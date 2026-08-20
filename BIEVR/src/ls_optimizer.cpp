// This Levenberg-Marquardt Optimizer builds upon the code used in the nanogicp module in DLIO
// https://github.com/vectr-ucla/direct_lidar_inertial_odometry

#include "bievr_lio/ls_optimizer.h"

#include <Eigen/Eigenvalues>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_reduce.h>

#include <algorithm>
#include <numeric>
#include <random>
#include <vector>

#include "bievr_lio/log++.h"
#include "bievr_lio/utils.h"

namespace bievr {

namespace {

// Quantile (0..1) of a sorted vector.
double quantileSorted(const std::vector<double>& sorted, double q) {
  if (sorted.empty()) return 0.0;
  if (q <= 0.0) return sorted.front();
  if (q >= 1.0) return sorted.back();
  const double idx = q * (sorted.size() - 1);
  const size_t lo = static_cast<size_t>(std::floor(idx));
  const size_t hi = static_cast<size_t>(std::ceil(idx));
  if (lo == hi) return sorted[lo];
  const double frac = idx - lo;
  return sorted[lo] * (1.0 - frac) + sorted[hi] * frac;
}

Eigen::Quaterniond so3_exp(const Eigen::Vector3d& omega) {
  double theta_sq = omega.dot(omega);

  double theta;
  double imag_factor;
  double real_factor;
  if (theta_sq < 1e-10) {
    theta = 0;
    double theta_quad = theta_sq * theta_sq;
    imag_factor = 0.5 - 1.0 / 48.0 * theta_sq + 1.0 / 3840.0 * theta_quad;
    real_factor = 1.0 - 1.0 / 8.0 * theta_sq + 1.0 / 384.0 * theta_quad;
  } else {
    theta = std::sqrt(theta_sq);
    double half_theta = 0.5 * theta;
    imag_factor = std::sin(half_theta) / theta;
    real_factor = std::cos(half_theta);
  }

  return Eigen::Quaterniond(real_factor, imag_factor * omega.x(), imag_factor * omega.y(),
                            imag_factor * omega.z());
}

Transform perturb(const Transform& T0, const Eigen::Matrix<double, 6, 1>& xi) {
  Transform delta = Transform::Identity();
  delta.linear() = so3_exp(xi.head<3>()).toRotationMatrix();
  delta.translation() = xi.tail<3>();
  return T0 * delta;
}

}  // namespace

LsqRegistration::LsqRegistration(const BIEVRMap& map, const Pointcloud& geometry_source,
                                 const Pointcloud& intensity_source,
                                 const Intensities& intensity_values,
                                 const RegistrationConfig& config)
    : config_(config),
      map_(map),
      points_j_(geometry_source),
      intensity_points_j_(intensity_source),
      intensity_values_j_(intensity_values) {}

Transform LsqRegistration::computeTransformation(const Transform& T_W_L_init) {
  Transform x0 = T_W_L_init;

  skew_points_j_.resize(points_j_.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, points_j_.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t i = r.begin(); i != r.end(); ++i) {
                        skew_points_j_[i] = skew(points_j_[i]);
                      }
                    });

  intensity_skew_points_j_.resize(intensity_points_j_.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, intensity_points_j_.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t i = r.begin(); i != r.end(); ++i) {
                        intensity_skew_points_j_[i] = skew(intensity_points_j_[i]);
                      }
                    });

  // Round-5 photometric safety: reset per-frame diagnostics.
  photo_diag_ = PhotometricDiagnostics();
  photo_diag_.photometric_scale = config_.photometric_scale;
  photo_samples_.clear();

  const bool photo_requested = config_.photometric_residual || config_.shadow_photometric;
  const bool photo_active =
      config_.photometric_residual &&
      config_.photometric_elapsed_s >= config_.photometric_warmup_s;

  const Transform T_W_L_init_saved = T_W_L_init;
  bool first_cost = true;
  double y_initial = 0.0;

  if (config_.lm_debug_print) {
    LOG(I, "***************** optimize *****************");
  }

  for (int i = 0; i < config_.max_iterations && !converged_; i++) {
    Transform delta;
    bool accepted = false;
    const double y0 = stepLm(x0, delta, accepted);
    if (first_cost) {
      y_initial = y0;
      first_cost = false;
    }
    ++photo_diag_.lm_iterations;
    if (!accepted && isConverged(delta)) {
      // Small step that failed LM accept: treat as converged.
      converged_ = true;
      break;
    }
    if (!accepted) {
      LOG(W, "lm not converged!!");
      break;
    }
    converged_ = isConverged(delta);
  }

  photo_diag_.lm_initial_cost = y_initial;
  photo_diag_.lm_trial_steps = photo_diag_.lm_accepted + photo_diag_.lm_rejected;
  photo_diag_.reject_rate =
      photo_diag_.lm_trial_steps > 0
          ? static_cast<double>(photo_diag_.lm_rejected) / photo_diag_.lm_trial_steps
          : 0.0;

  // Frame pose correction (total).
  {
    const Eigen::Vector3d dt = x0.translation() - T_W_L_init_saved.translation();
    photo_diag_.pose_delta_translation_m = dt.norm();
    const Eigen::AngleAxisd aa(T_W_L_init_saved.rotation().transpose() * x0.rotation());
    photo_diag_.pose_delta_rotation_deg = std::abs(aa.angle()) * 180.0 / M_PI;
  }

  // Round-5: final-pose photometric diagnostics (uses the PRE-UPDATE map: the
  // pipeline runs registration before integrating the current scan).
  if (photo_requested) {
    Matrix66 H;
    Vector6 b;
    const double y_final = linearize(x0, &H, &b, /*collect_photo_stats=*/true);
    photo_diag_.lm_final_cost = y_final;
    finalizePhotoDiagnostics(x0, final_geo_acc_, final_photo_acc_);
    if (config_.photometric_fd_check && !photo_diag_.fd_done) {
      runPhotoFdCheck(x0);
    }
  }

  return x0;
}

bool LsqRegistration::isConverged(const Transform& delta) const {
  Eigen::Matrix3d R = delta.linear() - Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = delta.translation();

  Eigen::Matrix3d r_delta = 1.0 / config_.rotation_epsilon * R.array().abs();
  Eigen::Vector3d t_delta = 1.0 / config_.transformation_epsilon * t.array().abs();

  return std::max(r_delta.maxCoeff(), t_delta.maxCoeff()) < 1;
}

double LsqRegistration::linearizeGeometry(const Transform& T_W_L, bool compute_jacobians,
                                          Accumulator& acc) const {
  Accumulator result = tbb::parallel_deterministic_reduce(
      tbb::blocked_range<size_t>(0, points_j_.size()),
      acc,  // identity
      [&](const tbb::blocked_range<size_t>& r, Accumulator local_acc) -> Accumulator {
        for (size_t i = r.begin(); i != r.end(); ++i) {
          Point p_W = T_W_L.linear() * points_j_[i] + T_W_L.translation();
          size_t hash = map_.hashIndex(p_W);
          const Voxel* voxel = map_.getVoxel(hash);
          if (!voxel) {
            if (!map_.nearestVoxel(p_W, hash)) continue;
            voxel = map_.getVoxel(hash);
            if (!voxel) continue;
          }

          const double inv_size = map_.inv_px_size;
          const auto& T_C_W = voxel->T_C_W_;

          Rotation R_o_j = T_C_W.linear() * T_W_L.linear();
          const Point p_o = T_C_W * p_W;

          const double x = p_o.x() * inv_size;
          const double y = p_o.y() * inv_size;

          double I = 0.0;

          if (!compute_jacobians) {
            if (config_.img_residual) {
              if (!getSubPixelValue(voxel, x, y, I)) continue;
            }
            const double r = p_o.z() - I;
            local_acc.add(r, nullptr);
            continue;
          }

          double dIdx = 0.0;
          double dIdy = 0.0;
          if (config_.img_residual) {
            if (!sampleValueAndGradient(voxel, x, y, I, dIdx, dIdy)) continue;
          }

          Eigen::Matrix<double, 3, 6> SE3_Jac;
          SE3_Jac.block<3, 3>(0, 3) = R_o_j;  // d p_o / d t
          SE3_Jac.block<3, 3>(0, 0).noalias() = -R_o_j * skew_points_j_[i];

          Eigen::Matrix<double, 1, 2> I_Jac;
          if (config_.img_jacobian) {
            I_Jac(0, 0) = dIdx;
            I_Jac(0, 1) = dIdy;
            I_Jac *= inv_size;
          }

          Row6 J = SE3_Jac.row(2) - (I_Jac * SE3_Jac.topRows<2>());

          const double r = p_o.z() - I;
          local_acc.add(r, &J);
        }

        return local_acc;
      },
      [](const Accumulator& a, const Accumulator& b) -> Accumulator {
        Accumulator out = a;
        out.merge(b);
        return out;
      });
  acc = result;
  return acc.error_sum;
}

namespace {

// Bilinear sample of the (shared) voxel map weight at subpixel (x,y), using only
// valid corners and interpolating the actual weight values. Round-5 maturity gate.
inline bool sampleMapWeight(const Voxel* voxel, double x, double y, double& weight) {
  const int x0 = std::floor(x);
  const int y0 = std::floor(y);
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;
  const int max_x = voxel->bump_weights_.cols() - 1;
  const int max_y = voxel->bump_weights_.rows() - 1;
  if (x0 < 0 || y0 < 0 || x1 > max_x || y1 > max_y) return false;

  const auto& V = voxel->bump_weights_;
  const double dx = x - x0, dy = y - y0, dx1 = 1.0 - dx, dy1 = 1.0 - dy;
  const double v00 = V(y0, x0), v01 = V(y0, x1), v10 = V(y1, x0), v11 = V(y1, x1);
  const double w0 = dx1 * dy1 * (v00 > 0 ? 1 : 0);
  const double w1 = dx * dy1 * (v01 > 0 ? 1 : 0);
  const double w2 = dx1 * dy * (v10 > 0 ? 1 : 0);
  const double w3 = dx * dy * (v11 > 0 ? 1 : 0);
  const double ws = w0 + w1 + w2 + w3;
  if (ws == 0.0) return false;
  weight = (w0 * v00 + w1 * v01 + w2 * v10 + w3 * v11) / ws;
  return true;
}

}  // namespace

double LsqRegistration::linearizePhotometric(const Transform& T_W_L, bool compute_jacobians,
                                             Accumulator& acc, bool collect_stats) {
  const double lambda = config_.photometric_scale;
  const double min_weight = config_.photometric_min_map_weight;

  // For the final diagnostics we run serially so photo_samples_ can be filled
  // without data races; for the LM iterations we use the parallel reduce.
  if (collect_stats) {
    for (size_t i = 0; i < intensity_points_j_.size(); ++i) {
      ++photo_diag_.candidates;

      Point p_W = T_W_L.linear() * intensity_points_j_[i] + T_W_L.translation();
      size_t hash = map_.hashIndex(p_W);
      const Voxel* voxel = map_.getVoxel(hash);
      if (!voxel) {
        if (!map_.nearestVoxel(p_W, hash)) continue;
        voxel = map_.getVoxel(hash);
        if (!voxel) continue;
      }

      const double inv_size = map_.inv_px_size;
      const auto& T_C_W = voxel->T_C_W_;
      const Point p_o = T_C_W * p_W;
      const double x = p_o.x() * inv_size;
      const double y = p_o.y() * inv_size;

      double map_weight = 0.0;
      if (!sampleMapWeight(voxel, x, y, map_weight)) continue;
      if (map_weight < min_weight) {
        ++photo_diag_.immature_matches;
        continue;
      }

      const double I_i = intensity_values_j_(0, i);
      double I_P = 0.0;
      if (!compute_jacobians) {
        if (!getSubPixelIntensityValue(voxel, x, y, I_P)) continue;
        const double r = lambda * (I_i - I_P);
        acc.add(r, nullptr);
        continue;
      }

      double dIdu = 0.0, dIdv = 0.0;
      if (!sampleIntensityValueAndGradient(voxel, x, y, I_P, dIdu, dIdv)) continue;

      Eigen::Matrix<double, 3, 6> SE3_Jac;
      SE3_Jac.block<3, 3>(0, 3) = T_C_W.linear() * T_W_L.linear();
      SE3_Jac.block<3, 3>(0, 0).noalias() =
          -SE3_Jac.block<3, 3>(0, 3) * intensity_skew_points_j_[i];

      Eigen::RowVector2d photo_grad;
      photo_grad << dIdu, dIdv;
      photo_grad *= inv_size;
      Row6 J_photo = -(photo_grad * SE3_Jac.topRows<2>());

      const double r = lambda * (I_i - I_P);
      const Row6 J_scaled = lambda * J_photo;
      acc.add(r, &J_scaled);

      ++photo_diag_.valid_matches;
      PhotoSample sample;
      sample.p_j = intensity_points_j_[i];
      sample.I_i = I_i;
      sample.r = I_i - I_P;
      sample.J = J_photo;
      sample.grad_norm = photo_grad.norm();
      photo_samples_.push_back(sample);
    }
    return acc.error_sum;
  }

  Accumulator result = tbb::parallel_deterministic_reduce(
      tbb::blocked_range<size_t>(0, intensity_points_j_.size()),
      acc,  // identity
      [&](const tbb::blocked_range<size_t>& r, Accumulator local_acc) -> Accumulator {
        for (size_t i = r.begin(); i != r.end(); ++i) {
          Point p_W = T_W_L.linear() * intensity_points_j_[i] + T_W_L.translation();
          size_t hash = map_.hashIndex(p_W);
          const Voxel* voxel = map_.getVoxel(hash);
          if (!voxel) {
            if (!map_.nearestVoxel(p_W, hash)) continue;
            voxel = map_.getVoxel(hash);
            if (!voxel) continue;
          }

          const double inv_size = map_.inv_px_size;
          const auto& T_C_W = voxel->T_C_W_;
          const Point p_o = T_C_W * p_W;

          const double x = p_o.x() * inv_size;
          const double y = p_o.y() * inv_size;

          // Map-maturity gate: only use photo matches whose (shared) map weight
          // W(u,v) >= photometric_min_map_weight.
          double map_weight = 0.0;
          if (!sampleMapWeight(voxel, x, y, map_weight)) continue;
          if (map_weight < min_weight) {
            continue;
          }

          const double I_i = intensity_values_j_(0, i);

          double I_P = 0.0;
          if (!compute_jacobians) {
            if (!getSubPixelIntensityValue(voxel, x, y, I_P)) continue;
            const double r = lambda * (I_i - I_P);
            local_acc.add(r, nullptr);
            continue;
          }

          double dIdu = 0.0;
          double dIdv = 0.0;
          if (!sampleIntensityValueAndGradient(voxel, x, y, I_P, dIdu, dIdv)) continue;

          Eigen::Matrix<double, 3, 6> SE3_Jac;
          SE3_Jac.block<3, 3>(0, 3) = T_C_W.linear() * T_W_L.linear();
          SE3_Jac.block<3, 3>(0, 0).noalias() =
              -SE3_Jac.block<3, 3>(0, 3) * intensity_skew_points_j_[i];

          // Photometric Jacobian: -grad(I_P) * d(u,v)/d(xi). There is no z term
          // (the intensity residual does not involve p_o.z()).
          Eigen::RowVector2d photo_grad;
          photo_grad << dIdu, dIdv;
          photo_grad *= inv_size;
          Row6 J_photo = -(photo_grad * SE3_Jac.topRows<2>());

          const double r = lambda * (I_i - I_P);
          const Row6 J_scaled = lambda * J_photo;
          local_acc.add(r, &J_scaled);

          if (collect_stats) {
            ++photo_diag_.valid_matches;
            LsqRegistration::PhotoSample sample;
            sample.p_j = intensity_points_j_[i];
            sample.I_i = I_i;
            sample.r = I_i - I_P;  // unscaled residual
            sample.J = J_photo;    // unscaled Jacobian
            sample.grad_norm = (photo_grad).norm();
            photo_samples_.push_back(sample);
          }
        }

        return local_acc;
      },
      [](const Accumulator& a, const Accumulator& b) -> Accumulator {
        Accumulator out = a;
        out.merge(b);
        return out;
      });
  acc = result;
  return acc.error_sum;
}

double LsqRegistration::linearize(const Transform& T_W_L, Matrix66* H, Vector6* b,
                                  bool collect_photo_stats) {
  const bool compute_jacobians = (H != nullptr && b != nullptr);

  const bool photo_requested = config_.photometric_residual || config_.shadow_photometric;
  const bool photo_active =
      config_.photometric_residual &&
      config_.photometric_elapsed_s >= config_.photometric_warmup_s;

  Accumulator geo_acc;
  geo_acc.huber_delta = config_.huber_delta;
  geo_acc.collect_statistics = compute_jacobians;
  linearizeGeometry(T_W_L, compute_jacobians, geo_acc);

  Accumulator photo_acc;
  photo_acc.huber_delta = config_.huber_delta;
  photo_acc.collect_statistics = compute_jacobians;
  if (photo_requested) {
    if (collect_photo_stats) photo_samples_.clear();
    linearizePhotometric(T_W_L, compute_jacobians, photo_acc, collect_photo_stats);
  }

  Accumulator total = geo_acc;
  if (photo_active) total.merge(photo_acc);

  if (compute_jacobians) {
    *H = total.H;
    *b = total.b;
    // Remember how many points contributed correspondences in this (Jacobian)
    // linearization so the pipeline can report the effective point count.
    num_geometry_effective_points_ = geo_acc.count;
    num_photometric_effective_points_ = photo_acc.count;
    num_effective_points_ = total.count;
    geometry_rmse_ =
        std::sqrt(geo_acc.residual_squared_sum / std::max(static_cast<double>(geo_acc.count), 1.0));
    photometric_rmse_ = std::sqrt(photo_acc.residual_squared_sum /
                                  std::max(static_cast<double>(photo_acc.count), 1.0));
    if (collect_photo_stats) {
      final_geo_acc_ = geo_acc;
      final_photo_acc_ = photo_acc;
    }
  }

  return total.error_sum;
}

bool LsqRegistration::stepLm(Transform& x0, Transform& delta, bool& accepted) {
  Matrix66 H;
  Vector6 b;
  double y0 = linearize(x0, &H, &b);

  if (lm_lambda_ < 0.0) {
    lm_lambda_ = config_.lm_init_lambda_factor * H.diagonal().array().abs().maxCoeff();
  }

  double nu = 2.0;
  for (int i = 0; i < config_.lm_max_iterations; i++) {
    Eigen::LDLT<Matrix66> solver(H + lm_lambda_ * Matrix66::Identity());
    Vector6 d = solver.solve(-b);
    delta.setIdentity();
    delta.linear() = so3_exp(d.head<3>()).toRotationMatrix();
    delta.translation() = d.tail<3>();

    Transform xi = x0 * delta;
    double yi = linearize(xi);
    double rho = (y0 - yi) / (d.dot(lm_lambda_ * d - b));

    if (rho < 0) {
      ++photo_diag_.lm_rejected;
      if (isConverged(delta)) {
        accepted = false;
        return true;
      }
      lm_lambda_ *= nu;
      nu *= 2;
      continue;
    }

    x0 = xi;
    lm_lambda_ *= std::max(1.0 / 3.0, 1 - std::pow(2 * rho - 1, 3));
    ++photo_diag_.lm_accepted;
    accepted = true;
    return true;
  }

  accepted = false;
  return false;
}

void LsqRegistration::finalizePhotoDiagnostics(const Transform& T_W_L,
                                               const Accumulator& geo_acc,
                                               const Accumulator& photo_acc) {
  const bool photo_active =
      config_.photometric_residual &&
      config_.photometric_elapsed_s >= config_.photometric_warmup_s;
  auto& d = photo_diag_;
  d.match_ratio = d.candidates > 0 ? static_cast<double>(d.valid_matches) / d.candidates : 0.0;
  d.photo_effective_residuals = photo_acc.count;
  d.geo_cost = geo_acc.error_sum;
  d.photo_cost = photo_acc.error_sum;

  // Residual / Jacobian / gradient percentiles from the collected samples.
  std::vector<double> abs_r, signed_r, j_norm, grad, j_dof[6];
  abs_r.reserve(photo_samples_.size());
  signed_r.reserve(photo_samples_.size());
  j_norm.reserve(photo_samples_.size());
  grad.reserve(photo_samples_.size());
  for (int k = 0; k < 6; ++k) j_dof[k].reserve(photo_samples_.size());
  for (const auto& s : photo_samples_) {
    abs_r.push_back(std::abs(s.r));
    signed_r.push_back(s.r);
    grad.push_back(s.grad_norm);
    j_norm.push_back(s.J.norm());
    for (int k = 0; k < 6; ++k) j_dof[k].push_back(std::abs(s.J(0, k)));
  }
  auto pct = [](std::vector<double>& v, double q) {
    std::sort(v.begin(), v.end());
    return quantileSorted(v, q);
  };
  d.r_abs_p50 = pct(abs_r, 0.50);
  d.r_abs_p75 = pct(abs_r, 0.75);
  d.r_abs_p90 = pct(abs_r, 0.90);
  d.r_abs_p95 = pct(abs_r, 0.95);
  d.r_abs_max = abs_r.empty() ? 0.0 : *std::max_element(abs_r.begin(), abs_r.end());
  d.r_signed_median = pct(signed_r, 0.50);
  d.grad_p50 = pct(grad, 0.50);
  d.grad_p75 = pct(grad, 0.75);
  d.grad_p90 = pct(grad, 0.90);
  d.grad_p95 = pct(grad, 0.95);
  d.J_norm_p50 = pct(j_norm, 0.50);
  d.J_norm_p90 = pct(j_norm, 0.90);
  d.J_norm_p99 = pct(j_norm, 0.99);
  d.J_norm_max = j_norm.empty() ? 0.0 : *std::max_element(j_norm.begin(), j_norm.end());
  for (int k = 0; k < 6; ++k) d.J_dof_p90[k] = pct(j_dof[k], 0.90);

  // Hessian / gradient norms. photo H/b are lambda^2-scaled (the Accumulator
  // scales r and J by lambda before add()).
  d.H_geo_fro = geo_acc.H.norm();
  d.H_photo_fro = photo_acc.H.norm();
  d.H_photo_trace = photo_acc.H.trace();
  d.b_geo_norm = geo_acc.b.norm();
  d.b_photo_norm = photo_acc.b.norm();
  d.R_H_scaled = d.H_geo_fro > 0 ? d.H_photo_fro / d.H_geo_fro : 0.0;

  const double lambda = config_.photometric_scale;
  const double lambda_sq = lambda * lambda;
  if (lambda_sq > 0.0) {
    const double H_photo_unscaled = d.H_photo_fro / lambda_sq;
    const double b_photo_unscaled = d.b_photo_norm / lambda_sq;
    d.R_H_unscaled = d.H_geo_fro > 0 ? H_photo_unscaled / d.H_geo_fro : 0.0;
    d.R_b_unscaled = d.b_geo_norm > 0 ? b_photo_unscaled / d.b_geo_norm : 0.0;
    // Per-frame Hessian-balance reference: lambda such that
    // lambda^2 * |H_photo|_unscaled ~ 0.1 * |H_geo|.
    if (d.R_H_unscaled > 0.0) {
      d.lambda_ref = std::sqrt(0.1 / d.R_H_unscaled);
    }
  }

  // Photo Hessian eigenvalues (ascending), on the lambda-scaled Hessian.
  Eigen::SelfAdjointEigenSolver<Matrix66> eig(photo_acc.H);
  d.photo_eigenvalues = eig.eigenvalues();

  // Photo-induced pose step: geometry-only vs joint LM step at this pose, using
  // the current LM damping. Diagnostic only; does not change the estimator.
  if (photo_active && photo_acc.count > 0) {
    const double mu = lm_lambda_ >= 0.0 ? lm_lambda_ : 0.0;
    const Matrix66 Hg = geo_acc.H;
    const Vector6 bg = geo_acc.b;
    const Matrix66 Hp = photo_acc.H;  // already lambda^2-scaled
    const Vector6 bp = photo_acc.b;
    Eigen::LDLT<Matrix66> g_solver(Hg + mu * Matrix66::Identity());
    Eigen::LDLT<Matrix66> j_solver(Hg + Hp + mu * Matrix66::Identity());
    const Vector6 dg = g_solver.solve(-bg);
    const Vector6 dj = j_solver.solve(-(bg + bp));
    const Vector6 dphoto = dj - dg;
    d.photo_step_translation_m = dphoto.tail<3>().norm();
    const Eigen::AngleAxisd aa(so3_exp(dphoto.head<3>()).toRotationMatrix());
    d.photo_step_rotation_deg = std::abs(aa.angle()) * 180.0 / M_PI;
  }
}

void LsqRegistration::runPhotoFdCheck(const Transform& T_W_L) {
  if (photo_samples_.empty()) return;

  // Sample up to 40 random mature matches for the spot check.
  std::mt19937 rng(12345);
  std::vector<PhotoSample> chosen;
  chosen.reserve(40);
  if (photo_samples_.size() <= 40) {
    chosen = photo_samples_;
  } else {
    std::vector<size_t> idx(photo_samples_.size());
    std::iota(idx.begin(), idx.end(), 0);
    std::shuffle(idx.begin(), idx.end(), rng);
    for (size_t k = 0; k < 40; ++k) chosen.push_back(photo_samples_[idx[k]]);
  }

  auto residual_at = [&](const Transform& T, const PhotoSample& s) -> double {
    const Point p_W = T.linear() * s.p_j + T.translation();
    size_t hash = map_.hashIndex(p_W);
    const Voxel* voxel = map_.getVoxel(hash);
    if (!voxel) {
      if (!map_.nearestVoxel(p_W, hash)) return std::numeric_limits<double>::quiet_NaN();
      voxel = map_.getVoxel(hash);
      if (!voxel) return std::numeric_limits<double>::quiet_NaN();
    }
    const Point p_o = voxel->T_C_W_ * p_W;
    const double x = p_o.x() * map_.inv_px_size;
    const double y = p_o.y() * map_.inv_px_size;
    double weight = 0.0;
    if (!sampleMapWeight(voxel, x, y, weight)) return std::numeric_limits<double>::quiet_NaN();
    if (weight < config_.photometric_min_map_weight) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    double I_P = 0.0;
    if (!getSubPixelIntensityValue(voxel, x, y, I_P)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return s.I_i - I_P;
  };

  const double eps = 1e-6;
  for (const auto& s : chosen) {
    Eigen::Matrix<double, 6, 1> J_num;
    bool ok = true;
    for (int dof = 0; dof < 6; ++dof) {
      Eigen::Matrix<double, 6, 1> xi_p = Eigen::Matrix<double, 6, 1>::Zero();
      Eigen::Matrix<double, 6, 1> xi_m = Eigen::Matrix<double, 6, 1>::Zero();
      xi_p(dof) = eps;
      xi_m(dof) = -eps;
      const double rp = residual_at(perturb(T_W_L, xi_p), s);
      const double rm = residual_at(perturb(T_W_L, xi_m), s);
      if (std::isnan(rp) || std::isnan(rm)) {
        ok = false;
        break;
      }
      J_num(0, dof) = (rp - rm) / (2.0 * eps);
    }
    if (!ok) continue;
    const double max_abs = J_num.cwiseAbs().maxCoeff();
    const double scale = std::max(max_abs, 1e-8);
    for (int dof = 0; dof < 6; ++dof) {
      const double rel = std::abs(s.J(0, dof) - J_num(0, dof)) / scale;
      fd_rel_errors_.push_back(rel);
    }
  }

  if (fd_rel_errors_.size() >= 100) {
    std::sort(fd_rel_errors_.begin(), fd_rel_errors_.end());
    photo_diag_.fd_samples = static_cast<int>(fd_rel_errors_.size());
    photo_diag_.fd_median_rel_err = quantileSorted(fd_rel_errors_, 0.50);
    photo_diag_.fd_p95_rel_err = quantileSorted(fd_rel_errors_, 0.95);
    photo_diag_.fd_done = true;
  }
}

}  // namespace bievr
