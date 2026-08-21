#ifndef BIEVR_LIO_LS_OPTIMIZER_H_
#define BIEVR_LIO_LS_OPTIMIZER_H_

#include <memory>

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/common.h"

namespace bievr {

using Matrix66 = Eigen::Matrix<double, 6, 6>;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Row6 = Eigen::Matrix<double, 1, 6, Eigen::RowMajor>;

struct RegistrationConfig {
  double lm_init_lambda_factor = 1e-9;
  double rotation_epsilon = 1e-4;
  double transformation_epsilon = 1e-5;
  double huber_delta = 0.1;
  int max_iterations = 20;
  int lm_max_iterations = 20;
  bool lm_debug_print = false;
  bool img_residual = true;
  bool img_jacobian = true;
  // COIN-BIEVR photometric residual: master switch and the scaling constant
  // lambda that balances meter-scale geometry residuals against the 0..255
  // intensity residuals. The COIN-BIEVR supplement does not specify the exact
  // value (Section 55 of the reproduction plan).
  bool photometric_residual = false;
  double photometric_scale = 1.0;
  // Round-5 photometric safety (engineering safeguards / TBD, not paper params):
  //   shadow_photometric: compute photo residual/J/H/b diagnostics WITHOUT
  //     merging into the LM solve (used by the C0 shadow run; never changes
  //     the trajectory).
  //   photometric_warmup_s / photometric_elapsed_s: photo residual enters the
  //     LM only after the pipeline has been in Running for photometric_warmup_s
  //     seconds (photometric_elapsed_s is set per frame by the pipeline).
  //   photometric_min_map_weight: map-maturity gate; a photo match is only used
  //     when the (shared) voxel weight W(u,v) >= this value.
  //   photometric_fd_check: run a real-map finite-difference Jacobian spot
  //     check on a sample of matches (round-5 validation; off by default).
  bool shadow_photometric = false;
  double photometric_warmup_s = 0.0;
  double photometric_elapsed_s = 0.0;
  double photometric_min_map_weight = 1.0;
  bool photometric_fd_check = false;
  // Round-7: in shadow mode, evaluate the DIRECT robustified photo contribution
  // across the fixed lambda grid (no /lambda^2 unscaling; see
  // runRobustShadowScan). Diagnostics only.
  bool shadow_lambda_scan = false;
};

// Per-frame photometric safety diagnostics (round 5). Filled by the final
// linearization of LsqRegistration::computeTransformation() when any photo work
// is requested (shadow or real). See the round-5 instruction for definitions.
struct PhotometricDiagnostics {
  // Coverage
  int candidates = 0;        // intensity source points considered
  int valid_matches = 0;     // mature matches that produced a residual
  int immature_matches = 0;  // skipped by the map-maturity gate
  double match_ratio = 0.0;  // valid_matches / candidates
  // Unscaled photometric residuals r = I_i - I_P (intensity units, 0..255).
  double r_abs_p50 = 0, r_abs_p75 = 0, r_abs_p90 = 0, r_abs_p95 = 0, r_abs_max = 0;
  double r_signed_median = 0;
  // Intensity-map gradient magnitude (per meter).
  double grad_p50 = 0, grad_p75 = 0, grad_p90 = 0, grad_p95 = 0;
  // Unscaled photometric Jacobian ||J_photo|| (1x6) and per-DOF |J|.
  double J_norm_p50 = 0, J_norm_p90 = 0, J_norm_p99 = 0, J_norm_max = 0;
  double J_dof_p90[6] = {0, 0, 0, 0, 0, 0};
  // Hessian / gradient: photo quantities are accumulated as the real optimizer
  // does (r and J scaled by lambda, then Huber IRLS in Accumulator::add), so
  // H_photo is piecewise lambda^2 (quadratic region) / lambda (linear region).
  double H_geo_fro = 0.0, H_photo_fro = 0.0, H_photo_trace = 0.0;
  double b_geo_norm = 0.0, b_photo_norm = 0.0;
  double R_H_scaled = 0.0;  // |H_photo| / |H_geo|  (direct, at this run's lambda)
  // DEPRECATED_INVALID (round-7): these were computed by dividing H_photo /
  // b_photo by lambda^2, which is only valid in the Huber quadratic region and
  // must NOT be used as calibrated quantities. Kept as zeroed placeholders for
  // CSV compatibility.
  double R_H_unscaled = 0.0;  // DEPRECATED_INVALID
  double R_b_unscaled = 0.0;  // DEPRECATED_INVALID
  double lambda_ref = 0.0;    // DEPRECATED_INVALID
  Eigen::Matrix<double, 6, 1> photo_eigenvalues = Eigen::Matrix<double, 6, 1>::Zero();
  // Photo-induced pose step difference (C-lambda only; 0 for shadow).
  double photo_step_translation_m = 0.0;
  double photo_step_rotation_deg = 0.0;
  // LM stability
  int lm_iterations = 0;
  int lm_trial_steps = 0, lm_accepted = 0, lm_rejected = 0;
  double reject_rate = 0.0;
  double lm_initial_cost = 0.0, lm_final_cost = 0.0;
  double geo_cost = 0.0, photo_cost = 0.0;
  double pose_delta_translation_m = 0.0, pose_delta_rotation_deg = 0.0;
  int photo_effective_residuals = 0;
  double photometric_scale = 0.0;
  // Real-map finite-difference Jacobian check (aggregated across frames).
  int fd_samples = 0;
  double fd_median_rel_err = 0.0, fd_p95_rel_err = 0.0;
  bool fd_done = false;
  // Round-6 three-level derivative validation.
  // Level A: image-space derivative (real map).
  int fd_level_a_samples = 0;
  double fd_level_a_median = 0.0, fd_level_a_p95 = 0.0, fd_level_a_sign = 0.0;
  // Level B: fixed-correspondence 6-DOF pose Jacobian.
  int fd_level_b_points = 0, fd_level_b_scalars = 0;
  double fd_level_b_median = 0.0, fd_level_b_p95 = 0.0, fd_level_b_sign = 0.0;
  double fd_level_b_rx_median = 0.0, fd_level_b_rx_p95 = 0.0, fd_level_b_rx_sign = 0.0;
  double fd_level_b_ry_median = 0.0, fd_level_b_ry_p95 = 0.0, fd_level_b_ry_sign = 0.0;
  double fd_level_b_rz_median = 0.0, fd_level_b_rz_p95 = 0.0, fd_level_b_rz_sign = 0.0;
  double fd_level_b_tx_median = 0.0, fd_level_b_tx_p95 = 0.0, fd_level_b_tx_sign = 0.0;
  double fd_level_b_ty_median = 0.0, fd_level_b_ty_p95 = 0.0, fd_level_b_ty_sign = 0.0;
  double fd_level_b_tz_median = 0.0, fd_level_b_tz_p95 = 0.0, fd_level_b_tz_sign = 0.0;
  double fd_level_b_analytic_p50 = 0.0, fd_level_b_analytic_p90 = 0.0, fd_level_b_analytic_p99 = 0.0;
  double fd_level_b_numeric_p50 = 0.0, fd_level_b_numeric_p90 = 0.0, fd_level_b_numeric_p99 = 0.0;
  // Level C: full-lookup switching (diagnostic only).
  double fd_level_c_voxel_switch = 0.0, fd_level_c_cell_switch = 0.0;
  double fd_level_c_validity_switch = 0.0;
  int fd_level_c_noswitch_samples = 0;
  double fd_level_c_noswitch_median = 0.0, fd_level_c_noswitch_p95 = 0.0;
  // Round-8: near-range (< 1.5 m) photo match statistics (Livox Avia near-range
  // intensity artifacts; diagnostic only, no filtering applied).
  double photo_near_fraction = 0.0;          // photo matches with range < 1.5 m
  double photo_near_r_p50 = 0.0, photo_near_r_p90 = 0.0;
  double photo_far_r_p50 = 0.0, photo_far_r_p90 = 0.0;
  // Round-10: full near (<1.5 m) / far (>=1.5 m) photometric breakdown over the
  // matched photo samples (diagnostic only, no filtering): residual, Huber
  // inlier fraction, intensity-gradient magnitude and J norm percentiles, plus
  // the near/total direct robustified photo-Hessian fraction at this lambda.
  double photo_near_r_p95 = 0.0, photo_far_r_p95 = 0.0;
  double photo_near_inlier_fraction = 0.0, photo_far_inlier_fraction = 0.0;
  double photo_near_grad_p50 = 0.0, photo_near_grad_p90 = 0.0;
  double photo_far_grad_p50 = 0.0, photo_far_grad_p90 = 0.0;
  double photo_near_J_p50 = 0.0, photo_near_J_p90 = 0.0;
  double photo_far_J_p50 = 0.0, photo_far_J_p90 = 0.0;
  double photo_near_H_fraction = 0.0;  // H_photo_near / H_photo_total (direct)
  // Round-10: fraction of the intensity SOURCE points (sampled intensity points,
  // IMU frame) with range < 1.5 m. Filled by the pipeline.
  double source_near_fraction = 0.0;
  // Round-8: geometry weak eigenvalues (lambda1<=lambda2<=lambda3) and the
  // two-weak-direction flag (10*lambda1 > lambda2), from the Eq.7 degeneracy
  // analysis. Filled by the pipeline (intensity_samples.weak_eigenvalues).
  double geo_lambda1 = 0.0, geo_lambda2 = 0.0, geo_lambda3 = 0.0;
  int geo_two_weak_flag = 0;
  // Round-9: weak-direction / Eq.7-8 audit (diagnostic only; sections 42-49).
  // All per-frame raw values; the analysis script aggregates percentiles.
  Eigen::Vector3d eta_world = Eigen::Vector3d::Zero();  // eta (world frame)
  // Temporal metrics vs the previous frame (|eta_t^T eta_{t-1}| and the
  // modulo-sign angle in degrees).
  double eta_abs_dot_prev = 1.0;
  double eta_angle_prev_deg = 0.0;
  // ||P_w(t) - P_w(t-1)||_F with P_w = v1 v1^T + v2 v2^T (sign/basis agnostic).
  double weak_subspace_diff_F = 0.0;
  // Selected top-100 voxel turnover (0 when no previous frame).
  double selected_voxel_jaccard = 0.0;
  double selected_voxel_retention = 0.0;
  // Eq.8 selected / candidate score distributions.
  double selected_score_p10 = 0.0, selected_score_p50 = 0.0, selected_score_p90 = 0.0,
         selected_score_max = 0.0;
  double candidate_score_p50 = 0.0, candidate_score_p90 = 0.0, candidate_score_p99 = 0.0;
  // Directional Hessian / gradient information projected onto the two weak
  // eigenvectors (converted to the IMU/body frame of the right perturbation):
  // q = d^T H d, s = d^T b. H_photo is the DIRECT robustified production
  // Hessian at this frame's lambda (never H/lambda^2).
  double q_geo_v1 = 0.0, q_geo_v2 = 0.0, q_photo_v1 = 0.0, q_photo_v2 = 0.0;
  double s_geo_v1 = 0.0, s_geo_v2 = 0.0, s_photo_v1 = 0.0, s_photo_v2 = 0.0;
  // Round-9 real-data serial-vs-production photometric parity (shadow C0 runs).
  // Filled by the pipeline via runPhotometricParity at the final pose.
  int parity_count_serial = 0, parity_count_parallel = 0;
  double parity_cost_rel = 0.0, parity_H_rel = 0.0, parity_b_rel = 0.0;
};

// Round-7: per-(lambda, frame) evaluation of the DIRECT robustified photometric
// contribution at the geometry-only LM solution (shadow; never enters solve).
struct PhotoScaleEvaluation {
  double lambda = 0.0;
  double raw_huber_knee = 0.0;            // huber_delta / lambda
  double huber_inlier_fraction = 0.0;     // |lambda*r| <= huber_delta
  double H_photo_fro = 0.0;
  double H_geo_fro = 0.0;
  double R_H = 0.0;                       // |H_photo| / |H_geo| (direct)
  double b_photo_norm = 0.0;
  double b_geo_norm = 0.0;
  double photo_cost = 0.0;                // sum rho(lambda*r)
  double geo_cost = 0.0;
  double pred_translation_m = 0.0;        // |d_joint - d_geo| (translation)
  double pred_rotation_deg = 0.0;         // AngleAxis angle of (d_joint - d_geo)
};

// Bilinear sample of `image` at subpixel (x, y). `weights` acts as the
// validity mask; corners with weight <= 0 are ignored (re-normalized). Returns
// false if the 2x2 stencil is out of bounds or has no valid corners.
inline bool sampleImageValue(const Eigen::MatrixXf& image, const Eigen::MatrixXf& weights,
                             const double x, const double y, double& value) {
  const int x0 = std::floor(x);
  const int y0 = std::floor(y);

  const int x1 = x0 + 1;
  const int y1 = y0 + 1;

  const int max_x = image.cols() - 1;
  const int max_y = image.rows() - 1;
  if (x0 < 0 || y0 < 0 || x1 > max_x || y1 > max_y) return false;

  const double dx = x - x0;
  const double dy = y - y0;
  const double dx1 = 1.0 - dx;
  const double dy1 = 1.0 - dy;

  const int v00 = weights(y0, x0) > 0, v01 = weights(y0, x1) > 0;
  const int v10 = weights(y1, x0) > 0, v11 = weights(y1, x1) > 0;

  const double w0 = dx1 * dy1 * v00;
  const double w1 = dx * dy1 * v01;
  const double w2 = dx1 * dy * v10;
  const double w3 = dx * dy * v11;

  const double wsum = w0 + w1 + w2 + w3;
  if (wsum == 0.0) return false;

  value = (w0 * image(y0, x0) + w1 * image(y0, x1) + w2 * image(y1, x0) + w3 * image(y1, x1)) /
          wsum;
  return true;
}

// Combined bilinear sample + central-difference gradient, generic over the
// image channel. Shares floor/bounds/fraction work and reuses the 4 inner
// corner reads. Returns false if the center 2x2 stencil is out of bounds or
// has no valid corners. Gradients are set to 0 when their 4x2 stencil is out of
// bounds or has no valid corners. (Same numerics as the original BIEVR
// sampleValueAndGradient, now shared by the height and intensity maps.)
inline bool sampleImageValueAndGradient(const Eigen::MatrixXf& image,
                                        const Eigen::MatrixXf& weights, const double x,
                                        const double y, double& value, double& dIdx,
                                        double& dIdy) {
  const int x0 = std::floor(x);
  const int y0 = std::floor(y);
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;

  const int max_x = image.cols() - 1;
  const int max_y = image.rows() - 1;
  if (x0 < 0 || y0 < 0 || x1 > max_x || y1 > max_y) return false;

  const auto& M = image;
  const auto& V = weights;

  const double dx = x - x0;
  const double dy = y - y0;
  const double dx1 = 1.0 - dx;
  const double dy1 = 1.0 - dy;

  const int v00 = V(y0, x0) > 0, v01 = V(y0, x1) > 0;
  const int v10 = V(y1, x0) > 0, v11 = V(y1, x1) > 0;
  {
    const double w0 = dx1 * dy1 * v00;
    const double w1 = dx * dy1 * v01;
    const double w2 = dx1 * dy * v10;
    const double w3 = dx * dy * v11;
    const double ws = w0 + w1 + w2 + w3;
    if (ws == 0.0) return false;
    value = (w0 * M(y0, x0) + w1 * M(y0, x1) + w2 * M(y1, x0) + w3 * M(y1, x1)) / ws;
  }

  dIdx = 0.0;
  dIdy = 0.0;

  if (x0 >= 1 && x1 + 1 <= max_x) {
    const int xm = x0 - 1;
    const int xp = x1 + 1;

    const int am0 = V(y0, xm) > 0, am2 = V(y1, xm) > 0;
    const double wm0 = dx1 * dy1 * am0;
    const double wm1 = dx * dy1 * v00;
    const double wm2 = dx1 * dy * am2;
    const double wm3 = dx * dy * v10;
    const double wms = wm0 + wm1 + wm2 + wm3;

    const int ap1 = V(y0, xp) > 0, ap3 = V(y1, xp) > 0;
    const double wp0 = dx1 * dy1 * v01;
    const double wp1 = dx * dy1 * ap1;
    const double wp2 = dx1 * dy * v11;
    const double wp3 = dx * dy * ap3;
    const double wps = wp0 + wp1 + wp2 + wp3;

    if (wms > 0.0 && wps > 0.0) {
      const double vm =
          (wm0 * M(y0, xm) + wm1 * M(y0, x0) + wm2 * M(y1, xm) + wm3 * M(y1, x0)) / wms;
      const double vp =
          (wp0 * M(y0, x1) + wp1 * M(y0, xp) + wp2 * M(y1, x1) + wp3 * M(y1, xp)) / wps;
      dIdx = 0.5 * (vp - vm);
    }
  }

  if (y0 >= 1 && y1 + 1 <= max_y) {
    const int ym = y0 - 1;
    const int yp = y1 + 1;

    const int am0 = V(ym, x0) > 0, am1 = V(ym, x1) > 0;
    const double wm0 = dx1 * dy1 * am0;
    const double wm1 = dx * dy1 * am1;
    const double wm2 = dx1 * dy * v00;
    const double wm3 = dx * dy * v01;
    const double wms = wm0 + wm1 + wm2 + wm3;

    const int ap2 = V(yp, x0) > 0, ap3 = V(yp, x1) > 0;
    const double wp0 = dx1 * dy1 * v10;
    const double wp1 = dx * dy1 * v11;
    const double wp2 = dx1 * dy * ap2;
    const double wp3 = dx * dy * ap3;
    const double wps = wp0 + wp1 + wp2 + wp3;

    if (wms > 0.0 && wps > 0.0) {
      const double vm =
          (wm0 * M(ym, x0) + wm1 * M(ym, x1) + wm2 * M(y0, x0) + wm3 * M(y0, x1)) / wms;
      const double vp =
          (wp0 * M(y1, x0) + wp1 * M(y1, x1) + wp2 * M(yp, x0) + wp3 * M(yp, x1)) / wps;
      dIdy = 0.5 * (vp - vm);
    }
  }

  return true;
}

inline bool getSubPixelValue(const Voxel* voxel, const double x, const double y, double& value) {
  return sampleImageValue(voxel->bump_smoothed_, voxel->bump_weights_, x, y, value);
}

inline bool getSubPixelIntensityValue(const Voxel* voxel, const double x, const double y,
                                      double& value) {
  return sampleImageValue(voxel->intensity_img_, voxel->bump_weights_, x, y, value);
}

// Combined bilinear sample + central-difference gradient.
// Shares floor/bounds/fraction work and reuses the 4 inner corner reads.
// Returns false if the center 2x2 stencil is out of bounds or has no valid corners.
// Gradients are set to 0 when their 4x2 stencil is out of bounds or has no valid corners.
inline bool sampleValueAndGradient(const Voxel* voxel, const double x, const double y,
                                   double& value, double& dIdx, double& dIdy) {
  return sampleImageValueAndGradient(voxel->bump_smoothed_, voxel->bump_weights_, x, y, value,
                                     dIdx, dIdy);
}

inline bool sampleIntensityValueAndGradient(const Voxel* voxel, const double x, const double y,
                                            double& value, double& dIdx, double& dIdy) {
  return sampleImageValueAndGradient(voxel->intensity_img_, voxel->bump_weights_, x, y, value,
                                     dIdx, dIdy);
}

// ---------------------------------------------------------------------------
// Round-6 exact photometric sampler (COIN-BIEVR photometric residual only).
//
// The residual value AND the intensity gradient come from the SAME masked
// normalized bilinear interpolation:
//   I(x,y) = (sum m_k w_k I_k) / (sum m_k w_k),   m_k = (bump_weights_ > 0)
// and the gradient is the EXACT derivative of that interpolation (quotient
// rule), NOT a central difference. Units are intensity per pixel.
//
// This does NOT change the geometry height gradient (sampleValueAndGradient)
// nor the Eq.6 intensity-information central difference.
// ---------------------------------------------------------------------------
struct IntensitySample {
  bool valid = false;
  double value = 0.0;
  // dI/dx_pixel, dI/dy_pixel (pixel units)
  Eigen::Vector2d gradient_pixel = Eigen::Vector2d::Zero();
  int x0 = 0, y0 = 0;
  double frac_x = 0.0, frac_y = 0.0;  // a = x - x0, b = y - y0
  double valid_weight_sum = 0.0;      // S = sum m_k w_k
};

inline bool sampleIntensityBilinearWithGradient(const Voxel* voxel, double x, double y,
                                                IntensitySample& out) {
  const int x0 = std::floor(x);
  const int y0 = std::floor(y);
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;
  const int max_x = voxel->intensity_img_.cols() - 1;
  const int max_y = voxel->intensity_img_.rows() - 1;
  if (x0 < 0 || y0 < 0 || x1 > max_x || y1 > max_y) return false;

  const double a = x - x0;
  const double b = y - y0;
  const double a1 = 1.0 - a;
  const double b1 = 1.0 - b;

  // Validity comes from the shared voxel weight (bump_weights_ > 0). A valid
  // intensity value of exactly 0 is still a valid measurement.
  const auto& I = voxel->intensity_img_;
  const auto& W = voxel->bump_weights_;
  const double m00 = W(y0, x0) > 0 ? 1.0 : 0.0;
  const double m10 = W(y0, x1) > 0 ? 1.0 : 0.0;
  const double m01 = W(y1, x0) > 0 ? 1.0 : 0.0;
  const double m11 = W(y1, x1) > 0 ? 1.0 : 0.0;

  // Geometry bilinear weights.
  const double w00 = a1 * b1, w10 = a * b1, w01 = a1 * b, w11 = a * b;
  const double S = m00 * w00 + m10 * w10 + m01 * w01 + m11 * w11;
  if (S <= 1e-12) return false;  // no valid corner: sample invalid

  const double I00 = I(y0, x0), I10 = I(y0, x1), I01 = I(y1, x0), I11 = I(y1, x1);
  const double N = m00 * w00 * I00 + m10 * w10 * I10 + m01 * w01 * I01 + m11 * w11 * I11;
  out.value = N / S;

  // Exact derivatives (quotient rule) w.r.t. a (= x in pixels, da/dx = 1):
  //   dw00/da = -(1-b), dw10/da = (1-b), dw01/da = -b, dw11/da = b
  const double dw00_da = -b1, dw10_da = b1, dw01_da = -b, dw11_da = b;
  const double Nx = m00 * dw00_da * I00 + m10 * dw10_da * I10 + m01 * dw01_da * I01 +
                    m11 * dw11_da * I11;
  const double Sx = m00 * dw00_da + m10 * dw10_da + m01 * dw01_da + m11 * dw11_da;
  out.gradient_pixel.x() = (Nx * S - N * Sx) / (S * S);

  //   dw00/db = -(1-a), dw10/db = -a, dw01/db = (1-a), dw11/db = a
  const double dw00_db = -a1, dw10_db = -a, dw01_db = a1, dw11_db = a;
  const double Ny = m00 * dw00_db * I00 + m10 * dw10_db * I10 + m01 * dw01_db * I01 +
                    m11 * dw11_db * I11;
  const double Sy = m00 * dw00_db + m10 * dw10_db + m01 * dw01_db + m11 * dw11_db;
  out.gradient_pixel.y() = (Ny * S - N * Sy) / (S * S);

  out.x0 = x0;
  out.y0 = y0;
  out.frac_x = a;
  out.frac_y = b;
  out.valid_weight_sum = S;
  out.valid = true;
  return true;
}

struct Accumulator {
  int count = 0;
  double error_sum = 0.0;
  Matrix66 H = Matrix66::Zero();
  Vector6 b = Vector6::Zero();
  double huber_delta = 0.2;  // default delta

  // Optional statistics collection (enabled per-accumulator for the dashboard).
  bool collect_statistics = false;
  int inlier_count = 0;
  double residual_squared_sum = 0.0;
  double residual_absolute_sum = 0.0;
  double max_absolute_residual = 0.0;

  inline void add(double r, const Row6* J) {
    ++count;
    double abs_r = std::abs(r);
    bool inlier = abs_r <= huber_delta;
    double w = inlier ? 1.0 : huber_delta / abs_r;

    error_sum += inlier ? 0.5 * r * r : huber_delta * (abs_r - 0.5 * huber_delta);

    if (collect_statistics) {
      if (inlier) ++inlier_count;
      residual_squared_sum += r * r;
      residual_absolute_sum += abs_r;
      max_absolute_residual = std::max(max_absolute_residual, abs_r);
    }

    if (J) {
      const Vector6 wJ = w * J->transpose();
      H.noalias() += wJ * (*J);
      b.noalias() += wJ * r;
    }
  }

  inline void merge(const Accumulator& other) {
    count += other.count;
    error_sum += other.error_sum;
    H += other.H;
    b += other.b;
    if (collect_statistics) {
      inlier_count += other.inlier_count;
      residual_squared_sum += other.residual_squared_sum;
      residual_absolute_sum += other.residual_absolute_sum;
      max_absolute_residual = std::max(max_absolute_residual, other.max_absolute_residual);
    }
  }
};

class LsqRegistration {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Joint geometry + photometric (COIN-BIEVR) registration. `intensity_source`
  // / `intensity_values` may be empty, in which case only the geometry residual
  // is optimized (identical to original BIEVR when config_.photometric_residual
  // is also false).
  LsqRegistration(const BIEVRMap& map, const Pointcloud& geometry_source,
                  const Pointcloud& intensity_source, const Intensities& intensity_values,
                  const RegistrationConfig& config = RegistrationConfig());

  // Geometry-only convenience constructor (original BIEVR signature), delegating
  // to the joint constructor with empty intensity sets.
  LsqRegistration(const BIEVRMap& map, const Pointcloud& geometry_source,
                  const RegistrationConfig& config = RegistrationConfig())
      : LsqRegistration(map, geometry_source, Pointcloud(), Intensities(), config) {}
  virtual ~LsqRegistration() = default;

  Transform computeTransformation(const Transform& T_W_L_init);

  // Number of source points that found a valid map correspondence in the last
  // linearization with Jacobians (i.e. the points that actually constrained the
  // pose). Reported on the dashboard as "Effective Points".
  int numEffectivePoints() const { return num_effective_points_; }
  // Breakdown of the effective points by residual type.
  int numGeometryEffectivePoints() const { return num_geometry_effective_points_; }
  int numPhotometricEffectivePoints() const { return num_photometric_effective_points_; }
  // RMSE of the last Jacobian linearization, per residual type (scaled
  // photometric residual for the photometric one).
  double geometryRMSE() const { return geometry_rmse_; }
  double photometricRMSE() const { return photometric_rmse_; }
  // Round-5 photometric safety diagnostics of the final pose linearization.
  const PhotometricDiagnostics& photometricDiagnostics() const { return photo_diag_; }

  // Round-9 P0 regression hook: photometric linearization through BOTH the
  // serial diagnostic path and the parallel production path at the same pose,
  // returning the achieved count/cost/H/b relative errors (diagnostics only).
  struct PhotometricParity {
    int count_serial = 0, count_parallel = 0;
    double cost_serial = 0.0, cost_parallel = 0.0;
    Matrix66 H_serial = Matrix66::Zero(), H_parallel = Matrix66::Zero();
    Vector6 b_serial = Vector6::Zero(), b_parallel = Vector6::Zero();
    double cost_rel = 0.0, H_rel = 0.0, b_rel = 0.0;
  };
  PhotometricParity runPhotometricParity(const Transform& T_W_L);

  // Round-9: Eq.7 weak eigenvectors (world frame) used only to compute the
  // directional Hessian/gradient audit at the final pose. Must be set before
  // computeTransformation. Diagnostic only; never changes the solve.
  void setWeakDirections(const Eigen::Vector3d& v1_W, const Eigen::Vector3d& v2_W) {
    v1_W_ = v1_W;
    v2_W_ = v2_W;
  }

 private:
  bool isConverged(const Transform& delta) const;

  // Round-9: SINGLE per-point photometric term implementation shared by the
  // serial diagnostic path, the parallel production accumulator and the FD
  // diagnostics, so the linearization math can never diverge between them.
  // Returns the match status; on kValid it fills r (unscaled residual
  // I_i - I_P), J_photo (unscaled, exactly one inv_px_size chain rule),
  // grad_per_meter and the voxel hash. `map_weight` is filled on kImmature too.
  enum class PhotoMatchStatus { kValid, kNoVoxel, kNoSample, kImmature };
  PhotoMatchStatus evaluatePhotometricTerm(const Transform& T_W_L, size_t i, double* I_P,
                                           double* map_weight, double* r, Row6* J_photo,
                                           Eigen::RowVector2d* grad_per_meter,
                                           size_t* voxel_hash) const;

  double linearizeGeometry(const Transform& T_W_L, bool compute_jacobians, Accumulator& acc) const;
  // Non-const: with collect_stats it also fills photo_diag_ / photo_samples_
  // (run serially to avoid data races on the shared sample vector).
  double linearizePhotometric(const Transform& T_W_L, bool compute_jacobians, Accumulator& acc,
                              bool collect_stats);
  double linearize(const Transform& T_W_L, Matrix66* H = nullptr, Vector6* b = nullptr,
                   bool collect_photo_stats = false);

  bool stepLm(Transform& x0, Transform& delta, bool& accepted);

  // Round-5: final-pose photo diagnostics, photo-induced step and the
  // finite-difference Jacobian spot check.
  void finalizePhotoDiagnostics(const Transform& T_W_L, const Accumulator& geo_acc,
                                const Accumulator& photo_acc);
  void runPhotoFdDiagnostics(const Transform& T_W_L);
  // Round-6 three-level derivative FD (image-space / fixed-correspondence /
  // full-lookup switching). Accumulates pools across frames.
  void runLevelAFd(const Transform& T_W_L);
  void runLevelBFd(const Transform& T_W_L);
  void runLevelCFd(const Transform& T_W_L);
  // Round-7: direct robustified photometric contribution across the fixed
  // lambda grid at the geometry-only solution (shadow only, never merges).
  void runRobustShadowScan(const Accumulator& geo_acc);

 public:
  // Per-frame results of the round-7 robust shadow lambda scan (one entry per
  // grid lambda), populated when config_.shadow_lambda_scan is enabled.
  const std::vector<PhotoScaleEvaluation>& robustShadowScan() const {
    return robust_scan_frame_;
  }

 private:
  // Per-match sample collected by the final photometric linearization.
  struct PhotoSample {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p_j;   // source point (IMU frame)
    double I_i = 0.0;      // source filtered intensity
    double r = 0.0;        // unscaled residual I_i - I_P
    Eigen::Matrix<double, 1, 6> J = Eigen::Matrix<double, 1, 6>::Zero();  // unscaled analytic J
    double grad_norm = 0.0;  // intensity gradient magnitude (per meter)
  };
  std::vector<PhotoSample> photo_samples_;
  // Round-7 per-frame robust shadow lambda scan results (one per grid lambda).
  std::vector<PhotoScaleEvaluation> robust_scan_frame_;

  RegistrationConfig config_;
  double lm_lambda_ = -1.0;
  const BIEVRMap& map_;
  const Pointcloud& points_j_;
  std::vector<M3> skew_points_j_;
  const Pointcloud& intensity_points_j_;
  const Intensities& intensity_values_j_;
  std::vector<M3> intensity_skew_points_j_;
  // Round-9: Eq.7 weak eigenvectors (world frame) for the directional audit.
  Eigen::Vector3d v1_W_ = Eigen::Vector3d::Zero(), v2_W_ = Eigen::Vector3d::Zero();
  bool converged_ = false;
  int num_effective_points_ = 0;
  int num_geometry_effective_points_ = 0;
  int num_photometric_effective_points_ = 0;
  double geometry_rmse_ = 0.0;
  double photometric_rmse_ = 0.0;
  PhotometricDiagnostics photo_diag_;
  Accumulator final_geo_acc_;   // final-pose geometry accumulator (round-5 diag)
  Accumulator final_photo_acc_; // final-pose photometric accumulator (round-5 diag)
  // Round-6 FD pools. STATIC because the pipeline constructs a fresh optimizer
  // every frame; the pools must persist across frames (single-process,
  // sequential odometry node) until the sample targets are reached.
  static std::vector<double> fd_a_errors_;
  static long fd_a_sign_agree_, fd_a_sign_total_;
  static std::vector<double> fd_b_errors_;
  static std::vector<double> fd_b_errors_dof_[6];
  static std::vector<double> fd_b_analytic_[6], fd_b_numeric_[6];
  static long fd_b_sign_agree_[6], fd_b_sign_total_[6];
  static long fd_b_points_;
  static long fd_c_total_pert_, fd_c_voxel_switch_, fd_c_cell_switch_, fd_c_validity_switch_;
  static std::vector<double> fd_c_noswitch_errors_;
  static bool first_frame_;  // reset the static pools on the first frame
};

}  // namespace bievr
#endif  // BIEVR_LIO_LS_OPTIMIZER_H_