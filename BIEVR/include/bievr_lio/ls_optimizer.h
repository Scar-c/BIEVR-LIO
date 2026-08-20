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

 private:
  bool isConverged(const Transform& delta) const;

  double linearizeGeometry(const Transform& T_W_L, bool compute_jacobians, Accumulator& acc) const;
  double linearizePhotometric(const Transform& T_W_L, bool compute_jacobians,
                              Accumulator& acc) const;
  double linearize(const Transform& T_W_L, Matrix66* H = nullptr, Vector6* b = nullptr);

  bool stepLm(Transform& x0, Transform& delta);

  RegistrationConfig config_;
  double lm_lambda_ = -1.0;
  const BIEVRMap& map_;
  const Pointcloud& points_j_;
  std::vector<M3> skew_points_j_;
  const Pointcloud& intensity_points_j_;
  const Intensities& intensity_values_j_;
  std::vector<M3> intensity_skew_points_j_;
  bool converged_ = false;
  int num_effective_points_ = 0;
  int num_geometry_effective_points_ = 0;
  int num_photometric_effective_points_ = 0;
  double geometry_rmse_ = 0.0;
  double photometric_rmse_ = 0.0;
};

}  // namespace bievr
#endif  // BIEVR_LIO_LS_OPTIMIZER_H_