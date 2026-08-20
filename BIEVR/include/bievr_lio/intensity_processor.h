#ifndef BIEVR_LIO_INTENSITY_PROCESSOR_H_
#define BIEVR_LIO_INTENSITY_PROCESSOR_H_

#include <Eigen/Core>
#include <string>

#include "bievr_lio/common.h"

// COIN-BIEVR intensity preprocessing (Section 5-9 of the reproduction plan).
//
// Per-frame per-point intensity normalization for irregular LiDAR scans:
//   1. Spherical projection of the LiDAR-frame point cloud into a (w x h)
//      intensity image (Eq. 1 of the COIN-BIEVR paper).
//   2. Optional Ouster line-artifact removal (vertical high-pass then
//      horizontal low-pass, per COIN-LIO).
//   3. Sparse brightness normalization: the brightness I_B is the average over
//      a box window that only counts non-empty pixels
//        I_F = s * I / (I_B + 1)
//   4. Optional masked Gaussian blur, then clamp to [0, 255].
//   5. The filtered value is written back to the per-point intensity channel of
//      the *original* 3D point via the point->pixel index map.
//
// The output Intensities row is index-aligned with the input point cloud, so it
// stays aligned with the (transformed / undistorted) spatial cloud downstream.
// When multiple points project into the same pixel, the closest one (smallest
// range) owns the pixel; all other points keep their raw (scaled) intensity.

namespace bievr {

struct IntensityProcessorConfig {
  bool enabled = true;

  std::string projection = "spherical";  // "spherical" (Eq. 1) or "ouster_lut"
  int image_width = 1024;                // spherical image width [pix]
  int image_height = 64;                 // spherical image height [pix]
  double vertical_fov_deg = 50.0;        // vertical FOV of the sensor [deg]

  // Brightness window (in pixels). 0 = full image (global sparse mean).
  int brightness_window_u = 0;
  int brightness_window_v = 0;

  double normalization_scale = 140.0;  // s in I_F = s * I / (I_B + 1)

  bool remove_lines = false;  // Ouster line-artifact removal (default: off)
  bool gaussian_blur = false;  // masked Gaussian blur after normalization

  double raw_intensity_scale = 1.0;  // raw intensity scaling before projection
  bool use_ouster_lut = false;       // whether a factory point->pixel LUT is used
};

// Per-pixel result of the spherical projection of a single scan.
struct ProjectedIntensityImage {
  Eigen::MatrixXf intensity;   // per-pixel intensity of the owning point
  Eigen::MatrixXi point_index; // index into the input cloud, -1 = empty
  Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> valid;  // mask
};

// Produces per-point filtered intensities from a LiDAR-frame point cloud.
class IntensityProcessor {
 public:
  explicit IntensityProcessor(const IntensityProcessorConfig& config = IntensityProcessorConfig());
  void configure(const IntensityProcessorConfig& config) { config_ = config; }
  const IntensityProcessorConfig& config() const { return config_; }

  // Projects `points_L` (LiDAR frame), normalizes the intensity image and
  // returns one filtered intensity value per input point, index-aligned.
  Intensities process(const Pointcloud& points_L, const IntensityView& raw_intensity);

  // Exposed for unit testing / debugging: project + normalize without writeback.
  ProjectedIntensityImage project(const Pointcloud& points_L,
                                  const IntensityView& raw_intensity) const;

  // Sparse brightness normalization of a (sparse) intensity image using only
  // non-empty pixels (mask > 0): I_F = s * I / (I_B + 1), optional line removal
  // and masked blur, then clamp to [0, 255]. Exposed for unit testing.
  Eigen::MatrixXf normalizeImage(const Eigen::MatrixXf& intensity_image,
                                 const Eigen::MatrixXi& mask) const;

 private:
  // Sparse box-average brightness I_B = sum(M*I) / max(sum(M), 1).
  Eigen::MatrixXd brightnessImage(const ProjectedIntensityImage& image) const;

  // COIN-LIO line-artifact removal (vertical HPF then horizontal LPF).
  void removeLines(Eigen::MatrixXf& image, const Eigen::MatrixXi& mask) const;

  // Masked separable Gaussian blur (only valid neighbours contribute).
  void gaussianBlur(Eigen::MatrixXf& image, const Eigen::MatrixXi& mask) const;

  IntensityProcessorConfig config_;
};

}  // namespace bievr

#endif  // BIEVR_LIO_INTENSITY_PROCESSOR_H_
