#ifndef BIEVR_LIO_INTENSITY_PROCESSOR_H_
#define BIEVR_LIO_INTENSITY_PROCESSOR_H_

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "bievr_lio/common.h"

// COIN-BIEVR intensity preprocessing (Section 5-9 of the reproduction plan).
//
// Per-frame per-point intensity normalization for irregular LiDAR scans:
//   1. Spherical projection of the LiDAR-frame point cloud into a (w x h)
//      intensity image (Eq. 1 of the COIN-BIEVR paper). The projection is
//      abstracted behind projectPoint() so an Ouster factory LUT projector can
//      later replace the spherical one without touching normalization / point
//      back-assignment / the map pipeline.
//   2. Optional Ouster line-artifact removal (vertical high-pass then
//      horizontal low-pass, per COIN-LIO).
//   3. Sparse brightness normalization: the brightness I_B is the average over
//      a box window that only counts non-empty pixels
//        I_F = s * I / (I_B + 1)
//   4. Optional masked Gaussian blur, then clamp to [0, 255].
//   5. The normalized value I_F(v_i,u_i) is written back to *every* 3D point
//      that projects into the image (via the point -> pixel map), not only to
//      the nearest-point owner of each pixel. This keeps every map point and
//      every photometric residual in the normalized intensity domain even when
//      several points collide into the same image pixel.
//
// The output Intensities row is index-aligned with the input point cloud, so it
// stays aligned with the (transformed / undistorted) spatial cloud downstream.
//
// When config_.enabled == false (i.e. `intensity.preprocessing.enabled: false`)
// the processor runs a debug/ablation bypass instead of returning an empty row:
//   raw intensity -> raw_intensity_scale -> sanitize -> clamp [0,255],
// keeping the point/intensity index alignment intact. This is NOT the COIN-BIEVR
// paper default.

namespace bievr {

struct IntensityProcessorConfig {
  bool enabled = true;  // preprocessing enabled; false = debug bypass mode

  std::string projection = "spherical";  // "spherical" (Eq. 1) or "ouster_lut"
  int image_width = 1024;                // spherical image width [pix]
  int image_height = 64;                 // spherical image height [pix]
  double vertical_fov_deg = 50.0;        // vertical FOV of the sensor [deg]

  // Brightness window (in pixels). 0 = full image (global sparse mean).
  int brightness_window_u = 0;
  int brightness_window_v = 0;

  double normalization_scale = 140.0;  // s in I_F = s * I / (I_B + 1)

  bool remove_lines = false;   // Ouster line-artifact removal (default: off)
  bool gaussian_blur = false;  // masked Gaussian blur after normalization

  double raw_intensity_scale = 1.0;  // raw intensity scaling before projection
  bool use_ouster_lut = false;       // whether a factory point->pixel LUT is used

  // Round-10: ENWIDE Ouster (OS0-128) factory-calibrated projection. Loaded by
  // config_loader.h from the official os_enwide.json (converted to the committed
  // YAML metadata file config/ouster/enwide_metadata.yaml; see provenance).
  // projection must be "ouster_lut" for these to take effect.
  std::string ouster_metadata_path = "";  // official os_enwide metadata (YAML)
  std::vector<double> ouster_beam_altitude_angles;  // [deg], descending (beam 0..127)
  std::vector<double> ouster_beam_azimuth_angles;   // [deg]
  std::vector<int> ouster_pixel_shift_by_row;       // 128 entries
  double ouster_beam_offset_m = 0.02767;            // lidar_origin_to_beam_origin_mm * 1e-3
  int ouster_u_shift = 0;                           // ENWIDE official image/u_shift = 0
  double ouster_intensity_scale = 0.25;             // COIN-LIO image/intensity_scale
  bool sparse_brightness = true;                    // true = masked sparse window (Avia);
                                                    // false = full-window box incl. empty
                                                    // pixels (COIN-LIO Ouster cv::blur)
};

// Per-pixel result of the projection of a single scan.
struct ProjectedIntensityImage {
  Eigen::MatrixXf intensity;   // per-pixel intensity of the owning point
  Eigen::MatrixXi point_index; // pixel -> owner point, -1 = empty
  Eigen::MatrixXf range;       // pixel -> owner range (nearest-point selection)
  Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> valid;  // mask
};

// Per-frame result of intensity preprocessing.
struct IntensityProcessingResult {
  // Index-aligned filtered intensity (one value per input point).
  Intensities filtered;

  // Input point count (kept separate from `filtered` because the pipeline may
  // std::move() `filtered` out of this struct before reading the diagnostics).
  size_t input_points = 0;

  // Original point -> normalization image pixel (linear index v*width+u).
  // -1 means the point could not be projected (defensive; with the clamped
  // spherical projection every finite point is expected to be valid).
  std::vector<int32_t> point_pixel_idx;

  // Low-overhead diagnostics (computed during projection / back-assignment).
  size_t num_valid = 0;          // points that received a filtered value
  size_t num_unique_pixels = 0;  // occupied image pixels
  size_t num_collisions = 0;     // points projected into an already-occupied pixel
  size_t num_invalid = 0;        // points that could not be projected
  // Projection boundary diagnostics (projectPoint returns the raw pixel; the
  // clamp to the image edges is applied in process() and counted here).
  size_t horizontal_boundary_adjusted = 0;  // u clamped to [0, W-1]
  size_t horizontal_wrap_adjusted = 0;      // subset: u >= W (azimuth -pi wrap)
  size_t vertical_top_clamped = 0;          // v < 0 -> 0
  size_t vertical_bottom_clamped = 0;       // v >= H -> H-1
  double filtered_min = 0.0;                // min / max / mean of the filtered values
  double filtered_max = 0.0;
  double filtered_mean = 0.0;
  // Filtered intensity distribution (256-bin histogram over [0,255], computed
  // during back-assignment; light, no sorting).
  double filtered_std = 0.0;
  double filtered_p01 = 0.0, filtered_p05 = 0.0, filtered_p50 = 0.0;
  double filtered_p95 = 0.0, filtered_p99 = 0.0;
  size_t filtered_sat0 = 0;    // points with filtered value == 0
  size_t filtered_sat255 = 0;  // points with filtered value >= 255
  std::array<int, 256> filtered_histogram{};  // 256 bins over [0, 256)
  // Raw intensity distribution (Livox reflectivity is 0..255).
  double raw_min = 0.0, raw_max = 0.0, raw_p50 = 0.0, raw_p95 = 0.0, raw_p99 = 0.0;
  // Observed elevation distribution (deg), arcsin(z/|p|); 720-bin histogram.
  double elevation_min_deg = 0.0, elevation_max_deg = 0.0;
  double elevation_p01_deg = 0.0, elevation_p99_deg = 0.0;
};

// Produces per-point filtered intensities from a LiDAR-frame point cloud.
class IntensityProcessor {
 public:
  explicit IntensityProcessor(const IntensityProcessorConfig& config = IntensityProcessorConfig());
  void configure(const IntensityProcessorConfig& config) { config_ = config; }
  const IntensityProcessorConfig& config() const { return config_; }

  // Projects + normalizes and returns the per-point filtered intensities (see
  // IntensityProcessingResult). Debug bypass when config_.enabled == false.
  // `rings` (optional, index-aligned): Ouster ring/beam index per point. When
  // projection == "ouster_lut" and rings is provided, the intensity image is
  // built ring-based (row = ring, col = geometric azimuth column), matching the
  // COIN-LIO dense-image construction (organized row == ring on the Ouster
  // driver); otherwise the geometric elevation-LUT projector is used.
  IntensityProcessingResult process(const Pointcloud& points_L,
                                    const IntensityView& raw_intensity,
                                    const IntensityView* rings = nullptr);

  // Point -> image pixel projection (raw, unclamped pixel). Returns false only
  // for non-finite or zero-range points. The returned (u, v) may lie outside
  // [0,W)x[0,H); clampPixel() (called by process/project) clamps it into the
  // image and counts boundary adjustments, so a sensor-FOV / image-size mismatch
  // shows up in the diagnostics instead of dropping points. When projection ==
  // "ouster_lut" this is the COIN-LIO factory-calibrated Ouster projector
  // (beam-offset spherical model + beam-altitude lookup, u_shift applied); it
  // returns false for points outside the beam vertical FOV (no clamping needed).
  bool projectPoint(const Point& p, int& u, int& v) const;

  // Exposed for unit testing / debugging: project + normalize without writeback.
  ProjectedIntensityImage project(const Pointcloud& points_L,
                                  const IntensityView& raw_intensity) const;

  // Sparse brightness normalization of a (sparse) intensity image using only
  // non-empty pixels (mask > 0): I_F = s * I / (I_B + 1), optional line removal
  // and masked blur, then clamp to [0, 255]. Exposed for unit testing.
  Eigen::MatrixXf normalizeImage(const Eigen::MatrixXf& intensity_image,
                                 const Eigen::MatrixXi& mask) const;

 private:
  // COIN-LIO factory-calibrated Ouster projector (projectPoint port). Returns
  // false for points outside the beam vertical FOV (the LUT rows bound the image
  // exactly, so vertical clamping is never needed).
  bool projectPointOuster(const Point& p, int& u, int& v) const;

  // Geometric azimuth column only (COIN-LIO projectPoint u part). Used for the
  // ring-based Ouster image construction (row = ring).
  bool projectPointOusterCol(const Point& p, int& u) const;

  // Sparse box-average brightness I_B = sum(M*I) / max(sum(M), 1).
  Eigen::MatrixXd brightnessImage(const ProjectedIntensityImage& image) const;

  // Full-window box average over ALL pixels (including empty/zero), matching the
  // COIN-LIO Ouster cv::blur brightness (used when sparse_brightness == false).
  Eigen::MatrixXd brightnessImageFull(const ProjectedIntensityImage& image) const;

  // COIN-LIO official ENWIDE line-artifact removal (vertical high-pass then
  // horizontal low-pass FIR, official config/line_removal.yaml coefficients).
  void removeLinesOuster(Eigen::MatrixXf& image) const;

  // Clamps a raw projected pixel into the image and reports how it was adjusted
  // as a bitmask: 1 = horizontal (u), 2 = vertical top (v<0), 4 = vertical
  // bottom (v>=H), 8 = horizontal wrap (u >= W, azimuth -pi boundary).
  int clampPixel(int& u, int& v) const;

  // COIN-LIO line-artifact removal (vertical HPF then horizontal LPF).
  void removeLines(Eigen::MatrixXf& image, const Eigen::MatrixXi& mask) const;

  // Masked separable Gaussian blur (only valid neighbours contribute).
  void gaussianBlur(Eigen::MatrixXf& image, const Eigen::MatrixXi& mask) const;

  IntensityProcessorConfig config_;
};

}  // namespace bievr

#endif  // BIEVR_LIO_INTENSITY_PROCESSOR_H_
