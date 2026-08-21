#include "bievr_lio/intensity_processor.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "bievr_lio/log++.h"

namespace bievr {

namespace {

constexpr double kPi = 3.14159265358979323846;

// Lightweight fixed-bin histogram used for per-frame distribution statistics
// (no sorting, O(N) increments). Values are clamped into [lo, hi).
class SimpleHistogram {
 public:
  SimpleHistogram(int num_bins, double lo, double hi)
      : bins_(num_bins, 0), lo_(lo), hi_(hi), total_(0) {}

  void add(double v) {
    if (!std::isfinite(v)) return;
    double t = (v - lo_) / (hi_ - lo_) * static_cast<double>(bins_.size());
    int idx = static_cast<int>(t);
    if (idx < 0) idx = 0;
    if (idx >= static_cast<int>(bins_.size())) idx = static_cast<int>(bins_.size()) - 1;
    ++bins_[idx];
    ++total_;
  }

  // Quantile in [0,100]; returns the bin-center value of the reaching bin.
  double percentile(double q) const {
    if (total_ == 0) return 0.0;
    const double target = q / 100.0 * static_cast<double>(total_);
    int64_t acc = 0;
    for (size_t i = 0; i < bins_.size(); ++i) {
      acc += bins_[i];
      if (acc >= target) {
        return lo_ + (static_cast<double>(i) + 0.5) / static_cast<double>(bins_.size()) *
                         (hi_ - lo_);
      }
    }
    return hi_;
  }

  int64_t bin(size_t i) const { return i < bins_.size() ? bins_[i] : 0; }
  int64_t total() const { return total_; }

 private:
  std::vector<int> bins_;
  double lo_, hi_;
  int64_t total_;
};

}  // namespace

// 1D clamped box average along the rows/cols via prefix sums. `values` holds
// the per-pixel signal and `mask` its validity; returns the per-pixel average
// of `values` over the box window, counting only valid pixels (0 where none).
Eigen::MatrixXd maskedBoxAverage(const Eigen::MatrixXf& values, const Eigen::MatrixXi& mask,
                                 int window_u, int window_v) {
  const int H = values.rows();
  const int W = values.cols();
  if (H == 0 || W == 0) return Eigen::MatrixXd();

  Eigen::MatrixXd Pv = Eigen::MatrixXd::Zero(H + 1, W + 1);
  Eigen::MatrixXd Pm = Eigen::MatrixXd::Zero(H + 1, W + 1);
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      const double v = mask(y, x) > 0 ? static_cast<double>(values(y, x)) : 0.0;
      const double m = mask(y, x) > 0 ? 1.0 : 0.0;
      Pv(y + 1, x + 1) = v + Pv(y, x + 1) + Pv(y + 1, x) - Pv(y, x);
      Pm(y + 1, x + 1) = m + Pm(y, x + 1) + Pm(y + 1, x) - Pm(y, x);
    }
  }

  const int half_u = window_u / 2;
  const int half_v = window_v / 2;
  Eigen::MatrixXd brightness(H, W);
  for (int y = 0; y < H; ++y) {
    const int r0 = std::max(0, y - half_v);
    const int r1 = std::min(H, y + half_v + 1);
    for (int x = 0; x < W; ++x) {
      const int c0 = std::max(0, x - half_u);
      const int c1 = std::min(W, x + half_u + 1);
      const double sum_v = Pv(r1, c1) - Pv(r0, c1) - Pv(r1, c0) + Pv(r0, c0);
      const double sum_m = Pm(r1, c1) - Pm(r0, c1) - Pm(r1, c0) + Pm(r0, c0);
      brightness(y, x) = sum_v / std::max(sum_m, 1.0);
    }
  }
  return brightness;
}

IntensityProcessor::IntensityProcessor(const IntensityProcessorConfig& config) : config_(config) {}

bool IntensityProcessor::projectPoint(const Point& p, int& u, int& v) const {
  if (config_.projection == "ouster_lut") {
    return projectPointOuster(p, u, v);
  }
  const double r = p.norm();
  if (!std::isfinite(r) || r < 1e-6) return false;

  const double az = std::atan2(p.y(), p.x());
  const double el = std::asin(std::max(-1.0, std::min(1.0, p.z() / r)));

  const int W = config_.image_width;
  const int H = config_.image_height;
  const double fov_rad = kPi * config_.vertical_fov_deg / 180.0;

  // Raw pixel (may lie outside the image; clamped by the caller).
  u = static_cast<int>(std::lround(-W / (2.0 * kPi) * az + W / 2.0));
  v = static_cast<int>(std::lround(-H / fov_rad * el + H / 2.0));
  return true;
}

// Geometric azimuth column only (COIN-LIO projectPoint u part). Used for the
// ring-based Ouster image construction (row = ring). Returns false for
// non-finite / zero-range points or when the column is outside [0, cols).
bool IntensityProcessor::projectPointOusterCol(const Point& p, int& u) const {
  const int cols = config_.image_width;
  const double beam_offset = config_.ouster_beam_offset_m;
  const double L = std::hypot(p.x(), p.y()) - beam_offset;
  const double R = std::sqrt(p.z() * p.z() + L * L);
  if (!std::isfinite(R) || R < 1e-6) return false;
  const double phi = std::atan2(p.y(), p.x());
  const double fx = -static_cast<double>(cols) / (2.0 * kPi);
  const double cx = cols / 2.0;
  u = static_cast<int>(std::lround(fx * phi + cx - config_.ouster_u_shift));
  if (u < 0 || u >= cols) return false;
  return true;
}

// COIN-LIO Projector::projectPoint port (src/projector.cpp, COIN-LIO commit
// 76729cc4feb3649cbd79d28f82d9f62a2c82889b): spherical model with the beam
// offset, K matrix from the factory beam-altitude angles, and subpixel row
// interpolation against the elevation lookup table. u_shift is applied on the
// column (0 for ENWIDE). Returns false for points outside the beam vertical FOV
// (the LUT rows bound the image exactly, so vertical clamping is never needed).
bool IntensityProcessor::projectPointOuster(const Point& p, int& u, int& v) const {
  const auto& elev = config_.ouster_beam_altitude_angles;  // degrees, descending
  const int rows = config_.image_height;
  const int cols = config_.image_width;
  if (elev.size() < 2 || static_cast<int>(elev.size()) != rows) return false;

  // Beam altitudes in radians (COIN-LIO converts the loaded angles to rad).
  std::vector<double> elev_rad(elev.size());
  for (size_t i = 0; i < elev.size(); ++i) elev_rad[i] = elev[i] * kPi / 180.0;

  const double fy = -static_cast<double>(rows) /
                    std::abs(elev_rad[0] - elev_rad[elev_rad.size() - 1]);
  const double fx = -static_cast<double>(cols) / (2.0 * kPi);
  const double cy = rows / 2.0;
  const double cx = cols / 2.0;

  const double beam_offset = config_.ouster_beam_offset_m;
  const double L = std::hypot(p.x(), p.y()) - beam_offset;
  const double R = std::sqrt(p.z() * p.z() + L * L);
  if (!std::isfinite(R) || R < 1e-6) return false;
  const double phi = std::atan2(p.y(), p.x());
  const double theta = std::asin(std::max(-1.0, std::min(1.0, p.z() / R)));

  double uf = fx * phi + cx;

  if (theta > elev_rad[0] || theta < elev_rad[rows - 1]) return false;

  // Row via the elevation LUT with subpixel interpolation (exact COIN-LIO logic).
  double vf;
  auto greater = (std::upper_bound(elev_rad.rbegin(), elev_rad.rend(), theta) + 1).base();
  auto smaller = greater + 1;
  if (greater == elev_rad.end()) {
    vf = rows - 1;
  } else {
    vf = std::distance(elev_rad.begin(), greater);
    vf += (*greater - theta) / (*greater - *smaller);
  }

  // Apply u_shift on the column (ENWIDE official u_shift = 0).
  uf -= config_.ouster_u_shift;

  u = static_cast<int>(std::lround(uf));
  v = static_cast<int>(std::lround(vf));
  if (u < 0 || u >= cols || v < 0 || v >= rows) return false;
  return true;
}

int IntensityProcessor::clampPixel(int& u, int& v) const {
  const int W = config_.image_width;
  const int H = config_.image_height;
  int flags = 0;
  if (u < 0 || u >= W) {
    flags |= 1;                       // horizontal boundary
    if (u >= W) flags |= 8;           // azimuth -pi wrap (u == W)
    u = std::max(0, std::min(W - 1, u));
  }
  if (v < 0) {
    flags |= 2;                       // vertical top
    v = 0;
  } else if (v >= H) {
    flags |= 4;                       // vertical bottom
    v = H - 1;
  }
  return flags;
}

ProjectedIntensityImage IntensityProcessor::project(const Pointcloud& points_L,
                                                    const IntensityView& raw_intensity) const {
  const int W = config_.image_width;
  const int H = config_.image_height;
  const double scale = static_cast<double>(config_.raw_intensity_scale);

  ProjectedIntensityImage image;
  image.intensity = Eigen::MatrixXf::Zero(H, W);
  image.point_index = Eigen::MatrixXi::Constant(H, W, -1);
  image.range = Eigen::MatrixXf::Constant(H, W, std::numeric_limits<float>::max());
  image.valid = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>::Zero(H, W);

  if (points_L.empty()) return image;

  for (size_t i = 0; i < points_L.size(); ++i) {
    int u = 0, v = 0;
    if (!projectPoint(points_L[i], u, v)) continue;
    clampPixel(u, v);

    const double r = points_L[i].norm();
    // Closest point owns the pixel.
    const float val = static_cast<float>(raw_intensity.size() > i
                                             ? static_cast<double>(raw_intensity(0, i)) * scale
                                             : 0.0);
    if (image.valid(v, u) == 0 || r < image.range(v, u)) {
      image.intensity(v, u) = val;
      image.point_index(v, u) = static_cast<int>(i);
      image.range(v, u) = static_cast<float>(r);
      image.valid(v, u) = 1;
    }
  }
  return image;
}

Eigen::MatrixXd IntensityProcessor::brightnessImage(const ProjectedIntensityImage& image) const {
  const int W = config_.image_width;
  const int H = config_.image_height;
  const int window_u = config_.brightness_window_u;
  const int window_v = config_.brightness_window_v;

  // Window <= 0 means the full image as a single window: the sparse global mean
  // over all non-empty pixels (only non-empty pixels are counted).
  if (window_u <= 0 && window_v <= 0) {
    double sum_v = 0.0;
    double sum_m = 0.0;
    for (int y = 0; y < H; ++y) {
      for (int x = 0; x < W; ++x) {
        if (image.valid(y, x) == 0) continue;
        sum_v += image.intensity(y, x);
        sum_m += 1.0;
      }
    }
    const double mean = sum_m > 0.0 ? sum_v / sum_m : 0.0;
    return Eigen::MatrixXd::Constant(H, W, mean);
  }

  const int wu = window_u > 0 ? window_u : W;
  const int wv = window_v > 0 ? window_v : H;
  return maskedBoxAverage(image.intensity, image.point_index, wu, wv);
}

// Full-window box average over ALL pixels (including empty/zero), matching the
// COIN-LIO Ouster `cv::blur` brightness used before normalization. Used when
// sparse_brightness == false (ENWIDE Ouster reproduction).
Eigen::MatrixXd IntensityProcessor::brightnessImageFull(const ProjectedIntensityImage& image) const {
  const int W = config_.image_width;
  const int H = config_.image_height;
  const int window_u = config_.brightness_window_u > 0 ? config_.brightness_window_u : W;
  const int window_v = config_.brightness_window_v > 0 ? config_.brightness_window_v : H;

  // Plain box average (zeros included) via prefix sums.
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(H + 1, W + 1);
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      P(y + 1, x + 1) =
          static_cast<double>(image.intensity(y, x)) + P(y, x + 1) + P(y + 1, x) - P(y, x);
    }
  }
  const int half_u = window_u / 2;
  const int half_v = window_v / 2;
  Eigen::MatrixXd brightness(H, W);
  for (int y = 0; y < H; ++y) {
    const int r0 = std::max(0, y - half_v);
    const int r1 = std::min(H, y + half_v + 1);
    for (int x = 0; x < W; ++x) {
      const int c0 = std::max(0, x - half_u);
      const int c1 = std::min(W, x + half_u + 1);
      const double area = static_cast<double>((r1 - r0) * (c1 - c0));
      const double sum = P(r1, c1) - P(r0, c1) - P(r1, c0) + P(r0, c0);
      brightness(y, x) = area > 0.0 ? sum / area : 0.0;
    }
  }
  return brightness;
}

void IntensityProcessor::removeLines(Eigen::MatrixXf& image, const Eigen::MatrixXi& mask) const {
  const int H = image.rows();
  const int W = image.cols();
  if (H < 3 || W < 3) return;

  // Vertical high-pass: [-1, 2, -1]^T  (removes the constant/D.C. component).
  Eigen::MatrixXf hpf = Eigen::MatrixXf::Zero(H, W);
  for (int y = 1; y < H - 1; ++y) {
    for (int x = 0; x < W; ++x) {
      hpf(y, x) = -image(y - 1, x) + 2.0f * image(y, x) - image(y + 1, x);
    }
  }
  // Horizontal low-pass: [1, 2, 1] / 4  (extracts the line component).
  Eigen::MatrixXf lpf = Eigen::MatrixXf::Zero(H, W);
  for (int y = 0; y < H; ++y) {
    for (int x = 1; x < W - 1; ++x) {
      lpf(y, x) = 0.25f * (hpf(y, x - 1) + 2.0f * hpf(y, x) + hpf(y, x + 1));
    }
  }

  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      if (mask(y, x) <= 0) continue;
      image(y, x) -= lpf(y, x);
      if (image(y, x) < 0.0f) image(y, x) = 0.0f;
    }
  }
}

// COIN-LIO official ENWIDE line-artifact removal (src/image_processing.cpp
// ImageProcessor::removeLines, COIN-LIO commit
// 76729cc4feb3649cbd79d28f82d9f62a2c82889b): vertical high-pass FIR then
// horizontal low-pass FIR, subtract the line signal and clamp to >= 0.
// Coefficients are the official config/line_removal.yaml (SHA256
// db0be90e9187a48ff4ba227f86f57dbaafe282fddf0740f335ee27ff9a06ca7f), which is
// symmetric (linear-phase), so correlation == convolution.
void IntensityProcessor::removeLinesOuster(Eigen::MatrixXf& image) const {
  const int H = image.rows();
  const int W = image.cols();
  constexpr double kHp[33] = {-0.00122687,  -0.00152587,  0.0009631,    0.00382838,
                              0.00071422,   -0.00765637,  -0.00681285,  0.01015542,
                              0.01944999,   -0.00536835,  -0.03792929,  -0.01565801,
                              0.05816374,   0.07138264,   -0.07402277,  -0.30572514,
                              0.5802669,    -0.30572514,  -0.07402277,  0.07138264,
                              0.05816374,   -0.01565801,  -0.03792929,  -0.00536835,
                              0.01944999,   0.01015542,   -0.00681285,  -0.00765637,
                              0.00071422,   0.00382838,   0.0009631,    -0.00152587,
                              -0.00122687};
  constexpr double kLp[32] = {-0.0013038,  -0.00117813, -0.00102349, -0.00051396, 0.000759,
                              0.00322145,  0.00724004,  0.01304552,  0.02066957,  0.02990645,
                              0.04030759,  0.0512121,   0.06181081,  0.07123596,  0.07866427,
                              0.08341891,  0.08505541,  0.08341891,  0.07866427,  0.07123596,
                              0.06181081,  0.0512121,   0.04030759,  0.02990645,  0.02066957,
                              0.01304552,  0.00724004,  0.00322145,  0.000759,    -0.00051396,
                              -0.00102349, -0.00117813};

  // Vertical high-pass (per column).
  Eigen::MatrixXf hpf = Eigen::MatrixXf::Zero(H, W);
  for (int x = 0; x < W; ++x) {
    for (int y = 0; y < H; ++y) {
      double acc = 0.0;
      for (int k = 0; k < 33; ++k) {
        const int yy = y + k - 16;
        if (yy < 0 || yy >= H) continue;
        acc += kHp[k] * image(yy, x);
      }
      hpf(y, x) = static_cast<float>(acc);
    }
  }
  // Horizontal low-pass (per row) of the high-passed image.
  Eigen::MatrixXf lpf = Eigen::MatrixXf::Zero(H, W);
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      double acc = 0.0;
      for (int k = 0; k < 32; ++k) {
        const int xx = x + k - 15;
        if (xx < 0 || xx >= W) continue;
        acc += kLp[k] * hpf(y, xx);
      }
      lpf(y, x) = static_cast<float>(acc);
    }
  }
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      image(y, x) -= lpf(y, x);
      if (image(y, x) < 0.0f) image(y, x) = 0.0f;
    }
  }
}

void IntensityProcessor::gaussianBlur(Eigen::MatrixXf& image, const Eigen::MatrixXi& mask) const {
  const int H = image.rows();
  const int W = image.cols();
  if (H < 3 || W < 3) return;

  // Separable 3x3 Gaussian [1,2,1]/4 per axis, restricted to valid pixels.
  Eigen::MatrixXf tmp = Eigen::MatrixXf::Zero(H, W);
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      if (mask(y, x) <= 0) continue;
      double wsum = 0.0, acc = 0.0;
      for (int dx = -1; dx <= 1; ++dx) {
        const int xx = x + dx;
        if (xx < 0 || xx >= W || mask(y, xx) <= 0) continue;
        const double w = (dx == 0) ? 2.0 : 1.0;
        acc += w * image(y, xx);
        wsum += w;
      }
      if (wsum > 0.0) tmp(y, x) = static_cast<float>(acc / wsum);
    }
  }
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      if (mask(y, x) <= 0) continue;
      double wsum = 0.0, acc = 0.0;
      for (int dy = -1; dy <= 1; ++dy) {
        const int yy = y + dy;
        if (yy < 0 || yy >= H || mask(yy, x) <= 0) continue;
        const double w = (dy == 0) ? 2.0 : 1.0;
        acc += w * tmp(yy, x);
        wsum += w;
      }
      if (wsum > 0.0) image(y, x) = static_cast<float>(acc / wsum);
    }
  }
}

IntensityProcessingResult IntensityProcessor::process(const Pointcloud& points_L,
                                                      const IntensityView& raw_intensity,
                                                      const IntensityView* rings) {
  IntensityProcessingResult out;
  const size_t N = points_L.size();
  out.input_points = N;
  out.filtered.resize(1, N);
  out.point_pixel_idx.assign(N, -1);
  out.num_valid = 0;
  out.num_unique_pixels = 0;
  out.num_collisions = 0;
  out.num_invalid = 0;
  out.filtered_min = 0.0;
  out.filtered_max = 0.0;
  out.filtered_mean = 0.0;
  if (N == 0) return out;

  // Ring-based Ouster image construction (row = ring, col = geometric azimuth).
  const bool ring_based = config_.projection == "ouster_lut" && rings != nullptr;

  // -------------------------------------------------------------------
  // Debug/ablation bypass mode (preprocessing.enabled == false).
  // This is NOT the COIN-BIEVR paper default: it keeps the point/intensity
  // index alignment but skips brightness normalization.
  // -------------------------------------------------------------------
  if (!config_.enabled) {
    double sum = 0.0, sum_sq = 0.0;
    SimpleHistogram fhist(256, 0, 256);
    out.filtered_min = std::numeric_limits<double>::max();
    out.filtered_max = std::numeric_limits<double>::lowest();
    out.raw_min = std::numeric_limits<double>::max();
    out.raw_max = std::numeric_limits<double>::lowest();
    for (size_t i = 0; i < N; ++i) {
      double v = raw_intensity.size() > i ? raw_intensity(0, i) * config_.raw_intensity_scale
                                          : 0.0;
      if (!std::isfinite(v)) v = 0.0;  // sanitize
      v = std::max(0.0, std::min(255.0, v));  // clamp
      out.filtered(0, i) = static_cast<float>(v);
      sum += v;
      sum_sq += v * v;
      out.filtered_min = std::min(out.filtered_min, v);
      out.filtered_max = std::max(out.filtered_max, v);
      out.raw_min = std::min(out.raw_min, v);
      out.raw_max = std::max(out.raw_max, v);
      fhist.add(v);
    }
    out.num_valid = N;
    out.filtered_mean = sum / static_cast<double>(N);
    out.filtered_std = std::sqrt(std::max(0.0, sum_sq / static_cast<double>(N) -
                                                   out.filtered_mean * out.filtered_mean));
    out.filtered_p01 = fhist.percentile(1.0);
    out.filtered_p05 = fhist.percentile(5.0);
    out.filtered_p50 = fhist.percentile(50.0);
    out.filtered_p95 = fhist.percentile(95.0);
    out.filtered_p99 = fhist.percentile(99.0);
    for (size_t i = 0; i < out.filtered_histogram.size(); ++i) {
      out.filtered_histogram[i] = fhist.bin(i);
      if (i == 0) out.filtered_sat0 = static_cast<size_t>(fhist.bin(i));
      if (i + 1 >= out.filtered_histogram.size()) {
        out.filtered_sat255 = static_cast<size_t>(fhist.bin(i));
      }
    }
    out.raw_p50 = out.filtered_p50;
    out.raw_p95 = out.filtered_p95;
    out.raw_p99 = out.filtered_p99;
    return out;
  }

  // -------------------------------------------------------------------
  // 1. Project every point; build the sparse image (nearest owner per pixel)
  //    and the per-point point -> pixel map.
  // -------------------------------------------------------------------
  ProjectedIntensityImage image;
  image.intensity = Eigen::MatrixXf::Zero(config_.image_height, config_.image_width);
  image.point_index = Eigen::MatrixXi::Constant(config_.image_height, config_.image_width, -1);
  image.range =
      Eigen::MatrixXf::Constant(config_.image_height, config_.image_width,
                                std::numeric_limits<float>::max());
  image.valid = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic>::Zero(
      config_.image_height, config_.image_width);
  const int W = config_.image_width;

  // Per-frame distribution diagnostics (light, O(N)).
  SimpleHistogram raw_hist(256, 0, 256);   // Livox reflectivity 0..255
  SimpleHistogram elev_hist(720, -90, 90); // 0.25 deg bins
  out.raw_min = std::numeric_limits<double>::max();
  out.raw_max = std::numeric_limits<double>::lowest();
  out.elevation_min_deg = std::numeric_limits<double>::max();
  out.elevation_max_deg = std::numeric_limits<double>::lowest();
  const double kRad2Deg = 180.0 / kPi;

  for (size_t i = 0; i < N; ++i) {
    const double r = points_L[i].norm();
    const double raw_v = raw_intensity.size() > i ? raw_intensity(0, i) * config_.raw_intensity_scale
                                                  : 0.0;
    raw_hist.add(raw_v);
    out.raw_min = std::min(out.raw_min, raw_v);
    out.raw_max = std::max(out.raw_max, raw_v);

    if (std::isfinite(r) && r > 1e-6) {
      const double el_deg =
          kRad2Deg * std::asin(std::max(-1.0, std::min(1.0, points_L[i].z() / r)));
      elev_hist.add(el_deg);
      out.elevation_min_deg = std::min(out.elevation_min_deg, el_deg);
      out.elevation_max_deg = std::max(out.elevation_max_deg, el_deg);
    }

    int u = 0, v = 0;
    bool projected;
    if (ring_based) {
      // Row = Ouster ring/beam index; column = geometric azimuth.
      const int ring = static_cast<int>(std::lround((*rings)(0, i)));
      projected = ring >= 0 && ring < config_.image_height &&
                  projectPointOusterCol(points_L[i], u);
      v = ring;
    } else {
      projected = projectPoint(points_L[i], u, v);
    }
    if (!projected) {
      out.point_pixel_idx[i] = -1;
      ++out.num_invalid;
      continue;
    }

    // Clamp into the image (Scheme B) and count boundary adjustments so a
    // sensor-FOV / image-size mismatch is visible in the diagnostics instead of
    // silently dropping or piling up points.
    const int flags = clampPixel(u, v);
    if (flags & 1) ++out.horizontal_boundary_adjusted;
    if (flags & 8) ++out.horizontal_wrap_adjusted;
    if (flags & 2) ++out.vertical_top_clamped;
    if (flags & 4) ++out.vertical_bottom_clamped;

    const int linear = v * W + u;
    out.point_pixel_idx[i] = linear;

    if (image.valid(v, u) == 0) {
      ++out.num_unique_pixels;
    } else {
      ++out.num_collisions;
    }

    const float val = static_cast<float>(raw_v);
    // Closest point owns the pixel (defines the representative raw intensity).
    if (image.valid(v, u) == 0 || r < image.range(v, u)) {
      image.intensity(v, u) = val;
      image.point_index(v, u) = static_cast<int>(i);
      image.range(v, u) = static_cast<float>(r);
      image.valid(v, u) = 1;
    }
  }

  out.raw_p50 = raw_hist.percentile(50.0);
  out.raw_p95 = raw_hist.percentile(95.0);
  out.raw_p99 = raw_hist.percentile(99.0);
  out.elevation_p01_deg = elev_hist.percentile(1.0);
  out.elevation_p99_deg = elev_hist.percentile(99.0);

  // -------------------------------------------------------------------
  // 2. Sparse brightness normalization.
  // -------------------------------------------------------------------
  const Eigen::MatrixXi valid_mask = (image.point_index.array() >= 0).cast<int>();
  image.intensity = normalizeImage(image.intensity, valid_mask);

  // -------------------------------------------------------------------
  // 3. Back-assign I_F(v_i,u_i) to EVERY projected point (not only the pixel
  //    owners), so points sharing a pixel all stay in the normalized domain.
  // -------------------------------------------------------------------
  double sum = 0.0, sum_sq = 0.0;
  SimpleHistogram fhist(256, 0, 256);
  out.filtered_min = std::numeric_limits<double>::max();
  out.filtered_max = std::numeric_limits<double>::lowest();
  for (size_t i = 0; i < N; ++i) {
    const int linear = out.point_pixel_idx[i];
    if (linear < 0) {
      out.filtered(0, i) = 0.0f;  // defensive: such points are pre-filtered upstream
      continue;
    }
    const int v = linear / W;
    const int u = linear % W;
    const double val = static_cast<double>(image.intensity(v, u));
    out.filtered(0, i) = static_cast<float>(val);
    ++out.num_valid;
    sum += val;
    sum_sq += val * val;
    out.filtered_min = std::min(out.filtered_min, val);
    out.filtered_max = std::max(out.filtered_max, val);
    fhist.add(val);
  }
  out.filtered_mean = out.num_valid > 0 ? sum / static_cast<double>(out.num_valid) : 0.0;
  out.filtered_std =
      out.num_valid > 0
          ? std::sqrt(std::max(0.0, sum_sq / static_cast<double>(out.num_valid) -
                                         out.filtered_mean * out.filtered_mean))
          : 0.0;
  out.filtered_p01 = fhist.percentile(1.0);
  out.filtered_p05 = fhist.percentile(5.0);
  out.filtered_p50 = fhist.percentile(50.0);
  out.filtered_p95 = fhist.percentile(95.0);
  out.filtered_p99 = fhist.percentile(99.0);
  out.filtered_sat0 = static_cast<size_t>(fhist.bin(0));
  out.filtered_sat255 = static_cast<size_t>(fhist.bin(255));
  for (size_t i = 0; i < out.filtered_histogram.size(); ++i) {
    out.filtered_histogram[i] = static_cast<int>(fhist.bin(i));
  }

  return out;
}

Eigen::MatrixXf IntensityProcessor::normalizeImage(const Eigen::MatrixXf& intensity_image,
                                                   const Eigen::MatrixXi& mask) const {
  Eigen::MatrixXf image = intensity_image;

  // COIN-LIO Ouster: scale the raw intensity channel by image/intensity_scale
  // before line removal / brightness normalization (reflectivity_=false path).
  if (config_.projection == "ouster_lut") {
    image *= static_cast<float>(config_.ouster_intensity_scale);
  }

  if (config_.remove_lines) {
    if (config_.projection == "ouster_lut") {
      removeLinesOuster(image);
    } else {
      removeLines(image, mask);
    }
  }

  ProjectedIntensityImage img;
  img.intensity = image;
  img.point_index = mask;
  img.valid = (mask.array() > 0).cast<uint8_t>();
  // Avia: sparse brightness (only non-empty pixels). Ouster ENWIDE: full-window
  // box over all pixels (COIN-LIO cv::blur semantics).
  const Eigen::MatrixXd I_B =
      config_.sparse_brightness ? brightnessImage(img) : brightnessImageFull(img);
  const double s = config_.normalization_scale;

  for (int v = 0; v < image.rows(); ++v) {
    for (int u = 0; u < image.cols(); ++u) {
      if (mask(v, u) <= 0) continue;
      const double I = image(v, u);
      double I_F = s * I / (I_B(v, u) + 1.0);
      image(v, u) = static_cast<float>(I_F);
    }
  }

  if (config_.gaussian_blur) {
    gaussianBlur(image, mask);
  }

  for (int v = 0; v < image.rows(); ++v) {
    for (int u = 0; u < image.cols(); ++u) {
      if (mask(v, u) <= 0) continue;
      image(v, u) = std::max(0.0f, std::min(255.0f, image(v, u)));  // truncate to 255
    }
  }

  return image;
}

}  // namespace bievr
