#include "bievr_lio/intensity_processor.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "bievr_lio/log++.h"

namespace bievr {

namespace {

constexpr double kPi = 3.14159265358979323846;

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

}  // namespace

IntensityProcessor::IntensityProcessor(const IntensityProcessorConfig& config) : config_(config) {}

bool IntensityProcessor::projectPoint(const Point& p, int& u, int& v) const {
  const double r = p.norm();
  if (!std::isfinite(r) || r < 1e-6) return false;

  const double az = std::atan2(p.y(), p.x());
  const double el = std::asin(std::max(-1.0, std::min(1.0, p.z() / r)));

  const int W = config_.image_width;
  const int H = config_.image_height;
  const double fov_rad = kPi * config_.vertical_fov_deg / 180.0;

  u = static_cast<int>(std::lround(-W / (2.0 * kPi) * az + W / 2.0));
  v = static_cast<int>(std::lround(-H / fov_rad * el + H / 2.0));

  // Clamp into the image (Scheme B): keep every finite point in the normalized
  // domain. A sensor-FOV / image-size mismatch shows up in the collision
  // diagnostics (points piling up on the boundary rows) instead of dropping
  // points from the full-cloud map update.
  u = std::max(0, std::min(W - 1, u));
  v = std::max(0, std::min(H - 1, v));
  return true;
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
                                                      const IntensityView& raw_intensity) {
  IntensityProcessingResult out;
  const size_t N = points_L.size();
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

  // -------------------------------------------------------------------
  // Debug/ablation bypass mode (preprocessing.enabled == false).
  // This is NOT the COIN-BIEVR paper default: it keeps the point/intensity
  // index alignment but skips brightness normalization.
  // -------------------------------------------------------------------
  if (!config_.enabled) {
    double sum = 0.0;
    out.filtered_min = std::numeric_limits<double>::max();
    out.filtered_max = std::numeric_limits<double>::lowest();
    for (size_t i = 0; i < N; ++i) {
      double v = raw_intensity.size() > i ? raw_intensity(0, i) * config_.raw_intensity_scale
                                          : 0.0;
      if (!std::isfinite(v)) v = 0.0;  // sanitize
      v = std::max(0.0, std::min(255.0, v));  // clamp
      out.filtered(0, i) = static_cast<float>(v);
      sum += v;
      out.filtered_min = std::min(out.filtered_min, v);
      out.filtered_max = std::max(out.filtered_max, v);
    }
    out.num_valid = N;
    out.filtered_mean = sum / static_cast<double>(N);
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

  for (size_t i = 0; i < N; ++i) {
    int u = 0, v = 0;
    if (!projectPoint(points_L[i], u, v)) {
      out.point_pixel_idx[i] = -1;
      ++out.num_invalid;
      continue;
    }

    const int linear = v * W + u;
    out.point_pixel_idx[i] = linear;

    if (image.valid(v, u) == 0) {
      ++out.num_unique_pixels;
    } else {
      ++out.num_collisions;
    }

    const double r = points_L[i].norm();
    const float val = static_cast<float>(raw_intensity.size() > i
                                             ? static_cast<double>(raw_intensity(0, i)) *
                                                   config_.raw_intensity_scale
                                             : 0.0);
    // Closest point owns the pixel (defines the representative raw intensity).
    if (image.valid(v, u) == 0 || r < image.range(v, u)) {
      image.intensity(v, u) = val;
      image.point_index(v, u) = static_cast<int>(i);
      image.range(v, u) = static_cast<float>(r);
      image.valid(v, u) = 1;
    }
  }

  // -------------------------------------------------------------------
  // 2. Sparse brightness normalization.
  // -------------------------------------------------------------------
  const Eigen::MatrixXi valid_mask = (image.point_index.array() >= 0).cast<int>();
  image.intensity = normalizeImage(image.intensity, valid_mask);

  // -------------------------------------------------------------------
  // 3. Back-assign I_F(v_i,u_i) to EVERY projected point (not only the pixel
  //    owners), so points sharing a pixel all stay in the normalized domain.
  // -------------------------------------------------------------------
  double sum = 0.0;
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
    out.filtered_min = std::min(out.filtered_min, val);
    out.filtered_max = std::max(out.filtered_max, val);
  }
  out.filtered_mean = out.num_valid > 0 ? sum / static_cast<double>(out.num_valid) : 0.0;

  return out;
}

Eigen::MatrixXf IntensityProcessor::normalizeImage(const Eigen::MatrixXf& intensity_image,
                                                   const Eigen::MatrixXi& mask) const {
  Eigen::MatrixXf image = intensity_image;

  if (config_.remove_lines) {
    removeLines(image, mask);
  }

  // Sparse brightness normalization: only non-empty pixels are averaged into
  // the brightness denominator (COIN-BIEVR modification of COIN-LIO's cv::blur).
  ProjectedIntensityImage img;
  img.intensity = image;
  img.point_index = mask;
  img.valid = (mask.array() > 0).cast<uint8_t>();
  const Eigen::MatrixXd I_B = brightnessImage(img);
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
