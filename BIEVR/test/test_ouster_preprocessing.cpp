// Round 10-B: ENWIDE Ouster OS0-128 preprocessing tests.
//
// 1. Metadata / projector: with the official os_enwide.json beam-altitude
//    angles, the Ouster lookup projector reproduces the EXACT COIN-LIO
//    Projector::projectPoint mapping (oracle compiled from src/projector.cpp,
//    COIN-LIO commit 76729cc4feb3649cbd79d28f82d9f62a2c82889b). Image 1024x128.
// 2. Boundary behaviour: points outside the beam vertical FOV are rejected (no
//    vertical clamping needed), horizontal u is bounded, projection is unique.
// 3. Line-artifact removal: the official line_removal.yaml FIR removes a
//    vertical column stripe from a synthetic intensity image.

#include "bievr_lio/intensity_processor.h"

#include <cmath>
#include <iostream>
#include <vector>

namespace {

int fail(const char* msg) {
  std::cerr << "FAIL: " << msg << '\n';
  return 1;
}

// Official os_enwide.json beam_altitude_angles (deg, descending, 128 entries;
// SHA256 of the source file 618374dbb4c81a212639f39b527e6882427fb79e19d7e3eee05688f02b1b3b3e).
const double kAlt[128] = {45.9052, 44.8424, 44.0789, 43.5846, 42.8873, 41.8531, 41.078,
                          40.5824, 39.8995, 38.8838, 38.1174, 37.6003, 36.9415, 35.9344,
                          35.1667, 34.6484, 33.9634, 32.9849, 32.2361, 31.7065, 31.0251,
                          30.0755, 29.3154, 28.7748, 28.0967, 27.1558, 26.425, 25.8632,
                          25.1982, 24.2863, 23.5444, 22.9819, 22.2994, 21.4067, 20.6839,
                          20.1105, 19.4305, 18.5671, 17.8335, 17.2394, 16.5615, 15.7173,
                          15.0131, 14.3984, 13.7123, 12.8975, 12.1827, 11.5676, 10.873,
                          10.0777, 9.38252, 8.74699, 8.02339, 7.27778, 6.58227, 5.91644,
                          5.22367, 4.47776, 3.78217, 3.10614, 2.40401, 1.69778, 0.992155,
                          0.285974, -0.405973, -1.11203, -1.80823, -2.52406, -3.23622,
                          -3.9123, -4.59801, -5.3439, -6.04652, -6.6924, -7.39795, -8.14355,
                          -8.86702, -9.50272, -10.1978, -10.9731, -11.6977, -12.323, -13.0176,
                          -13.8124, -14.5385, -15.1333, -15.8274, -16.6415, -17.3795, -17.9536,
                          -18.6571, -19.4905, -20.2306, -20.794, -21.4968, -22.3593, -23.0919,
                          -23.6345, -24.3463, -25.228, -25.9733, -26.495, -27.1959, -28.0965,
                          -28.8748, -29.3755, -30.0854, -31.0049, -31.7765, -32.2661, -32.9849,
                          -33.9332, -34.7084, -35.1867, -35.9142, -36.8713, -37.6703, -38.1374,
                          -38.8536, -39.8493, -40.6524, -41.108, -41.8429, -42.8471, -43.6545,
                          -44.1088, -44.8521, -45.8749};

bievr::IntensityProcessor makeOusterProcessor() {
  bievr::IntensityProcessorConfig cfg;
  cfg.enabled = true;
  cfg.projection = "ouster_lut";
  cfg.image_width = 1024;
  cfg.image_height = 128;
  cfg.ouster_beam_altitude_angles.assign(kAlt, kAlt + 128);
  cfg.ouster_beam_offset_m = 27.67 * 1e-3;
  cfg.ouster_u_shift = 0;
  return bievr::IntensityProcessor(cfg);
}

bool projTo(const bievr::IntensityProcessor& p, double x, double y, double z, int& u, int& v) {
  return p.projectPoint(bievr::Point(x, y, z), u, v);
}

}  // namespace

int main() {
  const auto proc = makeOusterProcessor();

  // --- 1. Exact COIN-LIO oracle fixtures (projectPoint continuous -> rounded) ---
  // (u, v, ok): (512.0, 0.777) -> (512, 1); (0,0,5) out of FOV -> false.
  struct F {
    double x, y, z;
    int u, v;
    bool ok;
  };
  const F fixtures[] = {
      {10, 0, 10, 512, 1, true},   {0, 10, 10, 256, 1, true},  {-10, 0, 10, 0, 1, true},
      {10, 10, 0, 384, 63, true},  {5, 5, 5, 384, 14, true},   {0, 0, 5, -1, -1, false},
      {1, 1, 1, 384, 13, true},    {30, 0, 0.5, 512, 62, true}, {-15, -3, 2, 992, 53, true},
      {8, -4, 6, 588, 16, true},   {2, 3, 1, 352, 41, true},   {40, 20, 5, 436, 54, true},
  };
  for (const auto& f : fixtures) {
    int u = 0, v = 0;
    const bool ok = projTo(proc, f.x, f.y, f.z, u, v);
    if (ok != f.ok || (ok && (u != f.u || v != f.v))) {
      std::cerr << "oracle mismatch for (" << f.x << ", " << f.y << ", " << f.z << "): got ("
                << u << ", " << v << ", " << ok << ") want (" << f.u << ", " << f.v << ", "
                << f.ok << ")\n";
      return fail("Ouster projector does not match the COIN-LIO oracle");
    }
  }

  // --- 2. Boundary / uniqueness over a dense scan ---
  // A half-sphere of points: all should project within [0,1024)x[0,128) when
  // inside the vertical FOV, none outside.
  int valid = 0, invalid = 0;
  std::vector<char> occupied(1024 * 128, 0);
  int collisions = 0;
  for (double az = -M_PI; az <= M_PI; az += 0.02) {
    for (double el = -0.9; el <= 0.9; el += 0.03) {
      const double r = 5.0;
      const bievr::Point p(r * std::cos(el) * std::cos(az), r * std::cos(el) * std::sin(az),
                           r * std::sin(el));
      int u = 0, v = 0;
      if (!projTo(proc, p.x(), p.y(), p.z(), u, v)) {
        ++invalid;
        continue;
      }
      if (u < 0 || u >= 1024 || v < 0 || v >= 128) {
        return fail("projector returned out-of-image pixel");
      }
      ++valid;
      if (occupied[v * 1024 + u]) ++collisions;
      occupied[v * 1024 + u] = 1;
    }
  }
  const double unique_ratio =
      valid > 0 ? 1.0 - static_cast<double>(collisions) / valid : 0.0;
  std::cout << "valid=" << valid << " invalid=" << invalid
            << " unique_pixel_ratio=" << unique_ratio << '\n';
  if (unique_ratio < 0.98) return fail("Ouster projection uniqueness < 98%");

  // --- 3. Line-artifact removal ---
  // Synthetic image with HORIZONTAL row stripes (the Ouster line artifact is a
  // per-row offset; COIN-LIO's vertical high-pass removes the row-direction
  // D.C. component). With the official FIR line removal ON, the even/odd row
  // contrast must be clearly smaller than with line removal OFF (both go
  // through the same brightness normalization / truncation).
  bievr::IntensityProcessorConfig base;
  base.projection = "ouster_lut";
  base.image_width = 1024;
  base.image_height = 128;
  base.sparse_brightness = false;
  base.gaussian_blur = false;

  auto row_contrast = [&](bool remove_lines) {
    bievr::IntensityProcessorConfig c = base;
    c.remove_lines = remove_lines;
    bievr::IntensityProcessor nproc(c);
    Eigen::MatrixXf img = Eigen::MatrixXf::Constant(128, 1024, 50.0f);
    for (int y = 0; y < 128; y += 2)
      for (int x = 0; x < 1024; ++x) img(y, x) += 40.0f;
    Eigen::MatrixXi mask = Eigen::MatrixXi::Ones(128, 1024);
    const Eigen::MatrixXf out = nproc.normalizeImage(img, mask);
    // Mean even-row value vs mean odd-row value -> contrast measure.
    double even = 0.0, odd = 0.0;
    int ne = 0, no = 0;
    for (int y = 0; y < 128; ++y)
      for (int x = 0; x < 1024; ++x) {
        if (y % 2 == 0) {
          even += out(y, x);
          ++ne;
        } else {
          odd += out(y, x);
          ++no;
        }
      }
    even /= ne;
    odd /= no;
    return std::abs(even - odd);
  };

  const double c_on = row_contrast(true);
  const double c_off = row_contrast(false);
  std::cout << "row stripe contrast line_removal on=" << c_on << " off=" << c_off << '\n';
  if (c_on >= 0.5 * c_off) return fail("line removal did not attenuate the row stripe");

  return 0;
}
