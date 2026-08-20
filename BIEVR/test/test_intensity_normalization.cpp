// COIN-BIEVR reproduction plan, Section 43:
// Sparse brightness normalization must average only non-empty pixels.
//   image:
//     0   0   50   0
//     0   0  100   0
//     0   0  150   0
//   brightness I_B = (50+100+150)/3 = 100  (empty pixels are NOT counted as 0)
//   I_F = 140 * I / (I_B + 1)

#include "bievr_lio/intensity_processor.h"

#include <cmath>
#include <iostream>

namespace {

bool near(double a, double b, double tol = 1e-6) { return std::abs(a - b) <= tol; }

int fail(const char* msg) {
  std::cerr << msg << '\n';
  return 1;
}

}  // namespace

int main() {
  bievr::IntensityProcessorConfig cfg;
  cfg.image_width = 4;
  cfg.image_height = 3;
  cfg.brightness_window_u = 0;  // full image
  cfg.brightness_window_v = 0;
  cfg.normalization_scale = 140.0;
  cfg.remove_lines = false;
  cfg.gaussian_blur = false;
  bievr::IntensityProcessor processor(cfg);

  Eigen::MatrixXf I(3, 4);
  I << 0, 0, 50, 0, 0, 0, 100, 0, 0, 0, 150, 0;
  Eigen::MatrixXi mask(3, 4);
  mask << 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0;

  const Eigen::MatrixXf F = processor.normalizeImage(I, mask);

  const double IB = 100.0;  // sparse mean of {50, 100, 150}
  if (!near(F(0, 2), 140.0 * 50 / (IB + 1.0), 1e-4)) return fail("I_F(50) mismatch");
  if (!near(F(1, 2), 140.0 * 100 / (IB + 1.0), 1e-4)) return fail("I_F(100) mismatch");
  if (!near(F(2, 2), 140.0 * 150 / (IB + 1.0), 1e-4)) return fail("I_F(150) mismatch");
  if (F(0, 0) != 0.0f || F(0, 3) != 0.0f) return fail("empty pixels were modified");

  return 0;
}
