// Round 6, exact masked-bilinear intensity gradient (Level A, synthetic).
// sampleIntensityBilinearWithGradient() must return the EXACT derivative of the
// masked normalized bilinear interpolation, covering the validity masks:
//   4 / 3 / 2-adjacent / 2-diagonal / 1 / 0 valid corners.

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/common.h"
#include "bievr_lio/ls_optimizer.h"

#include <cmath>
#include <iostream>
#include <random>
#include <vector>

namespace {

int fail(const char* msg) {
  std::cerr << msg << '\n';
  return 1;
}

bool near(double a, double b, double tol = 1e-9) { return std::abs(a - b) <= tol; }

// 2x2 cell with corners (x0..x0+1, y0..y0+1); weights m (1 = valid) and
// intensity values I.
struct Cell {
  double m00, m10, m01, m11;
  double I00, I10, I01, I11;
};

void fillVoxel(bievr::Voxel& v, const Cell& c, int x0 = 1, int y0 = 1) {
  v.intensity.intensity_img_.resize(4, 4);
  v.bump_weights_.resize(4, 4);
  v.intensity.intensity_img_.setZero();
  v.bump_weights_.setZero();
  v.intensity.intensity_img_(y0, x0) = c.I00;
  v.intensity.intensity_img_(y0, x0 + 1) = c.I10;
  v.intensity.intensity_img_(y0 + 1, x0) = c.I01;
  v.intensity.intensity_img_(y0 + 1, x0 + 1) = c.I11;
  v.bump_weights_(y0, x0) = c.m00;
  v.bump_weights_(y0, x0 + 1) = c.m10;
  v.bump_weights_(y0 + 1, x0) = c.m01;
  v.bump_weights_(y0 + 1, x0 + 1) = c.m11;
}

// Reference masked value: N / S.
double refValue(const Cell& c, double a, double b) {
  const double w00 = (1 - a) * (1 - b), w10 = a * (1 - b), w01 = (1 - a) * b, w11 = a * b;
  const double S = c.m00 * w00 + c.m10 * w10 + c.m01 * w01 + c.m11 * w11;
  const double N = c.m00 * w00 * c.I00 + c.m10 * w10 * c.I10 + c.m01 * w01 * c.I01 +
                   c.m11 * w11 * c.I11;
  return N / S;
}

}  // namespace

int main() {
  // --- Case 6: 0 valid corners -> invalid --------------------------------
  {
    bievr::Voxel v;
    fillVoxel(v, Cell{0, 0, 0, 0, 1, 2, 3, 4});
    bievr::IntensitySample s;
    if (sampleIntensityBilinearWithGradient(&v, 1.5, 1.5, s)) return fail("0-corner must be invalid");
  }

  // --- Case 1: 4 valid corners; value and gradient vs explicit formula -----
  {
    bievr::Voxel v;
    fillVoxel(v, Cell{1, 1, 1, 1, 10, 20, 30, 40});
    const double a = 0.5, b = 0.5;
    bievr::IntensitySample s;
    if (!sampleIntensityBilinearWithGradient(&v, 1.0 + a, 1.0 + b, s)) return fail("4-corner sample invalid");
    // value = mean = 25
    if (!near(s.value, 25.0, 1e-9)) return fail("4-corner value mismatch");
    // oracle gradient (all 4 valid, S=1):
    //   dI/dx = (1-b)(I10-I00) + b(I11-I01) = 0.5*10 + 0.5*10 = 10
    //   dI/dy = (1-a)(I01-I00) + a(I11-I10) = 0.5*20 + 0.5*20 = 20
    if (!near(s.gradient_pixel.x(), 10.0, 1e-9)) return fail("4-corner dI/dx mismatch");
    if (!near(s.gradient_pixel.y(), 20.0, 1e-9)) return fail("4-corner dI/dy mismatch");
  }

  // --- Case 5: 1 valid corner -> value = that corner, gradient ~ 0 ---------
  {
    bievr::Voxel v;
    fillVoxel(v, Cell{1, 0, 0, 0, 7, 0, 0, 0});
    const double a = 0.25, b = 0.75;
    bievr::IntensitySample s;
    if (!sampleIntensityBilinearWithGradient(&v, 1.0 + a, 1.0 + b, s)) {
      return fail("1-corner sample should be valid");
    }
    if (!near(s.value, 7.0, 1e-9)) return fail("1-corner value mismatch");
    if (std::abs(s.gradient_pixel.x()) > 1e-9 || std::abs(s.gradient_pixel.y()) > 1e-9) {
      return fail("1-corner gradient must be ~0 (no fabricated gradient)");
    }
  }

  // --- Random synthetic cells (4 valid) vs image-space FD (Level A) ---------
  {
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    std::uniform_real_distribution<double> frac(0.05, 0.95);
    std::uniform_real_distribution<double> val(0.0, 255.0);
    const double eps = 1e-4;
    std::vector<double> errs;
    for (int trial = 0; trial < 2000; ++trial) {
      bievr::Voxel v;
      fillVoxel(v, Cell{1, 1, 1, 1, val(rng), val(rng), val(rng), val(rng)});
      const double a = frac(rng), b = frac(rng);
      bievr::IntensitySample s;
      if (!sampleIntensityBilinearWithGradient(&v, 1.0 + a, 1.0 + b, s)) continue;
      // FD value-only at (a +/- eps)
      const double v_px = refValue(Cell{1, 1, 1, 1, v.intensity.intensity_img_(1, 1), v.intensity.intensity_img_(1, 2),
                                        v.intensity.intensity_img_(2, 1), v.intensity.intensity_img_(2, 2)},
                                   a + eps, b);
      const double v_mx = refValue(Cell{1, 1, 1, 1, v.intensity.intensity_img_(1, 1), v.intensity.intensity_img_(1, 2),
                                        v.intensity.intensity_img_(2, 1), v.intensity.intensity_img_(2, 2)},
                                   a - eps, b);
      const double v_py = refValue(Cell{1, 1, 1, 1, v.intensity.intensity_img_(1, 1), v.intensity.intensity_img_(1, 2),
                                        v.intensity.intensity_img_(2, 1), v.intensity.intensity_img_(2, 2)},
                                   a, b + eps);
      const double v_my = refValue(Cell{1, 1, 1, 1, v.intensity.intensity_img_(1, 1), v.intensity.intensity_img_(1, 2),
                                        v.intensity.intensity_img_(2, 1), v.intensity.intensity_img_(2, 2)},
                                   a, b - eps);
      const double fd_x = (v_px - v_mx) / (2 * eps);
      const double fd_y = (v_py - v_my) / (2 * eps);
      const double scale_x = std::max({1.0, std::abs(s.gradient_pixel.x()), std::abs(fd_x)});
      const double scale_y = std::max({1.0, std::abs(s.gradient_pixel.y()), std::abs(fd_y)});
      errs.push_back(std::abs(s.gradient_pixel.x() - fd_x) / scale_x);
      errs.push_back(std::abs(s.gradient_pixel.y() - fd_y) / scale_y);
    }
    std::sort(errs.begin(), errs.end());
    if (errs.size() < 1000) return fail("insufficient random-cell samples");
    const double med = errs[errs.size() / 2];
    const double p95 = errs[static_cast<size_t>(0.95 * (errs.size() - 1))];
    if (med > 1e-3) return fail("Level A synthetic median error > 1e-3");
    if (p95 > 1e-2) return fail("Level A synthetic P95 error > 1e-2");
  }

  // --- 3 / 2-adjacent / 2-diagonal valid corners vs FD ---------------------
  {
    const Cell cases[] = {
        Cell{1, 1, 1, 0, 10, 20, 30, 40},  // 3 corners
        Cell{1, 1, 0, 0, 10, 20, 30, 40},  // 2 adjacent
        Cell{1, 0, 0, 1, 10, 20, 30, 40},  // 2 diagonal
    };
    const double eps = 1e-4;
    for (const auto& c : cases) {
      bievr::Voxel v;
      fillVoxel(v, c);
      const double a = 0.4, b = 0.4;
      bievr::IntensitySample s;
      if (!sampleIntensityBilinearWithGradient(&v, 1.0 + a, 1.0 + b, s)) {
        return fail("masked sample should be valid");
      }
      const double fd_x = (refValue(c, a + eps, b) - refValue(c, a - eps, b)) / (2 * eps);
      const double fd_y = (refValue(c, a, b + eps) - refValue(c, a, b - eps)) / (2 * eps);
      const double ex = std::abs(s.gradient_pixel.x() - fd_x) /
                        std::max({1.0, std::abs(s.gradient_pixel.x()), std::abs(fd_x)});
      const double ey = std::abs(s.gradient_pixel.y() - fd_y) /
                        std::max({1.0, std::abs(s.gradient_pixel.y()), std::abs(fd_y)});
      if (ex > 1e-3 || ey > 1e-3) return fail("masked-case gradient vs FD mismatch");
    }
  }

  return 0;
}
