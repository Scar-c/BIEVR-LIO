// COIN-BIEVR reproduction plan, Section 45:
// Height and intensity are updated in the same pixel update with the same
// range weight (Eq. 4-5):
//   I1 = 50, w1 = 0.2  (range 5.0 -> min(0.5, 1/5))
//   I2 = 100, w2 = 0.1 (range 10.0 -> min(0.5, 1/10))
//   I_P = (0.2*50 + 0.1*100) / 0.3 = 66.6667,  W = 0.3

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/common.h"

#include <cmath>
#include <iostream>

namespace {

bool near(double a, double b, double tol = 1e-4) { return std::abs(a - b) <= tol; }

int fail(const char* msg) {
  std::cerr << msg << '\n';
  return 1;
}

}  // namespace

int main() {
  bievr::BIEVRMap::Config mcfg;
  mcfg.max_size = 1000;
  mcfg.voxel_size = 2.0;  // one voxel holds the whole cloud
  mcfg.px_size = 0.05;
  mcfg.weighted = true;
  mcfg.smooth = false;
  mcfg.norm_tol_deg = 3.0;
  bievr::BIEVRMap map(mcfg);

  bievr::Pointcloud cloud;
  cloud.resize(7);
  bievr::Intensities inten(1, 7);
  std::vector<double> ranges(7);

  // 4 points on the z=0 plane (establish the voxel normal).
  cloud[0] << 0.0, 0.0, 0.0;
  cloud[1] << 0.8, 0.0, 0.0;
  cloud[2] << 0.0, 0.8, 0.0;
  cloud[3] << 0.8, 0.8, 0.0;
  ranges[0] = ranges[1] = ranges[2] = ranges[3] = 3.0;
  inten(0, 0) = inten(0, 1) = inten(0, 2) = inten(0, 3) = 1.0f;

  // Two test points at the SAME (x,y) -> same pixel, different z/range/intensity.
  cloud[4] << 0.3, 0.4, 0.0;
  ranges[4] = 5.0;   // w = 0.2
  inten(0, 4) = 50.0f;
  cloud[5] << 0.3, 0.4, 0.1;
  ranges[5] = 10.0;  // w = 0.1
  inten(0, 5) = 100.0f;

  // A point in a second voxel: BIEVR only processes voxel groups when the hash
  // changes, so a cloud that lies entirely in one voxel is not integrated.
  cloud[6] << 2.5, 0.5, 0.2;
  ranges[6] = 3.0;
  inten(0, 6) = 1.0f;

  if (!map.integratePoints(cloud, &ranges, &inten)) return fail("integratePoints failed");

  const bievr::Voxel* voxel = map.getVoxel(map.hashIndex(cloud[4]));
  if (!voxel) return fail("voxel not observed");

  // Locate the pixel the two test points land in (same (x,y) -> same pixel).
  const bievr::Point p_O = voxel->T_C_W_ * cloud[4];
  const int x = static_cast<int>(std::round(p_O.x() / 0.05));
  const int y = static_cast<int>(std::round(p_O.y() / 0.05));
  if (x < 0 || x >= voxel->bump_img_.cols() || y < 0 || y >= voxel->bump_img_.rows()) {
    return fail("test pixel out of image bounds");
  }

  const double w1 = 0.2, w2 = 0.1;
  if (!near(voxel->bump_weights_(y, x), w1 + w2, 1e-6)) return fail("pixel weight mismatch");
  if (!near(voxel->intensity_img_(y, x), (w1 * 50 + w2 * 100) / (w1 + w2), 1e-3)) {
    return fail("weighted intensity average mismatch");
  }
  // Height must be updated with the exact same weight. The voxel local frame is
  // tilted (PCA normal of the small point set), so the expected value is
  // computed from the actual projected z of the two points.
  const bievr::Point p_O_A = voxel->T_C_W_ * cloud[4];
  const bievr::Point p_O_B = voxel->T_C_W_ * cloud[5];
  const double expect_H = (w1 * p_O_A(2) + w2 * p_O_B(2)) / (w1 + w2);
  if (!near(voxel->bump_img_(y, x), expect_H, 1e-4)) return fail("weighted height mismatch");

  return 0;
}
