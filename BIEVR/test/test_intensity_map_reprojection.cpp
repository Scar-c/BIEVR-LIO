// COIN-BIEVR reproduction plan, Section 46:
// When the voxel normal changes beyond the tolerance, reprojectImage() must
// carry BOTH the height surface and the intensity texture to the new local
// frame, keeping them strictly co-registered (same resolution, same support).

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/common.h"

#include <cmath>
#include <iostream>

namespace {

int fail(const char* msg) {
  std::cerr << msg << '\n';
  return 1;
}

double angleDeg(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
  const Eigen::AngleAxisd aa(a.transpose() * b);
  return std::abs(aa.angle()) * 180.0 / M_PI;
}

}  // namespace

int main() {
  bievr::BIEVRMap::Config mcfg;
  mcfg.max_size = 1000;
  mcfg.voxel_size = 2.0;
  mcfg.px_size = 0.05;
  mcfg.weighted = true;
  mcfg.smooth = false;
  mcfg.intensity_enabled = true;
  mcfg.norm_tol_deg = 3.0;
  bievr::BIEVRMap map(mcfg);

  // --- Batch 1: tilted plane z = 0.3*x, checkerboard intensity -----------
  bievr::Pointcloud cloud1;
  cloud1.resize(10);
  bievr::Intensities inten1(1, 10);
  std::vector<double> ranges1(10, 5.0);
  int k = 0;
  for (int iy = 0; iy < 3; ++iy) {
    for (int ix = 0; ix < 3; ++ix) {
      const double x = 0.1 + 0.6 * ix;
      const double y = 0.1 + 0.6 * iy;
      cloud1[k] << x, y, 0.3 * x;
      inten1(0, k) = ((ix + iy) % 2 == 0) ? 40.0f : 200.0f;  // checkerboard
      ++k;
    }
  }
  // Point in a second voxel so the grid voxel group is actually processed.
  cloud1[9] << 2.5, 0.5, 0.5;
  inten1(0, 9) = 100.0f;
  if (!map.integratePoints(cloud1, &ranges1, &inten1)) return fail("batch1 failed");

  const bievr::Voxel* voxel = map.getVoxel(map.hashIndex(cloud1[0]));
  if (!voxel) return fail("voxel not observed after batch1");
  const Eigen::Matrix3d T_C_W_1 = voxel->T_C_W_.linear();
  if (voxel->intensity.intensity_img_.rows() == 0 || voxel->height.bump_img_.rows() == 0) {
    return fail("images not created after batch1");
  }
  if (voxel->intensity.intensity_img_.rows() != voxel->height.bump_img_.rows() ||
      voxel->intensity.intensity_img_.cols() != voxel->height.bump_img_.cols()) {
    return fail("intensity and height images not same size after batch1");
  }

  // Checkerboard must be present in the intensity map.
  bool saw_dark = false, saw_bright = false;
  for (int y = 0; y < voxel->intensity.intensity_img_.rows(); ++y) {
    for (int x = 0; x < voxel->intensity.intensity_img_.cols(); ++x) {
      if (voxel->bump_weights_(y, x) <= 0.f) continue;
      const float v = voxel->intensity.intensity_img_(y, x);
      if (v < 100.f) saw_dark = true;
      if (v >= 100.f) saw_bright = true;
    }
  }
  if (!saw_dark || !saw_bright) return fail("checkerboard intensity missing after batch1");

  // --- Batch 2: near-vertical plane x = 0.2 (normal shift >> 3 deg) -------
  bievr::Pointcloud cloud2;
  cloud2.resize(10);
  bievr::Intensities inten2(1, 10);
  std::vector<double> ranges2(10, 5.0);
  k = 0;
  for (int iy = 0; iy < 3; ++iy) {
    for (int iz = 0; iz < 3; ++iz) {
      const double y = 0.1 + 0.6 * iy;
      const double z = 0.1 + 0.6 * iz;
      cloud2[k] << 0.2, y, z;
      inten2(0, k) = 120.0f;
      ++k;
    }
  }
  cloud2[9] << 2.5, 0.5, 1.5;
  inten2(0, 9) = 100.0f;
  if (!map.integratePoints(cloud2, &ranges2, &inten2)) return fail("batch2 failed");

  voxel = map.getVoxel(map.hashIndex(cloud1[0]));
  if (!voxel) return fail("voxel lost after batch2");
  const Eigen::Matrix3d T_C_W_2 = voxel->T_C_W_.linear();

  // The normal change must have triggered a reprojection.
  if (angleDeg(T_C_W_1, T_C_W_2) < 1.0) return fail("no reprojection triggered by batch2");

  // After reprojection the two maps must still be strictly co-registered.
  if (voxel->intensity.intensity_img_.rows() != voxel->height.bump_img_.rows() ||
      voxel->intensity.intensity_img_.cols() != voxel->height.bump_img_.cols()) {
    return fail("intensity/height size divergence after reprojection");
  }
  int valid_pixels = 0;
  for (int y = 0; y < voxel->height.bump_img_.rows(); ++y) {
    for (int x = 0; x < voxel->height.bump_img_.cols(); ++x) {
      if (voxel->bump_weights_(y, x) > 0.f) ++valid_pixels;
    }
  }
  if (valid_pixels == 0) return fail("reprojected maps empty after batch2");

  return 0;
}
