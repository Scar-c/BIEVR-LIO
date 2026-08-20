// Second review, Test 11: Eq. (8) score must be invariant to the eigenvector
// sign in abs_components mode: score(eta) == score(-eta). This guarantees the
// voxel ranking does not change when the eigen solver flips the sign of v1/v2.
// Also checks that paper_signed mode flips sign (confirming the modes differ).

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/common.h"
#include "bievr_lio/intensity_sampling.h"

#include <cmath>
#include <iostream>
#include <vector>

namespace {

int fail(const char* msg) {
  std::cerr << msg << '\n';
  return 1;
}

}  // namespace

int main() {
  bievr::BIEVRMap::Config mcfg;
  mcfg.max_size = 100;
  mcfg.voxel_size = 2.0;
  mcfg.px_size = 0.05;
  mcfg.weighted = false;
  mcfg.smooth = false;
  mcfg.intensity_enabled = true;
  bievr::BIEVRMap map(mcfg);

  // Dense 3x3 block (0.05 m spacing -> adjacent pixels) with an intensity ramp,
  // so the observed voxel has non-zero intensity information. A tenth point in
  // a second voxel makes the grid voxel group get processed.
  bievr::Pointcloud cloud;
  cloud.resize(10);
  bievr::Intensities inten(1, 10);
  std::vector<double> ranges(10, 4.0);
  int k = 0;
  for (int iy = 0; iy < 3; ++iy) {
    for (int ix = 0; ix < 3; ++ix) {
      cloud[k] << 0.1 + 0.05 * ix, 0.1 + 0.05 * iy, 0.0;
      inten(0, k) = static_cast<float>(10.0 + 10.0 * ix);
      ++k;
    }
  }
  cloud[9] << 2.5, 0.5, 0.0;
  inten(0, 9) = 0.0f;
  if (!map.integratePoints(cloud, &ranges, &inten)) return fail("map integration failed");

  const bievr::Voxel* voxel = map.getVoxel(map.hashIndex(cloud[0]));
  if (!voxel) return fail("voxel not observed");
  if (voxel->intensity_information_.norm() < 1e-9) return fail("voxel has no intensity information");

  std::vector<size_t> hashes = {map.hashIndex(cloud[0])};
  const Eigen::Vector3d eta(1.0, 0.0, 0.0);

  const auto s_pos = bievr::scoreIntensityVoxels(map, hashes, eta, "abs_components");
  const auto s_neg = bievr::scoreIntensityVoxels(map, hashes, -eta, "abs_components");
  if (s_pos.size() != 1 || s_neg.size() != 1) return fail("score lookup failed");
  if (std::abs(s_pos[0].first - s_neg[0].first) > 1e-12) {
    return fail("abs_components score is not sign invariant");
  }

  const auto p_pos = bievr::scoreIntensityVoxels(map, hashes, eta, "paper_signed");
  const auto p_neg = bievr::scoreIntensityVoxels(map, hashes, -eta, "paper_signed");
  if (std::abs(p_pos[0].first + p_neg[0].first) > 1e-9) {
    return fail("paper_signed mode should flip sign with -eta");
  }

  return 0;
}
