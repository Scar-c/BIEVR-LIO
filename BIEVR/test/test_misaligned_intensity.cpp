// Second review, Test 10: misaligned intensity protection.
// integratePoints() must reject an intensity row whose size does not match the
// cloud: geometry is still updated, but NO intensity (especially not a
// zero-filled row) may be merged into the map.

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/common.h"

#include <iostream>

namespace {

int fail(const char* msg) {
  std::cerr << msg << '\n';
  return 1;
}

}  // namespace

int main() {
  bievr::BIEVRMap::Config mcfg;
  mcfg.max_size = 1000;
  mcfg.voxel_size = 2.0;
  mcfg.px_size = 0.05;
  mcfg.weighted = false;
  mcfg.smooth = false;
  mcfg.intensity_enabled = true;
  bievr::BIEVRMap map(mcfg);

  // 12 points spanning >= 2 voxels (BIEVR only processes voxel groups on a
  // hash change), so the voxel group is actually integrated.
  const int N = 12;
  bievr::Pointcloud cloud;
  cloud.resize(N);
  for (int i = 0; i < N; ++i) {
    cloud[i] << 0.1 + 0.6 * (i % 3), 0.1 + 0.6 * ((i / 3) % 3), 0.0;
  }
  cloud[N - 1] << 2.5, 0.5, 0.2;

  // Misaligned intensity: N-1 values for an N-point cloud.
  bievr::Intensities inten(1, N - 1);
  inten.setConstant(200.0f);
  std::vector<double> ranges(N, 4.0);

  // In release this logs a warning and skips the intensity update; in debug it
  // asserts. Either way the map must not be contaminated.
  map.integratePoints(cloud, &ranges, &inten);

  const bievr::Voxel* voxel = map.getVoxel(map.hashIndex(cloud[0]));
  if (!voxel) return fail("voxel not observed");

  // Geometry still updated...
  if (voxel->bump_weights_.sum() == 0.0f) return fail("geometry update broken");
  // ...but the intensity map must stay untouched (no 200.0 / no 0.0 injected).
  if (voxel->intensity_img_.sum() != 0.0f) {
    return fail("misaligned intensity contaminated the map");
  }

  return 0;
}
