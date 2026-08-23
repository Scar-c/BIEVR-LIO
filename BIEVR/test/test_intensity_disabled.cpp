// Second review, Test 12: with the map's intensity channel disabled
// (intensity.enabled=false), the voxel must never allocate or update the
// intensity raster nor compute intensity information, while geometry still
// works. This verifies the master switch really cuts the extra intensity work.

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/common.h"

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
  mcfg.intensity_enabled = false;  // master switch off
  bievr::BIEVRMap map(mcfg);

  // 9-point grid + far point (spans >= 2 voxels so the group is processed).
  bievr::Pointcloud cloud;
  cloud.resize(10);
  bievr::Intensities inten(1, 10);
  std::vector<double> ranges(10, 4.0);
  int k = 0;
  for (int iy = 0; iy < 3; ++iy) {
    for (int ix = 0; ix < 3; ++ix) {
      cloud[k] << 0.1 + 0.6 * ix, 0.1 + 0.6 * iy, 0.0;
      inten(0, k) = 100.0f;
      ++k;
    }
  }
  cloud[9] << 2.5, 0.5, 0.2;
  inten(0, 9) = 100.0f;

  // Even with an aligned intensity row supplied, the disabled map must not use
  // it (update_intensity requires the map's intensity channel to be enabled).
  if (!map.integratePoints(cloud, &ranges, &inten)) return fail("map integration failed");

  const bievr::Voxel* voxel = map.getVoxel(map.hashIndex(cloud[0]));
  if (!voxel) return fail("voxel not observed");

  // Geometry is unaffected...
  if (voxel->bump_weights_.sum() == 0.0f) return fail("geometry update broken");
  if (voxel->height.bump_img_.rows() == 0) return fail("height image missing");
  // ...but no intensity raster is ever allocated...
  if (voxel->intensity.intensity_img_.rows() != 0) return fail("intensity image allocated while disabled");
  // ...and no intensity information is computed.
  if (voxel->intensity.intensity_information_.norm() != 0.0) {
    return fail("intensity information computed while disabled");
  }

  return 0;
}
