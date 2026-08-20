// COIN-BIEVR reproduction plan, Section 47:
// Intensity information (Eq. 6) computed on a dense 3x3 block of adjacent
// pixels (points 0.05 m apart -> one pixel per point):
//   10 20 30  (varies along one world axis)
//   10 20 30
//   10 20 30
// must yield an iota with a single non-zero component; the transposed pattern
// must yield the orthogonal single component. (The voxel local frame is tilted,
// so "image x" is some fixed world direction; the check is rotation-agnostic.)

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/common.h"

#include <iostream>

namespace {

int fail(const char* msg) {
  std::cerr << msg << '\n';
  return 1;
}

// Builds a dense 3x3 block on the z=0 plane inside a single voxel with
// intensity `value(ix,iy)`, integrates it and returns the voxel. A tenth point
// in a second voxel is added because BIEVR only processes voxel groups on a
// hash change (a single-voxel cloud is not integrated).
const bievr::Voxel* buildGradientVoxel(bievr::BIEVRMap& map, double (*value)(int, int)) {
  bievr::Pointcloud cloud;
  cloud.resize(10);
  bievr::Intensities inten(1, 10);
  std::vector<double> ranges(10, 4.0);
  int k = 0;
  for (int iy = 0; iy < 3; ++iy) {
    for (int ix = 0; ix < 3; ++ix) {
      cloud[k] << 0.1 + 0.05 * ix, 0.1 + 0.05 * iy, 0.0;
      inten(0, k) = static_cast<float>(value(ix, iy));
      ++k;
    }
  }
  cloud[9] << 2.5, 0.5, 0.0;
  inten(0, 9) = 0.0f;
  if (!map.integratePoints(cloud, &ranges, &inten)) return nullptr;
  return map.getVoxel(map.hashIndex(cloud[0]));
}

double alongX(int ix, int /*iy*/) { return 10.0 + 10.0 * ix; }
double alongY(int /*ix*/, int iy) { return 10.0 + 10.0 * iy; }

}  // namespace

int main() {
  bievr::BIEVRMap::Config mcfg;
  mcfg.max_size = 100;
  mcfg.voxel_size = 2.0;
  mcfg.px_size = 0.05;
  mcfg.weighted = false;
  mcfg.smooth = false;
  mcfg.intensity_enabled = true;
  bievr::BIEVRMap map1(mcfg);
  const bievr::Voxel* voxel1 = buildGradientVoxel(map1, alongX);
  if (!voxel1) return fail("voxel not built (pattern along X)");
  const auto iotaX = voxel1->intensity_information_;

  bievr::BIEVRMap map2(mcfg);
  const bievr::Voxel* voxel2 = buildGradientVoxel(map2, alongY);
  if (!voxel2) return fail("voxel not built (pattern along Y)");
  const auto iotaY = voxel2->intensity_information_;

  const bool iX_x = iotaX.x() > 1e-9, iX_y = iotaX.y() > 1e-9;
  const bool iY_x = iotaY.x() > 1e-9, iY_y = iotaY.y() > 1e-9;

  // Each pattern must activate exactly one image axis.
  if (iX_x == iX_y) return fail("pattern along X should activate a single axis");
  if (iY_x == iY_y) return fail("pattern along Y should activate a single axis");
  // The two orthogonal patterns must activate orthogonal axes.
  if (iX_x != iY_y || iX_y != iY_x) return fail("orthogonal patterns give non-orthogonal iota");

  return 0;
}
