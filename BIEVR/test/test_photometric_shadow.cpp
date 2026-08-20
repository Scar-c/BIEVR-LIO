// Round 5, shadow photometric mode:
// With shadow_photometric=true and photometric_residual=false, the photometric
// residual/J/H/b diagnostics are computed but NEVER merged into the LM solve,
// so the resulting pose must be identical to the pure-geometry run (to machine
// precision) while photo diagnostics are populated.

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/common.h"
#include "bievr_lio/ls_optimizer.h"

#include <cmath>
#include <iostream>
#include <vector>

namespace {

int fail(const char* msg) {
  std::cerr << msg << '\n';
  return 1;
}

// 5x5 dense grid (0.05 m -> adjacent pixels, mature) + far point.
void buildCloud(bievr::Pointcloud& cloud, bievr::Intensities& inten, std::vector<double>& ranges) {
  const int N = 25 + 1;
  cloud.resize(N);
  inten = bievr::Intensities(1, N);
  ranges.assign(N, 4.0);
  int k = 0;
  for (int iy = 0; iy < 5; ++iy) {
    for (int ix = 0; ix < 5; ++ix) {
      cloud[k] << 0.1 + 0.05 * ix, 0.1 + 0.05 * iy, 0.0;
      inten(0, k) = static_cast<float>(10.0 * ix + 5.0 * iy);
      ++k;
    }
  }
  cloud[25] << 2.5, 0.5, 0.0;
  inten(0, 25) = 0.0f;
}

}  // namespace

int main() {
  bievr::BIEVRMap::Config mcfg;
  mcfg.max_size = 100;
  mcfg.voxel_size = 2.0;
  mcfg.px_size = 0.05;
  mcfg.weighted = false;  // every pixel W = 1.0 (mature)
  mcfg.smooth = false;
  mcfg.intensity_enabled = true;
  bievr::BIEVRMap map(mcfg);

  bievr::Pointcloud cloud;
  bievr::Intensities inten;
  std::vector<double> ranges;
  buildCloud(cloud, inten, ranges);
  if (!map.integratePoints(cloud, &ranges, &inten)) return fail("map build failed");

  // Intensity source: same grid, constant intensity (nonzero photo residual).
  bievr::Intensities inten_200 = bievr::Intensities::Constant(1, cloud.size(), 200.0);

  bievr::RegistrationConfig reg_geo;
  reg_geo.photometric_residual = false;
  bievr::LsqRegistration opt_geo(map, cloud, bievr::Pointcloud(), bievr::Intensities(), reg_geo);
  const bievr::Transform pose_geo = opt_geo.computeTransformation(bievr::Transform::Identity());

  bievr::RegistrationConfig reg_shadow;
  reg_shadow.photometric_residual = false;
  reg_shadow.shadow_photometric = true;
  bievr::LsqRegistration opt_shadow(map, cloud, cloud, inten_200, reg_shadow);
  const bievr::Transform pose_shadow = opt_shadow.computeTransformation(bievr::Transform::Identity());

  // Shadow must not change the pose (geometry-only solve in both cases).
  if ((pose_shadow.translation() - pose_geo.translation()).norm() > 1e-9) {
    return fail("shadow mode changed the pose");
  }
  if ((pose_shadow.rotation() - pose_geo.rotation()).norm() > 1e-9) {
    return fail("shadow mode changed the orientation");
  }

  // But the photo diagnostics must be populated.
  const auto& diag = opt_shadow.photometricDiagnostics();
  if (diag.valid_matches <= 0) return fail("shadow mode produced no photo matches");
  if (diag.candidates <= 0) return fail("shadow mode produced no photo candidates");

  return 0;
}
