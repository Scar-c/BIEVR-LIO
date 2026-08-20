// Round 5, photometric warmup gate:
// photometric_residual only enters the LM after the pipeline has been in
// Running for photometric_warmup_s seconds. Before warmup the pose must equal
// the pure-geometry pose; after warmup the photometric term may pull it.
// (photometric_warmup_s is an engineering safeguard / TBD, not a paper param.)

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/common.h"
#include "bievr_lio/ls_optimizer.h"

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
  mcfg.weighted = false;  // mature
  mcfg.smooth = false;
  mcfg.intensity_enabled = true;
  bievr::BIEVRMap map(mcfg);

  const int N = 26;
  bievr::Pointcloud cloud;
  cloud.resize(N);
  bievr::Intensities inten(1, N);
  std::vector<double> ranges(N, 4.0);
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
  if (!map.integratePoints(cloud, &ranges, &inten)) return fail("map build failed");

  // Intensity source: same grid with a small constant offset -> a gentle,
  // stable photo pull (residual ~5, pull ~cm on this synthetic map).
  bievr::Intensities inten_off = inten;
  for (int i = 0; i < inten_off.size(); ++i) inten_off(0, i) += 5.0f;

  bievr::RegistrationConfig reg_geo;
  reg_geo.photometric_residual = false;
  bievr::LsqRegistration opt_geo(map, cloud, bievr::Pointcloud(), bievr::Intensities(), reg_geo);
  const bievr::Transform pose_geo = opt_geo.computeTransformation(bievr::Transform::Identity());

  const double warmup = 10.0;

  // Pre-warmup: photo computed but not merged -> pose identical to geometry.
  bievr::RegistrationConfig reg_warm;
  reg_warm.photometric_residual = true;
  reg_warm.photometric_scale = 0.2;
  reg_warm.photometric_warmup_s = warmup;
  reg_warm.photometric_elapsed_s = 0.0;
  bievr::LsqRegistration opt_warm(map, cloud, cloud, inten_off, reg_warm);
  const bievr::Transform pose_warm = opt_warm.computeTransformation(bievr::Transform::Identity());
  if ((pose_warm.translation() - pose_geo.translation()).norm() > 1e-9) {
    return fail("pre-warmup photometric residual entered the LM");
  }
  if (opt_warm.photometricDiagnostics().photo_effective_residuals <= 0) {
    return fail("photo residual not computed pre-warmup");
  }

  // Post-warmup: photo merged -> pose differs from geometry.
  bievr::RegistrationConfig reg_active;
  reg_active.photometric_residual = true;
  reg_active.photometric_scale = 0.2;
  reg_active.photometric_warmup_s = warmup;
  reg_active.photometric_elapsed_s = 2.0 * warmup;
  bievr::LsqRegistration opt_active(map, cloud, cloud, inten_off, reg_active);
  const bievr::Transform pose_active =
      opt_active.computeTransformation(bievr::Transform::Identity());
  if (opt_active.photometricDiagnostics().photo_effective_residuals <= 0) {
    return fail("photo residual not computed post-warmup");
  }
  if ((pose_active.translation() - pose_geo.translation()).norm() < 1e-6) {
    return fail("post-warmup photometric residual did not affect the pose");
  }

  return 0;
}
