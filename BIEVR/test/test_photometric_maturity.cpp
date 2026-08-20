// Round 5, map-maturity gate:
// a photometric match is only used when the shared voxel weight W(u,v) >=
// photometric_min_map_weight (default 1.0). With weighted=false every pixel has
// W = 1.0 (mature -> matches). With weighted=true and a single far-range point
// per pixel (w = min(0.5, 1/r) < 1) the pixels are immature -> no photo matches.

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
  const int N = 26;

  // --- Mature map (weighted=false -> every pixel W = 1.0) -----------------
  {
    bievr::BIEVRMap::Config mcfg;
    mcfg.max_size = 100;
    mcfg.voxel_size = 2.0;
    mcfg.px_size = 0.05;
    mcfg.weighted = false;
    mcfg.smooth = false;
    mcfg.intensity_enabled = true;
    bievr::BIEVRMap map(mcfg);

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
    if (!map.integratePoints(cloud, &ranges, &inten)) return fail("mature map build failed");

    bievr::Intensities inten_off = bievr::Intensities::Constant(1, cloud.size(), 200.0);
    bievr::RegistrationConfig reg;
    reg.shadow_photometric = true;
    bievr::LsqRegistration opt(map, cloud, cloud, inten_off, reg);
    opt.computeTransformation(bievr::Transform::Identity());
    const auto& diag = opt.photometricDiagnostics();
    if (diag.valid_matches <= 0) {
      return fail("mature map should produce photo matches");
    }
  }

  // --- Immature map (weighted=true, single far-range point per pixel, W<1) --
  {
    bievr::BIEVRMap::Config mcfg;
    mcfg.max_size = 100;
    mcfg.voxel_size = 2.0;
    mcfg.px_size = 0.05;
    mcfg.weighted = true;   // w = min(0.5, 1/r)
    mcfg.smooth = false;
    mcfg.intensity_enabled = true;
    bievr::BIEVRMap map(mcfg);

    bievr::Pointcloud cloud;
    cloud.resize(N);
    bievr::Intensities inten(1, N);
    std::vector<double> ranges(N, 10.0);  // w = 0.1 per point -> W = 0.1 < 1.0
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
    if (!map.integratePoints(cloud, &ranges, &inten)) return fail("immature map build failed");

    bievr::Intensities inten_off = bievr::Intensities::Constant(1, cloud.size(), 200.0);
    bievr::RegistrationConfig reg;
    reg.shadow_photometric = true;
    reg.photometric_min_map_weight = 1.0;
    bievr::LsqRegistration opt(map, cloud, cloud, inten_off, reg);
    opt.computeTransformation(bievr::Transform::Identity());
    const auto& diag = opt.photometricDiagnostics();
    if (diag.valid_matches != 0) {
      return fail("immature map (W<1) must reject all photo matches");
    }
    if (diag.immature_matches <= 0) {
      return fail("immature map should report skipped (immature) matches");
    }
  }

  return 0;
}
