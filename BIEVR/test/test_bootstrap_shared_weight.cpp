// Round 3, bootstrap shared-weight test.
//
// COIN-BIEVR height and intensity share the voxel weights (bump_weights_).
// A geometry-only bootstrap integratePoints() would create pixels with
// W(u,v) > 0 but intensity == 0, biasing every later weighted intensity
// average. This test verifies:
//   Case A (intensity OFF): the original geometry-only bootstrap still works.
//   Case B (intensity ON, fixed path): the first map is initialized jointly
//     (geometry + intensity together, as tryInitMap() does), so a pixel from a
//     valid intensity point carries ~100 (not 100*w/(W_geometry_history + w)).
//   It also demonstrates the old buggy sequence (geometry-only then intensity)
//   to prove the fix changes the result.

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/common.h"

#include <iostream>
#include <vector>

namespace {

int fail(const char* msg) {
  std::cerr << msg << '\n';
  return 1;
}

struct CloudData {
  bievr::Pointcloud cloud;
  bievr::Intensities inten;
  std::vector<double> ranges;
};

// 3x3 grid on the z=0 plane (single voxel) + one far point in a second voxel
// (BIEVR only processes voxel groups on a hash change). All intensities = 100.
CloudData makeCloud() {
  CloudData d;
  const int N = 10;
  d.cloud.resize(N);
  d.inten = bievr::Intensities::Constant(1, N, 100.0f);
  d.ranges.assign(N, 4.0);
  int k = 0;
  for (int iy = 0; iy < 3; ++iy) {
    for (int ix = 0; ix < 3; ++ix) {
      d.cloud[k] << 0.1 + 0.6 * ix, 0.1 + 0.6 * iy, 0.0;
      ++k;
    }
  }
  d.cloud[9] << 2.5, 0.5, 0.0;
  return d;
}

// Mean intensity over valid pixels of a voxel.
double intensityMean(const bievr::Voxel& v) {
  double sum = 0.0;
  size_t cnt = 0;
  for (int y = 0; y < v.intensity.intensity_img_.rows(); ++y) {
    for (int x = 0; x < v.intensity.intensity_img_.cols(); ++x) {
      if (v.bump_weights_(y, x) > 0.f) {
        sum += v.intensity.intensity_img_(y, x);
        ++cnt;
      }
    }
  }
  return cnt > 0 ? sum / static_cast<double>(cnt) : 0.0;
}

bievr::BIEVRMap::Config mapConfig(bool intensity_on) {
  bievr::BIEVRMap::Config mcfg;
  mcfg.max_size = 100;
  mcfg.voxel_size = 2.0;
  mcfg.px_size = 0.05;
  mcfg.weighted = false;  // w_new = 1 -> clean numbers
  mcfg.smooth = false;
  mcfg.intensity_enabled = intensity_on;
  return mcfg;
}

}  // namespace

int main() {
  // --- Case A: intensity OFF, original geometry-only bootstrap -------------
  {
    bievr::BIEVRMap map(mapConfig(/*intensity_on=*/false));
    CloudData d = makeCloud();
    if (!map.integratePoints(d.cloud, &d.ranges, nullptr)) return fail("OFF bootstrap failed");
    const bievr::Voxel* voxel = map.getVoxel(map.hashIndex(d.cloud[0]));
    if (!voxel) return fail("OFF bootstrap: voxel not observed");
    if (voxel->bump_weights_.sum() == 0.0f) return fail("OFF bootstrap: geometry not updated");
    if (voxel->intensity.intensity_img_.rows() != 0) return fail("OFF bootstrap: intensity raster allocated");
  }

  // --- Case B: intensity ON, fixed joint bootstrap (as tryInitMap) ---------
  {
    bievr::BIEVRMap map(mapConfig(/*intensity_on=*/true));
    CloudData d = makeCloud();
    // First map initialization integrates geometry AND intensity together.
    if (!map.integratePoints(d.cloud, &d.ranges, &d.inten)) return fail("ON bootstrap failed");
    const bievr::Voxel* voxel = map.getVoxel(map.hashIndex(d.cloud[0]));
    if (!voxel) return fail("ON bootstrap: voxel not observed");
    const double mean = intensityMean(*voxel);
    if (std::abs(mean - 100.0) > 1e-3) {
      std::cerr << "ON bootstrap mean intensity " << mean << " != 100\n";
      return fail("ON bootstrap: joint init must give ~100, no zero-history bias");
    }
  }

  // --- Old buggy sequence: geometry-only THEN intensity (must be biased) ---
  {
    bievr::BIEVRMap map(mapConfig(/*intensity_on=*/true));
    CloudData d = makeCloud();
    // OLD: bias init builds a geometry-only map (W>0, I=0)...
    if (!map.integratePoints(d.cloud, &d.ranges, nullptr)) return fail("buggy pass1 failed");
    // ...then the first real intensity update averages against the zero history.
    if (!map.integratePoints(d.cloud, &d.ranges, &d.inten)) return fail("buggy pass2 failed");
    const bievr::Voxel* voxel = map.getVoxel(map.hashIndex(d.cloud[0]));
    if (!voxel) return fail("buggy sequence: voxel not observed");
    const double mean = intensityMean(*voxel);
    if (mean > 90.0) {
      std::cerr << "expected biased (<90) intensity after geometry-only bootstrap, got " << mean
                << '\n';
      return fail("old geometry-only bootstrap did not bias the shared-weight intensity map");
    }
  }

  return 0;
}
