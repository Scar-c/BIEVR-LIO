// Round-11 regression: sampleIntensityPoints() output must be identical under
// serial (1 TBB worker) and default parallel execution, after the
// std::vector<bool> -> std::vector<uint8_t> observed-flag fix (the packed-bool
// proxy caused cross-word read-modify-write races between workers writing
// logically disjoint indices).
//
// Fixture: a map with several occupied voxels (distinct intensity patterns) and
// an undistorted source cloud containing both occupied (observed) and empty
// (unobserved) voxels, with mixed observed/unobserved indices sharing storage
// words. The fixture avoids score/distance ties so the sampling is fully
// deterministic; serial == parallel is then a pure race-regression guard.

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/common.h"
#include "bievr_lio/intensity_sampling.h"

#include <tbb/global_control.h>

#include <cmath>
#include <iostream>
#include <vector>

namespace {

int fail(const char* msg) {
  std::cerr << "FAIL: " << msg << '\n';
  return 1;
}

}  // namespace

int main() {
  bievr::BIEVRMap::Config mcfg;
  mcfg.max_size = 100;
  mcfg.voxel_size = 2.0;
  mcfg.px_size = 0.05;
  mcfg.weighted = false;  // every pixel W = 1.0
  mcfg.smooth = false;
  mcfg.intensity_enabled = true;
  bievr::BIEVRMap map(mcfg);

  // Map: 5x5 grid (0.1..0.5 m, 0.1 m spacing -> one point per downsample voxel)
  // in voxel (0,0) + three 2x2 planes (0.1 m spacing) creating three more
  // OBSERVED voxels (>= kNMinValid=4 points; distinct intensity patterns).
  const int N = 25 + 12;
  bievr::Pointcloud cloud;
  cloud.resize(N);
  bievr::Intensities inten(1, N);
  std::vector<double> ranges(N, 4.0);
  int k = 0;
  for (int iy = 0; iy < 5; ++iy) {
    for (int ix = 0; ix < 5; ++ix, ++k) {
      cloud[k] << 0.1 + 0.1 * ix, 0.1 + 0.1 * iy, 0.0;
      inten(0, k) = static_cast<float>(10.0 * ix + 5.0 * iy);
    }
  }
  const double far_xy[3][2] = {{4.1, 0.2}, {6.1, 0.2}, {8.1, 0.2}};
  for (int f = 0; f < 3; ++f) {
    for (int iy = 0; iy < 2; ++iy) {
      for (int ix = 0; ix < 2; ++ix, ++k) {
        cloud[k] << far_xy[f][0] + 0.1 * ix, far_xy[f][1] + 0.1 * iy, 0.0;
        inten(0, k) = static_cast<float>(40.0 * (f + 1) + 10.0 * ix + 5.0 * iy);
      }
    }
  }
  if (!map.integratePoints(cloud, &ranges, &inten)) return fail("map build failed");

  // Undistorted source: the map cloud plus points in EMPTY voxels (unobserved),
  // interleaved so observed/unobserved indices share storage words.
  const int M = N + 3;
  bievr::Pointcloud src;
  src.resize(M);
  bievr::Intensities src_i(1, M);
  for (int i = 0; i < N; ++i) {
    src[i] = cloud[i];
    src_i(0, i) = inten(0, i);
  }
  src[N] << 2.1, 0.2, 0.0;    // empty voxel (1,0) -> unobserved
  src[N + 1] << 2.1, 2.2, 0.0;  // empty voxel (1,1) -> unobserved
  src[N + 2] << 4.1, 2.2, 0.0;  // empty voxel (2,1) -> unobserved
  src_i(0, N) = 80.0f;
  src_i(0, N + 1) = 90.0f;
  src_i(0, N + 2) = 100.0f;

  bievr::IntensitySamplingConfig cfg;
  cfg.enabled = true;
  cfg.num_voxels = 10;  // > observed voxel count (4): all candidates selected
  cfg.downsample_resolution_m = 0.1;
  cfg.weak_eigen_ratio = 10.0;
  cfg.normalize_eta = true;

  // Serial execution (1 TBB worker).
  bievr::IntensitySampleSet ser;
  {
    tbb::global_control gc(tbb::global_control::max_allowed_parallelism, 1);
    ser = bievr::sampleIntensityPoints(map, src, src_i, bievr::Transform::Identity(), cfg);
  }
  // Default parallel execution.
  const bievr::IntensitySampleSet par =
      bievr::sampleIntensityPoints(map, src, src_i, bievr::Transform::Identity(), cfg);

  std::cout << "serial:   observed=" << ser.observed_voxels
            << " selected=" << ser.selected_voxels << " points=" << ser.points.size() << '\n';
  std::cout << "parallel: observed=" << par.observed_voxels
            << " selected=" << par.selected_voxels << " points=" << par.points.size() << '\n';

  if (ser.observed_voxels != par.observed_voxels) return fail("observed_voxels differ");
  if (ser.selected_voxels != par.selected_voxels) return fail("selected_voxels differ");
  if (ser.points.size() != par.points.size()) return fail("selected point count differs");
  if (ser.selected_voxel_hashes.size() != par.selected_voxel_hashes.size()) {
    return fail("selected voxel hash count differs");
  }
  if (ser.observed_voxels == 0 || ser.points.empty()) return fail("fixture produced no sampling");
  for (size_t i = 0; i < ser.points.size(); ++i) {
    if ((ser.points[i] - par.points[i]).norm() > 1e-12) return fail("selected point differs");
    if (std::abs(ser.intensities(0, i) - par.intensities(0, i)) > 1e-9) {
      return fail("selected intensity differs");
    }
  }
  for (size_t i = 0; i < ser.selected_voxel_hashes.size(); ++i) {
    if (ser.selected_voxel_hashes[i] != par.selected_voxel_hashes[i]) {
      return fail("selected voxel hash differs");
    }
  }
  return 0;
}
