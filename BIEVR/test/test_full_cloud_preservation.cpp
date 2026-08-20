// Second review, Test 8: full-cloud preservation under collision.
// Even with image pixel collisions, filtered_intensity keeps size == N and the
// map integration still processes all N points (never "unique image pixels"),
// matching the COIN-BIEVR "full undistorted cloud without downsampling" update.

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/common.h"
#include "bievr_lio/intensity_processor.h"

#include <cmath>
#include <iostream>

namespace {

int fail(const char* msg) {
  std::cerr << msg << '\n';
  return 1;
}

}  // namespace

int main() {
  bievr::IntensityProcessorConfig cfg;
  cfg.image_width = 64;
  cfg.image_height = 16;
  cfg.vertical_fov_deg = 50.0;
  cfg.normalization_scale = 140.0;
  bievr::IntensityProcessor processor(cfg);

  // 6 unique directions (3 azimuth x 2 elevation) x 4 ranges = 24 points that
  // collide into 6 pixels, plus one far point in a second voxel (BIEVR only
  // processes voxel groups on a hash change).
  const double azs[3] = {0.1, 0.3, 0.5};
  const double els[2] = {-0.02, 0.02};
  const double range_values[4] = {4.0, 4.5, 5.0, 5.5};
  const int N = 24 + 1;
  bievr::StampedIntensityPointcloud cloud;
  cloud.resize(N);
  int k = 0;
  for (int ia = 0; ia < 3; ++ia) {
    for (int ie = 0; ie < 2; ++ie) {
      for (int ir = 0; ir < 4; ++ir) {
        const double r = range_values[ir];
        cloud[k](0) = r * std::cos(els[ie]) * std::cos(azs[ia]);
        cloud[k](1) = r * std::cos(els[ie]) * std::sin(azs[ia]);
        cloud[k](2) = r * std::sin(els[ie]);
        cloud[k](3) = k * 0.001;
        cloud[k](4) = 500.0 + k;  // raw values > 255
        ++k;
      }
    }
  }
  cloud[k] << 9.0, 0.5, 0.2;
  cloud[k](3) = k * 0.001;
  cloud[k](4) = 100.0;

  const bievr::Pointcloud spatial = cloud;
  const bievr::StampedIntensityPointcloud& ccloud = cloud;
  const auto result = processor.process(spatial, ccloud.intensities());

  if (static_cast<size_t>(result.filtered.size()) != static_cast<size_t>(N)) {
    return fail("filtered size != N under collision");
  }
  if (result.num_valid != static_cast<size_t>(N)) return fail("num_valid != N");
  if (result.num_collisions == 0) return fail("expected collisions in the test cloud");
  // Every filtered value is normalized (no raw leak).
  for (int i = 0; i < N; ++i) {
    const double v = result.filtered(0, i);
    if (!(v >= 0.0 && v <= 255.0)) return fail("raw value leaked into filtered");
  }

  // Map integration must process all N points (not unique pixels).
  bievr::BIEVRMap::Config mcfg;
  mcfg.max_size = 1000;
  mcfg.voxel_size = 2.0;
  mcfg.px_size = 0.05;
  mcfg.weighted = false;
  mcfg.smooth = false;
  mcfg.intensity_enabled = true;
  bievr::BIEVRMap map(mcfg);

  bievr::Pointcloud world = spatial;
  std::vector<double> ranges(N, 5.0);
  if (!map.integratePoints(world, &ranges, &result.filtered)) return fail("map integration failed");

  size_t total_points = 0;
  map.forEachVoxel([&total_points](size_t, const bievr::Voxel& v) { total_points += v.num_points_; });
  if (total_points != static_cast<size_t>(N)) {
    std::cerr << "map integrated " << total_points << " != " << N << '\n';
    return fail("map integration dropped points (full-cloud semantics violated)");
  }

  return 0;
}
