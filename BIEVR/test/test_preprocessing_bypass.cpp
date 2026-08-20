// Second review, Test 9: preprocessing bypass (intensity.enabled=true,
// preprocessing.enabled=false) must return an index-aligned, non-empty,
// finite intensity row equal to clamp(raw * raw_scale). It must never return
// an empty vector.

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
  cfg.enabled = false;  // debug/ablation bypass mode
  cfg.image_width = 64;
  cfg.image_height = 16;
  cfg.vertical_fov_deg = 50.0;
  cfg.raw_intensity_scale = 2.0;
  bievr::IntensityProcessor processor(cfg);

  const int N = 50;
  bievr::StampedIntensityPointcloud cloud;
  cloud.resize(N);
  for (int i = 0; i < N; ++i) {
    const double az = 2.0 * M_PI * static_cast<double>(i % 32 - 15) / 32.0;
    const double el = 0.02;
    const double r = 5.0;
    cloud[i](0) = r * std::cos(el) * std::cos(az);
    cloud[i](1) = r * std::cos(el) * std::sin(az);
    cloud[i](2) = r * std::sin(el);
    cloud[i](3) = i * 0.001;
    cloud[i](4) = 20.0 + 5.0 * i;  // some raw values exceed 255 after *2
  }

  const bievr::Pointcloud spatial = cloud;
  const bievr::StampedIntensityPointcloud& ccloud = cloud;
  const auto result = processor.process(spatial, ccloud.intensities());

  if (static_cast<size_t>(result.filtered.size()) != static_cast<size_t>(N)) {
    return fail("bypass must return size == N (not empty)");
  }
  if (result.num_valid != static_cast<size_t>(N)) return fail("bypass num_valid != N");
  for (int i = 0; i < N; ++i) {
    const double expected = std::max(0.0, std::min(255.0, (20.0 + 5.0 * i) * 2.0));
    const double v = result.filtered(0, i);
    if (!std::isfinite(v)) return fail("bypass produced non-finite value");
    if (std::abs(v - expected) > 1e-6) {
      std::cerr << "index " << i << ": got " << v << " expected " << expected << '\n';
      return fail("bypass value mismatch (index alignment broken)");
    }
  }

  return 0;
}
