// Second review, Test 7: projection collision must normalize ALL projected
// points. Two points at the same azimuth/elevation (same image pixel) but
// different range: the nearest one owns the pixel, but BOTH must receive the
// normalized I_F of that pixel (no raw-domain leak for the loser).

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

  // Same direction -> same pixel; raw values > 255 so a raw leak is detectable.
  const double az = 0.3, el = 0.02;
  bievr::StampedIntensityPointcloud cloud;
  cloud.resize(2);
  cloud[0](0) = 4.0 * std::cos(el) * std::cos(az);
  cloud[0](1) = 4.0 * std::cos(el) * std::sin(az);
  cloud[0](2) = 4.0 * std::sin(el);
  cloud[0](3) = 0.0;
  cloud[0](4) = 1000.0;  // nearer point, owns the pixel
  cloud[1](0) = 8.0 * std::cos(el) * std::cos(az);
  cloud[1](1) = 8.0 * std::cos(el) * std::sin(az);
  cloud[1](2) = 8.0 * std::sin(el);
  cloud[1](3) = 0.001;
  cloud[1](4) = 2000.0;  // loser

  const bievr::Pointcloud spatial = cloud;
  const bievr::StampedIntensityPointcloud& ccloud = cloud;
  const auto result = processor.process(spatial, ccloud.intensities());

  if (result.filtered.size() != 2) return fail("filtered size != 2");
  // Both must be in the normalized domain [0, 255] (no raw 1000/2000 leak).
  for (int i = 0; i < 2; ++i) {
    const double v = result.filtered(0, i);
    if (!(v >= 0.0 && v <= 255.0)) {
      std::cerr << "point " << i << " not normalized: " << v << '\n';
      return fail("collision loser must be normalized");
    }
  }
  // Sharing one pixel -> identical filtered value.
  if (std::abs(result.filtered(0, 0) - result.filtered(0, 1)) > 1e-4) {
    return fail("points sharing a pixel must receive the same filtered value");
  }
  if (result.num_collisions != 1) return fail("expected 1 collision");
  if (result.num_unique_pixels != 1) return fail("expected 1 unique pixel");
  if (result.num_valid != 2) return fail("expected 2 valid points");

  return 0;
}
