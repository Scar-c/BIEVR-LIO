// COIN-BIEVR reproduction plan, Section 44:
// Per-point intensity must stay index-aligned with the point cloud through the
// intensity processor (transform / undistortion preserve order by construction,
// so the processor is the piece that must not scramble indices).

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

  const int N = 192;  // 64 azimuths x 3 elevations = 192 distinct pixels
  bievr::StampedIntensityPointcloud cloud;
  cloud.resize(N);
  for (int i = 0; i < N; ++i) {
    // Azimuth spread so the spherical projection maps every point to a distinct
    // in-bounds pixel: u = round(63 - (i%64)) in 0..63.
    const double az = 2.0 * M_PI * static_cast<double>(i % 64 - 31) / 64.0;
    // Elevation decoupled from the azimuth (i/64) so (u,v) pairs are unique:
    // v in {7, 8, 9} for i/64 in {0, 1, 2}.
    const double el = 0.05 - 0.05 * static_cast<double>(i / 64);
    const double r = 5.0;
    cloud[i](0) = r * std::cos(el) * std::cos(az);
    cloud[i](1) = r * std::cos(el) * std::sin(az);
    cloud[i](2) = r * std::sin(el);
    cloud[i](3) = i * 0.001;            // time
    cloud[i](4) = 1000.0 + i;           // intensity
  }

  const bievr::Pointcloud spatial = cloud;  // LiDAR frame
  const bievr::StampedIntensityPointcloud& ccloud = cloud;
  const bievr::Intensities filtered = processor.process(spatial, ccloud.intensities()).filtered;

  if (static_cast<size_t>(filtered.size()) != static_cast<size_t>(N)) {
    return fail("filtered intensity size != point cloud size");
  }
  // Every point owns a pixel, so every filtered value must be a normalized
  // intensity in [0, 255] (a scrambled index would leak the raw 1000+i value).
  for (int i = 0; i < N; ++i) {
    const double v = filtered(0, i);
    if (!(v >= 0.0 && v <= 255.0)) {
      std::cerr << "index " << i << " leaked raw intensity " << v << '\n';
      return fail("index misalignment in intensity processor");
    }
  }

  // Determinism: processing the same cloud twice yields identical output.
  const bievr::Intensities filtered2 = processor.process(spatial, ccloud.intensities()).filtered;
  if (!filtered.isApprox(filtered2, 1e-12)) return fail("intensity processor not deterministic");

  return 0;
}
