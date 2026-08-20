// Round 3, config smoke test:
// Both config/params.yaml (BIEVR baseline) and config/params_coin_bievr.yaml
// (COIN-BIEVR) must parse as COMPLETE algorithm configs through the same
// loader used by the ROS1 wrapper (loadConfigFromYaml). The COIN-BIEVR preset
// must not rely on hidden defaults for any algorithm section.

#include "bievr_lio/config_loader.h"

#include <iostream>
#include <string>
#include <vector>

namespace {

int fail(const char* msg) {
  std::cerr << msg << '\n';
  return 1;
}

bool near(double a, double b, double tol = 1e-9) { return (a - b) <= tol && (b - a) <= tol; }

}  // namespace

int main() {
#ifndef BIEVR_CONFIG_SOURCE_DIR
#error "BIEVR_CONFIG_SOURCE_DIR must be defined by the build (path to config/)"
#endif
  const std::string config_dir = BIEVR_CONFIG_SOURCE_DIR;
  // The ROS1 launch always passes a params file AND a sensor config file
  // (sensor config supplies topics/extrinsics/ranges), so the smoke test loads
  // them together exactly like the wrapper does.
  const std::string sensor_file = config_dir + "/sensor_configs/enwide.yaml";

  // --- BIEVR baseline: params.yaml + sensor config -------------------------
  {
    bievr::Config config;
    if (!bievr::loadConfigFromYaml({config_dir + "/params.yaml", sensor_file}, config)) {
      return fail("params.yaml + sensor config failed to parse");
    }
    const auto& hc = config.pipeline_config;
    if (hc.intensity.enabled) return fail("params.yaml must have intensity.enabled=false");
    if (!near(hc.map.voxel_size, 0.5)) return fail("params.yaml map.voxel_size not parsed");
    if (!near(hc.map.px_size, 0.05)) return fail("params.yaml map.pixel_size_m not parsed");
    if (!near(hc.preprocess.downsample_resolution, 0.1)) {
      return fail("params.yaml preprocess.downsample_resolution_m not parsed");
    }
    if (!near(hc.registration.huber_delta, 0.1)) return fail("params.yaml huber_delta not parsed");
    if (!near(hc.imu.window_length_s, 10.0)) return fail("params.yaml imu.window_s not parsed");
    if (!hc.preprocess.informed_sampling) return fail("params.yaml informed_sampling not parsed");
    if (!hc.map.smooth || !hc.map.weighted) return fail("params.yaml map smooth/weighted not parsed");
  }

  // --- COIN-BIEVR: params_coin_bievr.yaml + sensor config ------------------
  {
    bievr::Config config;
    if (!bievr::loadConfigFromYaml({config_dir + "/params_coin_bievr.yaml", sensor_file}, config)) {
      return fail("params_coin_bievr.yaml + sensor config failed to parse");
    }
    const auto& hc = config.pipeline_config;
    if (!hc.intensity.enabled) return fail("params_coin_bievr.yaml must have intensity.enabled=true");
    // The preset must be complete (no reliance on hidden defaults for the
    // algorithm sections that matter for COIN-BIEVR).
    if (!near(hc.map.voxel_size, 0.5)) return fail("coin preset map.voxel_size not parsed");
    if (!near(hc.map.px_size, 0.05)) return fail("coin preset map.pixel_size_m not parsed");
    if (!near(hc.preprocess.downsample_resolution, 0.1)) {
      return fail("coin preset preprocess.downsample_resolution_m not parsed");
    }
    if (!near(hc.registration.huber_delta, 0.1)) return fail("coin preset huber_delta not parsed");
    if (!near(hc.imu.window_length_s, 10.0)) return fail("coin preset imu.window_s not parsed");
    if (hc.intensity.sampling.num_voxels != 100) {
      return fail("coin preset intensity.sampling.num_voxels not parsed");
    }
    if (hc.intensity.sampling.score_mode != "abs_components") {
      return fail("coin preset intensity.sampling.score_mode not parsed");
    }
    if (!near(hc.intensity.photometric_scale, 0.02)) {
      return fail("coin preset intensity.optimization.photometric_scale not parsed");
    }
    if (hc.intensity.preprocessing.image_width != 1024) {
      return fail("coin preset intensity.preprocessing.image_width not parsed");
    }
  }

  return 0;
}
