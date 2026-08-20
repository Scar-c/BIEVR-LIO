#ifndef BIEVR_LIO_PIPELINE_H_
#define BIEVR_LIO_PIPELINE_H_

#include <typeindex>

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/imu_integrator.h"
#include "bievr_lio/intensity_processor.h"
#include "bievr_lio/intensity_sampling.h"
#include "bievr_lio/log++.h"
#include "bievr_lio/ls_optimizer.h"
#include "bievr_lio/preprocess.h"
#include "bievr_lio/utils.h"

namespace bievr {

// COIN-BIEVR intensity configuration (Section 39-40 of the reproduction plan).
struct IntensityConfig {
  bool enabled = true;  // master switch; false = original BIEVR behaviour

  IntensityProcessorConfig preprocessing;
  IntensitySamplingConfig sampling;
  bool optimization_enabled = true;
  // Photometric scale lambda. The COIN-BIEVR supplement does not give a value.
  double photometric_scale = 0.02;
  // Round-5 photometric safety (engineering safeguards / TBD, not paper params):
  //   shadow_diagnostics: compute photo residual/J/H/b diagnostics without
  //     merging into the LM solve (C0 shadow run; never changes the trajectory).
  //   photometric_warmup_s: photo residual enters the LM only after Running has
  //     lasted this long.
  bool shadow_diagnostics = false;
  double photometric_warmup_s = 10.0;
};

class Pipeline {
 public:
  struct Config {
    Config() = default;
    PreprocessConfig preprocess;
    ImuConfig imu;
    RegistrationConfig registration;
    BIEVRMap::Config map;
    IntensityConfig intensity;  // COIN-BIEVR
    bool print_timing = false;
    bool publish_all_clouds = false;
    bool print_debug = false;      // when true, lower the log level to show DEBUG messages
    bool print_dashboard = false;  // when true, print the fancy live status dashboard
    // Path to the ASCII art shown at the top of the dashboard (e.g. bievr_ascii.txt).
    std::string dashboard_ascii_path = "";
    Transform T_I_L = Transform::Identity();  // Transform from LiDAR to IMU frame
    std::string map_frame = "map";
    std::string body_frame = "body";
    std::string log_path = "";
    // Optional per-frame intensity preprocessing diagnostics CSV (round-4
    // validation). Empty = disabled. When set, writes <path>/... see
    // writeIntensityDiagnostics(). Diagnostics only; not part of the paper.
    std::string intensity_diagnostics_path = "";
    // Optional per-frame photometric safety diagnostics CSV (round-5).
    // Empty = disabled. Diagnostics only.
    std::string photo_diagnostics_path = "";
    // Optional per-(lambda, frame) robust shadow scan CSV (round-7). Empty = disabled.
    std::string robust_shadow_scan_path = "";

    size_t min_points_for_map_init = 100;
    size_t map_size_running_threshold = 5;
    size_t informed_sample_count = 300;
    size_t min_map_size_for_imu_opt = 100;
    size_t min_imu_integrators_for_opt = 5;
    double gravity_prior_weight = 5.0;
  };

  explicit Pipeline(const Config& config);
  virtual ~Pipeline() = default;

  void processFrame(const std::vector<ImuMeasurement>& imu_data,
                    const StampedIntensityPointcloud& pointcloud);

  template <typename T>
  void registerPublisher(std::function<void(const T&, const Header&, const std::string& topic,
                                            const std::string& child_frame)>
                             func) {
    auto wrapper = [func](const void* val, const Header& header, const std::string& topic,
                          const std::string& child_frame) {
      func(*static_cast<const T*>(val), header, topic, child_frame);
    };
    publishers_[typeid(T)] = wrapper;
  }

 private:
  enum class Phase { NeedBias, NeedMap, Running };

  // Pipeline helpers
  bool initializeBias(const std::vector<ImuMeasurement>& imu_data, const Pointcloud& pointcloud);
  void tryInitMap(uint64_t stamp, const State& x_j_pred, const Transform& T_W_I_init,
                  const Pointcloud& undistorted, const IntensityView& intensities,
                  const Intensities& filtered_intensities, std::vector<double>& ranges,
                  const Header& header);
  void sampleSource(const Pointcloud& undistorted, const Transform& T_W_I_init,
                    Pointcloud& filtered, Pointcloud& coarse, Pointcloud& fine) const;
  void publishIntensityDebug(const Header& header, const Transform& T_W_I,
                             const IntensitySampleSet& samples, const IntensityPointcloud* map_cloud);

  // Writes one row per frame of the intensity preprocessing diagnostics when
  // config_.intensity_diagnostics_path is non-empty (and intensity is enabled).
  void writeIntensityDiagnostics(uint64_t stamp, const IntensityProcessingResult& result,
                                 const IntensitySampleSet& samples, double preprocess_ms);

  // Writes one row per frame of the round-5 photometric safety diagnostics when
  // config_.photo_diagnostics_path is non-empty and photo work is requested.
  void writePhotometricDiagnostics(uint64_t stamp, const PhotometricDiagnostics& diag);

  // Writes one row per (frame, lambda) of the round-7 robust shadow scan.
  void writeRobustShadowScan(uint64_t stamp, const std::vector<PhotoScaleEvaluation>& scan);

  // State and optimization management
  bool addState(const uint64_t time, const Quaternion& quat, const V3& p, const V3& v);
  bool addImuIntegrator(ImuIntegratorPtr imu_integrator);
  bool optimizeInertialWindow();

  // Publishing
  template <typename T>
  void publish(const T& value, const Header& header, const std::string& topic,
               const std::string& child_frame = "") const {
    auto it = publishers_.find(typeid(T));
    if (it != publishers_.end()) {
      it->second(static_cast<const void*>(&value), header, topic, child_frame);
    } else {
      LOG(E, "No publisher registered in pipeline for type.");
    }
  }

  void publishFrame(const Header& header, const Transform& T_W_I, const Pointcloud& full_registered,
                    const Pointcloud& source_filtered, const Pointcloud& source_coarse,
                    const Pointcloud& source_fine, const Pointcloud& undistorted,
                    const IntensityView& intensities);
  void publishLatestState(const Header& header);
  void publishDebugClouds(const Pointcloud& source_filtered, const Pointcloud& source_coarse,
                          const Pointcloud& source_fine, const Pointcloud& undistorted_cloud,
                          const IntensityView& intensities, const Transform& T_W_I,
                          const Header& header);

  // Logging
  void logTUM(double timestamp, const Transform& pose);

  Config config_;
  std::map<uint64_t, State> states_;
  std::map<uint64_t, ImuIntegratorPtr> imu_integrators_;

  size_t seq_counter_ = 0;
  Phase phase_ = Phase::NeedBias;
  std::unique_ptr<BiasInitializer> bias_initializer_;
  std::shared_ptr<BIEVRMap> map_;
  IntensityProcessor intensity_processor_;  // COIN-BIEVR per-frame normalization
  // Logged once: high vertical-clamp fraction suggests a wrong FOV/image size.
  bool intensity_clamp_warned_ = false;
  // Optional per-frame intensity diagnostics CSV logger (debug config only).
  std::shared_ptr<std::ofstream> intensity_diag_csv_;
  std::shared_ptr<std::ofstream> intensity_hist_csv_;
  // Optional per-frame photometric safety diagnostics CSV (round-5).
  std::shared_ptr<std::ofstream> photo_diag_csv_;
  // Optional per-(lambda, frame) robust shadow scan CSV (round-7).
  std::shared_ptr<std::ofstream> robust_scan_csv_;
  // Stamp when the pipeline entered Phase::Running (used by the photometric
  // warmup gate). 0 = not yet running.
  uint64_t running_start_time_ = 0;
  V3 acc_bias_ = V3::Zero();
  V3 gyro_bias_ = V3::Zero();
  V3 gravity_dir_ = V3(0, 0, 1);
  // Latest gyro reading, used to report the angular velocity in the odometry twist.
  V3 latest_gyro_ = V3::Zero();
  // Accelerometer scale resolved during bias estimation (1 if raw, g if the IMU
  // reports gravity-normalized accelerations). Applied to all incoming IMU data.
  double imu_acc_scale_ = 1.0;

  using PublishFunction =
      std::function<void(const void*, const Header&, const std::string&, const std::string&)>;
  std::unordered_map<std::type_index, PublishFunction> publishers_;
  std::shared_ptr<std::ofstream> tum_log_;

  // Accumulated state for the live status dashboard (printDashboard in utils).
  DashboardState dashboard_;
};
}  // namespace bievr

#endif  // BIEVR_LIO_PIPELINE_H_
