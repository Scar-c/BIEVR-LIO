#include "bievr_lio/pipeline.h"

#include <fstream>
#include <sstream>

#include "bievr_lio/inertial_factor.h"
#include "bievr_lio/prior_factor.h"
#include "bievr_lio/timing.h"
#include "bievr_lio/undistort.h"
#include "bievr_lio/utils.h"

namespace bievr {

namespace {

// Builds a non-owning view onto an owning Intensities row so it can be passed
// where an IntensityView is expected (e.g. IntensityPointcloud construction).
inline IntensityView toIntensityView(const Intensities& intensities) {
  return IntensityView(intensities.data(), intensities.cols(),
                       Eigen::InnerStride<>(intensities.outerStride()));
}

// Reconstructs the voxel-wise intensity map as a 3D point cloud (COIN-BIEVR
// Fig. 1 / Fig. 4 style): each valid pixel is lifted to (u*px, v*px, height)
// in the voxel-local frame and transformed to world, carrying the intensity.
// Capped at `max_voxels` so debug publishing never stalls the pipeline.
IntensityPointcloud buildIntensityMapCloud(const BIEVRMap& map, size_t max_voxels) {
  Pointcloud points;
  Intensities intensities;
  size_t voxels_used = 0;
  map.forEachVoxel([&](size_t /*hash*/, const Voxel& voxel) {
    if (voxels_used >= max_voxels) return;
    const int rows = voxel.height.bump_img_.rows();
    const int cols = voxel.height.bump_img_.cols();
    if (rows == 0 || cols == 0) return;
    ++voxels_used;

    size_t start = points.size();
    size_t n_valid = 0;
    for (int y = 0; y < rows; ++y) {
      for (int x = 0; x < cols; ++x) {
        if (voxel.bump_weights_(y, x) <= 0.f) continue;
        ++n_valid;
      }
    }
    points.resize(start + n_valid);
    intensities.resize(1, start + n_valid);

    const double px = map.pixel_size;
    size_t k = start;
    for (int y = 0; y < rows; ++y) {
      for (int x = 0; x < cols; ++x) {
        if (voxel.bump_weights_(y, x) <= 0.f) continue;
        points[k] = voxel.T_C_W_.inverse() *
                    Point(x * px, y * px, voxel.height.bump_img_(y, x));
        intensities(0, k) = voxel.intensity.intensity_img_(y, x);
        ++k;
      }
    }
  });
  return IntensityPointcloud(points, toIntensityView(intensities));
}

}  // namespace

Pipeline::Pipeline(const Config& config) : config_(config) {
  // The map's photometric channel is enabled only when the master switch is on,
  // so that intensity.enabled=false runs the original BIEVR map path.
  BIEVRMap::Config map_config = config_.map;
  map_config.intensity_enabled = config_.intensity.enabled;
  map_ = std::make_shared<BIEVRMap>(map_config);
  intensity_processor_.configure(config_.intensity.preprocessing);

  if (!config_.log_path.empty()) {
    LOG(I, "Logging to " << config_.log_path);
    tum_log_ = std::make_shared<std::ofstream>(config_.log_path, std::ios::trunc);
    if (!tum_log_->is_open()) {
      LOG(E, "Error opening output file.");
    }
  }

  if (config_.print_dashboard && !config_.dashboard_ascii_path.empty()) {
    std::ifstream ascii_file(config_.dashboard_ascii_path);
    if (ascii_file.is_open()) {
      // Indent every line so the art sits a few characters off the left margin.
      const std::string indent(6, ' ');
      std::string line;
      std::ostringstream art;
      while (std::getline(ascii_file, line)) {
        art << indent << line << "\n";
      }
      dashboard_.ascii = art.str();
    } else {
      LOG(W, "Could not open dashboard ASCII art at " << config_.dashboard_ascii_path);
    }
  }

  if (config_.print_dashboard) {
    printDashboardBanner(dashboard_.ascii, "BIEVR-LIO  WAITING FOR DATA");
  }

  // Optional per-frame intensity diagnostics CSV (round-4 validation; gated by
  // a debug config path, never active by default).
  if (!config_.intensity_diagnostics_path.empty()) {
    std::string hist_path = config_.intensity_diagnostics_path;
    if (hist_path.size() > 4 && hist_path.compare(hist_path.size() - 4, 4, ".csv") == 0) {
      hist_path = hist_path.substr(0, hist_path.size() - 4) + "_histogram.csv";
    } else {
      hist_path += "_histogram.csv";
    }
    intensity_diag_csv_ =
        std::make_shared<std::ofstream>(config_.intensity_diagnostics_path, std::ios::trunc);
    intensity_hist_csv_ = std::make_shared<std::ofstream>(hist_path, std::ios::trunc);
    if (intensity_diag_csv_->is_open() && intensity_hist_csv_->is_open()) {
      *intensity_diag_csv_
          << "stamp,input_points,valid_points,unique_pixels,collisions,invalid,"
             "horizontal_boundary_adjusted,vertical_top_clamped,vertical_bottom_clamped,"
             "vertical_clamped_ratio,filtered_min,filtered_max,filtered_mean,filtered_std,"
             "filtered_p01,filtered_p05,filtered_p50,filtered_p95,filtered_p99,"
             "filtered_sat0,filtered_sat255,raw_min,raw_max,raw_p50,raw_p95,raw_p99,"
             "elev_min_deg,elev_max_deg,elev_p01_deg,elev_p99_deg,"
             "intensity_preprocess_ms,intensity_map_voxels,observed_voxels,"
             "selected_intensity_voxels,selected_intensity_points\n";
    } else {
      LOG(E, "Failed to open intensity diagnostics CSV at "
                 << config_.intensity_diagnostics_path);
      intensity_diag_csv_.reset();
      intensity_hist_csv_.reset();
    }
  }

  // Optional per-frame photometric safety diagnostics CSV (round-5).
  if (!config_.photo_diagnostics_path.empty()) {
    photo_diag_csv_ =
        std::make_shared<std::ofstream>(config_.photo_diagnostics_path, std::ios::trunc);
    if (photo_diag_csv_->is_open()) {
      *photo_diag_csv_
          << "stamp,candidates,valid_matches,immature_matches,match_ratio,"
             "r_abs_p50,r_abs_p75,r_abs_p90,r_abs_p95,r_abs_max,r_signed_median,"
             "grad_p50,grad_p75,grad_p90,grad_p95,"
             "J_norm_p50,J_norm_p90,J_norm_p99,J_norm_max,"
             "Jdof_p90_rx,Jdof_p90_ry,Jdof_p90_rz,Jdof_p90_tx,Jdof_p90_ty,Jdof_p90_tz,"
             "H_geo_fro,H_photo_fro,H_photo_trace,b_geo_norm,b_photo_norm,"
             "R_H_unscaled,R_b_unscaled,R_H_scaled,lambda_ref,"
             "eig1,eig2,eig3,eig4,eig5,eig6,"
             "photo_step_t_mm,photo_step_r_deg,"
             "lm_iterations,lm_accepted,lm_rejected,reject_rate,"
             "lm_initial_cost,lm_final_cost,geo_cost,photo_cost,"
             "pose_delta_t_m,pose_delta_r_deg,photo_effective_residuals,"
             "fd_samples,fd_median_rel_err,fd_p95_rel_err,fd_done,photometric_scale,"
             "fd_level_a_samples,fd_level_a_median,fd_level_a_p95,fd_level_a_sign,"
             "fd_level_b_points,fd_level_b_scalars,fd_level_b_median,fd_level_b_p95,fd_level_b_sign,"
             "fd_b_rx_med,fd_b_rx_p95,fd_b_ry_med,fd_b_ry_p95,fd_b_rz_med,fd_b_rz_p95,"
             "fd_b_tx_med,fd_b_tx_p95,fd_b_ty_med,fd_b_ty_p95,fd_b_tz_med,fd_b_tz_p95,"
             "fd_b_analytic_p50,fd_b_analytic_p90,fd_b_analytic_p99,"
             "fd_b_numeric_p50,fd_b_numeric_p90,fd_b_numeric_p99,"
             "fd_c_voxel_switch,fd_c_cell_switch,fd_c_validity_switch,"
             "fd_c_noswitch_samples,fd_c_noswitch_median,fd_c_noswitch_p95,"
             "photo_near_fraction,photo_near_r_p50,photo_near_r_p90,photo_far_r_p50,photo_far_r_p90,"
             "geo_lambda1,geo_lambda2,geo_lambda3,geo_two_weak_flag,"
             "eta_world_x,eta_world_y,eta_world_z,eta_abs_dot_prev,eta_angle_prev_deg,"
             "weak_subspace_diff_F,selected_voxel_jaccard,selected_voxel_retention,"
             "selected_score_p10,selected_score_p50,selected_score_p90,selected_score_max,"
             "candidate_score_p50,candidate_score_p90,candidate_score_p99,"
             "q_geo_v1,q_geo_v2,q_photo_v1,q_photo_v2,"
             "s_geo_v1,s_geo_v2,s_photo_v1,s_photo_v2,"
             "parity_count_serial,parity_count_parallel,parity_cost_rel,parity_H_rel,parity_b_rel,"
             "source_near_fraction,photo_near_r_p95,photo_far_r_p95,"
             "photo_near_inlier_fraction,photo_far_inlier_fraction,"
             "photo_near_grad_p50,photo_near_grad_p90,photo_far_grad_p50,photo_far_grad_p90,"
             "photo_near_J_p50,photo_near_J_p90,photo_far_J_p50,photo_far_J_p90,"
             "photo_near_H_fraction\n";
    } else {
      LOG(E, "Failed to open photometric diagnostics CSV at "
                 << config_.photo_diagnostics_path);
      photo_diag_csv_.reset();
    }
  }

  // Optional per-(lambda, frame) robust shadow scan CSV (round-7).
  if (!config_.robust_shadow_scan_path.empty()) {
    robust_scan_csv_ =
        std::make_shared<std::ofstream>(config_.robust_shadow_scan_path, std::ios::trunc);
    if (robust_scan_csv_->is_open()) {
      *robust_scan_csv_
          << "stamp,lambda,raw_huber_knee,huber_inlier_fraction,H_photo_fro,H_geo_fro,R_H,"
             "b_photo_norm,b_geo_norm,photo_cost,geo_cost,pred_t_m,pred_r_deg\n";
    } else {
      LOG(E, "Failed to open robust shadow scan CSV at "
                 << config_.robust_shadow_scan_path);
      robust_scan_csv_.reset();
    }
  }
}

void Pipeline::processFrame(const std::vector<ImuMeasurement>& imu_data,
                            const StampedIntensityPointcloud& points_L) {
  timing::Timer step_timer("step");

  if (imu_data.empty()) {
    LOG(W, "No IMU data to process.");
    return;
  }

  // Cache the latest gyro reading so the odometry twist can report the current
  // angular velocity (already expressed in the body/IMU frame).
  latest_gyro_ = imu_data.back().gyro;

  // Remove points outside of intended range
  timing::Timer filter_timer("01_filter");
  StampedIntensityPointcloud points_filtered_L;
  filterMinMaxRange(points_L, points_filtered_L, config_.preprocess.min_range,
                    config_.preprocess.max_range);
  // The per-point time and intensity stay in points_filtered_L; we only take
  // zero-copy views onto those rows. Undistortion consumes the time view, intensity
  // rides through to publishing. Both views remain valid for the whole frame and
  // stay aligned by index with the spatial clouds derived below, since every
  // transform downstream preserves point order.
  const StampedIntensityPointcloud& filtered_L = points_filtered_L;
  const TimeView times = filtered_L.times();
  const IntensityView intensities = filtered_L.intensities();
  // Transform only the spatial coordinates into the IMU frame (single 3xN write);
  // time and intensity are never rewritten.
  const Pointcloud points_filtered_I = transformPoints(config_.T_I_L, points_filtered_L);
  filter_timer.Stop();

  // COIN-BIEVR: per-frame intensity normalization in the LiDAR frame. The
  // output is index-aligned with points_filtered_L (hence also with the
  // transformed / undistorted spatial clouds downstream). When the master
  // switch is off this block is not executed at all; when the master switch is
  // on but preprocessing is disabled, the processor runs a debug/ablation
  // bypass that still returns an index-aligned (non-empty) intensity row.
  const bool intensity_enabled = config_.intensity.enabled;
  Intensities filtered_intensity;
  IntensityProcessingResult intensity_result;
  double intensity_preprocess_ms = 0.0;
  if (intensity_enabled) {
    timing::Timer inten_timer("02a_intensity_preprocess");
    const Pointcloud points_lidar = filtered_L;
    // Ouster ring/beam channel (index-aligned with filtered_L), used by the
    // ring-based Ouster intensity image construction.
    const IntensityView rings = filtered_L.rings();
    const auto t0 = std::chrono::steady_clock::now();
    intensity_result = intensity_processor_.process(points_lidar, intensities, &rings);
    const auto t1 = std::chrono::steady_clock::now();
    filtered_intensity = std::move(intensity_result.filtered);
    intensity_preprocess_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    inten_timer.Stop();

    // Low-frequency diagnostic (log once): a significant vertical-clamp fraction
    // almost always means vertical_fov_deg / image size is misconfigured for the
    // sensor. Diagnostic only; it does not change the estimator.
    if (intensity_result.input_points > 0 && !intensity_clamp_warned_) {
      const double clamped_ratio =
          static_cast<double>(intensity_result.vertical_top_clamped +
                              intensity_result.vertical_bottom_clamped) /
          static_cast<double>(intensity_result.input_points);
      if (clamped_ratio > 0.01) {
        LOG(W, "More than 1% of LiDAR points are being clamped to the intensity-image "
               "vertical boundary (ratio " << clamped_ratio << "). Check vertical_fov_deg "
               "and sensor-specific projection settings.");
        intensity_clamp_warned_ = true;
      }
    }
  }

  if (phase_ == Phase::NeedBias) {
    // Estimate initial biases and orientation based on zero velocity assumption.
    // This also resolves imu_acc_scale_ (normalization detection).
    if (!initializeBias(imu_data, points_filtered_I)) return;
  }

  // Apply the accelerometer scale resolved during bias estimation if necessary.
  std::vector<ImuMeasurement> imu_scaled;
  const std::vector<ImuMeasurement>* imu_ptr = &imu_data;
  if (imu_acc_scale_ != 1.0) {
    imu_scaled = imu_data;
    for (auto& imu : imu_scaled) imu.acc *= imu_acc_scale_;
    imu_ptr = &imu_scaled;
  }
  const std::vector<ImuMeasurement>& imu = *imu_ptr;

  const State& x_i = states_.rbegin()->second;
  const Header header{points_filtered_L.end_stamp, static_cast<uint32_t>(x_i.id + 1),
                      config_.map_frame};

  // Propagate state based on IMU data
  timing::Timer preint_timer("02_preint");
  ImuIntegratorPtr imu_integrator =
      std::make_shared<ImuIntegrator>(config_.imu, acc_bias_, gyro_bias_);
  imu_integrator->integrate(imu);
  V3 G = gravity_dir_ * kGMagnitude;
  State x_j_pred;
  imu_integrator->predict(x_i.quat, x_i.p, x_i.v, G, x_j_pred.quat, x_j_pred.p, x_j_pred.v);
  preint_timer.Stop();

  if (points_filtered_I.empty()) {
    LOG(W, "No pointcloud data to process.");
    addState(imu.back().stamp, x_j_pred.quat, x_j_pred.p, x_j_pred.v);
    publishLatestState(header);
    return;
  }

  // Undistort pointcloud based on IMU integration
  timing::Timer undistortion_timer("03_undistortion");
  Pointcloud points_undistorted_I;
  undistortCloud(imu_integrator, x_i, points_filtered_I, times, points_filtered_L.stamp, G,
                 points_undistorted_I);
  undistortion_timer.Stop();

  std::vector<double> ranges;
  calculateRanges(points_undistorted_I, ranges);
  const Transform T_W_I_init(x_j_pred.quat, x_j_pred.p);
  if (phase_ == Phase::NeedMap) {
    tryInitMap(imu_data.back().stamp, x_j_pred, T_W_I_init, points_undistorted_I, intensities,
               filtered_intensity, ranges, header);
    return;
  }

  // COIN-LIO mapping/point_filter_num: sparse-skip the undistorted cloud for the
  // GEOMETRY registration + map-update path (keep every (N+1)-th point; N=4 in
  // the COIN-LIO ENWIDE config). The intensity preprocessing and sampling keep
  // the FULL cloud (COIN-LIO builds its intensity image from the full
  // undistorted cloud). Subsets stay index-aligned (cloud/ranges/intensity).
  Pointcloud reg_cloud;
  std::vector<double> reg_ranges;
  Intensities reg_intensity;
  Pointcloud* reg_cloud_p = &points_undistorted_I;
  std::vector<double>* reg_ranges_p = &ranges;
  Intensities* reg_intensity_p = &filtered_intensity;
  const int pf_num = config_.preprocess.point_filter_num;
  if (pf_num > 1) {
    const size_t step = static_cast<size_t>(pf_num);
    const size_t n_kept = (points_undistorted_I.size() + step - 1) / step;
    reg_cloud.resize(n_kept);
    reg_ranges.resize(n_kept);
    if (intensity_enabled && filtered_intensity.cols() > 0) {
      reg_intensity = Intensities(1, n_kept);
    }
    size_t k = 0;
    for (size_t i = 0; i < points_undistorted_I.size(); i += step, ++k) {
      reg_cloud[k] = points_undistorted_I[i];
      reg_ranges[k] = ranges[i];
      if (intensity_enabled && filtered_intensity.cols() > 0) {
        reg_intensity(0, k) = filtered_intensity(0, i);
      }
    }
    reg_cloud_p = &reg_cloud;
    reg_ranges_p = &reg_ranges;
    if (intensity_enabled && filtered_intensity.cols() > 0) reg_intensity_p = &reg_intensity;
  }

  // Select points from the source cloud that will be used for registration
  timing::Timer voxel_timer("04_sampling");
  Pointcloud source_filtered, source_coarse, source_fine;
  sampleSource(*reg_cloud_p, T_W_I_init, source_filtered, source_coarse, source_fine);
  voxel_timer.Stop();

  // COIN-BIEVR: intensity point sampling (top-N intensity voxels along the
  // geometry-weak direction, downsampled to 0.1 m).
  IntensitySampleSet intensity_samples;
  if (intensity_enabled && config_.intensity.sampling.enabled) {
    timing::Timer isamp_timer("04b_intensity_sampling");
    intensity_samples =
        sampleIntensityPoints(*map_, points_undistorted_I, filtered_intensity, T_W_I_init,
                              config_.intensity.sampling);
    isamp_timer.Stop();
  }

  // Perform the actual registration (joint geometry + photometric)
  timing::Timer align_timer("05_registration");
  RegistrationConfig reg_config = config_.registration;
  reg_config.photometric_residual =
      intensity_enabled && config_.intensity.optimization_enabled &&
      !intensity_samples.points.empty();
  reg_config.photometric_scale = config_.intensity.photometric_scale;
  // Round-5 photometric safety (shadow diagnostics, warmup gate, maturity gate,
  // optional real-map finite-difference Jacobian spot check).
  reg_config.shadow_photometric =
      intensity_enabled && config_.intensity.shadow_diagnostics &&
      !intensity_samples.points.empty();
  reg_config.photometric_warmup_s = config_.intensity.photometric_warmup_s;
  reg_config.photometric_elapsed_s =
      running_start_time_ > 0 ? nsToS(header.stamp - running_start_time_) : 0.0;
  reg_config.photometric_fd_check =
      intensity_enabled && config_.intensity.shadow_diagnostics &&
      config_.intensity.optimization_enabled == false;
  // Round-7: direct robustified photometric contribution across the fixed
  // lambda grid (shadow only; never merges into the solve).
  reg_config.shadow_lambda_scan =
      intensity_enabled && config_.intensity.shadow_diagnostics &&
      config_.intensity.optimization_enabled == false;
  // With the master switch off we use the geometry-only constructor so the
  // photometric source is never constructed / accumulated.
  LsqRegistration optimizer =
      intensity_enabled
          ? LsqRegistration(*map_, source_filtered, intensity_samples.points,
                            intensity_samples.intensities, reg_config)
          : LsqRegistration(*map_, source_filtered, reg_config);
  // Round-9: feed the Eq.7 weak eigenvectors (world) into the optimizer so the
  // final-pose directional Hessian/gradient audit is computed in the exact
  // perturbation frame. Diagnostic only.
  if (intensity_enabled) {
    optimizer.setWeakDirections(intensity_samples.weak_v1, intensity_samples.weak_v2);
  }
  const Transform T_W_I = optimizer.computeTransformation(T_W_I_init);
  const int n_effective_points = optimizer.numEffectivePoints();
  align_timer.Stop();

  // Transform the full cloud using the estimated pose and add it to the map
  timing::Timer map_timer("06_map");
  const Pointcloud points_registered = T_W_I * *reg_cloud_p;
  map_->integratePoints(points_registered, reg_ranges_p,
                        (intensity_enabled && reg_intensity_p->cols() > 0) ? reg_intensity_p
                                                                         : nullptr);
  map_timer.Stop();

  // Bookkeeping and optimization of the intertial part of the state
  addState(imu.back().stamp, T_W_I.quaternion(), T_W_I.translation(), x_j_pred.v);

  timing::Timer imu_opt("07_imu_opt");
  if (map_->size() > config_.min_map_size_for_imu_opt) {
    addImuIntegrator(imu_integrator);
  }
  optimizeInertialWindow();
  imu_opt.Stop();

  // Publish results
  timing::Timer pub_time("08_publish");
  publishFrame(header, T_W_I, points_registered, source_filtered, source_coarse, source_fine,
               points_undistorted_I, intensities);
  if (config_.publish_all_clouds) {
    // Debug topics: selected intensity points, weak direction, selected voxels
    // and the intensity-textured map (capped so debug publishing never stalls).
    IntensityPointcloud map_cloud;
    if (intensity_enabled) {
      map_cloud = buildIntensityMapCloud(*map_, 2000);
    }
    publishIntensityDebug(header, T_W_I, intensity_samples,
                          intensity_enabled ? &map_cloud : nullptr);
  }
  pub_time.Stop();

  step_timer.Stop();

  if (!config_.log_path.empty()) {
    logTUM(nsToS(points_L.end_stamp), T_W_I);
  }

  if (config_.print_timing) {
    LOG(I, "Timings:\n" << timing::Timing::Print());
  }

  if (config_.print_dashboard) {
    FrameStats stats;
    stats.geometry_points = static_cast<int>(source_filtered.size());
    stats.intensity_points = static_cast<int>(intensity_samples.points.size());
    stats.observed_voxels = static_cast<int>(intensity_samples.observed_voxels);
    stats.intensity_voxels = static_cast<int>(intensity_samples.selected_voxels);
    stats.geo_residuals = optimizer.numGeometryEffectivePoints();
    stats.photo_residuals = optimizer.numPhotometricEffectivePoints();
    stats.geo_rmse = optimizer.geometryRMSE();
    stats.photo_rmse = optimizer.photometricRMSE();
    stats.weak_lambda1 = intensity_samples.weak_eigenvalues.x();
    stats.weak_lambda2 = intensity_samples.weak_eigenvalues.y();
    stats.weak_lambda3 = intensity_samples.weak_eigenvalues.z();
    stats.intensity_preprocess_ms = timing::Timing::GetMeanSeconds("02a_intensity_preprocess") * 1e3;
    stats.intensity_sampling_ms = timing::Timing::GetMeanSeconds("04b_intensity_sampling") * 1e3;
    stats.intensity_map_voxels = 0;
    map_->forEachVoxel([&stats](size_t, const Voxel& v) {
      if (v.intensity.intensity_img_.rows() > 0) ++stats.intensity_map_voxels;
    });
    // Low-overhead intensity preprocessing diagnostics (empty when disabled).
    stats.intensity_input_points = static_cast<int>(intensity_result.input_points);
    stats.intensity_valid_points = static_cast<int>(intensity_result.num_valid);
    stats.intensity_unique_pixels = static_cast<int>(intensity_result.num_unique_pixels);
    stats.intensity_collisions = static_cast<int>(intensity_result.num_collisions);
    stats.intensity_invalid = static_cast<int>(intensity_result.num_invalid);
    stats.intensity_h_boundary =
        static_cast<int>(intensity_result.horizontal_boundary_adjusted);
    stats.intensity_v_top = static_cast<int>(intensity_result.vertical_top_clamped);
    stats.intensity_v_bottom = static_cast<int>(intensity_result.vertical_bottom_clamped);
    stats.intensity_v_clamped_ratio =
        intensity_result.input_points > 0
            ? static_cast<double>(intensity_result.vertical_top_clamped +
                                  intensity_result.vertical_bottom_clamped) /
                  static_cast<double>(intensity_result.input_points)
            : 0.0;
    stats.intensity_filtered_min = intensity_result.filtered_min;
    stats.intensity_filtered_max = intensity_result.filtered_max;
    stats.intensity_filtered_mean = intensity_result.filtered_mean;

    printDashboard(dashboard_, header.stamp, T_W_I, x_j_pred.v, acc_bias_, gyro_bias_,
                   timing::Timing::GetMeanSeconds("step"), timing::Timing::GetMaxSeconds("step"),
                   n_effective_points, stats);
  }

  // Per-frame intensity diagnostics CSV (round-4 validation, debug config only).
  if (intensity_enabled && intensity_diag_csv_) {
    writeIntensityDiagnostics(header.stamp, intensity_result, intensity_samples,
                              intensity_preprocess_ms);
  }

  // Per-frame photometric safety diagnostics CSV (round-5, debug config only).
  if (photo_diag_csv_ && (reg_config.photometric_residual || reg_config.shadow_photometric)) {
    auto diag = optimizer.photometricDiagnostics();
    // Round-8: geometry weak-eigenvalue diagnostics from the Eq.7 degeneracy
    // analysis (available per frame from the intensity sampling).
    diag.geo_lambda1 = intensity_samples.weak_eigenvalues.x();
    diag.geo_lambda2 = intensity_samples.weak_eigenvalues.y();
    diag.geo_lambda3 = intensity_samples.weak_eigenvalues.z();
    diag.geo_two_weak_flag = (10.0 * diag.geo_lambda1 > diag.geo_lambda2) ? 1 : 0;
    // Round-10: fraction of the intensity SOURCE points (IMU frame) with
    // range < 1.5 m (Avia near-range artifact exposure).
    if (!intensity_samples.points.empty()) {
      size_t near_src = 0;
      for (size_t pi = 0; pi < intensity_samples.points.size(); ++pi) {
        if (intensity_samples.points[pi].norm() < 1.5) ++near_src;
      }
      diag.source_near_fraction = static_cast<double>(near_src) / intensity_samples.points.size();
    }
    // Round-9 weak-direction audit (diagnostic only; sections 42-49).
    if (intensity_enabled) {
      diag.eta_world = intensity_samples.weak_direction;
      const Eigen::Matrix3d P_cur =
          intensity_samples.weak_v1 * intensity_samples.weak_v1.transpose() +
          intensity_samples.weak_v2 * intensity_samples.weak_v2.transpose();
      if (have_prev_audit_) {
        const double adot = std::clamp(
            std::abs(prev_eta_world_.dot(intensity_samples.weak_direction)), 0.0, 1.0);
        diag.eta_abs_dot_prev = adot;
        diag.eta_angle_prev_deg = std::acos(adot) * 180.0 / M_PI;
        diag.weak_subspace_diff_F = (P_cur - prev_weak_subspace_).norm();
        if (!prev_selected_hashes_.empty() && !intensity_samples.selected_voxel_hashes.empty()) {
          ankerl::unordered_dense::set<size_t> cur(intensity_samples.selected_voxel_hashes.begin(),
                                                   intensity_samples.selected_voxel_hashes.end());
          size_t inter = 0;
          for (const size_t h : prev_selected_hashes_) {
            if (cur.contains(h)) ++inter;
          }
          const size_t uni = prev_selected_hashes_.size() +
                             intensity_samples.selected_voxel_hashes.size() - inter;
          diag.selected_voxel_jaccard = uni > 0 ? static_cast<double>(inter) / uni : 0.0;
          diag.selected_voxel_retention =
              static_cast<double>(inter) / intensity_samples.selected_voxel_hashes.size();
        }
      } else {
        diag.eta_abs_dot_prev = 1.0;
        diag.eta_angle_prev_deg = 0.0;
      }
      // Eq.8 score distributions (selected subset vs all candidates).
      auto quantile = [](std::vector<double>& v, double q) {
        if (v.empty()) return 0.0;
        std::sort(v.begin(), v.end());
        const double idx = q * (v.size() - 1);
        const size_t lo = static_cast<size_t>(std::floor(idx));
        const size_t hi = static_cast<size_t>(std::ceil(idx));
        return lo == hi ? v[lo] : v[lo] * (1.0 - (idx - lo)) + v[hi] * (idx - lo);
      };
      if (!intensity_samples.voxel_scores.empty()) {
        std::vector<double> cand, sel;
        cand.reserve(intensity_samples.voxel_scores.size());
        for (const auto& s : intensity_samples.voxel_scores) cand.push_back(s.score);
        ankerl::unordered_dense::set<size_t> sel_set(
            intensity_samples.selected_voxel_hashes.begin(),
            intensity_samples.selected_voxel_hashes.end());
        sel.reserve(intensity_samples.selected_voxel_hashes.size());
        for (const auto& s : intensity_samples.voxel_scores) {
          if (sel_set.contains(s.hash)) sel.push_back(s.score);
        }
        diag.selected_score_p10 = quantile(sel, 0.10);
        diag.selected_score_p50 = quantile(sel, 0.50);
        diag.selected_score_p90 = quantile(sel, 0.90);
        diag.selected_score_max = sel.empty() ? 0.0
                                              : *std::max_element(sel.begin(), sel.end());
        diag.candidate_score_p50 = quantile(cand, 0.50);
        diag.candidate_score_p90 = quantile(cand, 0.90);
        diag.candidate_score_p99 = quantile(cand, 0.99);
      }
      have_prev_audit_ = true;
      prev_eta_world_ = intensity_samples.weak_direction;
      prev_weak_subspace_ = P_cur;
      prev_selected_hashes_ = intensity_samples.selected_voxel_hashes;
    }
    // Round-9 real-data serial-vs-production parity (section 51-53): run BOTH
    // photometric accumulation paths at the final pose and record the relative
    // errors. Computed in shadow mode (C0), where photo never enters the solve.
    if (reg_config.shadow_photometric && !intensity_samples.points.empty()) {
      const auto par = optimizer.runPhotometricParity(T_W_I);
      diag.parity_count_serial = par.count_serial;
      diag.parity_count_parallel = par.count_parallel;
      diag.parity_cost_rel = par.cost_rel;
      diag.parity_H_rel = par.H_rel;
      diag.parity_b_rel = par.b_rel;
    }
    writePhotometricDiagnostics(header.stamp, diag);
  }

  // Per-(lambda, frame) robust shadow scan CSV (round-7, debug config only).
  if (robust_scan_csv_ && reg_config.shadow_lambda_scan) {
    writeRobustShadowScan(header.stamp, optimizer.robustShadowScan());
  }
}

void Pipeline::writeIntensityDiagnostics(uint64_t stamp, const IntensityProcessingResult& result,
                                         const IntensitySampleSet& samples,
                                         double preprocess_ms) {
  if (!intensity_diag_csv_ || !intensity_hist_csv_) return;
  const double input = static_cast<double>(result.input_points);
  const double vclamp =
      static_cast<double>(result.vertical_top_clamped + result.vertical_bottom_clamped);
  const double vclamp_ratio = input > 0 ? vclamp / input : 0.0;

  *intensity_diag_csv_ << stamp << "," << result.input_points << "," << result.num_valid << ","
                       << result.num_unique_pixels << "," << result.num_collisions << ","
                       << result.num_invalid << "," << result.horizontal_boundary_adjusted << ","
                       << result.vertical_top_clamped << "," << result.vertical_bottom_clamped << ","
                       << vclamp_ratio << "," << result.filtered_min << "," << result.filtered_max
                       << "," << result.filtered_mean << "," << result.filtered_std << ","
                       << result.filtered_p01 << "," << result.filtered_p05 << ","
                       << result.filtered_p50 << "," << result.filtered_p95 << ","
                       << result.filtered_p99 << "," << result.filtered_sat0 << ","
                       << result.filtered_sat255 << "," << result.raw_min << "," << result.raw_max
                       << "," << result.raw_p50 << "," << result.raw_p95 << "," << result.raw_p99
                       << "," << result.elevation_min_deg << "," << result.elevation_max_deg << ","
                       << result.elevation_p01_deg << "," << result.elevation_p99_deg << ","
                       << preprocess_ms << "," << map_->size() << "," << samples.observed_voxels
                       << "," << samples.selected_voxels << "," << samples.points.size() << "\n";

  // One 256-bin filtered-intensity histogram row per frame.
  for (int i = 0; i < 256; ++i) {
    *intensity_hist_csv_ << result.filtered_histogram[i] << (i < 255 ? "," : "\n");
  }
}

void Pipeline::writePhotometricDiagnostics(uint64_t stamp, const PhotometricDiagnostics& d) {
  if (!photo_diag_csv_) return;
  *photo_diag_csv_ << stamp << "," << d.candidates << "," << d.valid_matches << ","
                   << d.immature_matches << "," << d.match_ratio << "," << d.r_abs_p50 << ","
                   << d.r_abs_p75 << "," << d.r_abs_p90 << "," << d.r_abs_p95 << ","
                   << d.r_abs_max << "," << d.r_signed_median << "," << d.grad_p50 << ","
                   << d.grad_p75 << "," << d.grad_p90 << "," << d.grad_p95 << ","
                   << d.J_norm_p50 << "," << d.J_norm_p90 << "," << d.J_norm_p99 << ","
                   << d.J_norm_max << "," << d.J_dof_p90[0] << "," << d.J_dof_p90[1] << ","
                   << d.J_dof_p90[2] << "," << d.J_dof_p90[3] << "," << d.J_dof_p90[4] << ","
                   << d.J_dof_p90[5] << "," << d.H_geo_fro << "," << d.H_photo_fro << ","
                   << d.H_photo_trace << "," << d.b_geo_norm << "," << d.b_photo_norm << ","
                   << d.R_H_unscaled << "," << d.R_b_unscaled << "," << d.R_H_scaled << ","
                   << d.lambda_ref << "," << d.photo_eigenvalues(0) << "," << d.photo_eigenvalues(1)
                   << "," << d.photo_eigenvalues(2) << "," << d.photo_eigenvalues(3) << ","
                   << d.photo_eigenvalues(4) << "," << d.photo_eigenvalues(5) << ","
                   << d.photo_step_translation_m * 1000.0 << "," << d.photo_step_rotation_deg << ","
                   << d.lm_iterations << "," << d.lm_accepted << "," << d.lm_rejected << ","
                   << d.reject_rate << "," << d.lm_initial_cost << "," << d.lm_final_cost << ","
                   << d.geo_cost << "," << d.photo_cost << "," << d.pose_delta_translation_m << ","
                   << d.pose_delta_rotation_deg << "," << d.photo_effective_residuals << ","
                   << d.fd_samples << "," << d.fd_median_rel_err << "," << d.fd_p95_rel_err << ","
                   << d.fd_done << "," << d.photometric_scale << "," << d.fd_level_a_samples << ","
                   << d.fd_level_a_median << "," << d.fd_level_a_p95 << "," << d.fd_level_a_sign
                   << "," << d.fd_level_b_points << "," << d.fd_level_b_scalars << ","
                   << d.fd_level_b_median << "," << d.fd_level_b_p95 << "," << d.fd_level_b_sign
                   << "," << d.fd_level_b_rx_median << "," << d.fd_level_b_rx_p95 << ","
                   << d.fd_level_b_ry_median << "," << d.fd_level_b_ry_p95 << ","
                   << d.fd_level_b_rz_median << "," << d.fd_level_b_rz_p95 << ","
                   << d.fd_level_b_tx_median << "," << d.fd_level_b_tx_p95 << ","
                   << d.fd_level_b_ty_median << "," << d.fd_level_b_ty_p95 << ","
                   << d.fd_level_b_tz_median << "," << d.fd_level_b_tz_p95 << ","
                   << d.fd_level_b_analytic_p50 << "," << d.fd_level_b_analytic_p90 << ","
                   << d.fd_level_b_analytic_p99 << "," << d.fd_level_b_numeric_p50 << ","
                   << d.fd_level_b_numeric_p90 << "," << d.fd_level_b_numeric_p99 << ","
                   << d.fd_level_c_voxel_switch << "," << d.fd_level_c_cell_switch << ","
                   << d.fd_level_c_validity_switch << "," << d.fd_level_c_noswitch_samples << ","
                   << d.fd_level_c_noswitch_median << "," << d.fd_level_c_noswitch_p95 << ","
                   << d.photo_near_fraction << "," << d.photo_near_r_p50 << ","
                   << d.photo_near_r_p90 << "," << d.photo_far_r_p50 << "," << d.photo_far_r_p90
                   << ","                    << d.geo_lambda1 << "," << d.geo_lambda2 << "," << d.geo_lambda3 << ","
                   << d.geo_two_weak_flag << "," << d.eta_world.x() << "," << d.eta_world.y()
                   << "," << d.eta_world.z() << "," << d.eta_abs_dot_prev << ","
                   << d.eta_angle_prev_deg << "," << d.weak_subspace_diff_F << ","
                   << d.selected_voxel_jaccard << "," << d.selected_voxel_retention << ","
                   << d.selected_score_p10 << "," << d.selected_score_p50 << ","
                   << d.selected_score_p90 << "," << d.selected_score_max << ","
                   << d.candidate_score_p50 << "," << d.candidate_score_p90 << ","
                   << d.candidate_score_p99 << "," << d.q_geo_v1 << "," << d.q_geo_v2 << ","
                   << d.q_photo_v1 << "," << d.q_photo_v2 << "," << d.s_geo_v1 << ","
                   << d.s_geo_v2 << "," << d.s_photo_v1 << "," << d.s_photo_v2 << ","
                   << d.parity_count_serial << "," << d.parity_count_parallel << ","
                   << d.parity_cost_rel << "," << d.parity_H_rel << "," << d.parity_b_rel << ","
                   << d.source_near_fraction << "," << d.photo_near_r_p95 << ","
                   << d.photo_far_r_p95 << "," << d.photo_near_inlier_fraction << ","
                   << d.photo_far_inlier_fraction << "," << d.photo_near_grad_p50 << ","
                   << d.photo_near_grad_p90 << "," << d.photo_far_grad_p50 << ","
                   << d.photo_far_grad_p90 << "," << d.photo_near_J_p50 << ","
                   << d.photo_near_J_p90 << "," << d.photo_far_J_p50 << ","
                   << d.photo_far_J_p90 << "," << d.photo_near_H_fraction << "\n";
}

void Pipeline::writeRobustShadowScan(uint64_t stamp,
                                     const std::vector<PhotoScaleEvaluation>& scan) {
  if (!robust_scan_csv_) return;
  for (const auto& e : scan) {
    *robust_scan_csv_ << stamp << "," << e.lambda << "," << e.raw_huber_knee << ","
                      << e.huber_inlier_fraction << "," << e.H_photo_fro << "," << e.H_geo_fro
                      << "," << e.R_H << "," << e.b_photo_norm << "," << e.b_geo_norm << ","
                      << e.photo_cost << "," << e.geo_cost << "," << e.pred_translation_m << ","
                      << e.pred_rotation_deg << "\n";
  }
}

bool Pipeline::initializeBias(const std::vector<ImuMeasurement>& imu_data,
                              const Pointcloud& pointcloud) {
  if (!bias_initializer_) {
    bias_initializer_ =
        std::make_unique<BiasInitializer>(config_.imu.t_init, config_.imu.normalized);
  }
  for (const auto& imu : imu_data) {
    if (bias_initializer_->addMeasurement(imu)) break;
  }
  if (!bias_initializer_->isReady()) return false;

  acc_bias_ = bias_initializer_->accBias();
  gyro_bias_ = bias_initializer_->gyroBias();
  imu_acc_scale_ = bias_initializer_->accScale();
  const Rotation R_est = bias_initializer_->initialOrientation();
  bias_initializer_.reset();

  LOG(I, "Bias initialized: " << acc_bias_.transpose() << " " << gyro_bias_.transpose());
  const Eigen::Vector3d euler = R_est.eulerAngles(2, 1, 0);  // yaw, pitch, roll
  LOG(I, "Initial Pitch: " << (180. / M_PI) * euler[1]);
  LOG(I, "Initial Roll: " << (180. / M_PI) * euler[2]);

  State x_init;
  x_init.quat = R_est;
  x_init.p = V3::Zero();
  x_init.v = V3::Zero();

  addState(imu_data.back().stamp, x_init.quat, x_init.p, x_init.v);

  if (pointcloud.empty()) {
    phase_ = Phase::NeedMap;
  } else if (!config_.intensity.enabled) {
    // Original BIEVR geometry-only bootstrap. Kept byte-for-byte equivalent to
    // the intensity.enabled=false A/B baseline (21121698f273d6fbfffca57546b940edb1de2ff0).
    const Transform T_W_I_init(x_init.quat, x_init.p);
    std::vector<double> ranges(pointcloud.size(), 0.f);
    for (size_t i = 0; i < pointcloud.size(); ++i) {
      ranges[i] = pointcloud[i].head<3>().norm();
    }
    map_->integratePoints(T_W_I_init * pointcloud, &ranges);
    phase_ = Phase::Running;
    running_start_time_ = imu_data.back().stamp;
  } else {
    // COIN-BIEVR: do NOT build a geometry-only map during bias init. Height and
    // intensity share the voxel weights (bump_weights_), so a geometry-only
    // integratePoints would create pixels with W(u,v) > 0 but intensity == 0,
    // biasing every later weighted intensity average. Instead fall through to
    // NeedMap so the current frame is undistorted and the map is initialized
    // jointly (geometry + filtered intensity) in tryInitMap().
    phase_ = Phase::NeedMap;
  }
  return true;
}

void Pipeline::tryInitMap(uint64_t stamp, const State& x_j_pred, const Transform& T_W_I_init,
                          const Pointcloud& undistorted, const IntensityView& intensities,
                          const Intensities& filtered_intensities, std::vector<double>& ranges,
                          const Header& header) {
  if (undistorted.size() < config_.min_points_for_map_init) return;
  const Pointcloud registered = T_W_I_init * undistorted;
  map_->integratePoints(registered, &ranges,
                        config_.intensity.enabled ? &filtered_intensities : nullptr);
  addState(stamp, x_j_pred.quat, x_j_pred.p, x_j_pred.v);
  publishLatestState(header);
  // The registered cloud keeps the raw intensity row (index-aligned with the
  // points); filtered intensity is only used for the map.
  publish(IntensityPointcloud(registered, intensities), header, "points/registered");
  if (map_->size() > config_.map_size_running_threshold) {
    phase_ = Phase::Running;
    running_start_time_ = stamp;
  }
}

void Pipeline::sampleSource(const Pointcloud& undistorted, const Transform& T_W_I_init,
                            Pointcloud& filtered, Pointcloud& coarse, Pointcloud& fine) const {
  Pointcloud source_down;
  voxelDownsample(undistorted, source_down, config_.preprocess.downsample_resolution);
  if (config_.preprocess.informed_sampling) {
    sampleInformed(*map_, T_W_I_init, source_down, coarse, fine,
                   config_.preprocess.downsample_resolution, config_.informed_sample_count);
    filtered = fine + coarse;
  } else {
    filtered = source_down;
  }
}

void Pipeline::publishIntensityDebug(const Header& header, const Transform& T_W_I,
                                     const IntensitySampleSet& samples,
                                     const IntensityPointcloud* map_cloud) {
  if (!samples.points.empty()) {
    publish(IntensityPointcloud(T_W_I * samples.points, toIntensityView(samples.intensities)),
            header, "debug/intensity/selected_points");
  }
  publish(samples.weak_direction, header, "debug/degeneracy/direction");

  if (!samples.selected_voxel_hashes.empty() && map_) {
    std::vector<Eigen::Vector3d> centroid_list;
    std::vector<double> value_list;
    centroid_list.reserve(samples.selected_voxel_hashes.size());
    value_list.reserve(samples.selected_voxel_hashes.size());
    for (const size_t hash : samples.selected_voxel_hashes) {
      const Voxel* v = map_->getVoxel(hash);
      if (!v || v->num_points_ == 0) continue;
      centroid_list.push_back(v->sum_ / static_cast<double>(v->num_points_));
      value_list.push_back(v->intensity.intensity_information_.norm());
    }
    if (!centroid_list.empty()) {
      Pointcloud centroids;
      centroids.resize(centroid_list.size());
      Intensities vals(1, value_list.size());
      for (size_t i = 0; i < centroid_list.size(); ++i) {
        centroids[i] = centroid_list[i];
        vals(0, i) = value_list[i];
      }
      publish(IntensityPointcloud(centroids, toIntensityView(vals)), header,
              "debug/intensity/selected_voxels");
    }
  }

  if (map_cloud) {
    publish(*map_cloud, header, "debug/map/intensity");
  }
}

bool Pipeline::addState(const uint64_t time, const Quaternion& quat, const V3& p, const V3& v) {
  if (states_.find(time) != states_.end()) {
    LOG(D, "Time " << time << " already exists in the Pipeline. Skipping.");
    return false;
  }

  states_[time] = {seq_counter_++, quat, p, v};

  const double allowed_oldest_time_s = nsToS(time) - config_.imu.window_length_s;
  if (allowed_oldest_time_s < 0) return true;

  const uint64_t allowed_oldest_time = sToNs(allowed_oldest_time_s);
  for (auto it = states_.begin(); it != states_.end();) {
    // Remove states that are outside of the window length, along with their corresponding IMU
    // integrators
    if (it->first >= allowed_oldest_time) break;
    imu_integrators_.erase(it->first);
    it = states_.erase(it);
  }
  return true;
}

bool Pipeline::addImuIntegrator(ImuIntegratorPtr imu_integrator) {
  uint64_t time = imu_integrator->getFirstTime();
  if (states_.find(time) == states_.end()) {
    LOG(D, "Time " << time << " does not exist in the Pipeline. Skipping.");
    return false;
  }

  imu_integrators_[time] = imu_integrator;
  return true;
}

bool Pipeline::optimizeInertialWindow() {
  const size_t n_states = states_.size();
  const size_t n_imu = imu_integrators_.size();
  if (n_states == 0 || n_imu == 0) {
    LOG(D, "No states or IMU integrators to optimize.");
    return false;
  }

  // Skip if there are not enough IMU integrators to prevent instable optimization
  if (n_imu < config_.min_imu_integrators_for_opt) return false;

  ceres::Problem problem;
  ceres::Solver::Options options;
  options.logging_type = ceres::SILENT;
  ceres::LossFunction* loss_function = new ceres::TrivialLoss();

  bool first = true;
  for (auto integrator_it : imu_integrators_) {
    ImuIntegratorPtr& imu_integrator = integrator_it.second;
    const uint64_t t_i = imu_integrator->getFirstTime();
    const uint64_t t_j = imu_integrator->getLastTime();
    State& state_i = states_.at(t_i);
    State& state_j = states_.at(t_j);

    ceres::CostFunction* cost_function = new InertialFactor(imu_integrator);

    // Add preintegrated IMU cost factors between consecutive states, with shared bias and gravity
    // parameters
    problem.AddResidualBlock(cost_function, loss_function, state_i.quat.coeffs().data(),
                             state_i.p.data(), state_i.v.data(), state_j.quat.coeffs().data(),
                             state_j.p.data(), state_j.v.data(), acc_bias_.data(),
                             gyro_bias_.data(), gravity_dir_.data());
    // Keep poses fixed as we only want to optimize over velocities, biases and gravity direction
    problem.SetParameterBlockConstant(state_i.quat.coeffs().data());
    problem.SetParameterBlockConstant(state_j.quat.coeffs().data());
    problem.SetParameterBlockConstant(state_i.p.data());
    problem.SetParameterBlockConstant(state_j.p.data());

    if (first) {
      problem.SetParameterBlockConstant(state_i.v.data());
      first = false;
    }
  }

  // Add constant prior to add some rigidity to the optimization problem and prevent gravity from
  // drifting
  ceres::CostFunction* prior_cost_function =
      new PriorFactor(gravity_dir_, config_.gravity_prior_weight);
  problem.AddResidualBlock(prior_cost_function, loss_function, gravity_dir_.data());
  ceres::Manifold* gravity_manifold = new ceres::SphereManifold<3>();
  problem.SetManifold(gravity_dir_.data(), gravity_manifold);

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  return true;
}

void Pipeline::publishFrame(const Header& header, const Transform& T_W_I,
                            const Pointcloud& full_registered, const Pointcloud& source_filtered,
                            const Pointcloud& source_coarse, const Pointcloud& source_fine,
                            const Pointcloud& undistorted, const IntensityView& intensities) {
  if (config_.publish_all_clouds) {
    publishDebugClouds(source_filtered, source_coarse, source_fine, undistorted, intensities, T_W_I,
                       header);
  }
  publish(IntensityPointcloud(full_registered, intensities), header, "points/registered");
  publishLatestState(header);
}

void Pipeline::publishLatestState(const Header& header) {
  if (states_.empty()) {
    LOG(W, "No states to publish.");
    return;
  }

  const State& latest_state = states_.rbegin()->second;
  Odometry odom;
  odom.pose = Transform(latest_state.quat, latest_state.p);
  // The state velocity is expressed in the world frame; rotate it into the body
  // frame so it matches the odometry message's child frame. The angular velocity
  // comes straight from the latest gyro measurement (already in the body frame).
  odom.linear_velocity = latest_state.quat.conjugate() * latest_state.v;
  odom.angular_velocity = latest_gyro_;
  publish(odom, header, "odom", config_.body_frame);
  publish(acc_bias_, header, "bias/acc");
  publish(gyro_bias_, header, "bias/gyro");
}

void Pipeline::publishDebugClouds(const Pointcloud& source_filtered,
                                  const Pointcloud& source_coarse, const Pointcloud& source_fine,
                                  const Pointcloud& undistorted_cloud,
                                  const IntensityView& intensities, const Transform& T_W_I,
                                  const Header& header) {
  Pointcloud source_registered = T_W_I * source_filtered;
  Pointcloud fine_registered = T_W_I * source_fine;
  Pointcloud coarse_registered = T_W_I * source_coarse;
  publish(fine_registered, header, "points/fine");
  publish(coarse_registered, header, "points/coarse");
  publish(source_registered, header, "points/effective");
  Header body_header = header;
  body_header.frame = config_.body_frame;
  // The undistorted cloud keeps its original point order, so the snapshotted
  // intensity row still lines up with it.
  publish(IntensityPointcloud(undistorted_cloud, intensities), body_header, "points/undistorted");
}

void Pipeline::logTUM(double timestamp, const Transform& pose) {
  const Quaternion q(pose.linear());
  const V3& t = pose.translation();
  if (!tum_log_->is_open()) {
    LOG(E, "Error: Output file is not open for logging.");
    return;
  }

  (*tum_log_) << std::fixed;
  (*tum_log_) << timestamp << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " "
              << q.y() << " " << q.z() << " " << q.w() << "\n";
}

}  // namespace bievr
