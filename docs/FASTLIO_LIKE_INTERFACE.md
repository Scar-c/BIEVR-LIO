# FAST-LIO-like sensor-specific launch / configuration interface

## User entry points

```bash
roslaunch bievr_lio_ros mapping_avia.launch          # rviz on by default
roslaunch bievr_lio_ros mapping_avia.launch rviz:=false
roslaunch bievr_lio_ros mapping_mid360.launch
roslaunch bievr_lio_ros mapping_ouster.launch
```

Photometric optimization switch (C1 default ON, provisional lambda 0.001):
```bash
roslaunch bievr_lio_ros mapping_ouster.launch photo:=false   # C0: photo OFF +
                                                              # shadow diagnostics
```

Each launch selects the sensor preset AND its validated algorithm config, so the
user never writes `sensor_config_file:=...` by hand. The `photo` arg (default
true) selects the C1 preset (`params_coin_bievr_{avia,ouster_enwide}.yaml`,
intensity optimization ON, lambda 0.001) or the C0 preset
(`params_coin_bievr_{avia,ouster_enwide}_c0.yaml`, photo OFF + shadow
diagnostics ON); the C0 files are generated from the C1 files with only the
enabled/shadow lines differing. Optional topic override via
ROS remap:

```bash
roslaunch bievr_lio_ros mapping_avia.launch lidar_topic:=/custom/lidar imu_topic:=/custom/imu
```

## Architecture (current, unchanged)

The node reads the algorithm YAML and the sensor YAML from command-line arguments
(`--params_file`, `--sensor_config_file`, header-only `config_loader.h` /
yaml-cpp). Topics, extrinsic and LiDAR range live in the sensor config; map /
optimization / intensity / imu parameters live in the algorithm config. Frames:
`map.frame` (default `odom`) and `imu.frame` (default `imu`) come from the
algorithm config.

| launch | sensor config | algorithm config | lidar topic | imu topic | point_filter_num |
|---|---|---|---|---|---|
| mapping_avia.launch | sensor_configs/gamma.yaml (GEODE gamma device = Livox Avia) | params_coin_bievr_avia.yaml (validated) | /livox/lidar | /livox/imu | 1 |
| mapping_mid360.launch | sensor_configs/mid360.yaml (USES_CURRENT_DEFAULT) | params_coin_bievr_avia.yaml (shared Livox path) | /livox/lidar | /livox/imu | 1 (default) |
| mapping_ouster.launch | sensor_configs/enwide.yaml | params_coin_bievr_ouster_enwide.yaml (validated) | /ouster/points | /ouster/imu | 4 (indices 0,4,8,... ratio 1/4) |

A `config/mapping.yaml` common algorithm file is intentionally NOT introduced:
the current loader loads exactly one algorithm YAML, and splitting it would
either go unused or require a loader change. Complete per-sensor algorithm
configs are the validated units; the launches resolve them internally.

Mid360: shares the Livox gen1 CustomMsg parser with Avia (shared implementation,
sensor-specific preset). Its extrinsic in mid360.yaml is a placeholder
(USES_CURRENT_DEFAULT) and must be replaced before use; intensity preprocessing
uses the current default (Avia-derived) values - not a validated Mid360 preset.

## Parameter source classification

| Parameter | Source | Category | FAST-LIO analogue | Semantic difference |
|---|---|---|---|---|
| map.pixel_size_m | BIEVR_ORIGINAL | voxel image | filter_size_map | BIEVR pixel side length of the bump/intensity image, NOT a map voxel filter size |
| map.voxel_size_m | BIEVR_ORIGINAL | voxel map | filter_size_map | BIEVR voxel size |
| map.normal_tolerance_deg | BIEVR_ORIGINAL | voxel reprojection | - | normal-change threshold to re-project the voxel image |
| map.smooth / weighted / max_size / frame | BIEVR_ORIGINAL | map representation | - | BIEVR bump-image update semantics |
| preprocess.downsample_resolution_m | BIEVR_ORIGINAL | source sampling | filter_size_surf | BIEVR source downsample resolution |
| preprocess.informed_sampling | BIEVR_ORIGINAL | source sampling | - | BIEVR map-informed sampling (MID) |
| preprocess.point_filter_num | COIN_BIEVR_ADDED | geometry stride | mapping/point_filter_num | COIN-LIO stride N (indices 0,N,2N,...; ratio 1/N); Avia 1, Ouster 4 |
| optimization.huber_delta | BIEVR_ORIGINAL | optimizer | - | shared Huber delta (geometry + photometric) |
| optimization.img_residual / img_jacobian | BIEVR_ORIGINAL | optimizer | - | BIEVR geometry image residual |
| intensity.enabled | COIN_BIEVR_ADDED | master switch | - | COIN-BIEVR master switch |
| intensity.preprocessing.* | COIN_BIEVR_ADDED | projection/normalization | - | spherical or ouster_lut projection, FOV, image size, brightness window, normalization, line removal, blur, raw scale |
| intensity.sampling.* | COIN_BIEVR_ADDED | Eq.7/8 sampling | - | num_voxels (top-K), downsample, weak_eigen_ratio, normalize_eta, score_mode |
| intensity.optimization.* | COIN_BIEVR_ADDED | photometric LM | - | photometric_scale (lambda), shadow_diagnostics, photometric_warmup_s |
| imu.window_s / t_init | BIEVR_ORIGINAL | inertial | - | BIEVR inertial window |
| debug.* | ROS_INTERFACE_ONLY / BIEVR_ORIGINAL | logging | publish.* | trajectory/diagnostics paths (offline-only for process_bag) |
| max_num_threads | BIEVR_ORIGINAL | threading | - | TBB parallelism cap |
| sensor topics / calibration / ranges | SENSOR_PRESET | sensor binding | common lid_topic/imu_topic + extrinsic | topics + T_IMU_LIDAR + range per sensor preset |

## FAST-LIO parameters intentionally NOT copied

| FAST-LIO param | classification |
|---|---|
| preprocess.scan_line | NOT_APPLICABLE (BIEVR map is unstructured voxel-image based) |
| preprocess.blind | ALREADY_HANDLED_DIFFERENTLY (sensor config lidar.min_range_m) |
| preprocess.timestamp_unit | NOT_APPLICABLE (per-point time is auto-detected in conversions.h) |
| preprocess.scan_rate | NOT_APPLICABLE (times come from the messages) |
| mapping.filter_size_surf | EQUIVALENT_ROLE_DIFFERENT_SEMANTICS (BIEVR preprocess.downsample_resolution_m) |
| mapping.filter_size_map | EQUIVALENT_ROLE_DIFFERENT_SEMANTICS (BIEVR map.pixel_size_m / voxel_size_m; NOT renamed) |
| mapping.cube_side_length | NOT_APPLICABLE (BIEVR map is not an ikd-Tree with local map cubes) |
| mapping.det_range | EQUIVALENT_ROLE_DIFFERENT_SEMANTICS (sensor config lidar.max_range_m) |
| mapping.fov_degree | NOT_APPLICABLE (BIEVR registers against the full map) |
| extrinsic_est_en | NOT_APPLICABLE (extrinsic is fixed per sensor preset) |
| publish.path_en | NOT_APPLICABLE (no nav_msgs/Path is published; odometry is) |
| publish.scan_publish_en | ALREADY_HANDLED_DIFFERENTLY (/bievr_lio/points/registered always published) |
| publish.dense_publish_en | ALREADY_HANDLED_DIFFERENTLY (coarse/fine published only with publish_all_clouds) |
| publish.scan_bodyframe_pub_en | NOT_APPLICABLE (odometry child frame is the IMU frame) |

## BIEVR-specific parameters retained under their original names

voxel_size_m, pixel_size_m, normal_tolerance_deg, smooth, weighted,
downsample_resolution_m, informed_sampling (MID), img_residual, img_jacobian,
huber_delta, imu window/t_init, max_num_threads. No renaming to FAST-LIO names.

## COIN-BIEVR-specific parameters

intensity.enabled, preprocessing (projection/FOV/image/window/normalization/line
removal/blur/raw scale/ouster metadata/u_shift/intensity_scale/sparse
brightness), sampling (num_voxels/downsample/weak_eigen_ratio/normalize_eta/
score_mode), optimization (photometric_scale/shadow_diagnostics/warmup),
preprocess.point_filter_num.

HARD_CODED_NOT_EXPOSED (implemented but not configurable):
- Eq.7 eta construction rule (v1+v2 if 10*lambda1>lambda2 else v1)
- Eq.8 abs_components default vs paper_signed option is exposed; the exact
  signed/abs semantics choice is fixed per score_mode
- Huber objective rho(lambda*r_photo) with the shared huber_delta
- photometric_min_map_weight default 1.0 (configurable via code default only)
- map maturity / warmup mechanics
- LM parameters (lm_init_lambda_factor, epsilons, max_iterations)
- Ouster destagger semantics (row = ring, col = geometric azimuth)

## Published topics (rviz/mapping.rviz shows only these + TF)

- /bievr_lio/odom (nav_msgs/Odometry, frame = map.frame, child = imu.frame) + TF
- /bievr_lio/points/registered (accumulated registered cloud)
- /bievr_lio/bias/acc, /bievr_lio/bias/gyro
- with publish_all_clouds: /bievr_lio/points/{coarse,fine},
  /bievr_lio/debug/intensity/{selected_points,selected_voxels},
  /bievr_lio/debug/degeneracy/direction, /bievr_lio/debug/map/intensity

## Backward compatibility

The old `process_bag.launch` / `process_topics.launch`, all existing YAML files
and the offline process_bag benchmark scripts are unchanged and continue to work.
The new launchers are purely additive.
## Troubleshooting

Symptom: `Error in XmlRpcClient::writeRequest: write error (Connection refused).`
repeated at launch while roscore appears alive.

Cause observed: a long-lived roscore whose `rosmaster` accumulated stale node
registrations (from force-killed nodes) and intermittently refused XMLRPC
connections. Note that `kill <roscore-wrapper-pid>` alone does NOT stop the
`rosmaster --core` child that actually owns port 11311.

Fix:
```bash
pkill -9 -f "rosmaster --core"    # kill the real master process
pkill -9 -f "/opt/ros/noetic/lib/rosout"
pkill -9 -f "bin/roscore"
roscore &                           # start ONE fresh master
```
Verify: `rosparam list` should show only /rosdistro, /rosversion and fresh
/roslaunch/uris; the Connection refused errors disappear and all mapping_*.launch
files load cleanly.
