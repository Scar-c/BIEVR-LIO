# COIN-BIEVR Handoff Package

基于官方 BIEVR-LIO 复现 COIN-BIEVR 的外部代码审查交接包。

## 1. Repository / Baseline / Branch / HEAD

| Item | Value |
|---|---|
| Upstream repo | https://github.com/ethz-asl/BIEVR-LIO.git |
| Baseline SHA | `21121698f273d6fbfffca57546b940edb1de2ff0` (`main`, "Merge pull request #5 from ethz-asl/feature/humble-ci") |
| Branch | `coin_bievr` |
| HEAD SHA | `e08613fd77254c00a568baf477d6fce967afed7b` |
| Backup branch | `coin_bievr_backup_before_rebase` @ `80c3a9f` (pre-reorganization full-state WIP commit, keeps the entire implementation in one place) |
| Rebased from | clean `main` at baseline SHA |

## 2. Commit Table

`git log --oneline --decorate 2112169..HEAD`

| # | Commit | SHA | Subject | COIN-BIEVR paper Eq. / plan section |
|---|---|---|---|---|
| 1 | `460a04a` | 460a04a | refactor(map): carry per-point intensity through pending points (MapPoint) | plan §11, §57 (MapPoint) |
| 2 | `d9d31f9` | d9d31f9 | feat(map): voxel-wise intensity map with shared weighting | paper Eq. 4-6; plan §10-18 |
| 3 | `4be5874` | 4be5874 | feat(preprocess): per-frame intensity normalization | paper Eq. 1 + sparse brightness; plan §5-9 |
| 4 | `cbf9f78` | cbf9f78 | fix(processor): point index 0 vs empty-pixel sentinel | (implementation fix) |
| 5 | `13821f0` | 13821f0 | feat(sampling): complementary intensity sampling | paper Eq. 6-8; plan §18-27 |
| 6 | `e2218e1` | e2218e1 | feat(registration): generic bilinear sampling + photometric interpolation/Jacobian | paper Eq. 12 / photometric Jacobian; plan §28-30 |
| 7 | `0861f48` | 0861f48 | feat(registration): joint geometry + photometric LM | paper Eq. 13 (joint cost); plan §31-34, §58 |
| 8 | `7dd2a6f` | 7dd2a6f | feat(pipeline): integrate the COIN-BIEVR intensity pipeline | paper Fig. 2 flow; plan §35-37, §41-42 |
| 9 | `f615cf8` | f615cf8 | feat(config): intensity configuration block + regression switch | plan §39-40 |
| 10 | `e08613f` | e08613f | test: unit tests for the COIN-BIEVR modules | plan §43-48 |

## 3. Per-commit Changed Files

**1. `460a04a` refactor(map): MapPoint**
- `BIEVR/include/bievr_lio/bievr_map.h`
- `BIEVR/src/bievr_map.cpp`

**2. `d9d31f9` feat(map): voxel intensity map**
- `BIEVR/include/bievr_lio/bievr_map.h`
- `BIEVR/src/bievr_map.cpp`

**3. `4be5874` feat(preprocess): intensity normalization**
- `BIEVR/CMakeLists.txt` (add `src/intensity_processor.cpp`)
- `BIEVR/include/bievr_lio/intensity_processor.h` (new)
- `BIEVR/src/intensity_processor.cpp` (new)

**4. `cbf9f78` fix(processor): point index 0 sentinel**
- `BIEVR/src/intensity_processor.cpp`

**5. `13821f0` feat(sampling): complementary intensity sampling**
- `BIEVR/CMakeLists.txt` (add `src/intensity_sampling.cpp`)
- `BIEVR/include/bievr_lio/intensity_sampling.h` (new)
- `BIEVR/src/intensity_sampling.cpp` (new)

**6. `e2218e1` feat(registration): photometric interpolation/Jacobian**
- `BIEVR/include/bievr_lio/ls_optimizer.h`

**7. `0861f48` feat(registration): joint LM**
- `BIEVR/include/bievr_lio/ls_optimizer.h`
- `BIEVR/src/ls_optimizer.cpp`

**8. `7dd2a6f` feat(pipeline): pipeline integration**
- `BIEVR/include/bievr_lio/pipeline.h`
- `BIEVR/include/bievr_lio/utils.h`
- `BIEVR/src/pipeline.cpp`
- `BIEVR/src/utils.cpp`

**9. `f615cf8` feat(config): configuration block**
- `BIEVR/include/bievr_lio/config_loader.h`
- `config/params.yaml`

**10. `e08613f` test: unit tests**
- `BIEVR/CMakeLists.txt` (`BIEVR_BUILD_TESTS` + CTest targets)
- `BIEVR/test/test_intensity_normalization.cpp` (new)
- `BIEVR/test/test_point_intensity_alignment.cpp` (new)
- `BIEVR/test/test_intensity_map_weighted_update.cpp` (new)
- `BIEVR/test/test_intensity_map_reprojection.cpp` (new)
- `BIEVR/test/test_intensity_gradient.cpp` (new)
- `BIEVR/test/test_photometric_jacobian.cpp` (new)

## 4. Diff Overview

`git diff --stat 2112169..HEAD` — 21 files, +1981 / -82.
`git diff --name-status 2112169..HEAD` — 4 new headers/sources (intensity_processor, intensity_sampling), 6 new tests, 11 modified.

## 5. Build Status

| Build | Status |
|---|---|
| Core library (`cmake -S BIEVR -DBIEVR_BUILD_TESTS=ON`, Release) | PASS, 0 warnings |
| ROS1 catkin (`catkin build bievr_lio bievr_ros_common bievr_lio_ros`, Noetic) | PASS, 0 warnings, 0 failed |
| ROS2 wrapper | NOT BUILT (environment is ROS1-only; wrapper untouched by these changes) |

## 6. Test Status

CTest (`-DBIEVR_BUILD_TESTS=ON`): **6/6 pass**

| Test | Verifies |
|---|---|
| test_intensity_normalization | sparse brightness averages only non-empty pixels (plan §43) |
| test_point_intensity_alignment | point <-> intensity index alignment (plan §44) |
| test_intensity_map_weighted_update | height & intensity share pixel weight (plan §45, Eq. 4-5) |
| test_intensity_map_reprojection | height+intensity co-registered after normal reprojection (plan §46) |
| test_intensity_gradient | Eq. 6 iota direction per gradient pattern (plan §47) |
| test_photometric_jacobian | analytic photometric Jacobian vs 6-DOF finite difference (plan §48) |

## 7. intensity.enabled=false Regression Status

`intensity.enabled: false` must reproduce original BIEVR geometry/map/sampling/LM.

**By construction (verified at code-path level):**
- `pipeline.cpp`: with `intensity.enabled == false`, no intensity sampling is run,
  `photometric_residual` is forced false, and the map update passes
  `intensities = nullptr`.
- `bievr_map.cpp`: `integratePoints(..., nullptr)` leaves `intensity_img_` untouched
  (stays all-zero); geometry hashing/grouping/normal/bump-image math is unchanged
  (commit 1 is a pure internal representation refactor `Vector4d -> MapPoint`,
  identical arithmetic).
- `ls_optimizer.cpp`: `photometric_residual == false` runs only the original
  geometry linearization; the joint merge reduces to the geometry-only result.
- Geometry sampling (`sampleSource`) is untouched.

**Not yet verified on real data:** a runtime A/B trajectory regression
(pose-by-pose diff / ATE) requires a recorded dataset, which is not available in
this environment. Recommended: run the same bag with `intensity.enabled: true/false`
and diff the TUM trajectories (plan §4 Definition of Done).

## 8. Implemented Items (paper Eq. mapping)

| Item | Where |
|---|---|
| Per-frame intensity normalization | `intensity_processor.cpp` |
| Spherical projection (Eq. 1) | `IntensityProcessor::project` |
| Sparse brightness normalization `I_F = s·I/(I_B+1)`, non-empty only | `brightnessImage`/`normalizeImage` |
| Ouster line-artifact removal (vertical HPF + horizontal LPF) | `removeLines` (off by default) |
| Masked Gaussian blur / clamp to 255 | `gaussianBlur`/`normalizeImage` |
| Voxel-wise intensity map (Eq. 4-5 shared weight) | `bievr_map.cpp` `integratePoints` |
| Intensity map reprojection on normal change | `reprojectImage` |
| Voxel intensity information `iota` (Eq. 6) | `computeIntensityInformation` |
| Weak geometry direction `eta` (Eq. 7, `10*lambda1>lambda2`) | `estimateWeakGeometryDirection` |
| Voxel contribution `l_c = |(R_CW eta)·iota|` (Eq. 8) | `scoreIntensityVoxels` |
| Top-100 intensity voxels + 0.1 m intensity points | `selectTopIntensityVoxels`/`downsampleIntensityPoints` |
| Photometric residual + analytic Jacobian (Eq. 12) | `linearizePhotometric` |
| Joint geometry+photometric LM (Eq. 13, `H = H_g + λ²H_p`) | `LsqRegistration::linearize` |
| Full (undistorted, non-downsampled) cloud map update | `pipeline.cpp` |
| IMU propagation / sliding-window inertial optimization | inherited unchanged |
| LRU map lifecycle | inherited unchanged |

## 9. Deviations / TBD

Per plan §55 (paper does not specify exact values — do not claim them as official):

1. **`photometric_scale` (λ)** — `intensity.optimization.photometric_scale` default `0.02`, TBD. Paper only states the constant exists.
2. **Brightness window size** — `brightness_window_u/v` default `0` = global sparse mean (paper: "large window", no exact size).
3. **Image width/height/FOV per irregular LiDAR** — defaults `1024x64`, 50°; per-sensor values TBD in sensor YAMLs.
4. **Intensity map smoothing** — OFF (paper describes using `I_P` directly, Eq. 6).
5. **Eq. 8 sign/absolute direction** — implemented with absolute terms (eigenvector sign ambiguity; COIN-LIO uses `fabs`).
6. **`normalize_eta`** — default `true`; paper does not state normalization.
7. **Ouster point-to-pixel LUT** — `use_ouster_lut` is a flag only; factory LUT data not integrated yet (falls back to spherical projection).

All of the above are configurable and marked `TBD` in `config/params.yaml`.

## 10. Known Limitations

- Ouster factory LUT + destaggering not implemented (flag only).
- Intensity preprocessing runs single-threaded per frame (O(WH) box/global mean); acceptable but not TBB-parallelized.
- `buildIntensityMapCloud` debug topic is capped (2000 voxels) and only published with `publish_all_clouds`.
- BIEVR's pre-existing single-voxel quirk (a cloud lying entirely in one voxel is not integrated) is preserved; unit tests work around it. Not modified to keep baseline behaviour.
- `photometric_scale`, image geometry and per-sensor parameters unvalidated against the paper's experiments (no datasets run).

## 11. Unresolved Questions

1. Exact official `λ` (photometric_scale) value — needs tuning on ENWIDE TunnelS/D / GEODE FlatSurfacesS.
2. Exact brightness window size and image geometry used in the paper's experiments.
3. Whether the intensity map is Gaussian-smoothed in the official implementation.
4. Whether `eta` is normalized in the official implementation.
5. Ouster LUT source (factory metadata) to enable `use_ouster_lut`.

## 12. Next Benchmark Steps (plan §50)

- **Step A (geometry-rich, no regression):** Newer College QuadHard/Cloister — COIN-BIEVR must not break BIEVR.
- **Step B (Ouster intensity-degenerate):** ENWIDE TunnelS/TunnelD, RunwayS/RunwayD — target: BIEVR fails, COIN-BIEVR recovers (paper: TunnelS 0.432 m, TunnelD 0.369 m).
- **Step C (Livox irregular):** GEODE Shield1/4/5, FlatSurfacesS — especially FlatSurfacesS (paper: geometry-only diverges, COIN-BIEVR 0.064 m).
- Tune `photometric_scale` via dashboard Geo/Photo RMSE and per-stage timings.
- A/B regression with `intensity.enabled=false` on one sequence first (plan §4).
