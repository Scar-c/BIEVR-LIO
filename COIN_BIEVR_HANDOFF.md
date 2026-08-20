# COIN-BIEVR Handoff Package

基于官方 BIEVR-LIO 复现 COIN-BIEVR 的外部代码审查交接包。

## 1. Repository / Baseline / Branch / HEAD

| Item | Value |
|---|---|
| Upstream repo | https://github.com/ethz-asl/BIEVR-LIO.git |
| Baseline SHA | `21121698f273d6fbfffca57546b940edb1de2ff0` (`main`, "Merge pull request #5 from ethz-asl/feature/humble-ci") |
| Branch | `coin_bievr` |
| Implementation HEAD before documentation commit | `2d316d2` (round-8; round-7 impl head was `fa207c7`) |
| Backup branch | `coin_bievr_backup_before_rebase` @ `80c3a9f` (pre-reorganization full-state WIP commit, keeps the entire implementation in one place) |
| Rebased from | clean `main` at baseline SHA |

> Round 1 (commits 1-11, reviewed HEAD `5e93707`) covers the initial COIN-BIEVR
> implementation. Round 2 (commits 12-16) contains the second-review correctness
> fixes. Round 3 (commits 17-20) fixes the bootstrap shared-weight issue, makes
> the COIN-BIEVR config a complete preset and adds projection boundary
> diagnostics. Round 4 (commits 21-25) fixes a config-loader bug, adds per-frame
> intensity diagnostics + CSV export and runs the first real-data GEODE/Avia
> preprocessing validation. Round 5 (commits 26-30) adds photometric safety
> validation which FAILED the real-map Jacobian FD gate. Round 6 (commits 31-34)
> fixes the photometric derivative (exact masked-bilinear gradient) and validates
> it with the three-level FD. Round 7 (commits 35-38) fixes the robust-weighting
> semantics (removes the invalid lambda^2 unscaling) and runs the first
> trustworthy C0-vs-C-lambda validation on Shield1. Round 8 (commits 39-40) adds
> FlatSurfacesS photometric-degeneracy diagnostics + tooling; the FlatSurfacesS
> dataset itself is NOT available locally (see Section 7h).

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

Round 2 commits (second review fixes, appended on top of the reviewed HEAD):

| # | Commit | SHA | Subject |
|---|---|---|---|
| 12 | `98fa2e2` | 98fa2e2 | fix(intensity): normalize all projected points across pixel collisions |
| 13 | `6cc967a` | 6cc967a | fix(intensity): make preprocessing bypass preserve point alignment |
| 14 | `fe51c0c` | fe51c0c | fix(map): reject misaligned intensity updates; zero intensity work when disabled |
| 15 | `0e9093a` | 0e9093a | config: separate BIEVR baseline and COIN-BIEVR presets; Eq.8 score mode |
| 16 | `ef42b03` | ef42b03 | test(intensity): cover collisions, bypass, misaligned, sign invariance, disabled |

Round 3 commits (bootstrap + config + diagnostics, appended on top):

| # | Commit | SHA | Subject |
|---|---|---|---|
| 17 | `2b3d0a2` | 2b3d0a2 | fix(pipeline): initialize intensity map through undistorted NeedMap path |
| 18 | `8a8c5f0` | 8a8c5f0 | debug(intensity): report projection boundary clamping |
| 19 | `4c8152c` | 4c8152c | fix(config): make COIN-BIEVR preset a complete algorithm config |
| 20 | `7ebd41d` | 7ebd41d | test(intensity): bootstrap shared-weight + config smoke |

Round 4 commits (config-loader fix + diagnostics + tools, appended on top):

| # | Commit | SHA | Subject |
|---|---|---|---|
| 21 | `4b0fdc0` | 4b0fdc0 | config: add experimental Livox Avia COIN-BIEVR preset |
| 22 | `8a0042d` | 8a0042d | fix(config): nested YAML resolution no longer mutates the document |
| 23 | `e589c81` | e589c81 | feat(debug): per-frame intensity preprocessing diagnostics + CSV export |
| 24 | `7b5c8ff` | 7b5c8ff | tools: round-4 intensity preprocessing analysis + window-sweep scripts |

Round 5 commits (photometric safety validation, appended on top):

| # | Commit | SHA | Subject |
|---|---|---|---|
| 26 | `67e4dd7` | 67e4dd7 | feat(registration): shadow photometric linearization + maturity/warmup gating + real-map FD check |
| 27 | `b460750` | b460750 | feat(debug): per-frame photometric safety diagnostics CSV + pipeline wiring |
| 28 | `e1bd9fc` | e1bd9fc | test(registration): cover shadow mode, photometric warmup and map-maturity gating |
| 29 | `9b53153` | 9b53153 | tools: photometric safety analysis + round-5 sweep scripts |

Round 6 commits (photometric derivative correctness, appended on top):

| # | Commit | SHA | Subject |
|---|---|---|---|
| 31 | `f694db3` | f694db3 | fix(photo): use exact derivative of masked bilinear intensity sampling |
| 32 | `5097649` | 5097649 | test(photo): validate exact interpolation and fixed-correspondence Jacobian |
| 33 | `d8ce276` | d8ce276 | tools: analyze round-6 photometric derivative validation |

Round 7 commits (robust weighting semantics + first C0-vs-C-lambda validation):

| # | Commit | SHA | Subject |
|---|---|---|---|
| 35 | `c8ae0f2` | c8ae0f2 | fix(photo): remove invalid lambda-squared unscaling; direct robustified accumulation |
| 36 | `e4abac4` | e4abac4 | test(photo): cover Huber-aware photometric scale semantics |
| 37 | `fa207c7` | fa207c7 | tools: analyze round-7 photometric scale safety |

Round 8 commits (FlatSurfacesS diagnostics + tooling):

| # | Commit | SHA | Subject |
|---|---|---|---|
| 39 | `42005b4` | 42005b4 | feat(debug): expose FlatSurfaces photometric-degeneracy diagnostics |
| 40 | `2d316d2` | 2d316d2 | tools: add FlatSurfaces round-8 runner and analysis |

## 7b. Second Review Round (round-2 fixes)

See `COIN_BIEVR_SECOND_REVIEW_FIX_PLAN.md` for the review specification.

**P1 - projection collision (commit 12).** When several 3D points land in the
same spherical image pixel, only the nearest owner used to get the normalized
I_F; the losers kept their raw sensor-domain intensity, mixing normalized and
raw values into the voxel intensity map and the photometric residual. Fix: the
processor returns an `IntensityProcessingResult` with a per-point `point_pixel_idx`;
after sparse brightness normalization, `I_F(v_i,u_i)` is back-assigned to EVERY
successfully projected point, so all points sharing a pixel stay in the
normalized `[0,255]` domain while the image keeps its nearest-point owner
semantics (full-cloud map update is preserved, no points dropped).
`projectPoint()` abstracts the point->pixel projection (future Ouster LUT) and
clamps the elevation into the FOV so every finite point stays valid.

**P1 - preprocessing bypass (commit 13).** `preprocessing.enabled=false` no
longer returns an empty intensity row; it runs a debug/ablation bypass
(`clamp(raw * raw_scale)`) that keeps the point/intensity index alignment.
Not the paper default.

**P1 - map defensive rejection (commit 14).** `integratePoints()` rejects a
size-mismatched intensity row (assert in debug, warning + skip in release) and
never falls back to intensity=0 merged into the map.

**P2 - disabled zero path (commit 14).** `BIEVRMap::Config::intensity_enabled`
gates all voxel intensity work (allocation, reprojection, information update);
with `intensity.enabled=false` the pipeline also skips preprocessing, sampling
and the photometric LM (geometry-only LsqRegistration constructor), so runtime
and memory follow the original BIEVR path.

**P2 - config presets (commit 15).** `config/params.yaml` defaults to
`intensity.enabled: false` (BIEVR-safe baseline); the COIN-BIEVR preset is
`config/params_coin_bievr.yaml`. Eq. (8) score is configurable via
`score_mode` ("abs_components" default / "paper_signed").

**Tests.** Extended from 6 to 12 (see Section 6b).

## 7c. Round 3 (bootstrap + config + diagnostics)

**Bootstrap shared-weight fix (commit 17).** During bias initialization the old
code built a geometry-only map when a cloud was already available, even with the
intensity branch enabled. Because height and intensity share `bump_weights_`,
that left pixels with `W(u,v) > 0` but intensity == 0, biasing every later
weighted intensity average. With `intensity.enabled=true` the bias init now
falls through to `Phase::NeedMap`, so the current frame is undistorted and the
map is initialized jointly (full undistorted cloud + filtered intensity) in
`tryInitMap()`. The `intensity.enabled=false` bootstrap is byte-for-byte
unchanged.

**Projection boundary diagnostics (commit 18).** `projectPoint()` returns the raw
pixel; `clampPixel()` clamps and counts horizontal boundary adjustments (incl.
the azimuth -pi wrap `u==W`), vertical top/bottom clamps. The vertical-clamp
fraction is reported on the dashboard, and a log-once warning fires when >1% of
points are clamped (likely FOV/image-size mismatch). Diagnostic only.

**Complete COIN-BIEVR preset (commit 19).** `config/params_coin_bievr.yaml` is
now a complete algorithm config (map/preprocess/optimization/imu/debug/
max_num_threads/intensity) usable directly via `params:=params_coin_bievr`;
sensor configs stay in `config/sensor_configs/`. Correct ROS1 usage:

- BIEVR: `roslaunch bievr_lio_ros process_bag.launch sensor_config:=enwide params:=params rosbag:=...`
- COIN-BIEVR: `roslaunch bievr_lio_ros process_bag.launch sensor_config:=enwide params:=params_coin_bievr rosbag:=...`

**Tests.** Extended from 12 to 14 (`test_bootstrap_shared_weight`,
`test_config_smoke`).

## 7d. Round 4 (config-loader fix + diagnostics + real-data Avia validation)

**Bug found & fixed: `getNested` corrupted the YAML document (commit 22).**
`MergedYaml::getNested()` used yaml-cpp's non-const `operator[]` while walking
the dotted key; because `YAML::Node` is a ref-counted handle into the shared
document tree, the traversal could mutate nodes, so after a `getNested()` call
subsequent `as<T>()` lookups of the same keys failed and nested
`intensity.preprocessing` values (vertical_fov_deg, brightness window, ...)
silently fell back to C++ defaults (FOV stayed 50 deg, window 0x0 no matter the
YAML). Fixed with const-only traversal; regression covered in
`test_config_smoke` (Avia preset must load 77.2 / 41x7).

**Per-frame intensity diagnostics (commit 23).** `IntensityProcessingResult`
now carries filtered P1/P5/P50/P95/P99 + std + saturation@0/@255 + 256-bin
histogram, raw min/max/P50/P95/P99 and elevation min/max/P1/P99, all computed
with lightweight O(N) histograms. A CSV logger (gated by
`debug.intensity_diagnostics_path`, off by default) exports per-frame
`diagnostics.csv` and `diagnostics_histogram.csv`.

**Real-data GEODE/Avia validation (Shield 1, Livox Avia).** See
`results/intensity_preprocess/avia_shield1/` (gitignored). Short segment
starting at bag begin (vehicle stationary -> IMU bias init valid):
- vertical_fov_deg 77.2 matches the observed elevation [-38.5, 38.5] deg
  (P1/P99 [-34.3, 34.4]); mean vertical-clamp ratio ~0.06% (target <1%).
- collision ratio ~62% (expected for the irregular Avia pattern; no raw
  intensity leakage - covered by unit tests).
- Brightness-window sweep (0x0 / 41x7 / 81x15 / 161x31) shifts the filtered
  distribution (P50 193.5 / 149.5 / 129.5 / 137.5) with no 255-saturation.
- Geometry-only A/B (params vs params_coin_bievr_avia, photo OFF): trajectories
  agree to ~cm level (mean 54 mm / 0.19 deg over 139 m), with the expected
  1-frame timestamp offset from the round-3 NeedMap bootstrap; baseline is
  stable (no divergence) when the segment starts with a stationary period.

**Important validation caveat.** A segment that starts mid-motion breaks the
zero-velocity IMU bias init and makes BOTH baseline and COIN-BIEVR diverge -
this is a data/init issue, not a code regression. Baseline on a properly
initialized segment is stable on Shield 1.

## 7e. Round 5 (photometric residual safety validation)

See `results/photo_safety/avia_shield1/` (gitignored). Same 99 s GEODE/Avia
segment (stationary init), 41x7 window frozen.

**What was added.** Shadow photometric mode (`shadow_diagnostics`, photo
residual/J/H/b computed but never merged), map-maturity gate (shared W >= 1.0),
photometric warmup gate (`photometric_warmup_s`), per-frame photometric safety
CSV, and a real-map finite-difference Jacobian spot check. `PhotometricDiagnostics`
reports residual/gradient/J/H/b percentiles, per-DOF |J|, photo Hessian
eigenvalues, unscaled H/b ratios, per-frame lambda_ref, the photo-induced LM
step, and LM stability.

**Key results (Shield 1, Avia):**
- Shadow (C0): 969/992 frames with matches, ~255 valid matches/frame (mean),
  residual |r| P50/P90/P95 = 31.7/72.4/90.5, gradient P50 ~334/m, J norm P50 ~
  1245 (nonzero), Jdof P90 all nonzero (rz largest ~2310), unscaled
  H_photo/H_geo ~ 48.7, lambda_ref (median) = 0.0453. Shadow does NOT change the
  trajectory.
- **Real-map FD Jacobian check: FAIL** - median relative error 3.53, P95 87.6
  vs the required 1e-3 / 1e-2. The analytic central-difference photometric
  Jacobian does not match the local finite-difference derivative on the real
  texture (likely the 2-pixel central-difference gradient vs local bilinear
  derivative on noisy/sparse texture). Per the round-5 gate, C-lambda is not
  enabled.
- Lambda sweep (trajectory stability only): L0 (lambda=0) stable (139 m), L1
  (0.0045) stable (139 m), L2 (0.0136) stable (141 m), L3 (0.0453 = lambda_ref)
  DIVERGED (855 m). LM iterations / reject rate unchanged vs C0; photo-induced
  pose steps are small (P99 < 0.4 mm, max < 17 mm; rotation max < 0.19 deg).

**Round-5 verdict.** Photometric residual safety NOT established: the real-map
Jacobian FD gate fails, and at lambda_ref the trajectory diverges on Shield 1.
Classification: C0/L0 SAFE; L1/L2 MARGINAL (stable trajectory, FD gate not met);
L3 UNSAFE (diverged). Photometric_scale decision deferred to the coordinator;
do not enable C-lambda until the Jacobian accuracy issue is resolved.

## 7f. Round 6 (photometric derivative correctness)

**Bug fixed.** The photometric residual used the masked-bilinear VALUE
(getSubPixelIntensityValue) while the Jacobian gradient used a 2-pixel central
difference - two different functions, which diverged on real texture (round-5 FD
median rel err 3.53). Now `sampleIntensityBilinearWithGradient` returns the
value AND the exact quotient-rule derivative of the same masked normalized
bilinear interpolation (intensity per pixel). The residual and Jacobian come
from the SAME call. The geometry height gradient and the Eq.6 sampling
central-difference are untouched (baseline protected).

**Three-level FD validation (real Shield1/Avia map, shadow C0):**
- Level A (image-space): 1036 scalar derivatives, median 4.2e-12, P95 2.7e-9,
  sign 100% -> PASS.
- Level B (fixed-correspondence 6-DOF): 320 points / 1920 scalars, median
  1.3e-10, P95 6.0e-8, sign 100%; per-DOF all ~1e-9..1e-11; analytic == numeric
  J magnitudes -> PASS.
- Level C (full-lookup switching): voxel 0.02%, cell 0.12%, validity 0.016%
  switch ratios (GOOD); no-switch subset 208214 samples, median 7.4e-10 ->
  consistent with Level B.

**Round-5 lambda_ref invalidation.** Round-5 lambda_ref = 0.0453 MUST NOT be
treated as a valid calibrated photometric scale. The previous H/lambda^2
back-calculation ignores the lambda-dependence introduced by robust weighting.
Photometric-scale calibration is deferred until photometric derivative
correctness is established (now done) and robust-weighting semantics are
resolved. No lambda sweep was performed in round 6.

**Round-6 status.** Photometric linearization is now mathematically consistent
(Level A/B PASS). Photometric optimization is STILL NOT enabled; the next round
must independently address robust (Huber) weighting semantics and
photometric_scale calibration before a first trustworthy C0-vs-C-lambda
experiment.

## 7g. Round 7 (robust weighting semantics & first C0-vs-C-lambda validation)

**Round-5 lambda_ref invalidation.** Round-5 lambda_ref = 0.0453 is INVALID as a
calibrated scale. COIN-BIEVR optimizes rho(lambda*r_photo); since the Huber
weight depends on lambda*r, H_photo is piecewise lambda^2 (quadratic) / lambda
(linear) scaled and cannot generally be recovered by H_photo/lambda^2.
Round-7 therefore removes the /lambda^2 unscaling (R_H_unscaled, R_b_unscaled,
lambda_ref are zeroed/DEPRECATED_INVALID) and uses DIRECT robustified
accumulation at each lambda with the exact optimizer semantics.

**Phase A (C0 robustified shadow scan, Shield1/Avia, photo OFF).** For each of
9 lambdas, at the geometry-only solution the raw photo residual/J are
re-accumulated with rho(lambda*r) and the direct H_photo/b_photo/cost, Huber
inlier fraction, R_H and predicted pose influence are reported
(results/photo_scale/avia_shield1/robust_shadow_scan.csv):

| lambda | raw knee | inlier frac | R_H P90/P99/max | pred_t P99/max(mm) | pred_r P99/max(deg) | class |
|---|---|---|---|---|---|---|
| 0.0005 | 200 | 0.976 | 0.0074/0.014/0.021 | 0.37/0.76 | 0.0036/0.018 | SAFE |
| 0.0010 | 100 | 0.932 | 0.028/0.053/0.084 | 1.19/2.06 | 0.012/0.059 | SAFE |
| 0.0020 | 50 | 0.726 | 0.096/0.172/0.313 | 2.68/3.65 | 0.033/0.104 | SAFE |
| 0.0030 | 33.3 | 0.519 | 0.180/0.316/0.662 | 3.49/4.92 | 0.038/0.113 | MARGINAL |
| 0.0050 | 20 | 0.316 | 0.374/0.645/1.663 | 4.21/6.79 | 0.053/0.152 | UNSAFE |
| 0.0100 | 10 | 0.152 | 0.946/1.71/5.84 | 4.84/9.02 | 0.074/0.237 | UNSAFE |
| 0.0200 | 5 | 0.074 | 2.27/4.20/21.3 | 4.81/11.4 | 0.090/0.332 | UNSAFE |
| 0.0300 | 3.3 | 0.049 | 3.70/6.83/46.4 | 4.78/12.5 | 0.104/0.369 | UNSAFE |
| 0.0400 | 2.5 | 0.037 | 5.27/9.75/80.9 | 4.92/13.1 | 0.115/0.386 | UNSAFE |

No lambda from Phase A entered the estimator solve.

**Phase B (actual C-lambda, gated).** Per the round-7 gate, 0.010 was skipped
(SHADOW_UNSAFE). C0 / 0.001 / 0.003 all ran on the same segment:

| group | stable | path | reject | iter | R_H P90 | step_t P99/max(mm) | step_r P99/max(deg) | class |
|---|---|---|---|---|---|---|---|---|
| C0 | yes | 138.9 m | 0.722 | 4.0 | - | 0/0 | 0/0 | SAFE |
| 0.001 | yes | 139.7 m | 0.688 | 4.5 | 0.028 | 0.18/1.41 | 0.0021/0.042 | SAFE |
| 0.003 | yes | 141.5 m | 0.667 | 5.0 | 0.178 | 10.0/21.7 | 0.067/0.264 | SAFE |
| 0.010 | skipped | - | - | - | - | - | - | SKIPPED_BY_GATE |

C0-vs-C-lambda trajectory: rotation diff small (0.001: mean 0.21 deg; 0.003:
mean 0.40 deg); translation diff ~0.46 m (0.001) / ~1.19 m (0.003) mean, largely
a lever-arm effect of the small rotation offset over ~140 m (both paths stable,
< 1.2x C0). Both 0.001 and 0.003 are classified SAFE on Shield1. This is the
first trustworthy C0-vs-C-lambda data with correct derivative + robust semantics.

## 7h. Round 8 (FlatSurfacesS photometric-effectiveness validation)

**Dataset obtained and experiment run.** The FlatSurfacesS bag + GT were
provided (results/flatsurfaces_round8/): bag
/home/lc/algorithm_versa/bag/ENWIDE/flat_surfaces_smooth.bag (82 s, 821 Livox
Avia frames, /livox/lidar + /livox/imu), GT flat_surfaces_smooth.txt (23315
TUM poses, aligned time range). evo v1.31.1 used for APE (evo_ape tum -a,
trans_part, t_max_diff 0.1, offset 0, no offset search).

Four groups run from t0 (IMU init OK: acc ~9.75 g, gyro mean 0.77 deg/s):

| group | coverage | path(m) | APE RMSE | APE mean | APE max | t>1m | t>2m | class |
|---|---|---|---|---|---|---|---|---|
| B0 (BIEVR) | 99.5% | 67.7 | 1.52 | 1.37 | 3.48 | 24.5s | 30.8s | STABLE_BUT_POOR (geometry drift) |
| C0 (COIN/photo OFF) | 99.4% | 67.2 | 1.65 | 1.54 | 3.21 | 0s | 16.5s | STABLE_BUT_POOR |
| L1 (0.001) | 99.4% | 5542 | 821.9 | 537.8 | 3617 | - | - | FAIL (diverged) |
| L2 (0.003) | 99.4% | 10144 | 2232 | 1829 | 6519 | - | - | FAIL (diverged) |

FlatSurfacesS is confirmed geometry-degenerate: 79% of frames have the
two-weak-direction flag (10*lambda1 > lambda2); geometry lambda1 median ~0.9.

**Key result: the photometric residual does NOT rescue FlatSurfacesS in the
current implementation - it causes catastrophic divergence.** The intensity
sampling works (600-1100 photo matches/frame, residual P50 30-40), but the
direct robustified photo Hessian is far too strong relative to the degenerate
geometry Hessian (L1 R_H median 0.48, L2 R_H median 2.45 - vs Shield1's 0.028 /
0.18), so the photometric term dominates and drives the pose away. Near-range
(<1.5 m) photo matches are essentially absent (fraction ~0), so the Avia
near-range artifact is not the cause. L1 begins to leave the bounded envelope
~40-50 s in (after the 10 s photometric warmup).

Per round-8 rules this is reported, not fixed: no lambda tuning, no window
tuning, no preprocessing changes, no FlatSurfacesS re-runs. The likely causes
(the photo Hessian authority vs the degenerate geometry, Eq.8/eta selection, or
intensity-map consistency under a drifting geometry pose) are for the
coordinator to adjudicate.

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

`git diff --stat 2112169..HEAD` — 31 files, +3288 / -84.
`git diff --name-status 2112169..HEAD` — 6 new headers/sources (intensity_processor,
intensity_sampling), 14 tests (8 new), 14 modified + config presets.

## 5. Build Status

| Build | Status |
|---|---|
| Core library (`cmake -S BIEVR -DBIEVR_BUILD_TESTS=ON`, Release) | PASS, 0 warnings |
| ROS1 catkin (`catkin build bievr_lio bievr_ros_common bievr_lio_ros`, Noetic) | PASS, 0 warnings, 0 failed |
| ROS2 wrapper | NOT BUILT (environment is ROS1-only; wrapper untouched by these changes) |

## 6. Test Status

CTest (`-DBIEVR_BUILD_TESTS=ON`): **14/14 pass**

Round-1 tests (6/6):

| Test | Verifies |
|---|---|
| test_intensity_normalization | sparse brightness averages only non-empty pixels (plan §43) |
| test_point_intensity_alignment | point <-> intensity index alignment (plan §44) |
| test_intensity_map_weighted_update | height & intensity share pixel weight (plan §45, Eq. 4-5) |
| test_intensity_map_reprojection | height+intensity co-registered after normal reprojection (plan §46) |
| test_intensity_gradient | Eq. 6 iota direction per gradient pattern (plan §47) |
| test_photometric_jacobian | analytic photometric Jacobian vs 6-DOF finite difference (plan §48) |

Round-2 tests (6 new, second review Section 16):

| Test | Verifies |
|---|---|
| test_projection_collision | collision losers also receive the normalized I_F (no raw leak) |
| test_full_cloud_preservation | filtered size == N and map integrates all N points under collisions |
| test_preprocessing_bypass | preprocessing.enabled=false returns aligned, non-empty clamp(raw*scale) |
| test_misaligned_intensity | size-mismatched intensity rejected; map not contaminated |
| test_score_sign_invariance | Eq.8 abs_components score invariant to eigenvector sign |
| test_intensity_disabled | intensity raster never allocated / info never computed when disabled |

Round-3 tests (2 new):

| Test | Verifies |
|---|---|
| test_bootstrap_shared_weight | intensity-OFF keeps geometry bootstrap; intensity-ON joint init gives ~100 (no zero-history bias); old geometry-only-then-intensity sequence is biased |
| test_config_smoke | params.yaml and params_coin_bievr.yaml both parse completely (with a sensor config) via the ROS1 loader |

## 7. intensity.enabled=false Regression Status

`intensity.enabled: false` must reproduce original BIEVR geometry/map/sampling/LM.

**By construction (verified at code-path level):**
- `pipeline.cpp`: with `intensity.enabled == false`, no intensity preprocessing is
  run, no intensity sampling is run, `photometric_residual` is forced false, the
  geometry-only LsqRegistration constructor is used (no photometric source /
  accumulator), and the map update passes `intensities = nullptr`.
- `bievr_map.cpp`: with `config_.intensity_enabled == false` the voxel never
  allocates/updates/reprojects the intensity raster and never computes intensity
  information (round-2 fix, commit 14); geometry hashing/grouping/normal/bump-
  image math is unchanged (commit 1 is a pure internal representation refactor
  `Vector4d -> MapPoint`, identical arithmetic).
- `ls_optimizer.cpp`: `photometric_residual == false` runs only the original
  geometry linearization; the joint merge reduces to the geometry-only result.
- Geometry sampling (`sampleSource`) is untouched.
- Verified by unit test `test_intensity_disabled` (raster never allocated, info
  never computed while disabled) and the original geometry tests still passing
  with the switch off.

**Residual static overhead when disabled (documented, negligible):** `Voxel`
gains an empty `intensity_img_` (MatrixXf header, no allocation) and a
`Vector2d intensity_information_` (16 B/voxel); `MapPoint` gains one `float`
(4 B) per pending point. No dynamic intensity allocation occurs when disabled.

**Not yet verified on real data:** a runtime A/B trajectory regression
(pose-by-pose diff / ATE, and RSS/peak-memory comparison) requires a recorded
dataset, which is not available in this environment. Recommended: run the same
bag with `intensity.enabled: true/false` and diff the TUM trajectories
(plan §4 Definition of Done / second review Stage B).

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
