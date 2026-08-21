# Round 12 — GEODE Shield post-vector<bool>-fix re-benchmark (summary)

All runs: current HEAD 04717fd (Round11 vector<bool> fix present), Avia frozen
config (lambda 0.001, FOV 77.2, 1024x64, 41x7, 140, raw 1.0, point_filter_num 1),
full bags from t0, strict sequential, independent dirs, official GEODE evaluator
(gamma2GT_leica.py c6e9306 + evo_ape -va t_offset 0 t_max_diff 0.1).

## Results (APE RMSE, m)
| seq | B0 | C0 | C1 (A=B bitwise) | C1 gate | trend | paper BIEVR | paper COIN |
|---|---:|---:|---:|---|---:|---:|
| Shield1 | 0.3946 | 0.4384 | 0.4208 | PASS | MILD_WORSE (+6.6%) | 0.256 | 0.220 |
| Shield4 | 1.8874 | 2.0364 | 2.0483 | PASS | MILD_WORSE (+8.5%) | 0.275 | 0.245 |
| Shield5 | 3305 (DIVERGED) | 2.0162 | 2.0264 | PASS | B0_DIVERGED_C1_STABLE | 0.146 | 0.219 |

## C1-A vs C1-B determinism (all PASS, bitwise)
- Shield1: traj SHA e5921b4a... == e5921b4a..., photo 8b253d03... == ; APE diff 0.0000
- Shield4: traj SHA 43fe6f78... == ; APE diff 0.0000
- Shield5: traj SHA ceeec819... == ; APE diff 0.0000
Pairwise trans P99/max = 0.0000/0.0000 m on all three (rotation P99 ~0.17 deg is
the expected quaternion dot-approximation of identical poses).

## Old (pre-fix) vs new (post-fix) C1
| seq | old C1 (PRE_FIX / NOT_AUTHORITATIVE) | new C1 (POST_FIX, authoritative) | delta |
|---|---:|---:|---:|
| Shield1 | 0.4166 | 0.4208 | +0.0042 |
| Shield4 | 1.7772 | 2.0483 | +0.2711 (race-affected old value) |
| Shield5 | 2.0367 | 2.0264 | -0.0103 |

## C0 side-effect vs B0
- Shield1: +11.1% -> PIPELINE_LARGE_SIDE_EFFECT
- Shield4: +7.9% -> PIPELINE_SIDE_EFFECT
- Shield5: B0 diverged; C0 stable (2.016)

## Interpretation
- The vector<bool> race fix makes the Avia photometric path bitwise deterministic
  (C1-A == C1-B on all three Shields).
- Post-fix photometric trend: Shield1/4 are MILD_WORSE (photo slightly degrades
  vs current-head B0), i.e. the paper's "photo improves on Shield1/4" is NOT
  reproduced post-fix; Shield5 C1 is stable while the current-head B0 diverges
  (baseline regression, known and NOT fixed this round; original public SHA
  Shield5 B0 = 2.04 m stable).
- Shield4 old C1 1.7772 (pre-fix) is superseded by 2.0483 (post-fix): the race
  materially changed Shield4's photometric score.

## FlatSurfacesS (added post-hoc to the Round12 post-fix matrix)

Bag: /home/lc/algorithm_versa/bag/ENWIDE/flat_surfaces_smooth.bag (82 s, 821 frames,
Avia/gamma), GT flat_surfaces_smooth.txt. Evaluator: evo_ape tum -a trans_part
t_max_diff 0.1 offset 0 (Round8/9 semantics).

| run | APE RMSE | mean/median/max | notes |
|---|---:|---:|---|
| B0 | 1.5232 | 1.37/1.29/3.48 | identical to R9 (geometry deterministic) |
| C0 | 1.6547 | 1.54/1.56/3.21 | identical to R9 |
| C1-A | 0.0595 | 0.051/0.049/0.29 | traj SHA 6920bc2b... |
| C1-B | 0.0595 | 0.051/0.049/0.29 | traj+photo SHA equal -> gate PASS |

- C1 determinism: PASS (bitwise).
- Pre-fix R9 C1 was 0.0625 m; post-fix deterministic C1 = 0.0595 m (paper 0.064 m
  reproduced, now deterministically).
- Trend: CLEAR_IMPROVEMENT (C1 0.0595 vs B0 1.5232, ~96% better) - the FlatSurfaces
  photometric rescue is CONFIRMED post-race-fix and deterministic.
