# Round 10.7 — COIN-LIO mapping/point_filter_num=4 on TunnelD C1

## Change
Implemented COIN-LIO's `mapping/point_filter_num` semantics for the Ouster group:
`preprocess.point_filter_num` (default 1 = no skip) keeps every (N+1)-th point of
the UNDISTORTED cloud for the GEOMETRY registration + map-update path only. The
intensity preprocessing and sampling keep the FULL cloud (matching COIN-LIO, which
builds its intensity image from the full undistorted cloud). Set to 4 in
config/params_coin_bievr_ouster_enwide.yaml (COIN-LIO ENWIDE value: 1 of every 5
points).

Algorithm math unchanged elsewhere. Avia configs untouched (default 1).

## Results (TunnelD C1, lambda=0.001, Ouster, same evaluator)
| run | threads | APE RMSE | mean/median/max | path(m) | trajectory SHA256 |
|---|---:|---:|---:|---:|---|
| run1 | multi (0) | 1.2704 | 1.18/1.17/1.88 | 173.4 | 7b582e2ecf42a89f... |
| run2 | multi (0) | 0.5891 | 0.56/0.63/0.86 | 176.3 | 31a015af05213fd0... |
| run3 | multi (0) | 168.4711 | 152.7/138.2/274.8 | 590.7 | 96b2c8a640b4a3d8... |
| single1 | 1 | 0.5854 | 0.54/0.62/0.88 | 176.3 | a8ea08e009f813c0... |

Pairwise fork times (run-to-run, no GT):
- run1 vs run2: >0.5m @46.6s, >1m @47.8s, never >5m, end diff ~0
  (same basin; run2 == single-thread basin 0.59 vs 0.585)
- run1/run2 vs run3: >0.5m @35s, >5m @40s, >20m @45s (divergent draw forks at the
  same ~35s as the full-cloud runs)

## Interpretation
- point_filter_num=4 moves the DETERMINISTIC (single-thread) outcome from the
  divergent basin (full-cloud single-thread: 345 m) into the good basin
  (pf4 single-thread: 0.585 m, close to the paper's 0.369).
- The multi-thread good basin also improves from ~6 m (full cloud) to ~0.6-1.3 m
  (pf4). run2 (0.589) matches the deterministic basin almost exactly.
- The parallel run-to-run nondeterminism PERSISTS (1 of 3 multi-thread runs
  diverges, forking at ~35 s as before) - the underlying parallel race/order
  issue is not fixed by the sparse sampling.
- Runtime drops from ~2:40 to ~1:40 per run (5x fewer geometry points).

## Files
- code: BIEVR/include/bievr_lio/preprocess.h (+point_filter_num),
  BIEVR/include/bievr_lio/config_loader.h (load), BIEVR/src/pipeline.cpp (geometry
  subset for registration + map update)
- config: config/params_coin_bievr_ouster_enwide.yaml (point_filter_num: 4)
- results: results/round10_7_pointfilter4/tunneld/{run1,run2,run3,single1}/