# TunnelD C1 run-to-run A/B dataset

Strictly copied from the originals (source files untouched). Two groups, all with
the SAME byte-identical algorithm config (BASE_CFG SHA256 be7d3e41...; output
paths excluded, normalized config identical across all runs).

All runs: TunnelD (119 s, 1189 frames), COIN-BIEVR Ouster, photo ON, lambda=0.001,
max_num_threads=0 (multi-thread unless noted), same evaluator semantics.

## old_sequential/  (Round-10.5, 3x sequential multi-thread, idle machine)
- run1: APE 0.4716 m  (trajectory SHA ab33ee5c...)
- run2: APE 91.4185 m (SHA 8a06ddfe...)
- run3: APE 399.1063 m (SHA b9313280...)
Each run: trajectory.tum, photo.csv, config_used.yaml (+ hashes/timing/logs).

## new_coldcache/  (Round-10.6, manual protocol: fresh process, independent dirs,
hard process barrier + sync before each run; drop_caches NOT executed - protocol
deviation, page cache not actually dropped)
- run1: APE 6.1173 m  (SHA 8649eef5...)
- run2: APE 5.7484 m  (SHA 49390ef3...)
- run3: APE 159.9156 m (SHA bc9a5c4e...)
Each run: trajectory.tum, photo.csv, config_used.yaml, environment_before.txt,
environment_after.txt, timing.txt, hashes.txt, evaluation.txt.

## old_extra/  (additional arms, kept for completeness; NOT part of the core A/B)
- single1 / single2: Round-10.5 single-thread (max_num_threads=1), BITWISE
  identical (SHA 4efd2d4f...), APE 344.97 m (diverged basin)
- round10_C1_sequential_7p26: Round-10 run (7.26 m, SHA ad69aff7...)
- round10_C1_solo_96p45: Round-10 solo run (96.45 m, SHA 063d0819...)

## Lost runs (overwritten at the time; cannot be reconstructed)
- Round-10 concurrent C1 (0.596 m) - directory reused by the sequential run
- Round-10 solo C1 #1 (11.13 m) - directory reused by solo #2

## Key observation for first-divergence analysis
All runs share identical timestamps (1185 poses, t0=...32.36). Within-group and
cross-group first-divergence times are the natural run-to-run analysis signal
(no GT required). Note old_sequential/run1 and new_coldcache runs 1-2 land in a
bounded ~0.5-6 m basin; old run2/run3 and new run3 diverge (91-399 m).