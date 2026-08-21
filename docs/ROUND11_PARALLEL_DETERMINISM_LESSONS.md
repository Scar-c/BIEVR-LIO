# Round 11 — Parallel Determinism Lessons

Documentation of the `std::vector<bool>` packed-bit race found and fixed in the
COIN-BIEVR reproduction's intensity informed-sampling path, and the engineering
rules it establishes for any future parallel work in this project.

> **This was NOT an original BIEVR-LIO bug.**
>
> **This was NOT a COIN-BIEVR mathematical-design bug.**
>
> **This was an implementation bug introduced in our reproduction's newly added
> parallel photometric informed-sampling path.**

---

## 1. Symptom

TunnelD C1 (Ouster, lambda=0.001) showed catastrophic run-to-run nondeterminism
with an identical binary, identical config and identical bag:

```text
sequential multi-thread (Round10.5): 0.47 / 91.4 / 399.1 m
manual cold-cache protocol (Round10.6): 6.1 / 5.7 / 159.9 m
single-thread (Round10.5): 344.97 / 344.97 m (bitwise identical)
```

The divergent runs all forked from the good basin at the same ~34-37 s point in
the sequence. The trajectory SHA256 differed on every multi-thread run.

## 2. False leads

- **CPU contention**: a concurrent run (0.596 m) was first suspected. Strict
  sequential re-runs still showed the full spread -> contention is a trigger /
  scheduling condition, not the root cause.
- **Filesystem page cache / shared output state**: a manual cold-cache protocol
  (independent dirs, fresh processes, hard barriers, sync) did not remove the
  spread (one of three runs still diverged). Not the root cause.
- **Geometry stride** (point_filter_num=4): moving to COIN-LIO's 1-of-4 geometry
  stride moved the DETERMINISTIC basin from 345 m to 0.585 m, and the multi-thread
  good basin from ~6 m to ~0.6 m, but the parallel nondeterminism persisted
  (1 of 3 runs still diverged). A real improvement, but not the nondeterminism
  root cause.

## 3. Root cause

`sampleIntensityPoints()` in `BIEVR/src/intensity_sampling.cpp` used

```cpp
std::vector<bool> observed_flag(undistorted.size(), false);
tbb::parallel_for(tbb::blocked_range<size_t>(0, undistorted.size()),
                  [&](const tbb::blocked_range<size_t>& r) {
                    for (size_t i = r.begin(); i != r.end(); ++i) {
                      ...
                      observed_flag[i] = map.getVoxel(hash) != nullptr;
                    }
                  });
```

`std::vector<bool>` is a **bit-packed proxy container**. Different logical
indices that live in the same 64-bit storage word are written by different
workers as read-modify-write on that shared word -> **data race / undefined
behaviour**, even though the logical indices are disjoint.

Propagation chain:

```text
observed_flag race
  -> observed voxel set nondeterministic
  -> weak-direction / eta nondeterministic
  -> Eq.8 voxel score nondeterministic
  -> top selected intensity voxels nondeterministic
  -> photometric candidates / matches nondeterministic
  -> H_photo / b_photo nondeterministic
  -> small pose differences
  -> recursive map/state update
  -> TunnelD enters different basins at the ~35 s strongly-degenerate section
```

Final classification after the one-line fix (vector<bool> -> vector<uint8_t>):
**ROOT_CAUSE_CONFIRMED** — three multi-thread runs became bitwise identical to
each other AND to the single-thread reference (trajectory SHA a8ea08e0...,
APE 0.5854 m on TunnelD pf4).

## 4. Why BIEVR design was referenced but implementation pattern was not reused enough

Our intensity informed sampling is algorithmically isomorphic to BIEVR's original
map-informed sampling:

```text
undistorted cloud
  -> map voxel association
  -> voxel score
  -> top-K / informed selection
  -> registration / photometric optimization
```

The intended difference was only the **scorer** (BIEVR: geometry-informed /
MID score; COIN-BIEVR: weak-direction / Eq.8 / intensity-informed score).

The implementation, however, was written as a fresh standalone module
(`sampleIntensityPoints()`) for legitimate module-isolation reasons (so the
intensity OFF path keeps original BIEVR behaviour). When writing the fresh
module, the established parallel engineering patterns of the original BIEVR
sampling path were not reused or mimicked:

```text
preallocated ordinary POD/vector entries
+ each worker writes an independent object
+ deterministic grouping/selection
```

Instead, a `std::vector<bool>` was used as a parallel-written mask. The lesson:
**when the data flow is isomorphic to an existing module, reuse or strictly
mimic its parallel ownership, container types, sorting/grouping and
deterministic-ordering patterns, not just its formulas.**

## 5. Why std::vector<bool> is unsafe here

- It is a proxy container: `operator[]` returns a proxy, not a real reference.
- Elements are packed 1 per bit; adjacent logical indices share a storage word.
- Two threads writing different logical indices in the same word perform
  read-modify-write on the same bytes -> race even with disjoint indices.
- The race is timing-dependent and typically silent: unit tests and even full
  runs usually pass, but occasionally produce a corrupted flag -> a subtly wrong
  observed-voxel set.

Parallel-writable masks must use byte-addressable storage:

```cpp
std::vector<uint8_t> observed_flag(n, 0u);   // 0u / 1u
std::vector<char>    mask(n, 0);
// or ordinary POD struct vectors
```

## 6. How the single-thread oracle exposed the issue

- Round10.5: two single-thread runs were **bitwise identical** (344.97 m), while
  multi-thread runs spanned 0.47-399 m. A deterministic serial reference is the
  decisive discriminator between "algorithm behaviour" and "parallel execution
  corruption".
- After the geometry stride (pf4): single-thread 0.585 m (bitwise deterministic)
  became the reference for the fix.
- After the vector<bool> fix: multi-thread runs matched the single-thread
  trajectory **bitwise** (SHA identical, pairwise max trans diff 0.000e+00 m).

Rule: for any newly added parallel path, unit tests / build PASS are not enough;
a **serial-vs-parallel semantic parity test** is mandatory.

## 7. Why point_filter_num=4 changed the deterministic basin

COIN-LIO's `mapping/point_filter_num=4` (stride 4: retained indices 0,4,8,...,
ratio 0.25) applied to the geometry registration + map-update path:

```text
full-cloud  single-thread: 344.97 m   (divergent deterministic basin)
pf4         single-thread:   0.585 m  (good deterministic basin)
```

The stride changes the map point set and the registration input, which changes
the deterministic trajectory solution itself. It does NOT fix the parallel race
(1 of 3 multi-thread runs still diverged pre-fix). It is a separate,
legitimate improvement that moved the deterministic solution into the sub-meter
basin; the race fix then made multi-thread reproduce that basin bitwise.

## 8. Why one-variable repair was essential

The successful attribution used a strict protocol:

```text
one candidate
one patch
one deterministic validation
```

Only `std::vector<bool>` was changed. Other theoretical parallel risks
(`parallel_sort` tie-breaks, unordered_map iteration, map-integration ordering,
TBB reduction internals) were explicitly NOT touched. The result was immediate
and unambiguous: multi == single bitwise. Touching several candidates at once
would have made the root cause un-attributable.

## 9. New engineering rules

- **Rule 1**: `std::vector<bool>` is forbidden as a parallel-writable mask.
  Use `std::vector<uint8_t>` / `std::vector<char>` / ordinary POD struct vectors.
- **Rule 2**: if a new feature's data flow is `point -> voxel association ->
  score -> select`, first reuse / abstract / strictly mimic the existing BIEVR
  sampling implementation's parallel ownership, container types, sorting and
  deterministic ordering. Do not re-invent the infrastructure from formulas.
- **Rule 3**: every new parallel sampling/accumulation path must ship a serial
  fixture and a parallel fixture and compare selected IDs, voxel hashes,
  residual counts, H, b, and trajectory-level output where practical.
- **Rule 4**: before long-sequence benchmarks, run a repeated-run determinism
  gate (same bag, same config, same binary, multi x >= 2, bitwise identical or
  numeric thresholds PASS). Otherwise the benchmark is not admissible.
- **Rule 5**: distinguish "trigger / scheduling condition" (CPU load, cache
  state) from "algorithm implementation correctness". A race can be triggered
  by load but is fixed only by fixing the code.
- **Rule 6**: one variable per repair; validate determinism after each single
  change; never batch-repair parallel candidates.

## 10. Benchmark invalidation policy

- Old Round10 Shield C1 values were generated while the `std::vector<bool>`
  race existed in the intensity sampling path:

```text
Shield1 C1 = 0.4166   PRE_FIX / NOT_AUTHORITATIVE
Shield4 C1 = 1.7772   PRE_FIX / NOT_AUTHORITATIVE
Shield5 C1 = 2.0367   PRE_FIX / NOT_AUTHORITATIVE
```

- They are retained only as historical/pre-fix references and must not appear
  in the final paper-comparison main table.
- Round12 post-fix values become authoritative only if the C1-A vs C1-B
  duplicate determinism gate PASSES (trajectory SHA equal or numeric thresholds).
- The same policy applies to any future benchmark generated before a
  determinism gate.