# Parallel Architecture Refactor & Multimodal Voxel Design Audit

Design audit of the COIN-BIEVR reproduction's parallel architecture and a
unified surface-centric voxel proposal for geometry / LiDAR-intensity / future
camera layers. No core code is changed by this document.

Repository state audited: HEAD 93069a00 (includes the Round-11 `std::vector<bool>`
race fix; FlatSurfacesS post-fix rerun f40a433 is an ancestor).

---

## 1. Scope

- Current geometry (BIEVR height/bump) data flow, current intensity (COIN-BIEVR)
  data flow, and the points where they share or duplicate infrastructure.
- Voxel / map data structures and their memory ownership.
- Parallel ownership table for every point->voxel / voxel-update / sampling path.
- Height/intensity co-registration status (surface frame, pixel correspondence).
- Surface-frame update/reprojection lifecycle.
- A candidate `SurfaceVoxel` (height reference + intensity layer + future visual
  layer), shared point->voxel association, deterministic grouping, scorer
  abstraction, point-parallel vs voxel-parallel comparison.
- Minimal-change and clean-architecture variants, one recommendation.
- Migration phases R1-R5 with rollback boundaries and regression gates.

## 2. Current geometry pipeline

```text
raw LiDAR (CustomMsg / PointCloud2)
  -> conversions.h msgToPointcloud      (parallel per-point parse; PER_INDEX_EXCLUSIVE)
  -> StampedIntensityPointcloud (6-row: xyz, time, intensity, ring)
  -> filterMinMaxRange (preprocess.h)   (parallel range test -> serial index gather)
  -> filtered_L
  -> transformPoints(T_I_L)             (single 3xN write)
  -> undistortCloud                     (IMU deskew)
  -> points_undistorted_I + ranges      (index-aligned)
  -> point_filter_num stride            (geometry+map subset, index-aligned triples)
  -> sampleSource:
       voxelDownsample                  (parallel hash entries -> parallel_sort(hash,idx)
                                         -> serial group gather)
       sampleInformed                   (parallel hash -> parallel_sort(hash,idx)
                                         -> serial unique-voxel walk -> parallel score
                                         (map READ) -> partial_sort by score -> gather)
  -> source_filtered / coarse / fine
  -> LsqRegistration::linearizeGeometry (parallel_deterministic_reduce; map READ only,
                                         per-point POD accumulation)
  -> map integration (BIEVRMap::integratePoints):
       parallel hash + parallel_sort    (POD hashed_points, tie (hash,x))
       serial group extraction          (voxel insert on hash change)
       parallel per-voxel update        (PER_VOXEL_EXCLUSIVE: updateNormal,
                                         updateBumpImage -> integratePoints(MapPoint),
                                         computeScore)
  -> voxel bump_img_ / bump_smoothed_ / bump_weights_ update
```

## 3. Current intensity pipeline

```text
full undistorted cloud (points_undistorted_I) + filtered_intensity
  -> intensity preprocessing (IntensityProcessor::process)
       spherical or ouster_lut projection, per-point parallel loop (PER_INDEX_EXCLUSIVE)
       ring-based image build (Ouster), line removal, brightness, back-assignment
  -> sampleIntensityPoints:
       parallel hash entries            (POD VoxelHashIdx)
       parallel observed test           (std::vector<uint8_t> observed_flag - FIXED;
                                         was std::vector<bool>, Round-11 race)
       parallel_sort (hash, idx)        (strict total order)
       serial observed_hashes walk
       estimateWeakGeometryDirection    (A = sum n n^T, eigen - serial, Eq.7)
       scoreIntensityVoxels             (serial over observed hashes; Eq.8 abs_components)
       selectTopIntensityVoxels         (std::partial_sort by score - ties unstable)
       downsampleIntensityPoints        (parallel hash -> parallel_sort(hash, dist))
  -> intensity source points + intensities (IMU frame, 0.1 m)
  -> LsqRegistration::linearizePhotometric (shared per-point helper; serial diagnostics +
       parallel production via parallel_deterministic_reduce; map READ only)
  -> joint LM (geometry + photo H/b)
  -> intensity map update (same integratePoints; intensity_img_ updated with the
       SAME pixel weights as the height map)
```

## 4. Shared vs duplicated infrastructure

| Item | Geometry path | Intensity path | Map update | Classification |
|---|---|---|---|---|
| point->voxel hash | sampleInformed (preprocess.cpp) | sampleIntensityPoints (intensity_sampling.cpp) | integratePoints (bievr_map.cpp) | LOGICALLY_EQUIVALENT_BUT_DUPLICATED (3 separate parallel hash loops over the same cloud) |
| sort / grouping | parallel_sort(hash,idx) + serial walk | parallel_sort(hash,idx) + serial walk | parallel_sort(hash,x) + serial group | LOGICALLY_EQUIVALENT_BUT_DUPLICATED |
| voxel surface frame (T_C_W_/T_O_W_) | owned by Voxel | same Voxel | same Voxel | EXACT_SHARED |
| pixel grid / resolution | bump_img_ (px_size) | intensity_img_ (same dims) | shared bump_weights_ mask | EXACT_SHARED |
| pixel correspondence (u,v) | height(u,v) | I_lidar(u,v) | weights(u,v) | EXACT_SHARED (1:1) |
| reprojection on frame change | reprojectImage (bump) | rides along with the surface | - | EXACT_SHARED (intensity reprojected together) |
| scoring / selection | MID-like mean_img_dist_ (sampleInformed) | Eq.8 intensity score (sampleIntensityPoints) | - | INDEPENDENT (two separate scorers + two separate select/top-K paths) |
| association buffer types | VoxelHashIdx | VoxelHashIdx | hashed_points | LOGICALLY_EQUIVALENT_BUT_DUPLICATED |

## 5. Current voxel / map data structures

| Struct | Fields | Ownership / lifetime | Write owner | Copyable? |
|---|---|---|---|---|
| Voxel | observed_, T_C_W_, T_O_W_, bump_img_, bump_smoothed_, bump_weights_, intensity_img_, intensity_information_, outer_sum_, sum_, num_points_, mean_img_dist_, pending_points_ | persistent (owned by VoxelEntry in map_) | the per-voxel map-update task (exclusive) | copyable (Eigen matrices), heavy (~tens of KB once observed) |
| VoxelEntry | Voxel + lru_it | persistent | serial LRU splice + per-voxel task | - |
| BIEVRMap | map_ (ankerl::unordered_dense::map<size_t,VoxelEntry>), voxels_cache_ (std::list), corner_offsets_, neighbor_offsets_ | persistent | serial insertion; parallel phase reads map_ only | - |
| Height layer | bump_img_ (Eigen::MatrixXf, HxW), bump_smoothed_ | persistent per voxel | per-voxel task | - |
| Shared weight/mask | bump_weights_ (Eigen::MatrixXf, HxW) | persistent | per-voxel task | - |
| Intensity layer | intensity_img_ (Eigen::MatrixXf, HxW), intensity_information_ (Vector2d) | persistent (allocated only when intensity_enabled) | per-voxel task | - |
| PointVoxelHashIdx (geometry) | hash + idx | transient per frame | PER_INDEX_EXCLUSIVE | - |
| PointVoxelHashIdx (intensity) | hash + idx | transient per frame | PER_INDEX_EXCLUSIVE | - |
| hashed_points (map) | p_W + range + intensity | transient per frame | PER_INDEX_EXCLUSIVE | - |
| DownsampleEntry | hash + idx + dist | transient per frame | PER_INDEX_EXCLUSIVE | - |
| weak-direction structs | (in sampleIntensityPoints result) | transient per frame | serial | - |

## 6. Parallel ownership audit

| Region | Parallel unit | Reads | Writes | Write ownership | Shared mutable state | Deterministic? |
|---|---|---|---|---|---|---|
| msgToPointcloud parse | point | msg buffer | cloud col(i) | PER_INDEX_EXCLUSIVE | none | yes |
| filterMinMaxRange test | point | msg/cloud | std::vector<char> mask | PER_INDEX_EXCLUSIVE (char, disjoint) | none | yes |
| transformPoints / undistort | - (single write) | cloud | cloud | serial | - | yes |
| intensity preprocessing | point | cloud | point_pixel_idx / filtered | PER_INDEX_EXCLUSIVE | none | yes |
| voxelDownsample | point | cloud | entries[i] | PER_INDEX_EXCLUSIVE | none | yes |
| sampleInformed hash | point | cloud, map (read) | entries[i] | PER_INDEX_EXCLUSIVE | map read-only | yes |
| sampleInformed score | voxel | map (read) | scores[i] | PER_INDEX_EXCLUSIVE | map read-only | yes |
| sampleIntensityPoints hash+observed | point | cloud, map (read) | entries[i], observed_flag[i] | PER_INDEX_EXCLUSIVE (uint8_t) | map read-only | yes (post-fix) |
| intensity downsample | point | candidates | DownsampleEntry[i] | PER_INDEX_EXCLUSIVE | none | yes (sort key below) |
| map integratePoints hash+sort | point | cloud | hashed_points[i] | PER_INDEX_EXCLUSIVE | none | yes |
| map per-voxel update | voxel | its own group | its own Voxel | PER_VOXEL_EXCLUSIVE | map_ read-only in parallel phase | yes |
| linearizeGeometry | point | map (read) | accumulator reduce | REDUCTION (deterministic) | map read-only | yes |
| linearizePhotometric | point | map (read) | accumulator reduce | REDUCTION (deterministic) | map read-only | yes |
| forEachVoxel / debug publish | voxel | map | output | serial iteration | unordered_map iteration order | LATENT (output-only) |

## 7. Determinism risks

| Risk | Location | Classification |
|---|---|---|
| std::vector<bool> packed-bit RMW | sampleIntensityPoints observed_flag | ACTIVE_BUG -> FIXED (uint8_t) |
| parallel_sort equal keys without tie-break | downsampleIntensityPoints (hash, dist) - equidistant points in the same downsample voxel | LATENT_RISK |
| top-K partial_sort ties | selectTopIntensityVoxels (score only; no secondary key) | LATENT_RISK |
| unordered_map iteration | forEachVoxel / debug publishing | LATENT_RISK (output-only, never feeds the estimator) |
| floating reduction order | parallel_deterministic_reduce | SAFE_BY_OWNERSHIP (TBB deterministic reduction) |
| map_ mutation vs parallel read | all parallel phases read map_ only (insertions happen in the serial group pass) | SAFE_BY_OWNERSHIP |

## 8. Surface-frame lifecycle (current)

- `updateNormal`: recomputes the voxel plane (covariance eigen) when the normal
  angle exceeds `norm_tol_deg` (3 deg). The voxel is initialized (observed) once
  `num_points_ >= 4`.
- On a normal change, `updateBumpImage` -> `reprojectImage`:
  - computes the new image bounds from the voxel corners,
  - lifts every valid pixel to 3D with the OLD bump value and re-projects it into
    the NEW frame (`T_C1_C0`),
  - **intensity rides along with the (u,v,height) surface pixel** (the intensity
    is never used for the 3D lift) - so the intensity layer and the height layer
    always share the SAME frame and SAME pixel correspondence,
  - weights are re-projected with the same transform.
- Frequency: unbounded (every time the normal drifts > 3 deg); each reproject is
  O(pixels) and touches bump + intensity + weights.

## 9. Height/intensity co-registration (current)

1. Intensity image is fully attached to the BIEVR voxel's local surface frame:
   YES (EXACT_SHARED; same `T_C_W_`, same `T_O_W_`, same origin convention).
2. Intensity pixel (u,v) == bump/height pixel (u,v): YES (same `bump_weights_`
   grid, same dimensions).
3. Same voxel origin / normal / tangent basis / pixel size / image dimensions:
   YES - all derived from the single `T_C_W_` / `T_O_W_` and `px_size`.
4. On frame rotation/update: height and intensity are re-projected together in
   `reprojectImage` (intensity rides along the surface).
5. Independent coordinate system / projection for intensity: NO for the MAP
   raster (EXACT_SHARED). The per-frame Ouster intensity PREPROCESSING uses its
   own ring-based projection, but that is an input-side transform; the map layer
   is co-registered.
6. Shared vs duplicated: the map-layer attachment is EXACT_SHARED; the per-frame
   preprocessing and the association/sampling infrastructure are duplicated
   (see section 4).

## 10. Memory layout

Persistent per voxel (2 m voxel -> 41x41 px @ 0.05 m observed image):
- T_C_W_ + T_O_W_: 128 B
- bump_img_ + bump_smoothed_ + bump_weights_: 3 x 6.7 KB = 20.2 KB
- intensity_img_ (when enabled): 6.7 KB
- intensity_information_: 16 B; outer_sum_ 72 B; sum_ 24 B; counters ~16 B
- pending_points_: grows before maturity (transient, released after observed)
- Total observed+intensity voxel: ~27 KB.

Transient per frame (Ouster ~126k points):
- hashed_points: 126k x ~48 B = ~6 MB
- sampleInformed entries: 126k x 16 B = ~2 MB
- sampleIntensityPoints entries + observed_flag: ~2 MB + 126 KB
- DownsampleEntry: ~16 B x candidates
- intensity source / residual buffers: small
- Total transient: ~10 MB, allocated and freed every frame.

Future visual layer (est.): a patch/descriptor per surface pixel + view metadata
(see section 15) - estimated +4-8 KB per voxel with a reference view (LOW/MEDIUM
estimate; view-dependent references would multiply this - see risks).

## 11. Proposed SurfaceVoxel (candidate, two variants)

MINIMAL-CHANGE variant (keeps the current Voxel layout, adds explicit layer
grouping via comments/namespaces and a small VisualLayer slot):

```cpp
struct Voxel {
  // GeometryState - authoritative surface frame
  Transform T_C_W_, T_O_W_;
  M3 outer_sum_; V3 sum_; size_t num_points_; bool observed_;

  // HeightLayer - reference surface image (geometry)
  Eigen::MatrixXf bump_img_, bump_smoothed_;

  // SharedLayer - validity/weight mask (height + intensity + future visual)
  Eigen::MatrixXf bump_weights_;

  // IntensityLayer - co-registered radiometric layer
  Eigen::MatrixXf intensity_img_;
  Eigen::Vector2d intensity_information_;
  double mean_img_dist_;

  std::vector<MapPoint> pending_points_;
  // future: std::optional<VisualLayer> visual_;
};
```

CLEAN-ARCHITECTURE variant (new aggregate; layers as structs, all sharing the
GeometryState frame):

```cpp
struct GeometryState { Transform T_C_W_, T_O_W_; M3 outer_sum_; V3 sum_;
                       size_t num_points_; bool observed_; };
struct HeightLayer   { Eigen::MatrixXf image, smoothed; };
struct SharedMask    { Eigen::MatrixXf weights; };   // validity for all layers
struct IntensityLayer{ Eigen::MatrixXf image; Eigen::Vector2d information; };
struct VisualLayer   { ... view-dependent reference (future) ... };
struct SurfaceVoxel  { GeometryState geo; HeightLayer height; SharedMask mask;
                       IntensityLayer intensity; std::optional<VisualLayer> visual; };
```

Principle: `GeometryState` is the authoritative frame; `height(u,v)`,
`I_lidar(u,v)` and (future) `I_cam(u,v,view)` are all defined in it.

## 12. Shared point-to-voxel association

One association pass over the (undistorted, IMU-frame) cloud reused by geometry
sampling, intensity sampling and map integration:

```cpp
struct SharedPointVoxelAssociation {
  size_t hash;         // voxel hash (primary key)
  uint32_t point_idx;  // original index (secondary key, deterministic tie-break)
  float local_u, local_v, local_h;  // voxel-local surface coordinates (computed
                                    // only for the selected subset, or on demand)
  uint8_t flags;       // observed / in-range / intensity-valid ...
};
```

- Produced once per frame by a parallel point loop (PER_INDEX_EXCLUSIVE POD
  writes), sorted with (hash, point_idx) - a strict total order.
- Consumers: sampleSource (geometry), sampleIntensityPoints (observed test, weak
  direction), map integration (grouping), and future visual association.
- The per-layer workloads then run from the SAME sorted buffer (section 16).

## 13. Deterministic grouping

- Group key: `(hash, point_idx)`; the sort is stable in the sense of a strict
  total order (no equal keys), so grouping by hash produces a deterministic
  per-voxel point list.
- Top-K / selection secondary keys: score, then `hash`, then `point_idx` (add a
  tie-break to `selectTopIntensityVoxels`; add `idx` to the downsample
  comparator).
- The current `parallel_sort` comparators in sampleInformed and
  sampleIntensityPoints already use (hash, idx); the downsample comparator
  (hash, dist) and the top-K partial_sort need explicit tie-breaks (LATENT_RISK).

## 14. Scorer abstraction

```cpp
struct VoxelScorer {
  virtual double score(const SurfaceVoxel& voxel, const Point& point_w,
                       const Eigen::Vector3d& eta_world) const = 0;
};
struct GeometryInformedScorer : VoxelScorer { ... };   // MID-like (mean_img_dist_)
struct Eq8IntensityScorer  : VoxelScorer { ... };      // COIN-BIEVR Eq.8
struct CompositeScorer      : VoxelScorer { ... };     // future (geometry + photo + visual)
```

Both current scorers then share one candidate/group/select infrastructure
(voxel-scored entries -> deterministic sort -> top-K) instead of the two
independent samplers (`sampleInformed` and `sampleIntensityPoints`).

## 15. Future camera layer (interface only, no residual) [NOT CURRENT ROADMAP]

- Single camera first; stereo is an optional future extension (never required by
  the voxel infrastructure).
- Camera model abstraction (no fx/fy/cx/cy in the voxel core):
  `project(point_C) -> uv`, `unproject(pixel) -> dir_C`, `jacobian(...)`; Pinhole
  and equidistant/fisheye allowed.
- Per surface-voxel view-dependent metadata: T_C_L / T_C_I, timestamp, projection
  model id, image pyramid level, exposure/gain, reference view direction,
  surface visibility flag. Placement: a `VisualLayer` attached to the shared
  mask pixels (view-dependent references; see memory/risk notes).

## 16. Point-parallel vs voxel-parallel comparison

| Item | point-parallel | voxel-parallel |
|---|---|---|
| hash cost | one hash per point (cheap, cache-friendly) | needs grouping first (hash still per point) |
| cache locality | sequential over points (good) | grouped by voxel (scattered points) |
| write contention | PER_INDEX_EXCLUSIVE (none) | PER_VOXEL_EXCLUSIVE (none if one task per voxel) |
| height update | not applicable (map writes are per voxel) | per-voxel image update (natural) |
| intensity update | not applicable | per-voxel image update (natural) |
| visual extension | not applicable | per-voxel reference update (natural) |
| deterministic ordering | sort by (hash,idx) | same sort, then grouped |
| memory overhead | + one POD entry per point | + per-voxel work package |

Recommendation: **HYBRID** - point-parallel stage-1 (hash + POD association,
read-only map) followed by voxel-parallel stage-3 (one task per voxel with
exclusive ownership). Pure voxel-parallel would need a grouping pre-pass that
still hashes per point.

## 17. Minimal-change architecture (variant A)

- Keep the current `Voxel` layout and the three call sites, but:
  - extract the per-frame point->voxel hash+sort into one shared helper
    (`buildPointVoxelAssociation`) used by sampleSource / sampling / map
    integration (same output as today; algorithm-identical),
  - add tie-breaks to the downsample comparator and the top-K partial_sort,
  - keep `reprojectImage` as the single frame-lifecycle owner for height +
    intensity.
- Pros: small diff, per-phase rollback, no behavior change expected; directly
  removes the duplicated association loops.
- Cons: still three group/select implementations; scorer abstraction is added
  later (R3).

## 18. Clean-architecture variant (variant B)

- `SurfaceVoxel` aggregate with explicit layers (section 11), one shared
  association + one shared grouping, scorer interface, and a `VisualLayer` slot.
- Pros: single ownership model; future visual integration is a natural extension.
- Cons: larger refactor; must pass the same regression gates at every phase;
  higher transient-change risk.

## 19. Recommended architecture

**Recommendation: MINIMAL-CHANGE first (R1-R2), then the scorer abstraction
(R3) and only the `VisualLayer` interface slot (R4).** I.e., migrate toward the
clean architecture in stages, with the current Voxel layout as the baseline and
`SurfaceVoxel` as the R2 target for the layer grouping:

1. shared association: YES (one `buildPointVoxelAssociation`, reused by
   geometry sampling, intensity sampling, map integration);
2. grouping: one sorted buffer + a shared "unique voxel groups" walk used by all
   consumers;
3. parallel unit: HYBRID (point-parallel stage 1 + voxel-parallel stage 3);
4. height/intensity ownership: one `SharedMask` + one frame lifecycle
   (`reprojectImage` moves all layers);
5. frame policy: HYBRID - freeze after maturity (see section 20);
6. scorer abstraction: yes (R3), `GeometryInformedScorer` + `Eq8IntensityScorer`
   over one select/top-K path;
7. camera layer: a `VisualLayer` interface slot only (R4), no residual;
8. memory: persistent voxel stays ~27 KB; transient association buffers become a
   reusable frame-local pool (section 10).

## 19b. R1 implementation status (Phase 14, DONE)

- Extracted `buildPointVoxelAssociations` + `sortPointVoxelAssociations`
  (PointVoxelAssociation {hash, point_idx}) in
  BIEVR/include/bievr_lio/point_voxel_association.h; migrated sampleInformed,
  sampleIntensityPoints and findObservedVoxels; integratePoints DEFERRED
  (different sort key (hash, x) + MapPoint metadata - changing it would alter
  behavior).
- Comparator/tie-break semantics unchanged; no new locks/atomics.
- Bitwise parity confirmed on FlatSurfacesS C1, TunnelD pf4 C1 and Shield1 C1
  (trajectory + photo.csv SHA identical); runtime/RSS unchanged.
- R1b (total-order hardening): NOT STARTED.

## 19c. R2 implementation status (Phase 15, DONE)

Voxel restructured with explicit co-registered layers: HeightLayer
{bump_img_, bump_smoothed_} and IntensityLayer {intensity_img_,
intensity_information_}; surface frame, shared bump_weights_ mask, observed
flag and geometry statistics stay at Voxel level (one voxel, one authoritative
surface frame, multiple co-registered LiDAR layers). Pure member relocation -
matrix types, dimensions, initial values and allocation timing unchanged;
reprojectImage still moves height + intensity + weights together with height
as the 3D lift quantity.

Roadmap update: camera / VisualLayer extension is OUT OF CURRENT SCOPE. The
current architecture target is geometry + LiDAR intensity only. The former
R4/R5 visual phases are NOT PLANNED IN CURRENT ROADMAP.

## 19d. R3 implementation status (Phase 16, DONE)

Shared scored-candidate / selection infrastructure extracted
(scored_voxel_selection.h): shared ScoredVoxelCandidate type (replaces the
geometry VoxelScore and the intensity pair-based scores with identical values
and field order), shared unique-voxel walk (collectUniqueVoxels over the R1
association buffer, used by all three sampling-side walks), shared descending
comparator (score > score, no tie-break) and the two selection primitives kept
semantically distinct: sortScoredVoxelsDescending (geometry full sort: the
sorted tail feeds the coarse set) and selectTopKScoredVoxels (intensity
partial_sort top-K). Eq.8 kernel exposed as the pure shared function
intensityDirectionalScore with unchanged math; MID stays path-local (field
read + eligibility). No mega-candidate struct, no virtual machinery, no
threading change. 26/26 tests PASS (added T1-T5 scorer/selection parity).

CORE_ARCHITECTURE_REFACTOR_CLOSED: R1 (association) + R2 (ownership/lifecycle)
+ R3 (candidate/score/select) all complete. Further work is
profiling-driven optimization only. Camera/LIVO: OUT OF SCOPE.

Bitwise parity verified on FlatSurfacesS B0 (intensity-OFF), FlatSurfacesS C1,
TunnelD pf4 C1 and Shield1 C1 (trajectory + photo.csv SHA identical);
runtime/RSS unchanged.

## 20. Migration phases R1-R5

**R1 - No-behavior-change infrastructure extraction** (single commit group):
extract `buildPointVoxelAssociation` (parallel hash + (hash,idx) sort) and the
unique-voxel walk; make sampleInformed / sampleIntensityPoints / integratePoints
consume it.
- Parity gate: FlatSurfacesS trajectory SHA unchanged; TunnelD trajectory SHA
  unchanged; Shield C1 unchanged.

**R2 - Height/intensity ownership unification**: group Voxel fields into
GeometryState/HeightLayer/IntensityLayer (same memory layout or verified
equivalent), single `reprojectImage` lifecycle (already shared).
- Parity gate: numerical parity; memory/runtime deltas recorded.

**R3 - Sampling scorer abstraction**: one candidate/group/select path with
`GeometryInformedScorer` and `Eq8IntensityScorer`; remove the duplicated top-K.
- Parity gate: same SHA/APE gates.

**R4 - Future visual layer interface**: add the `VisualLayer` interface + camera
model abstraction + reference lifecycle; no visual residual.
- Parity gate: intensity-OFF path unchanged; build/tests PASS.

**R5 - Visual residual prototype**: future round, not designed in detail here.

Rollback: each phase is a single commit group with its own gate, independently
revertible.

## 21. Regression gates

- Functional parity: FlatSurfacesS C1 APE within 0.005 m of 0.0595 m; TunnelD
  pf4 C1 within 0.02 m of 0.5854 m.
- Determinism: multi x3 bitwise preferred; if not bitwise, translation max
  <= 1e-6 m and rotation max <= 1e-6 deg.
- OFF parity: intensity.enabled=false must preserve the BIEVR geometry path
  semantics.
- Build: 23/23 tests PASS (update the total if tests grow).

## 22. Risks

- P0 architecture: duplicated surface-frame lifecycle (none today - shared; the
  risk is if future layers re-introduce independent frames); map-layer
  inconsistent reprojection if a new layer is added without joining the shared
  `reprojectImage`.
- P1 performance: duplicated per-frame hash/sort (3 passes over the same cloud);
  large transient buffers (~10 MB/frame, no pooling).
- P1 memory: per-voxel persistent images (~27 KB each); visual view-dependent
  references could explode if not capped (reference-per-patch policy needed).
- P2 visual integration: view-dependent reference explosion; exposure/gain
  metadata; non-pinhole projection (kept abstract).

## 23. Open questions

- Freeze policy exact maturity threshold (how many updates / centroid jitter
  before freeze) - empirical, needs a future round.
- Whether `pending_points_` should be part of the shared association buffer or
  stay per-voxel.
- Whether the intensity source points should share the geometry downsample
  buffer (currently independent).
- The exact tie-break keys for the intensity downsample comparator (dist then
  idx is proposed).

---

## Known unrelated issue (not a blocker)

Shield5 current-head B0 diverges (catastrophic) while the original public SHA is
stable (~2.04 m): KNOWN_BASELINE_REGRESSION. Not fixed, not mixed into the
parallel refactor.