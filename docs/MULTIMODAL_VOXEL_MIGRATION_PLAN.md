# Multimodal Voxel Migration Plan (R1-R5)

Every phase: single commit group, own regression gate, independently revertible.

## R1 - No-behavior-change infrastructure extraction
- Extract one shared point->voxel association pass
  (`buildPointVoxelAssociation`: parallel hash + (hash, point_idx) sort) and one
  unique-voxel walk; reuse it in sampleInformed, sampleIntensityPoints and
  integratePoints.
- Gate: FlatSurfacesS / TunnelD / Shield C1 trajectory SHA unchanged.

## R2 - Height/intensity ownership unification
- Group Voxel fields into GeometryState / HeightLayer / IntensityLayer (same
  memory or verified equivalent); the shared `reprojectImage` lifecycle already
  moves height + intensity together - make it the single frame-owner.
- Gate: numerical parity; memory + runtime deltas recorded.

## R3 - Sampling scorer abstraction
- One candidate/group/select path with `GeometryInformedScorer` (MID-like) and
  `Eq8IntensityScorer`; add tie-breaks (score, hash, point_idx).
- Gate: same SHA/APE gates.

## R4 - Future visual layer interface
- `VisualLayer` interface slot + camera model abstraction (project/unproject/
  jacobian; pinhole + fisheye) + reference lifecycle. Single camera first. No
  visual residual.
- Gate: intensity-OFF path unchanged; build/tests PASS.

## R5 - Visual residual prototype (future round, not designed here)

Regression gates (all phases):
- FlatSurfacesS C1: |APE - 0.0595| <= 0.005 m
- TunnelD pf4 C1: |APE - 0.5854| <= 0.02 m
- Determinism: multi x3 bitwise preferred; else trans max <= 1e-6 m, rot max
  <= 1e-6 deg
- Intensity OFF: BIEVR geometry path semantics preserved
- Build: 23/23 tests PASS (update total if tests grow)

## R1 implementation status (Phase 14, DONE)

- Abstraction extracted: BIEVR/include/bievr_lio/point_voxel_association.h
  (PointVoxelAssociation {hash, point_idx} + buildPointVoxelAssociations +
  sortPointVoxelAssociations; PER_INDEX_EXCLUSIVE stage-1 writes; no
  std::vector<bool>).
- Callers migrated: sampleInformed (geometry), sampleIntensityPoints and
  findObservedVoxels (intensity).
- Caller DEFERRED: BIEVRMap::integratePoints (association carries MapPoint and
  sorts by (hash, x-position) - different semantics; migrating would change
  behavior, forbidden in R1).
- Comparator semantics: unchanged ((hash, point_idx) preserved). Tie-breaks NOT
  changed.
- Tests: test_point_voxel_association (T1 serial==parallel, T2 old-vs-new
  reference, T3 duplicate-hash grouping, T4 empty/single).
- Parity: FlatSurfacesS C1 (traj 6920bc2b..., photo 26fa8137...), TunnelD pf4
  C1 (traj a8ea08e0..., photo 591e1d18...), Shield1 C1 (traj e5921b4a..., photo
  8b253d03...) - ALL bitwise identical to the pre-R1 references.
- Performance (TunnelD): wall 1:37.88 vs 1:37.96; max RSS 147.5 vs 147.8 MB.
- R1b (deterministic total-order hardening): NOT STARTED.

## R2 implementation status (Phase 15, DONE)

- Voxel restructured: HeightLayer {bump_img_, bump_smoothed_} and IntensityLayer
  {intensity_img_, intensity_information_} as explicit co-registered layer
  structs; surface frame (T_C_W_/T_O_W_), shared weight mask (bump_weights_),
  observed flag and geometry statistics remain at Voxel level. Pure member
  relocation; matrix types/dims/initial values/allocation timing unchanged.
- reprojectImage (single shared lifecycle) unchanged: height stays the 3D
  lift/reference quantity; intensity rides along; weights move together.
- intensity_information_ (Eq. 6) grouped into IntensityLayer (voxel-aggregate
  [Ix, Iy] central-difference information, masked by shared weights).
- Camera/VisualLayer: OUT OF CURRENT SCOPE (roadmap updated).
- Tests: test_layer_ownership (T1 layer init incl. intensity-disabled no
  allocation; T2 shared pixel correspondence; T3 reprojection alignment; T4
  height-only path; T5 copy behavior).
- Parity (all bitwise identical to references): FlatSurfacesS B0
  (dce52ea7..., intensity-OFF), FlatSurfacesS C1 (6920bc2b.../26fa8137...),
  TunnelD pf4 C1 (a8ea08e0.../591e1d18...), Shield1 C1 (e5921b4a.../8b253d03...).
- Performance (TunnelD): wall 1:35.71 vs 1:37.88 (-1.3%), max RSS 145320 vs
  147452 KB (-1.4%); no regression.
- R1b tie-break hardening: NOT STARTED.
