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
