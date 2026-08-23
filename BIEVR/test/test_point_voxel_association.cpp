// Phase-14 R1: shared point->voxel association primitive regression tests.
//
// T1: serial (1 TBB worker) vs parallel association parity - hash, point_idx
//     and post-sort ordering must be identical.
// T2: old-vs-new fixture - the shared helper must reproduce the previous
//     (pre-extraction) parallel hash + (hash, idx) sort semantics exactly.
// T3: duplicate-hash grouping - group boundaries, indices and ordering for
//     multiple points in the same voxel.
// T4: empty / single-point inputs must not regress.

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/common.h"
#include "bievr_lio/point_voxel_association.h"

#include <tbb/global_control.h>

#include <cmath>
#include <iostream>
#include <tuple>
#include <vector>

namespace {

int fail(const char* msg) {
  std::cerr << "FAIL: " << msg << '\n';
  return 1;
}

bievr::BIEVRMap makeMap() {
  bievr::BIEVRMap::Config mcfg;
  mcfg.max_size = 100;
  mcfg.voxel_size = 2.0;
  mcfg.px_size = 0.05;
  mcfg.weighted = false;
  mcfg.smooth = false;
  mcfg.intensity_enabled = false;
  bievr::BIEVRMap map(mcfg);
  // A few points creating voxel hash boundaries (grid + two far points).
  const int N = 12;
  bievr::Pointcloud cloud;
  cloud.resize(N);
  std::vector<double> ranges(N, 4.0);
  int k = 0;
  for (int iy = 0; iy < 2; ++iy)
    for (int ix = 0; ix < 5; ++ix, ++k) cloud[k] << 0.1 + 0.1 * ix, 0.1 + 0.1 * iy, 0.0;
  cloud[10] << 4.1, 0.2, 0.0;
  cloud[11] << 6.1, 0.2, 0.0;
  map.integratePoints(cloud, &ranges);
  return map;
}

// T2 reference: the pre-extraction implementation skeleton.
void referenceAssociation(const bievr::BIEVRMap& map, const bievr::Pointcloud& points,
                          const bievr::Transform& T,
                          std::vector<bievr::PointVoxelAssociation>& out) {
  out.resize(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    const bievr::Point p_w = T * points[i];
    out[i] = {map.hashIndex(p_w), i};
  }
  std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) {
    return std::tie(a.hash, a.point_idx) < std::tie(b.hash, b.point_idx);
  });
}

}  // namespace

int main() {
  const bievr::BIEVRMap map = makeMap();

  // Deterministic point set: duplicates within one voxel + several voxels.
  const int N = 41;
  bievr::Pointcloud pts;
  pts.resize(N);
  for (int i = 0; i < N; ++i) {
    // 10 points in voxel (0,0), 10 in (1,0), 10 in (0,1), 10 in (1,1), 1 far.
    const double x = 0.1 + 0.05 * (i % 10);
    const double y = 0.1 + 0.05 * (i % 10);
    const int g = i / 10;
    pts[i] << (g == 1 ? 2.1 : g == 2 ? 0.1 : g == 3 ? 2.1 : 0.1) + 0.5 * (i % 2) * 0.0 + x * 0.0,
        (g == 2 ? 2.1 : g == 3 ? 2.1 : 0.1) + y * 0.0, 0.0;
  }
  // Simpler: 20 points in voxel (0,0) at distinct positions + 1 far point.
  bievr::Pointcloud pts2;
  pts2.resize(21);
  for (int i = 0; i < 20; ++i) pts2[i] << 0.1 + 0.05 * i, 0.1, 0.0;
  pts2[20] << 4.1, 0.2, 0.0;

  const bievr::Transform T = bievr::Transform::Identity();

  // ---- T1: serial vs parallel parity ----
  std::vector<bievr::PointVoxelAssociation> ser, par;
  {
    tbb::global_control gc(tbb::global_control::max_allowed_parallelism, 1);
    bievr::buildPointVoxelAssociations(map, pts2, T, ser);
    bievr::sortPointVoxelAssociations(ser);
  }
  bievr::buildPointVoxelAssociations(map, pts2, T, par);
  bievr::sortPointVoxelAssociations(par);
  if (ser.size() != par.size()) return fail("T1: size differs");
  for (size_t i = 0; i < ser.size(); ++i) {
    if (ser[i].hash != par[i].hash || ser[i].point_idx != par[i].point_idx) {
      return fail("T1: serial/parallel association differs");
    }
  }

  // ---- T2: old-vs-new fixture ----
  std::vector<bievr::PointVoxelAssociation> ref;
  referenceAssociation(map, pts2, T, ref);
  if (ref.size() != par.size()) return fail("T2: size differs");
  for (size_t i = 0; i < ref.size(); ++i) {
    if (ref[i].hash != par[i].hash || ref[i].point_idx != par[i].point_idx) {
      return fail("T2: helper output differs from the reference implementation");
    }
  }

  // ---- T3: duplicate-hash grouping ----
  // 20 points in voxel (0,0): the first group must have 20 entries with
  // point_idx strictly increasing, and a second group for the far point.
  size_t group0 = 0;
  const size_t h0 = par[0].hash;
  for (size_t i = 0; i < par.size(); ++i) {
    if (par[i].hash == h0) ++group0;
    else break;
  }
  if (group0 != 20) return fail("T3: group size for the duplicate-hash voxel is not 20");
  for (size_t i = 1; i < group0; ++i) {
    if (par[i].point_idx <= par[i - 1].point_idx) return fail("T3: point_idx not increasing");
  }
  if (par[group0].hash == h0) return fail("T3: group boundary wrong");

  // ---- T4: empty / single point ----
  {
    bievr::Pointcloud empty;
    std::vector<bievr::PointVoxelAssociation> e;
    bievr::buildPointVoxelAssociations(map, empty, T, e);
    bievr::sortPointVoxelAssociations(e);
    if (!e.empty()) return fail("T4: empty input produced associations");
  }
  {
    bievr::Pointcloud single;
    single.resize(1);
    single[0] << 0.2, 0.2, 0.0;
    std::vector<bievr::PointVoxelAssociation> s;
    bievr::buildPointVoxelAssociations(map, single, T, s);
    bievr::sortPointVoxelAssociations(s);
    if (s.size() != 1) return fail("T4: single-point size");
    if (s[0].point_idx != 0) return fail("T4: single-point idx");
  }

  std::cout << "T1 serial==parallel, T2 old==new, T3 grouping, T4 empty/single: PASS\n";
  return 0;
}
