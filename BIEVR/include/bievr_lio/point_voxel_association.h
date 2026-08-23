#ifndef BIEVR_LIO_POINT_VOXEL_ASSOCIATION_H_
#define BIEVR_LIO_POINT_VOXEL_ASSOCIATION_H_

#include <cstddef>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/common.h"

// Phase-14 R1: shared point -> voxel association primitive.
//
// Deduplicates the repeated parallel hash/sort skeleton used by the geometry
// informed sampling (sampleInformed) and the intensity sampling
// (sampleIntensityPoints). This is an infrastructure extraction: the hash
// formula, the POD fields and the (hash, point_idx) ordering are preserved
// exactly; no comparator / tie-break semantics are changed.
//
// Stage-1 writes are PER_INDEX_EXCLUSIVE (each worker writes its own POD
// entry); the map is read-only. No std::vector<bool>.
//
// The map-integration path (BIEVRMap::integratePoints) is intentionally NOT
// migrated: it associates (hash + MapPoint) and sorts by (hash, x-position),
// which differs from the (hash, point_idx) semantics used here.

namespace bievr {

// Point -> voxel association (voxel hash + original point index). Same fields
// and layout as the previous local `VoxelHashIdx` of preprocess.cpp and
// intensity_sampling.cpp.
struct PointVoxelAssociation {
  size_t hash;
  size_t point_idx;
};

// Parallel stage-1: transform each point with `T` and compute its voxel hash.
// Write ownership: PER_INDEX_EXCLUSIVE.
inline void buildPointVoxelAssociations(const BIEVRMap& map, const Pointcloud& points,
                                        const Transform& T,
                                        std::vector<PointVoxelAssociation>& associations) {
  associations.resize(points.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, points.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t i = r.begin(); i != r.end(); ++i) {
                        const Point p_w = T * points[i];
                        associations[i] = {map.hashIndex(p_w), i};
                      }
                    });
}

// Stage-2: sort by (hash, point_idx) - a strict total order (unchanged
// semantics; equivalent to the previous std::tie(a.hash, a.idx) comparator).
inline void sortPointVoxelAssociations(std::vector<PointVoxelAssociation>& associations) {
  tbb::parallel_sort(associations.begin(), associations.end(),
                     [](const PointVoxelAssociation& a, const PointVoxelAssociation& b) {
                       return a.hash < b.hash || (a.hash == b.hash && a.point_idx < b.point_idx);
                     });
}

}  // namespace bievr

#endif  // BIEVR_LIO_POINT_VOXEL_ASSOCIATION_H_