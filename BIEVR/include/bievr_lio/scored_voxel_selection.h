#ifndef BIEVR_LIO_SCORED_VOXEL_SELECTION_H_
#define BIEVR_LIO_SCORED_VOXEL_SELECTION_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

#include "bievr_lio/point_voxel_association.h"

// Phase-16 R3: shared candidate / score / select infrastructure.
//
// Deduplicates the repeated "candidate voxel list -> scored candidate list ->
// top-K selection" engineering skeleton used by the BIEVR geometry informed
// sampling (sampleInformed, MID-like score) and the COIN-BIEVR intensity
// informed sampling (sampleIntensityPoints, Eq.8 score). This is an
// infrastructure extraction only:
//
//   - MID and Eq.8 remain two distinct scoring algorithms; the scorers are
//     path-specific (Eq.8 exposes its pure kernel here as
//     intensityDirectionalScore with unchanged math).
//   - candidate generation order is preserved per path (both paths walk the
//     R1-sorted association buffer; collectUniqueVoxels produces the exact
//     same per-hash sequence the two paths previously built inline).
//   - selection semantics are preserved exactly: the geometry path keeps the
//     FULL descending sort (its sorted tail feeds the coarse set) via
//     sortScoredVoxelsDescending; the intensity path keeps the top-K partial
//     sort via selectTopKScoredVoxels. Both use the same descending
//     comparator (score > score) with NO tie-break, exactly as before.
//
// No std::vector<bool>, no mutex/atomic, no new parallel regions.

namespace bievr {

// Scored voxel candidate shared by the geometry (MID) and intensity (Eq.8)
// informed sampling paths. Field order mirrors the geometry VoxelScore
// {score, hash, idx}; the intensity path sets point_idx = 0 (unused there).
struct ScoredVoxelCandidate {
  double score;
  size_t hash;
  size_t point_idx;
};

// One distinct voxel group of a sorted association buffer, in buffer order:
// hash, first (minimum) point index of the group and group point count.
struct UniqueVoxel {
  size_t hash;
  size_t point_idx;
  size_t count;
};

// Unique-voxel walk over an R1-sorted PointVoxelAssociation buffer.
// Produces the same per-hash sequence (hash, first point index, count) that
// the two sampling paths previously built with inline walks over the same
// sorted buffer; input order is preserved.
inline std::vector<UniqueVoxel> collectUniqueVoxels(
    const std::vector<PointVoxelAssociation>& sorted) {
  std::vector<UniqueVoxel> out;
  out.reserve(sorted.size());
  for (size_t i = 0; i < sorted.size();) {
    const size_t hash = sorted[i].hash;
    const size_t first_idx = sorted[i].point_idx;
    size_t j = i + 1;
    while (j < sorted.size() && sorted[j].hash == hash) ++j;
    out.push_back({hash, first_idx, j - i});
    i = j;
  }
  return out;
}

// Descending-score comparator shared by both selection paths
// (score > score, no tie-break; current semantics preserved exactly).
inline bool scoredVoxelDescending(const ScoredVoxelCandidate& a,
                                  const ScoredVoxelCandidate& b) {
  return a.score > b.score;
}

// Geometry path selection: FULL descending sort. The sorted tail order feeds
// the coarse point set, so a top-K partial sort must NOT be substituted here.
inline void sortScoredVoxelsDescending(std::vector<ScoredVoxelCandidate>& scored) {
  tbb::parallel_sort(scored.begin(), scored.end(), scoredVoxelDescending);
}

// Intensity path selection: top-K descending partial sort, then resize to the
// selected count (min(k, size)). Same semantics as the former
// selectTopIntensityVoxels: partial_sort on the input order, first K kept.
inline void selectTopKScoredVoxels(std::vector<ScoredVoxelCandidate>& scored, size_t k) {
  if (scored.empty()) return;
  const size_t n_select = std::min(k, scored.size());
  std::partial_sort(scored.begin(), scored.begin() + static_cast<ptrdiff_t>(n_select),
                    scored.end(), scoredVoxelDescending);
  scored.resize(n_select);
}

// COIN-BIEVR Eq. (8) score kernel: directional alignment of the weak-geometry
// direction (projected into the voxel-local uv plane) with the voxel intensity
// information vector iota (Eq. 6). `abs_components` matches the production
// score_mode != "paper_signed" sign handling. Math unchanged.
inline double intensityDirectionalScore(const Eigen::Vector2d& eta_uv,
                                        const Eigen::Vector2d& iota,
                                        bool abs_components) {
  if (abs_components) {
    return std::abs(eta_uv.x()) * iota.x() + std::abs(eta_uv.y()) * iota.y();
  }
  return eta_uv.x() * iota.x() + eta_uv.y() * iota.y();
}

}  // namespace bievr

#endif  // BIEVR_LIO_SCORED_VOXEL_SELECTION_H_