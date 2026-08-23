#include "bievr_lio/point_voxel_association.h"
#include "bievr_lio/intensity_sampling.h"
#include "bievr_lio/scored_voxel_selection.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <utility>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

#include "unordered_dense/unordered_dense.h"

namespace bievr {

namespace {

struct DownsampleEntry {
  size_t hash;
  size_t idx;
  double dist;
};

}  // namespace

std::vector<size_t> findObservedVoxels(const BIEVRMap& map, const Pointcloud& undistorted,
                                       const Transform& T_W_I_prior) {
  // Shared point->voxel association primitive (Phase-14 R1; identical semantics).
  std::vector<PointVoxelAssociation> entries;
  buildPointVoxelAssociations(map, undistorted, T_W_I_prior, entries);
  sortPointVoxelAssociations(entries);

  const auto unique = collectUniqueVoxels(entries);
  std::vector<size_t> observed;
  observed.reserve(unique.size());
  for (const auto& uv : unique) {
    if (map.getVoxel(uv.hash)) observed.push_back(uv.hash);
  }
  return observed;
}

Eigen::Vector3d estimateWeakGeometryDirection(const BIEVRMap& map,
                                              const std::vector<size_t>& observed_hashes,
                                              double weak_eigen_ratio, bool normalize_eta,
                                              Eigen::Vector3d* eigenvalues_out,
                                              Eigen::Matrix3d* eigenvectors_out) {
  Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
  for (const size_t hash : observed_hashes) {
    const Voxel* voxel = map.getVoxel(hash);
    if (!voxel) continue;
    // The voxel normal is the Z axis of the local frame (see bievr_map.cpp).
    const Eigen::Vector3d normal = voxel->T_O_W_.linear().row(2).transpose();
    A.noalias() += normal * normal.transpose();
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(A);
  const Eigen::Vector3d eigenvalues = solver.eigenvalues();  // ascending
  const Eigen::Matrix3d V = solver.eigenvectors();

  Eigen::Vector3d eta;
  if (weak_eigen_ratio * eigenvalues(0) > eigenvalues(1)) {
    eta = V.col(0) + V.col(1);
  } else {
    eta = V.col(0);
  }
  if (normalize_eta) {
    const double n = eta.norm();
    if (n > 1e-12) eta /= n;
  }

  if (eigenvalues_out) *eigenvalues_out = eigenvalues;
  if (eigenvectors_out) *eigenvectors_out = V;
  return eta;
}

std::vector<ScoredVoxelCandidate> scoreIntensityVoxels(
    const BIEVRMap& map, const std::vector<size_t>& observed_hashes, const Eigen::Vector3d& eta_W,
    const std::string& score_mode) {
  std::vector<ScoredVoxelCandidate> scores;
  scores.reserve(observed_hashes.size());
  const bool abs_components = (score_mode != "paper_signed");
  for (const size_t hash : observed_hashes) {
    const Voxel* voxel = map.getVoxel(hash);
    if (!voxel) continue;
    // Map eta into the voxel-local frame and project onto its uv plane.
    const Eigen::Vector2d eta_uv = (voxel->T_C_W_.linear() * eta_W).head<2>();
    const Eigen::Vector2d& iota = voxel->intensity.intensity_information_;
    // COIN-BIEVR Eq. (8): shared pure kernel with the production sign handling
    // (abs_components for score_mode != "paper_signed").
    scores.push_back({intensityDirectionalScore(eta_uv, iota, abs_components), hash, 0});
  }
  return scores;
}

std::vector<size_t> selectTopIntensityVoxels(const std::vector<ScoredVoxelCandidate>& scores,
                                             size_t num_voxels) {
  if (scores.empty()) return {};

  std::vector<ScoredVoxelCandidate> sorted = scores;
  selectTopKScoredVoxels(sorted, num_voxels);

  std::vector<size_t> selected;
  selected.reserve(sorted.size());
  for (const auto& c : sorted) selected.push_back(c.hash);
  return selected;
}

void downsampleIntensityPoints(const Pointcloud& points, const Intensities& intensities,
                               double voxel_size, Pointcloud& points_down,
                               Intensities& intensities_down) {
  std::vector<DownsampleEntry> entries(points.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, points.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t i = r.begin(); i != r.end(); ++i) {
                        const Eigen::Vector3i voxel =
                            (points[i] / voxel_size).array().floor().cast<int>();
                        const Eigen::Vector3d voxel_center =
                            (voxel.cast<double>() + Eigen::Vector3d::Constant(0.5)) * voxel_size;
                        const double dist = (points[i] - voxel_center).squaredNorm();
                        entries[i] = {hashIndexVoxel(voxel.matrix()), i, dist};
                      }
                    });

  tbb::parallel_sort(entries.begin(), entries.end(),
                     [](const DownsampleEntry& a, const DownsampleEntry& b) {
                       return std::tie(a.hash, a.dist) < std::tie(b.hash, b.dist);
                     });

  std::vector<size_t> selected_indices;
  selected_indices.reserve(points.size());
  size_t curr_hash = std::numeric_limits<size_t>::max();
  for (const auto& entry : entries) {
    if (entry.hash != curr_hash) {
      selected_indices.push_back(entry.idx);
      curr_hash = entry.hash;
    }
  }

  points_down.resize(selected_indices.size());
  intensities_down.resize(1, selected_indices.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, selected_indices.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t i = r.begin(); i != r.end(); ++i) {
                        const size_t idx = selected_indices[i];
                        points_down[i] = points[idx];
                        intensities_down(0, i) = intensities(0, idx);
                      }
                    });
}

IntensitySampleSet sampleIntensityPoints(const BIEVRMap& map, const Pointcloud& undistorted,
                                         const Intensities& filtered_intensity,
                                         const Transform& T_W_I_prior,
                                         const IntensitySamplingConfig& config) {
  IntensitySampleSet result;
  if (!config.enabled || undistorted.empty()) return result;

  // Per-point world position + hash (single pass over the undistorted cloud),
  // using the shared point->voxel association primitive (Phase-14 R1; identical
  // hash / ordering semantics). observed_flag is byte-addressable storage (NOT
  // std::vector<bool>): the parallel workers write logically disjoint indices,
  // but the bit-packed proxy would perform read-modify-write on shared storage
  // words (data race / UB).
  std::vector<PointVoxelAssociation> entries;
  buildPointVoxelAssociations(map, undistorted, T_W_I_prior, entries);
  sortPointVoxelAssociations(entries);

  // Shared unique-voxel walk (Phase-16 R3); voxel-existence filter preserved.
  const auto unique = collectUniqueVoxels(entries);
  std::vector<size_t> observed_hashes;
  observed_hashes.reserve(unique.size());
  for (const auto& uv : unique) {
    if (map.getVoxel(uv.hash)) observed_hashes.push_back(uv.hash);
  }
  result.observed_voxels = observed_hashes.size();
  if (observed_hashes.empty()) return result;

  Eigen::Matrix3d V = Eigen::Matrix3d::Zero();
  result.weak_direction = estimateWeakGeometryDirection(map, observed_hashes, config.weak_eigen_ratio,
                                                        config.normalize_eta,
                                                        &result.weak_eigenvalues, &V);
  result.weak_v1 = V.col(0);
  result.weak_v2 = V.col(1);
  const std::vector<ScoredVoxelCandidate> scores =
      scoreIntensityVoxels(map, observed_hashes, result.weak_direction, config.score_mode);
  result.voxel_scores = scores;
  result.selected_voxel_hashes = selectTopIntensityVoxels(scores, config.num_voxels);
  result.selected_voxels = result.selected_voxel_hashes.size();
  if (result.selected_voxel_hashes.empty()) return result;

  ankerl::unordered_dense::set<size_t> selected_set(result.selected_voxel_hashes.begin(),
                                                    result.selected_voxel_hashes.end());

  // Keep the current-frame points that fall inside the selected intensity voxels.
  std::vector<size_t> candidate_indices;
  candidate_indices.reserve(undistorted.size());
  size_t curr_hash = std::numeric_limits<size_t>::max();
  bool curr_selected = false;
  for (const auto& entry : entries) {
    if (entry.hash != curr_hash) {
      curr_hash = entry.hash;
      curr_selected = selected_set.contains(entry.hash);
    }
    if (curr_selected) candidate_indices.push_back(entry.point_idx);
  }
  Pointcloud candidates;
  candidates.resize(candidate_indices.size());
  Intensities candidate_intensities(1, candidate_indices.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, candidate_indices.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t i = r.begin(); i != r.end(); ++i) {
                        const size_t idx = candidate_indices[i];
                        candidates[i] = undistorted[idx];
                        candidate_intensities(0, i) =
                            filtered_intensity.size() > idx ? filtered_intensity(0, idx) : 0.0;
                      }
                    });

  downsampleIntensityPoints(candidates, candidate_intensities, config.downsample_resolution_m,
                            result.points, result.intensities);
  return result;
}

}  // namespace bievr
