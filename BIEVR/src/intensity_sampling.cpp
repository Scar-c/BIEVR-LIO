#include "bievr_lio/point_voxel_association.h"
#include "bievr_lio/intensity_sampling.h"

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

  std::vector<size_t> observed;
  observed.reserve(entries.size());
  for (size_t i = 0; i < entries.size(); ++i) {
    if (i > 0 && entries[i].hash == entries[i - 1].hash) continue;
    if (map.getVoxel(entries[i].hash)) observed.push_back(entries[i].hash);
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

std::vector<std::pair<double, size_t>> scoreIntensityVoxels(
    const BIEVRMap& map, const std::vector<size_t>& observed_hashes, const Eigen::Vector3d& eta_W,
    const std::string& score_mode) {
  std::vector<std::pair<double, size_t>> scores;
  scores.reserve(observed_hashes.size());
  const bool abs_components = (score_mode != "paper_signed");
  for (const size_t hash : observed_hashes) {
    const Voxel* voxel = map.getVoxel(hash);
    if (!voxel) continue;
    // Map eta into the voxel-local frame and project onto its uv plane.
    const Eigen::Vector2d eta_uv = (voxel->T_C_W_.linear() * eta_W).head<2>();
    const Eigen::Vector2d& iota = voxel->intensity.intensity_information_;

    double score;
    if (abs_components) {
      // COIN-BIEVR Eq. (8) is written as a signed dot product. We use
      // component-wise absolute directional contribution to remove eigenvector
      // sign ambiguity (v <-> -v), following the implementation style of
      // COIN-LIO. The supplement does not specify the exact sign handling.
      score = std::abs(eta_uv.x()) * iota.x() + std::abs(eta_uv.y()) * iota.y();
    } else {
      score = eta_uv.x() * iota.x() + eta_uv.y() * iota.y();
    }
    scores.emplace_back(score, hash);
  }
  return scores;
}

std::vector<size_t> selectTopIntensityVoxels(const std::vector<std::pair<double, size_t>>& scores,
                                             size_t num_voxels) {
  if (scores.empty()) return {};

  std::vector<std::pair<double, size_t>> sorted = scores;
  const size_t n_select = std::min(num_voxels, sorted.size());
  std::partial_sort(sorted.begin(), sorted.begin() + static_cast<ptrdiff_t>(n_select), sorted.end(),
                    [](const auto& a, const auto& b) { return a.first > b.first; });

  std::vector<size_t> selected;
  selected.reserve(n_select);
  for (size_t i = 0; i < n_select; ++i) selected.push_back(sorted[i].second);
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
  std::vector<uint8_t> observed_flag(undistorted.size(), 0u);
  for (size_t i = 0; i < entries.size(); ++i) {
    observed_flag[entries[i].point_idx] =
        map.getVoxel(entries[i].hash) != nullptr ? 1u : 0u;
  }
  sortPointVoxelAssociations(entries);

  std::vector<size_t> observed_hashes;
  observed_hashes.reserve(entries.size());
  for (size_t i = 0; i < entries.size(); ++i) {
    if (i > 0 && entries[i].hash == entries[i - 1].hash) continue;
    if (observed_flag[entries[i].point_idx]) observed_hashes.push_back(entries[i].hash);
  }
  result.observed_voxels = observed_hashes.size();
  if (observed_hashes.empty()) return result;

  Eigen::Matrix3d V = Eigen::Matrix3d::Zero();
  result.weak_direction = estimateWeakGeometryDirection(map, observed_hashes, config.weak_eigen_ratio,
                                                        config.normalize_eta,
                                                        &result.weak_eigenvalues, &V);
  result.weak_v1 = V.col(0);
  result.weak_v2 = V.col(1);
  const std::vector<std::pair<double, size_t>> scores =
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
