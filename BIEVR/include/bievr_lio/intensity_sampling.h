#ifndef BIEVR_LIO_INTENSITY_SAMPLING_H_
#define BIEVR_LIO_INTENSITY_SAMPLING_H_

#include <utility>
#include <vector>

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/common.h"

// COIN-BIEVR intensity-voxel sampling (Sections 18-27 of the reproduction
// plan). Given the current undistorted scan, the registration pose prior and
// the map (which already holds per-voxel intensity information), selects the
// N=100 intensity voxels that contribute most along the geometry-weak
// direction eta (Eq. 6-8) and returns the current-frame intensity points that
// fall inside them, downsampled to 0.1 m.

namespace bievr {

struct IntensitySamplingConfig {
  bool enabled = true;
  size_t num_voxels = 100;                   // N_intensity_voxel
  double downsample_resolution_m = 0.1;      // intensity point downsampling
  double weak_eigen_ratio = 10.0;            // eta switch: 10*lambda1 > lambda2
  bool normalize_eta = true;                 // normalize eta after construction
  // Eq. (8) voxel contribution score:
  //   "abs_components" (default): |eta_u|*iota_x + |eta_v|*iota_y
  //   "paper_signed":            (eta_u*iota_x + eta_v*iota_y)
  // abs_components removes the eigenvector sign ambiguity (v <-> -v) and
  // follows the fabs-style directional contribution of COIN-LIO. The COIN-BIEVR
  // supplement writes a signed dot product; the exact sign handling is not
  // specified (Section 55 / second review Section 3).
  std::string score_mode = "abs_components";
};

struct IntensitySampleSet {
  Pointcloud points;          // IMU frame, downsampled, index-aligned with intensities
  Intensities intensities;    // filtered intensity per point (0..255)
  size_t observed_voxels = 0;   // # observed BIEVR voxels hit by the current scan
  size_t selected_voxels = 0;   // # selected intensity voxels (<= num_voxels)
  Eigen::Vector3d weak_direction = Eigen::Vector3d::Zero();     // eta (world)
  Eigen::Vector3d weak_eigenvalues = Eigen::Vector3d::Zero();   // lambda1..3
  // Round-9 audit: the two smallest eigenvectors (world), and the raw Eq.8
  // candidate scores for all observed voxels (diagnostic only).
  Eigen::Vector3d weak_v1 = Eigen::Vector3d::Zero(), weak_v2 = Eigen::Vector3d::Zero();
  std::vector<std::pair<double, size_t>> voxel_scores;
  std::vector<size_t> selected_voxel_hashes;
};

// Hashes of the observed BIEVR voxels that the current (undistorted, IMU-frame)
// scan hits when transformed by the registration pose prior.
std::vector<size_t> findObservedVoxels(const BIEVRMap& map, const Pointcloud& undistorted,
                                       const Transform& T_W_I_prior);

// Eq. (7): builds A = sum_{i in V} n_i n_i^T from map voxel normals and returns
// eta = v1 + v2 if 10*lambda1 > lambda2 else v1 (lambda sorted ascending).
// Optionally reports the eigenvalues and the eigenvector matrix V (columns are
// v1..v3, ascending eigenvalues). eta is in the world frame.
Eigen::Vector3d estimateWeakGeometryDirection(const BIEVRMap& map,
                                              const std::vector<size_t>& observed_hashes,
                                              double weak_eigen_ratio, bool normalize_eta,
                                              Eigen::Vector3d* eigenvalues_out = nullptr,
                                              Eigen::Matrix3d* eigenvectors_out = nullptr);

// Eq. (8): voxel contribution l_c = (R_CW eta) . iota using the voxel-local
// intensity information vector. `score_mode` selects "abs_components" (default,
// eigenvector-sign-agnostic, follows COIN-LIO's fabs style) or "paper_signed"
// (literal signed dot product). Returns (score, hash) pairs, unsorted.
std::vector<std::pair<double, size_t>> scoreIntensityVoxels(
    const BIEVRMap& map, const std::vector<size_t>& observed_hashes,
    const Eigen::Vector3d& eta_W, const std::string& score_mode = "abs_components");

// Selects the `num_voxels` highest-scoring voxels (partial sort).
std::vector<size_t> selectTopIntensityVoxels(const std::vector<std::pair<double, size_t>>& scores,
                                             size_t num_voxels);

// Full COIN-BIEVR intensity sampling: observed voxels -> eta -> top-N -> 0.1 m
// intensity points (index-aligned with `filtered_intensity`).
IntensitySampleSet sampleIntensityPoints(const BIEVRMap& map, const Pointcloud& undistorted,
                                         const Intensities& filtered_intensity,
                                         const Transform& T_W_I_prior,
                                         const IntensitySamplingConfig& config);

// Downsample `points` (with aligned intensities) at `voxel_size` m, keeping the
// point closest to each voxel center. Both outputs stay index-aligned.
void downsampleIntensityPoints(const Pointcloud& points, const Intensities& intensities,
                               double voxel_size, Pointcloud& points_down,
                               Intensities& intensities_down);

}  // namespace bievr

#endif  // BIEVR_LIO_INTENSITY_SAMPLING_H_
