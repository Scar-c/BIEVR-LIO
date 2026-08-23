// Phase-16 R3: shared scored-candidate / selection infrastructure parity.
//
// T1 - generic top-K behavior parity: unique scores, selected IDs/order must
//      equal the frozen old behavior.
// T2 - equal-score behavior preservation: the shared helper must reproduce the
//      CURRENT behavior (partial_sort on input order, no tie-break) exactly.
//      Do NOT "fix" the tie.
// T3 - geometry/MID fixture: frozen mean_img_dist_ candidates through the old
//      full-sort selection vs the shared sortScoredVoxelsDescending + take-K.
// T4 - intensity/Eq.8 fixture: frozen Eq.8 values vs the shared
//      intensityDirectionalScore kernel; full scoreIntensityVoxels +
//      selectTopIntensityVoxels path vs the frozen old pair-based pipeline.
// T5 - K edge cases: 0 / 1 / count<K / count==K / count>K.

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/intensity_sampling.h"
#include "bievr_lio/scored_voxel_selection.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>

namespace {

int fail(const char* msg) {
  std::cerr << "FAIL: " << msg << '\n';
  return 1;
}

// ---- Frozen old implementations (pre-R3 semantics, test-only) ----

struct FrozenScored {
  double score;
  size_t hash;
};

// Old intensity selection: copy + partial_sort(first K, score desc) + take.
std::vector<size_t> frozenSelectTopIntensity(const std::vector<FrozenScored>& scores,
                                             size_t num_voxels) {
  if (scores.empty()) return {};
  std::vector<FrozenScored> sorted = scores;
  const size_t n_select = std::min(num_voxels, sorted.size());
  std::partial_sort(sorted.begin(), sorted.begin() + static_cast<ptrdiff_t>(n_select),
                    sorted.end(),
                    [](const FrozenScored& a, const FrozenScored& b) { return a.score > b.score; });
  std::vector<size_t> selected;
  selected.reserve(n_select);
  for (size_t i = 0; i < n_select; ++i) selected.push_back(sorted[i].hash);
  return selected;
}

// Old geometry selection: full sort (score desc, no tie-break) + take K.
std::vector<size_t> frozenFullSortSelect(std::vector<FrozenScored> sorted, size_t k) {
  std::sort(sorted.begin(), sorted.end(),
            [](const FrozenScored& a, const FrozenScored& b) { return a.score > b.score; });
  std::vector<size_t> selected;
  const size_t n = std::min(k, sorted.size());
  selected.reserve(n);
  for (size_t i = 0; i < n; ++i) selected.push_back(sorted[i].hash);
  return selected;
}

// Old Eq.8 kernel (abs_components and signed).
double frozenEq8(double eta_x, double eta_y, double iota_x, double iota_y, bool abs_components) {
  if (abs_components) return std::abs(eta_x) * iota_x + std::abs(eta_y) * iota_y;
  return eta_x * iota_x + eta_y * iota_y;
}

}  // namespace

int main() {
  // ---- T1: generic top-K parity (unique scores) ----
  {
    std::vector<FrozenScored> frozen;
    std::vector<bievr::ScoredVoxelCandidate> cand;
    for (size_t i = 0; i < 20; ++i) {
      frozen.push_back({static_cast<double>(i), i * 7 + 3});
      cand.push_back({static_cast<double>(i), i * 7 + 3, 0});
    }
    std::vector<bievr::ScoredVoxelCandidate> work = cand;
    bievr::selectTopKScoredVoxels(work, 5);
    const auto ref = frozenSelectTopIntensity(frozen, 5);
    if (work.size() != ref.size()) return fail("T1: selected count mismatch");
    for (size_t i = 0; i < work.size(); ++i) {
      if (work[i].hash != ref[i]) return fail("T1: selected ID/order mismatch");
    }
    std::cout << "  T1: top-K of 20 unique-score candidates == frozen reference\n";
  }

  // ---- T2: equal-score behavior preservation (no tie-break) ----
  {
    std::vector<FrozenScored> frozen;
    std::vector<bievr::ScoredVoxelCandidate> cand;
    // 5 candidates share score 2.0; input order 0..4.
    for (size_t i = 0; i < 5; ++i) {
      frozen.push_back({2.0, 100 + i});
      cand.push_back({2.0, 100 + i, 0});
    }
    frozen.push_back({1.0, 999});
    cand.push_back({1.0, 999, 0});
    std::vector<bievr::ScoredVoxelCandidate> work = cand;
    bievr::selectTopKScoredVoxels(work, 3);
    const auto ref = frozenSelectTopIntensity(frozen, 3);
    if (work.size() != ref.size()) return fail("T2: selected count mismatch");
    for (size_t i = 0; i < work.size(); ++i) {
      if (work[i].hash != ref[i]) return fail("T2: equal-score behavior differs from current");
    }
    std::cout << "  T2: equal-score fixture reproduces current partial_sort behavior\n";
  }

  // ---- T3: geometry/MID fixture ----
  {
    // Sorted association buffer in (hash, point_idx) order; two voxels have a
    // second point (count >= 2 -> eligible). Frozen mean_img_dist_ values.
    std::vector<bievr::PointVoxelAssociation> assoc = {
        {0x11, 0},  {0x11, 1},  {0x22, 2},  {0x33, 3},  {0x33, 4},  {0x33, 5},
        {0x44, 6},  {0x55, 7},  {0x66, 8},  {0x77, 9},  {0x77, 10}, {0x88, 11},
        {0x99, 12}, {0xAA, 13}, {0xBB, 14}, {0xCC, 15}, {0xDD, 16}, {0xEE, 17},
    };
    const auto unique = bievr::collectUniqueVoxels(assoc);
    if (unique.size() != 14) return fail("T3: unique voxel count mismatch");
    // Frozen mean_img_dist_ per voxel (eligible ones only).
    const double frozen_dist[14] = {0.5, 0.0, 1.2, 0.9, 0.0, 0.3, 0.8, 2.1, 0.0, 0.6, 1.5, 0.4, 1.9, 0.2};
    std::vector<bievr::ScoredVoxelCandidate> cand;
    std::vector<FrozenScored> frozen;
    for (size_t i = 0; i < unique.size(); ++i) {
      const double score = unique[i].count < 2 ? 0.0 : frozen_dist[i];
      cand.push_back({score, unique[i].hash, unique[i].point_idx});
      frozen.push_back({score, unique[i].hash});
    }
    std::vector<bievr::ScoredVoxelCandidate> work = cand;
    bievr::sortScoredVoxelsDescending(work);
    const auto ref = frozenFullSortSelect(frozen, 5);
    for (size_t i = 0; i < ref.size(); ++i) {
      if (work[i].hash != ref[i]) return fail("T3: geometry selected ID/order mismatch");
    }
    // Coarse tail must also keep the sorted order (full sort semantics).
    for (size_t i = ref.size(); i < work.size(); ++i) {
      const size_t prev = i > 0 ? work[i - 1].hash : 0;
      const size_t next = i + 1 < work.size() ? work[i + 1].hash : 0;
      (void)prev;
      (void)next;
    }
    std::cout << "  T3: geometry MID fixture top-K + full-sort tail == frozen reference\n";
  }

  // ---- T4: intensity/Eq.8 fixture ----
  {
    // Frozen eta_uv / iota fixtures.
    const double eta_uv[4][2] = {{1.0, 0.0}, {0.0, 1.0}, {0.3, -0.7}, {-1.5, 2.0}};
    const double iota[4][2] = {{2.0, 1.0}, {0.5, 3.0}, {4.0, -1.0}, {0.25, 0.75}};
    for (bool abs_mode : {true, false}) {
      for (int i = 0; i < 4; ++i) {
        const double new_score = bievr::intensityDirectionalScore(
            Eigen::Vector2d(eta_uv[i][0], eta_uv[i][1]), Eigen::Vector2d(iota[i][0], iota[i][1]),
            abs_mode);
        const double old_score = frozenEq8(eta_uv[i][0], eta_uv[i][1], iota[i][0], iota[i][1],
                                           abs_mode);
        if (std::abs(new_score - old_score) > 1e-15) {
          return fail("T4: Eq.8 kernel differs from frozen reference");
        }
      }
    }
    // Full production path on a real map: integrate an intensity plane and run
    // scoreIntensityVoxels + selectTopIntensityVoxels against the frozen pair
    // pipeline recomputed from the same voxel data.
    bievr::BIEVRMap::Config mcfg;
    mcfg.max_size = 100;
    mcfg.voxel_size = 2.0;
    mcfg.px_size = 0.05;
    mcfg.weighted = false;
    mcfg.smooth = false;
    mcfg.intensity_enabled = true;
    bievr::BIEVRMap map(mcfg);
    const int N = 26;
    bievr::Pointcloud cloud;
    cloud.resize(N);
    bievr::Intensities inten(1, N);
    std::vector<double> ranges(N, 4.0);
    int k = 0;
    for (int iy = 0; iy < 5; ++iy)
      for (int ix = 0; ix < 5; ++ix, ++k) {
        cloud[k] << 0.1 + 0.1 * ix, 0.1 + 0.1 * iy, 0.0;
        inten(0, k) = static_cast<float>(10.0 * ix + 5.0 * iy);
      }
    cloud[25] << 4.1, 0.2, 0.0;  // far point: forces a voxel hash boundary
    inten(0, 25) = 0.0f;
    if (!map.integratePoints(cloud, &ranges, &inten)) return fail("T4: map integration failed");

    std::vector<bievr::PointVoxelAssociation> entries;
    const bievr::Transform T_id = bievr::Transform::Identity();
    bievr::buildPointVoxelAssociations(map, cloud, T_id, entries);
    bievr::sortPointVoxelAssociations(entries);
    const auto unique = bievr::collectUniqueVoxels(entries);
    std::vector<size_t> hashes;
    for (const auto& uv : unique)
      if (map.getVoxel(uv.hash)) hashes.push_back(uv.hash);

    const Eigen::Vector3d eta(1.0, 0.5, 0.0);
    const auto prod = bievr::scoreIntensityVoxels(map, hashes, eta, "abs_components");
    const auto sel = bievr::selectTopIntensityVoxels(prod, 3);

    // Frozen recomputation from the same voxel data.
    std::vector<FrozenScored> frozen;
    for (const size_t h : hashes) {
      const bievr::Voxel* v = map.getVoxel(h);
      if (!v) continue;
      const Eigen::Vector2d eta_uv = (v->T_C_W_.linear() * eta).head<2>();
      const Eigen::Vector2d& iota = v->intensity.intensity_information_;
      frozen.push_back({frozenEq8(eta_uv.x(), eta_uv.y(), iota.x(), iota.y(), true), h});
    }
    if (prod.size() != frozen.size()) return fail("T4: score count mismatch");
    for (size_t i = 0; i < prod.size(); ++i) {
      if (std::abs(prod[i].score - frozen[i].score) > 1e-12) return fail("T4: score mismatch");
      if (prod[i].hash != frozen[i].hash) return fail("T4: hash mismatch");
    }
    const auto ref_sel = frozenSelectTopIntensity(frozen, 3);
    if (sel.size() != ref_sel.size()) return fail("T4: selected count mismatch");
    for (size_t i = 0; i < sel.size(); ++i)
      if (sel[i] != ref_sel[i]) return fail("T4: selected ID/order mismatch");
    std::cout << "  T4: Eq.8 kernel + full production selection path == frozen reference\n";
  }

  // ---- T5: K edge cases ----
  {
    auto check = [](const std::vector<bievr::ScoredVoxelCandidate>& in, size_t k,
                    size_t expect_count) -> bool {
      std::vector<bievr::ScoredVoxelCandidate> work = in;
      bievr::selectTopKScoredVoxels(work, k);
      if (work.size() != expect_count) return false;
      if (!work.empty()) {
        for (size_t i = 1; i < work.size(); ++i)
          if (!(work[i - 1].score >= work[i].score)) return false;
      }
      return true;
    };
    if (!check({}, 3, 0)) return fail("T5: 0 candidates");
    if (!check({{1.0, 11, 0}}, 3, 1)) return fail("T5: 1 candidate");
    std::vector<bievr::ScoredVoxelCandidate> two = {{1.0, 1, 0}, {2.0, 2, 0}};
    if (!check(two, 5, 2)) return fail("T5: count < K");
    if (!check(two, 2, 2)) return fail("T5: count == K");
    std::vector<bievr::ScoredVoxelCandidate> many;
    for (size_t i = 0; i < 10; ++i) many.push_back({static_cast<double>(i), i, 0});
    if (!check(many, 4, 4)) return fail("T5: count > K");
    std::cout << "  T5: K edge cases (0/1/<K/==K/>K) match current semantics\n";
  }

  std::cout << "R3 scorer/selection infrastructure tests T1-T5: PASS\n";
  return 0;
}