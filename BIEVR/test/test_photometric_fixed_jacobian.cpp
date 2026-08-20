// Round 6, fixed-correspondence 6-DOF photometric Jacobian (Level B, local).
// A voxel with a NONLINEAR intensity texture (quadratic in pixel coords) so the
// exact masked-bilinear gradient matters. The analytic Jacobian (replicating
// linearizePhotometric's exact math) is compared with a fixed-correspondence
// finite difference, keeping the same voxel and the same bilinear cell.

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/common.h"
#include "bievr_lio/ls_optimizer.h"
#include "bievr_lio/utils.h"

#include <cmath>
#include <iostream>
#include <vector>

namespace {

using bievr::Point;
using bievr::Row6;
using bievr::Transform;

int fail(const char* msg) {
  std::cerr << msg << '\n';
  return 1;
}

Transform so3_exp(const Eigen::Vector3d& omega) {
  const double theta_sq = omega.dot(omega);
  Eigen::Quaterniond q;
  if (theta_sq < 1e-10) {
    q = Eigen::Quaterniond(1.0, 0.5 * omega.x(), 0.5 * omega.y(), 0.5 * omega.z());
  } else {
    const double theta = std::sqrt(theta_sq);
    const double half = 0.5 * theta;
    q = Eigen::Quaterniond(std::cos(half), std::sin(half) / theta * omega.x(),
                           std::sin(half) / theta * omega.y(), std::sin(half) / theta * omega.z());
  }
  Transform T = Transform::Identity();
  T.linear() = q.toRotationMatrix();
  return T;
}

Transform perturb(const Transform& T0, const Eigen::Matrix<double, 6, 1>& xi) {
  Transform delta = so3_exp(xi.head<3>());
  delta.translation() = xi.tail<3>();
  return T0 * delta;
}

}  // namespace

int main() {
  bievr::BIEVRMap::Config mcfg;
  mcfg.max_size = 100;
  mcfg.voxel_size = 2.0;
  mcfg.px_size = 0.05;
  mcfg.weighted = false;
  mcfg.smooth = false;
  mcfg.intensity_enabled = true;
  bievr::BIEVRMap map(mcfg);

  // Dense 7x7 grid (0.05 m spacing -> adjacent pixels) + far point.
  const int G = 7;
  const int N = G * G + 1;
  bievr::Pointcloud cloud;
  cloud.resize(N);
  bievr::Intensities inten(1, N);
  std::vector<double> ranges(N, 4.0);
  int k = 0;
  for (int iy = 0; iy < G; ++iy) {
    for (int ix = 0; ix < G; ++ix) {
      cloud[k] << 0.03 + 0.05 * ix, 0.03 + 0.05 * iy, 0.0;
      inten(0, k) = 0.0f;
      ++k;
    }
  }
  cloud[N - 1] << 2.5, 0.5, 0.0;
  inten(0, N - 1) = 0.0f;

  // Probe: discover the pixel each grid point lands in.
  bievr::BIEVRMap probe(mcfg);
  if (!probe.integratePoints(cloud, &ranges, &inten)) return fail("probe failed");
  const bievr::Voxel* pv = probe.getVoxel(probe.hashIndex(cloud[0]));
  if (!pv) return fail("probe voxel not observed");

  // Rebuild with a quadratic intensity texture (nonlinear in pixel coords).
  std::vector<int> pu(N, 0), pvv(N, 0);
  const double inv_px = map.inv_px_size;
  for (int i = 0; i < N; ++i) {
    const Point p_o = pv->T_C_W_ * cloud[i];
    pu[i] = static_cast<int>(std::round(p_o.x() * inv_px));
    pvv[i] = static_cast<int>(std::round(p_o.y() * inv_px));
  }
  const int center = (G / 2) * G + (G / 2);
  const double cu = pu[center], cv = pvv[center];
  for (int i = 0; i < N; ++i) {
    const double du = pu[i] - cu, dv = pvv[i] - cv;
    inten(0, i) = static_cast<float>(du * du + dv * dv);
  }
  if (!map.integratePoints(cloud, &ranges, &inten)) return fail("map rebuild failed");

  const Point p_j = cloud[center];
  const double I_i = inten(0, center);

  const Transform T0 = Transform::Identity();
  const Point p_W = T0 * p_j;
  size_t hash = map.hashIndex(p_W);
  const bievr::Voxel* voxel = map.getVoxel(hash);
  if (!voxel) {
    if (!map.nearestVoxel(p_W, hash)) return fail("no voxel");
    voxel = map.getVoxel(hash);
    if (!voxel) return fail("no voxel");
  }

  // Analytic Jacobian (exact masked-bilinear gradient + SE3 chain rule).
  bievr::IntensitySample smp;
  const Point p_o0 = voxel->T_C_W_ * p_W;
  const double u0 = p_o0.x() * inv_px, v0 = p_o0.y() * inv_px;
  if (!sampleIntensityBilinearWithGradient(voxel, u0, v0, smp)) return fail("sample failed");
  Eigen::RowVector2d grad;
  grad << smp.gradient_pixel.x(), smp.gradient_pixel.y();
  grad *= inv_px;
  Eigen::Matrix<double, 3, 6> SE3_Jac;
  SE3_Jac.block<3, 3>(0, 3) = voxel->T_C_W_.linear() * T0.linear();
  SE3_Jac.block<3, 3>(0, 0).noalias() = -SE3_Jac.block<3, 3>(0, 3) * bievr::skew(p_j);
  Row6 J_an = -(grad * SE3_Jac.topRows<2>());

  // Fixed-correspondence FD (same voxel, same bilinear cell).
  const double eps_rot = 1e-5, eps_tr = 1e-5;
  auto residual = [&](const Transform& T, int& cx, int& cy) -> double {
    const Point p_W_ = T * p_j;
    const Point p_o = voxel->T_C_W_ * p_W_;
    const double u = p_o.x() * inv_px, v = p_o.y() * inv_px;
    bievr::IntensitySample s;
    if (!sampleIntensityBilinearWithGradient(voxel, u, v, s)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    cx = s.x0;
    cy = s.y0;
    return I_i - s.value;
  };

  std::vector<double> errs;
  for (int dof = 0; dof < 6; ++dof) {
    const double eps = dof < 3 ? eps_rot : eps_tr;
    Eigen::Matrix<double, 6, 1> xp = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> xm = Eigen::Matrix<double, 6, 1>::Zero();
    xp(dof) = eps;
    xm(dof) = -eps;
    int cxp = 0, cyp = 0, cxm = 0, cym = 0;
    const double rp = residual(perturb(T0, xp), cxp, cyp);
    const double rm = residual(perturb(T0, xm), cxm, cym);
    if (std::isnan(rp) || std::isnan(rm)) continue;
    if (cxp != smp.x0 || cyp != smp.y0 || cxm != smp.x0 || cym != smp.y0) continue;
    const double J_num = (rp - rm) / (2 * eps);
    const double scale = std::max({1.0, std::abs(J_an(0, dof)), std::abs(J_num)});
    errs.push_back(std::abs(J_an(0, dof) - J_num) / scale);
  }

  if (errs.size() < 6) return fail("insufficient fixed-correspondence samples");
  std::sort(errs.begin(), errs.end());
  const double med = errs[errs.size() / 2];
  const double p95 = errs[static_cast<size_t>(0.95 * (errs.size() - 1))];
  if (med > 1e-3) return fail("fixed-correspondence median error > 1e-3");
  if (p95 > 1e-2) return fail("fixed-correspondence P95 error > 1e-2");

  return 0;
}
