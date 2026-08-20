// COIN-BIEVR reproduction plan, Section 48:
// Verify the analytic photometric Jacobian
//   J_photo = -grad(I_P) * d(u,v)/d(xi)
// against a six-dimensional finite-difference of the photometric residual
//   r(xi) = I_i - I_P(u(xi), v(xi)).
// The map texture is a linear ramp in *pixel* space (built with a two-pass
// integration: first integrate with dummy intensity to discover which pixel each
// point lands in, then re-integrate the same geometry with a pixel-space ramp),
// so the central-difference image gradient is exact.

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/common.h"
#include "bievr_lio/ls_optimizer.h"
#include "bievr_lio/utils.h"

#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <vector>

namespace {

using bievr::Point;
using bievr::Row6;
using bievr::Transform;
using bievr::Voxel;

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

// Residual as a function of the perturbation xi around T0.
double residual(const bievr::BIEVRMap& map, const Transform& T0, const Point& p_j, double I_i,
                const Eigen::Matrix<double, 6, 1>& xi) {
  const Transform T = perturb(T0, xi);
  const Point p_W = T * p_j;
  size_t hash = map.hashIndex(p_W);
  const Voxel* voxel = map.getVoxel(hash);
  if (!voxel) {
    if (!map.nearestVoxel(p_W, hash)) return std::numeric_limits<double>::quiet_NaN();
    voxel = map.getVoxel(hash);
    if (!voxel) return std::numeric_limits<double>::quiet_NaN();
  }
  const Point p_o = voxel->T_C_W_ * p_W;
  const double x = p_o.x() * map.inv_px_size;
  const double y = p_o.y() * map.inv_px_size;
  double I_P = 0.0;
  if (!getSubPixelIntensityValue(voxel, x, y, I_P)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return I_i - I_P;
}

bool analyticJacobian(const bievr::BIEVRMap& map, const Transform& T0, const Point& p_j, double I_i,
                      Row6& J) {
  const Point p_W = T0 * p_j;
  size_t hash = map.hashIndex(p_W);
  const Voxel* voxel = map.getVoxel(hash);
  if (!voxel) {
    if (!map.nearestVoxel(p_W, hash)) return false;
    voxel = map.getVoxel(hash);
    if (!voxel) return false;
  }
  const Point p_o = voxel->T_C_W_ * p_W;
  const double x = p_o.x() * map.inv_px_size;
  const double y = p_o.y() * map.inv_px_size;
  double I_P, dIdu, dIdv;
  if (!sampleIntensityValueAndGradient(voxel, x, y, I_P, dIdu, dIdv)) return false;
  (void)I_P;
  (void)I_i;

  Eigen::Matrix<double, 3, 6> SE3_Jac;
  SE3_Jac.block<3, 3>(0, 3) = voxel->T_C_W_.linear() * T0.linear();
  SE3_Jac.block<3, 3>(0, 0).noalias() = -SE3_Jac.block<3, 3>(0, 3) * bievr::skew(p_j);

  Eigen::RowVector2d grad;
  grad << dIdu, dIdv;
  grad *= map.inv_px_size;
  J = -(grad * SE3_Jac.topRows<2>());
  return true;
}

}  // namespace

int main() {
  bievr::BIEVRMap::Config mcfg;
  mcfg.max_size = 100;
  mcfg.voxel_size = 2.0;
  mcfg.px_size = 0.05;
  mcfg.weighted = false;
  mcfg.smooth = false;

  // Dense 5x5 grid (0.05 m spacing -> adjacent pixels) + a far point in a
  // second voxel so the grid voxel group is processed.
  std::vector<Point> grid;
  for (int iy = 0; iy < 5; ++iy) {
    for (int ix = 0; ix < 5; ++ix) {
      grid.emplace_back(0.1 + 0.05 * ix, 0.1 + 0.05 * iy, 0.0);
    }
  }
  grid.emplace_back(2.5, 0.5, 0.0);
  const int center = 2 * 5 + 2;  // central grid point

  bievr::Pointcloud cloud;
  cloud.resize(grid.size());
  for (size_t i = 0; i < grid.size(); ++i) cloud[i] = grid[i];
  std::vector<double> ranges(grid.size(), 4.0);

  // Pass 1 (probe): discover which pixel each point lands in. The geometry is
  // identical to pass 2, so the local frame (and thus the pixels) are the same.
  bievr::BIEVRMap mapProbe(mcfg);
  {
    bievr::Intensities inten(1, grid.size());
    inten.setZero();
    if (!mapProbe.integratePoints(cloud, &ranges, &inten)) return fail("probe integration failed");
  }
  const Voxel* probe = mapProbe.getVoxel(mapProbe.hashIndex(cloud[center]));
  if (!probe) return fail("probe voxel not observed");
  std::vector<std::pair<int, int>> pixels(grid.size());
  for (size_t i = 0; i < grid.size(); ++i) {
    const Point p_O = probe->T_C_W_ * cloud[i];
    pixels[i] = {static_cast<int>(std::round(p_O.x() / 0.05)),
                 static_cast<int>(std::round(p_O.y() / 0.05))};
  }

  // Pass 2: rebuild with a linear ramp in pixel space so the image gradient is
  // exact and the analytic central-difference gradient matches the true one.
  bievr::BIEVRMap map(mcfg);
  bievr::Intensities inten(1, grid.size());
  for (size_t i = 0; i < grid.size(); ++i) {
    inten(0, i) = (i >= 25)
                      ? 0.0f
                      : static_cast<float>(10.0 * pixels[i].first + 5.0 * pixels[i].second);
  }
  if (!map.integratePoints(cloud, &ranges, &inten)) return fail("map integration failed");

  const Point p_j = cloud[center];
  const double I_i = inten(0, center);

  const Transform T0 = Transform::Identity();
  Row6 J_analytic;
  if (!analyticJacobian(map, T0, p_j, I_i, J_analytic)) return fail("analytic jacobian failed");

  const double eps = 1e-6;
  Row6 J_numeric;
  for (int d = 0; d < 6; ++d) {
    Eigen::Matrix<double, 6, 1> xi_p = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> xi_m = Eigen::Matrix<double, 6, 1>::Zero();
    xi_p(d) = eps;
    xi_m(d) = -eps;
    const double r_p = residual(map, T0, p_j, I_i, xi_p);
    const double r_m = residual(map, T0, p_j, I_i, xi_m);
    if (std::isnan(r_p) || std::isnan(r_m)) return fail("numeric residual out of support");
    J_numeric(0, d) = (r_p - r_m) / (2.0 * eps);
  }

  double max_abs = 0.0;
  for (int d = 0; d < 6; ++d) max_abs = std::max(max_abs, std::abs(J_numeric(0, d)));
  const double scale = std::max(max_abs, 1.0);
  for (int d = 0; d < 6; ++d) {
    if (std::abs(J_analytic(0, d) - J_numeric(0, d)) > 1e-3 * scale) {
      std::cerr << "dof " << d << ": analytic " << J_analytic(0, d) << " numeric "
                << J_numeric(0, d) << '\n';
      return fail("photometric jacobian mismatch (finite difference)");
    }
  }

  return 0;
}
