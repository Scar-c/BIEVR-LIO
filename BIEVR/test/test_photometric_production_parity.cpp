// Round 9, P0 regression test: serial diagnostic and parallel production
// photometric accumulation must agree (count/cost/H/b) because both now share
// the SAME per-point photometric term implementation. This test FAILS if the
// duplicate inv_px_size in the production Jacobian is ever reintroduced.
//
// Gates (round-9 instruction sections 12-13): count identical; cost / H / b
// relative errors < 1e-9 (1e-10 preferred; TBB reduction order may force 1e-9).

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/common.h"
#include "bievr_lio/ls_optimizer.h"

#include <cmath>
#include <iostream>
#include <vector>

namespace {

int fail(const char* msg) {
  std::cerr << "FAIL: " << msg << '\n';
  return 1;
}

// 7x7 dense grid (0.05 m spacing -> adjacent pixels) + a far point that forces
// a second map voxel (a single-voxel cloud is never inserted - see the map
// hash-boundary logic in BIEVRMap::integratePoints).
// Map intensity = 10*ix + 5*iy (a plane with nonzero gradient). Source
// intensity = map + 2 for even (ix+iy) [Huber quadratic] and map + 200 for odd
// [Huber linear], giving mixed quadratic/linear robust residuals.
void buildCloud(bievr::Pointcloud& cloud, bievr::Intensities& inten,
                bievr::Intensities& inten_src) {
  const int N = 50;
  cloud.resize(N);
  inten = bievr::Intensities(1, N);
  inten_src = bievr::Intensities(1, N);
  int k = 0;
  for (int iy = 0; iy < 7; ++iy) {
    for (int ix = 0; ix < 7; ++ix) {
      cloud[k] << 0.1 + 0.05 * ix, 0.1 + 0.05 * iy, 0.0;
      const float base = static_cast<float>(10.0 * ix + 5.0 * iy);
      inten(0, k) = base;
      inten_src(0, k) = ((ix + iy) % 2 == 0) ? base + 2.0f : base + 200.0f;
      ++k;
    }
  }
  cloud[49] << 2.5, 0.5, 0.0;  // second voxel (1,0)
  inten(0, 49) = 0.0f;
  inten_src(0, 49) = 0.0f;
}

}  // namespace

int main() {
  bievr::BIEVRMap::Config mcfg;
  mcfg.max_size = 100;
  mcfg.voxel_size = 2.0;
  mcfg.px_size = 0.05;
  mcfg.weighted = false;  // every pixel W = 1.0 (mature)
  mcfg.smooth = false;
  mcfg.intensity_enabled = true;
  bievr::BIEVRMap map(mcfg);

  bievr::Pointcloud cloud;
  bievr::Intensities inten, inten_src;
  buildCloud(cloud, inten, inten_src);
  std::vector<double> ranges(cloud.size(), 4.0);
  if (!map.integratePoints(cloud, &ranges, &inten)) return fail("map build failed");

  bievr::RegistrationConfig reg;
  reg.photometric_residual = true;
  reg.photometric_scale = 0.02;  // lambda*r: 0.04 (quadratic) and 4.0 (linear)
  reg.huber_delta = 0.1;
  reg.photometric_warmup_s = 0.0;
  reg.photometric_elapsed_s = 0.0;
  reg.photometric_min_map_weight = 1.0;

  bievr::LsqRegistration opt(map, cloud, cloud, inten_src, reg);

  // Fixed evaluation pose (small translation + rotation so points land at
  // fractional pixels with a nonzero gradient and residual).
  bievr::Transform pose = bievr::Transform::Identity();
  pose.translation() << 0.02, 0.01, 0.005;
  Eigen::AngleAxisd aa(0.01, Eigen::Vector3d(0.3, 0.5, 0.2).normalized());
  pose.linear() = aa.toRotationMatrix();

  const auto p = opt.runPhotometricParity(pose);

  std::cout << "count serial/parallel: " << p.count_serial << " / " << p.count_parallel << '\n';
  std::cout << "cost serial/parallel:  " << p.cost_serial << " / " << p.cost_parallel << '\n';
  std::cout << "cost_rel: " << p.cost_rel << '\n';
  std::cout << "H_rel:    " << p.H_rel << '\n';
  std::cout << "b_rel:    " << p.b_rel << '\n';

  if (p.count_serial != p.count_parallel) {
    return fail("serial and parallel photometric counts differ");
  }
  if (p.count_serial <= 0) return fail("no photometric matches in the parity test");
  // Relative errors normalized by max(1, |x|); hard gate 1e-9.
  if (p.cost_rel >= 1e-9) return fail("photometric cost parity gate exceeded (1e-9)");
  if (p.H_rel >= 1e-9) return fail("photometric H parity gate exceeded (1e-9)");
  if (p.b_rel >= 1e-9) return fail("photometric b parity gate exceeded (1e-9)");

  return 0;
}
