// Round 7, robust weighting semantics for the photometric scale.
// Verifies the Accumulator-based accumulation implements rho(lambda*r_photo)
// (COIN-BIEVR Eq. 13) with the shared BIEVR Huber delta:
//   Case A: both in Huber quadratic region -> H/b scale as lambda^2.
//   Case B: both in Huber linear region    -> H/b scale as lambda.
//   Case C: crossing the Huber knee        -> matches Accumulator directly.
//   Case D: cost is rho(lambda*r), NOT lambda^2 * rho(r).

#include "bievr_lio/ls_optimizer.h"

#include <cmath>
#include <iostream>

namespace {

using bievr::Accumulator;
using bievr::Row6;

int fail(const char* msg) {
  std::cerr << msg << '\n';
  return 1;
}

bool near(double a, double b, double tol = 1e-12) { return std::abs(a - b) <= tol; }

// Accumulate a single residual at `lambda` with the same semantics as the
// optimizer (r and J scaled by lambda, then Huber IRLS).
Accumulator accumulate(double r_raw, const Row6& J, double lambda, double delta) {
  Accumulator acc;
  acc.huber_delta = delta;
  const Row6 J_scaled = lambda * J;
  acc.add(lambda * r_raw, &J_scaled);
  return acc;
}

// A single-nonzero-component Jacobian for easy hand-computed H/b.
Row6 jx(double v) {
  Row6 J = Row6::Zero();
  J(0, 0) = v;
  return J;
}

}  // namespace

int main() {
  const double delta = 0.1;
  const Row6 J = jx(1.0);
  const double r = 1.0;

  // --- Case A: both in Huber quadratic region (|lambda*r| <= delta) ---------
  {
    const double l1 = 0.01, l2 = 0.02;  // |0.01|,|0.02| <= 0.1 -> inliers
    const auto a1 = accumulate(r, J, l1, delta);
    const auto a2 = accumulate(r, J, l2, delta);
    const double ratio = (l2 / l1) * (l2 / l1);
    if (!near(a2.H(0, 0) / a1.H(0, 0), ratio)) {
      return fail("Case A: H lambda^2 ratio mismatch");
    }
    if (!near(a2.b(0, 0) / a1.b(0, 0), ratio)) {
      return fail("Case A: b lambda^2 ratio mismatch");
    }
  }

  // --- Case B: both in Huber linear region (|lambda*r| > delta) -------------
  {
    const double rb = 20.0;
    const double l1 = 0.01, l2 = 0.02;  // |0.2|,|0.4| > 0.1 -> outliers
    const auto a1 = accumulate(rb, J, l1, delta);
    const auto a2 = accumulate(rb, J, l2, delta);
    const double ratio = l2 / l1;  // H_photo ~ lambda in the linear region
    if (!near(a2.H(0, 0) / a1.H(0, 0), ratio)) {
      return fail("Case B: H lambda ratio mismatch");
    }
    if (!near(a2.b(0, 0) / a1.b(0, 0), ratio)) {
      return fail("Case B: b lambda ratio mismatch");
    }
  }

  // --- Case C: crossing the Huber knee (one in, one out) --------------------
  {
    const double rc = 15.0;
    const double l1 = 0.005;  // |0.075| <= 0.1 -> quadratic, w=1
    const double l2 = 0.020;  // |0.30| > 0.1 -> linear, w = 0.1/0.3 = 1/3
    const auto a1 = accumulate(rc, J, l1, delta);
    const auto a2 = accumulate(rc, J, l2, delta);
    // Hand-computed expectations.
    const double H1 = l1 * l1;                        // 1*lambda^2
    const double H2 = (1.0 / 3.0) * l2 * l2;          // w*lambda^2
    const double b1 = l1 * l1 * rc;                   // lambda^2 * r
    const double b2 = (1.0 / 3.0) * l2 * l2 * rc;     // w*lambda^2*r
    if (!near(a1.H(0, 0), H1) || !near(a2.H(0, 0), H2)) return fail("Case C: H mismatch");
    if (!near(a1.b(0, 0), b1) || !near(a2.b(0, 0), b2)) return fail("Case C: b mismatch");
  }

  // --- Case D: cost is rho(lambda*r), not lambda^2 * rho(r) -----------------
  {
    const double rd = 20.0;
    const double l = 0.02;  // |0.4| > 0.1 -> linear region
    const auto a = accumulate(rd, J, l, delta);
    const double cost_expected = delta * (l * rd - 0.5 * delta);  // Huber(lambda*r)
    const double cost_wrong = l * l * (delta * (rd - 0.5 * delta));  // lambda^2 * Huber(r)
    if (!near(a.error_sum, cost_expected)) return fail("Case D: cost != rho(lambda*r)");
    if (near(a.error_sum, cost_wrong, 1e-9)) {
      return fail("Case D: cost wrongly equals lambda^2 * rho(r)");
    }
  }

  return 0;
}
