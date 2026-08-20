#!/usr/bin/env python3
"""Round-6 photometric derivative validation analysis.

Reads the photometric safety CSV (shadow run) and reports the three-level
finite-difference validation (Level A image-space, Level B fixed-correspondence
pose Jacobian, Level C full-lookup switching) with the fixed PASS/FAIL gates:

  Level A/B: median normalized error < 1e-3, P95 < 1e-2, sign agreement > 99.5%

Usage:
  python3 scripts/analyze_photo_derivative.py results/photo_safety/avia_shield1/shadow_r6/photo.csv
"""
import argparse
import csv
import os
import statistics
import sys


def read_csv(p):
    with open(p, newline="") as f:
        return list(csv.DictReader(f))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="shadow photometric diagnostics CSV")
    args = ap.parse_args()

    rows = read_csv(args.csv)
    fd = [r for r in rows if r.get("fd_level_a_samples") and int(r["fd_level_a_samples"]) > 0]
    if not fd:
        print("Level A/B/C pools not finalized (no frames reached the sample targets).")
        sys.exit(2)
    r = fd[-1]

    print("================================================")
    print("Level A — image-space derivative (real map)")
    print("================================================")
    a_samples = int(r["fd_level_a_samples"])
    a_med, a_p95, a_sign = map(float, (r["fd_level_a_median"], r["fd_level_a_p95"], r["fd_level_a_sign"]))
    a_pass = a_med < 1e-3 and a_p95 < 1e-2 and a_sign > 99.5
    print(f"samples       : {a_samples}")
    print(f"median err    : {a_med:.4g}")
    print(f"P95 err       : {a_p95:.4g}")
    print(f"sign agree    : {a_sign:.2f}%")
    print(f"PASS/FAIL     : {'PASS' if a_pass else 'FAIL'}")

    print("\n================================================")
    print("Level B — fixed-correspondence pose Jacobian")
    print("================================================")
    b_points = int(r["fd_level_b_points"])
    b_scalars = int(r["fd_level_b_scalars"])
    b_med, b_p95, b_sign = map(float, (r["fd_level_b_median"], r["fd_level_b_p95"], r["fd_level_b_sign"]))
    b_pass = b_med < 1e-3 and b_p95 < 1e-2 and b_sign > 99.5
    print(f"points        : {b_points}")
    print(f"scalars       : {b_scalars}")
    print(f"overall med   : {b_med:.4g}")
    print(f"overall P95   : {b_p95:.4g}")
    print(f"overall sign  : {b_sign:.2f}%")
    for nm in ("rx", "ry", "rz", "tx", "ty", "tz"):
        m, p = float(r[f"fd_b_{nm}_med"]), float(r[f"fd_b_{nm}_p95"])
        print(f"  {nm}: med={m:.4g} P95={p:.4g}")
    print(f"analytic J P50/P90/P99 : {r['fd_b_analytic_p50']}/{r['fd_b_analytic_p90']}/{r['fd_b_analytic_p99']}")
    print(f"numeric  J P50/P90/P99 : {r['fd_b_numeric_p50']}/{r['fd_b_numeric_p90']}/{r['fd_b_numeric_p99']}")
    print(f"PASS/FAIL     : {'PASS' if b_pass else 'FAIL'}")

    print("\n================================================")
    print("Level C — full-lookup nonsmoothness (diagnostic)")
    print("================================================")
    print(f"voxel switch ratio   : {float(r['fd_c_voxel_switch']):.4f}%")
    print(f"cell switch ratio    : {float(r['fd_c_cell_switch']):.4f}%")
    print(f"validity switch ratio: {float(r['fd_c_validity_switch']):.4f}%")
    print(f"no-switch samples    : {r['fd_c_noswitch_samples']}")
    print(f"no-switch median/P95 : {r['fd_c_noswitch_median']}/{r['fd_c_noswitch_p95']}")

    print("\n================================================")
    print("Round-6 verdict")
    print("================================================")
    print(f"Level A: {'PASS' if a_pass else 'FAIL'}")
    print(f"Level B: {'PASS' if b_pass else 'FAIL'}")
    print(f"Level C: diagnostic only")
    if a_pass and b_pass:
        print("Photometric linearization is mathematically consistent.")
        print("Photometric optimization still NOT enabled (Huber/lambda deferred).")
    else:
        print("Derivative validation FAILED; do not enable photometric optimization.")


if __name__ == "__main__":
    main()
