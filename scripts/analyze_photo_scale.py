#!/usr/bin/env python3
"""Round-7 photometric scale safety analysis.

Phase A: reads the robust shadow scan CSV (per-frame, per-lambda direct
robustified photometric contribution) and writes robust_shadow_summary.csv/md
with per-lambda stats + SHADOW_SAFE/MARGINAL/UNSAFE classification.

Phase B: reads the actual C-lambda photo CSVs and trajectories (C0 + 0.001 +
0.003 + 0.010) and writes actual_lambda_summary.csv/md with stability gates.

Usage:
  python3 scripts/analyze_photo_scale.py results/photo_scale/avia_shield1
"""
import argparse
import csv
import math
import os
import statistics


def read_csv(p):
    with open(p, newline="") as f:
        return list(csv.DictReader(f))


def pct(vals, q):
    v = sorted(vals)
    return v[int(q * (len(v) - 1))]


def classify_shadow(lam_rows):
    t = [float(r["pred_t_m"]) for r in lam_rows]
    r_ = [float(r["pred_r_deg"]) for r in lam_rows]
    rh = [float(r["R_H"]) for r in lam_rows]
    if any(not math.isfinite(x) for x in t + r_ + rh):
        return "SHADOW_UNSAFE"
    if (pct(t, 0.99) <= 0.020 and max(t) <= 0.100 and pct(r_, 0.99) <= 0.05 and
            max(r_) <= 0.50 and pct(rh, 0.90) <= 0.10 and max(rh) <= 0.50):
        return "SHADOW_SAFE"
    if max(t) <= 0.200 and max(r_) <= 1.0 and max(rh) <= 1.0:
        return "SHADOW_MARGINAL"
    return "SHADOW_UNSAFE"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir", help="results/photo_scale/<seq>")
    args = ap.parse_args()
    D = args.dir

    # ---------------- Phase A: robust shadow scan ----------------
    rows = read_csv(os.path.join(D, "robust_shadow_scan.csv"))
    lambdas = sorted({float(r["lambda"]) for r in rows})
    lam_rows = {lam: [r for r in rows if float(r["lambda"]) == lam] for lam in lambdas}

    summary = []
    with open(os.path.join(D, "robust_shadow_summary.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["lambda", "raw_huber_knee", "inlier_mean", "inlier_p10", "inlier_p50",
                    "inlier_p90", "R_H_p50", "R_H_p90", "R_H_p99", "R_H_max",
                    "b_photo_p50", "b_photo_p90", "pred_t_p50", "pred_t_p90", "pred_t_p99",
                    "pred_t_max", "pred_r_p50", "pred_r_p90", "pred_r_p99", "pred_r_max",
                    "shadow_classification"])
        for lam in lambdas:
            lr = lam_rows[lam]
            inl = [float(r["huber_inlier_fraction"]) for r in lr]
            rh = [float(r["R_H"]) for r in lr]
            bp = [float(r["b_photo_norm"]) for r in lr]
            t = [float(r["pred_t_m"]) for r in lr]
            r_ = [float(r["pred_r_deg"]) for r in lr]
            knee = float(lr[0]["raw_huber_knee"])
            cls = classify_shadow(lr)
            row = [lam, f"{knee:.4g}", f"{statistics.mean(inl):.3f}", f"{pct(inl,0.10):.3f}",
                   f"{pct(inl,0.50):.3f}", f"{pct(inl,0.90):.3f}", f"{pct(rh,0.50):.5g}",
                   f"{pct(rh,0.90):.5g}", f"{pct(rh,0.99):.5g}", f"{max(rh):.5g}",
                   f"{pct(bp,0.50):.5g}", f"{pct(bp,0.90):.5g}", f"{pct(t,0.50)*1e3:.2f}",
                   f"{pct(t,0.90)*1e3:.2f}", f"{pct(t,0.99)*1e3:.2f}", f"{max(t)*1e3:.2f}",
                   f"{pct(r_,0.50):.4f}", f"{pct(r_,0.90):.4f}", f"{pct(r_,0.99):.4f}",
                   f"{max(r_):.4f}", cls]
            w.writerow(row)
            summary.append(row)

    with open(os.path.join(D, "robust_shadow_summary.md"), "w") as f:
        f.write("| lambda | knee | inlier mean | R_H P50/P90/P99/max | pred_t P99/max(mm) | "
                "pred_r P99/max(deg) | classification |\n")
        f.write("|---|---|---|---|---|---|---|\n")
        for r in summary:
            f.write(f"| {r[0]:.4f} | {r[1]} | {r[2]} | {r[6]}/{r[7]}/{r[8]}/{r[9]} | "
                    f"{r[14]}/{r[15]} | {r[18]}/{r[19]} | {r[20]} |\n")

    # ---------------- Phase B: actual C-lambda summary ----------------
    names = [("lambda_0", "C0"), ("lambda_0001", "0.001"), ("lambda_0003", "0.003"),
             ("lambda_0010", "0.010")]
    base_rows = read_csv(os.path.join(D, "lambda_0", "photo.csv"))
    base_reject = statistics.median(float(r["reject_rate"]) for r in base_rows)
    base_iter = statistics.median(float(r["lm_iterations"]) for r in base_rows)
    base_path = None

    out = []
    with open(os.path.join(D, "actual_lambda_summary.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["group", "lambda", "executed", "stable", "path_m", "reject_rate", "mean_iter",
                    "photo_inlier_mean", "R_H_p50", "R_H_p90", "R_H_p99", "R_H_max",
                    "step_t_p99_mm", "step_t_max_mm", "step_r_p99_deg", "step_r_max_deg",
                    "runtime_s", "classification"])
        for name, desc in names:
            photo = os.path.join(D, name, "photo.csv")
            traj = os.path.join(D, name, "trajectory.txt")
            if not (os.path.exists(photo) and os.path.exists(traj)):
                w.writerow([desc, name.replace("lambda_", ""), "SKIPPED_BY_GATE", "", "", "", "",
                            "", "", "", "", "", "", "", "", "", "", "SKIPPED_BY_GATE"])
                continue
            rows = read_csv(photo)
            # trajectory path length + stability
            d = {}
            for line in open(traj):
                s = line.split()
                if len(s) < 8:
                    continue
                d[float(s[0])] = (float(s[1]), float(s[2]), float(s[3]))
            ts = sorted(d)
            nan = any(any(math.isnan(x) or math.isinf(x) for x in v) for v in d.values())
            path = sum(math.dist(d[ts[i]], d[ts[i - 1]]) for i in range(1, len(ts)))
            if desc == "C0":
                base_path = path
            stable = (nan == 0) and (base_path is None or path <= 1.2 * base_path)
            reject = statistics.median(float(r["reject_rate"]) for r in rows)
            it = statistics.median(float(r["lm_iterations"]) for r in rows)
            inl = statistics.mean(float(r["photo_huber_inlier_fraction"]) for r in rows) \
                if "photo_huber_inlier_fraction" in rows[0] else 0.0
            rh = [float(r["R_H_scaled"]) for r in rows]
            st = [float(r["photo_step_t_mm"]) for r in rows]
            sr = [float(r["photo_step_r_deg"]) for r in rows]
            rt = 0.0
            timing = os.path.join(D, name, "timing.txt")
            if os.path.exists(timing):
                for line in open(timing):
                    if "Elapsed" in line and "(" in line:
                        try:
                            rt = float(line.split("(")[-1].split(")")[0].split(":")[-1])
                        except Exception:
                            rt = 0.0
            # classification (actual safety gates)
            fails = []
            if not stable:
                fails.append("trajectory")
            if pct(st, 0.99) > 50:
                pass  # P99 threshold -> MARGINAL
            if max(st) > 200:
                fails.append("step_t_max")
            if max(sr) > 1.0:
                fails.append("step_r_max")
            if reject > base_reject + 0.15:
                fails.append("reject+0.15")
            if it > base_iter + 2:
                fails.append("iter+2")
            if fails:
                cls = "UNSAFE"
            elif max(st) > 200 or max(sr) > 1.0 or pct(st, 0.99) > 50 or pct(sr, 0.99) > 0.2:
                cls = "MARGINAL"
            else:
                cls = "SAFE"
            row = [desc, name.replace("lambda_", ""), "yes", "yes" if stable else "NO",
                   f"{path:.1f}", f"{reject:.3f}", f"{it:.1f}", f"{inl:.3f}",
                   f"{pct(rh,0.50):.5g}", f"{pct(rh,0.90):.5g}", f"{pct(rh,0.99):.5g}",
                   f"{max(rh):.5g}", f"{pct(st,0.99):.2f}", f"{max(st):.2f}",
                   f"{pct(sr,0.99):.4f}", f"{max(sr):.4f}", f"{rt:.1f}", cls]
            w.writerow(row)
            out.append(row)

    with open(os.path.join(D, "actual_lambda_summary.md"), "w") as f:
        f.write("| Group | executed | stable | path(m) | reject | iter | R_H P50/P90/max | "
                "step_t P99/max(mm) | step_r P99/max(deg) | classification |\n")
        f.write("|---|---|---|---|---|---|---|---|---|---|\n")
        for r in out:
            f.write(f"| {r[0]} | {r[2]} | {r[3]} | {r[4]} | {r[5]} | {r[6]} | "
                    f"{r[8]}/{r[9]}/{r[11]} | {r[12]}/{r[13]} | {r[14]}/{r[15]} | {r[17]} |\n")


if __name__ == "__main__":
    main()
