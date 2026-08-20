#!/usr/bin/env python3
"""Round-8 FlatSurfacesS analysis.

Reads the four group trajectories + photo diagnostics under results/<label>/
and, if the GEODE official GT and evo_ape are available, computes APE. It
aggregates coverage, stability, divergence onset, photo/geometry diagnostics and
writes round8_summary.csv / round8_summary.md.

Usage:
  python3 scripts/analyze_flatsurfaces_round8.py results/flatsurfaces_round8 \
      --gt <Flat_Surfaces_Smooth_gt.tum> [--evaluator geode_rmse.py]
"""
import argparse
import csv
import math
import os
import shutil
import statistics
import subprocess
import sys


def read_csv(p):
    if not os.path.exists(p):
        return []
    with open(p, newline="") as f:
        return list(csv.DictReader(f))


def load_traj(p):
    d = {}
    for line in open(p):
        s = line.split()
        if len(s) < 8:
            continue
        d[round(float(s[0]), 3)] = tuple(map(float, s[1:8]))
    return d


def pct(vals, q):
    v = sorted(vals)
    return v[int(q * (len(v) - 1))]


def run_evo_ape(gt, est):
    """Run evo_ape tum (no offset search, t_max_diff 0.1, no alignment change)."""
    cmd = ["evo_ape", "tum", gt, est, "-a", "--t_max_diff", "0.1",
           "--n_to_align", "-1", "--pose_relation", "trans_part", "-as"]
    out = subprocess.run(cmd, capture_output=True, text=True)
    if out.returncode != 0:
        return None, out.stderr
    # Parse the 'Statistics' block.
    stats = {}
    for line in out.stdout.splitlines():
        if ":" in line and line.split(":")[0].strip() in ("rmse", "mean", "median", "max", "min"):
            try:
                stats[line.split(":")[0].strip()] = float(line.split(":")[1].strip().split()[0])
            except Exception:
                pass
    return stats, out.stdout


def divergence_onset(est, gt):
    """First time translation APE(t) is sustained (>1.0 s) above thresholds."""
    if not gt:
        return None, None, None
    common = sorted(set(est) & set(gt))
    if len(common) < 10:
        return None, None, None
    # TUM: x y z qx qy qz qw
    def trans_diff(a, b):
        return math.dist(a[0:3], b[0:3])
    times = []
    ape = []
    for t in common:
        times.append(t)
        ape.append(trans_diff(est[t], gt[t]))
    out = {}
    for thr in (0.5, 1.0, 2.0):
        onset = None
        i = 0
        while i < len(times):
            if ape[i] > thr:
                j = i
                while j < len(times) and times[j] - times[i] <= 1.0 and ape[j] > thr:
                    j += 1
                if j < len(times) and times[j] - times[i] >= 1.0 and all(
                        a > thr for a in ape[i:j]):
                    onset = times[i]
                    break
            i += 1
        out[thr] = onset
    return out[0.5], out[1.0], out[2.0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--gt", default="", help="Flat_Surfaces_Smooth GT TUM file")
    ap.add_argument("--evaluator", default="", help="GEODE official rmse.py (optional)")
    args = ap.parse_args()
    D = args.dir
    gt = load_traj(args.gt) if args.gt else None

    groups = [("B0", "B0", None), ("C0", "C0", None), ("L1_0p001", "L1", 0.001),
              ("L2_0p003", "L2", 0.003)]

    rows = []
    with open(os.path.join(D, "round8_summary.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["group", "lambda", "coverage", "path_m", "APE_RMSE", "APE_mean", "APE_median",
                    "APE_max", "t_APE_0.5", "t_APE_1.0", "t_APE_2.0", "photo_matches_mean",
                    "photo_inlier_fraction", "R_H_P50", "R_H_P90", "R_H_P99", "R_H_max",
                    "LM_iterations_mean", "reject_rate", "runtime", "classification"])
        for name, desc, lam in groups:
            traj = os.path.join(D, name, "trajectory.tum")
            if not os.path.exists(traj):
                w.writerow([desc, lam if lam else "", "INCOMPLETE", "", "", "", "", "", "", "", "",
                            "", "", "", "", "", "", "", "", "", "FAIL"])
                continue
            d = load_traj(traj)
            ts = sorted(d)
            nan = any(any(math.isnan(x) or math.isinf(x) for x in v) for v in d.values())
            path = sum(math.dist(d[ts[i]], d[ts[i - 1]]) for i in range(1, len(ts)))
            span = ts[-1] - ts[0] if ts else 0.0
            # coverage vs GT span (or nominal).
            gt_span = (max(gt) - min(gt)) if gt else span
            coverage = (span / gt_span) if gt_span > 0 else (1.0 if span > 0 else 0.0)
            ape = run_evo_ape(args.gt, traj)[0] if args.gt else None
            t05, t10, t20 = divergence_onset(d, gt)
            # photo diagnostics
            ph = read_csv(os.path.join(D, name, "photo.csv"))
            matches = statistics.mean(float(r["valid_matches"]) for r in ph) if ph else None
            rh = [float(r["R_H_scaled"]) for r in ph] if ph else []
            it = statistics.median(float(r["lm_iterations"]) for r in ph) if ph else None
            rr = statistics.median(float(r["reject_rate"]) for r in ph) if ph else None
            rt = 0.0
            timing = os.path.join(D, name, "timing.txt")
            if os.path.exists(timing):
                for line in open(timing):
                    if "Elapsed" in line and "(" in line:
                        try:
                            rt = float(line.split("(")[-1].split(")")[0].split(":")[-1])
                        except Exception:
                            rt = 0.0
            # classification
            if nan or coverage < 0.95:
                cls = "FAIL"
            elif ape is None:
                cls = "NUMERICALLY_STABLE" if not nan else "DIVERGED"
            elif ape["rmse"] < 0.50:
                cls = "STRONG_RESCUE"
            elif ape["rmse"] < 1.0:
                cls = "RESCUE_BUT_INACCURATE"
            else:
                cls = "STABLE_BUT_POOR"
            row = [desc, lam if lam else "", f"{coverage*100:.1f}%", f"{path:.2f}",
                   f"{ape['rmse']:.4f}" if ape else "N/A",
                   f"{ape['mean']:.4f}" if ape else "N/A",
                   f"{ape['median']:.4f}" if ape else "N/A",
                   f"{ape['max']:.4f}" if ape else "N/A",
                   f"{t05-ts[0]:.1f}" if t05 else "N/A",
                   f"{t10-ts[0]:.1f}" if t10 else "N/A",
                   f"{t20-ts[0]:.1f}" if t20 else "N/A",
                   f"{matches:.0f}" if matches is not None else "N/A",
                   "N/A",
                   f"{pct(rh,0.5):.5g}" if rh else "N/A",
                   f"{pct(rh,0.9):.5g}" if rh else "N/A",
                   f"{pct(rh,0.99):.5g}" if rh else "N/A",
                   f"{max(rh):.5g}" if rh else "N/A",
                   f"{it:.1f}" if it is not None else "N/A",
                   f"{rr:.3f}" if rr is not None else "N/A",
                   f"{rt:.1f}", cls]
            w.writerow(row)
            rows.append((desc, row, ape))

    with open(os.path.join(D, "round8_summary.md"), "w") as f:
        f.write("| Group | lambda | coverage | path(m) | APE RMSE | APE mean | APE max | "
                "t_APE0.5 | t_APE2.0 | classification |\n")
        f.write("|---|---|---|---|---|---|---|---|---|---|\n")
        for desc, row, ape in rows:
            f.write(f"| {desc} | {row[1]} | {row[2]} | {row[3]} | {row[4]} | {row[5]} | "
                    f"{row[7]} | {row[8]} | {row[10]} | {row[20]} |\n")
        if not args.gt:
            f.write("\nGT_EVALUATION_BLOCKED: evo_ape/GT unavailable or not provided.\n")


if __name__ == "__main__":
    main()
