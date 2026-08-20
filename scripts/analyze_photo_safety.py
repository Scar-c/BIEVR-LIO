#!/usr/bin/env python3
"""Round-5 photometric safety analysis.

Reads the per-frame photometric diagnostics CSVs produced by run_photo_safety.sh
(results/photo_safety/<seq>/...) and writes:
  shadow_summary.md / shadow_summary.csv
  lambda_safety_summary.md / lambda_safety_summary.csv

Usage:
  python3 scripts/analyze_photo_safety.py results/photo_safety/avia_shield1 [--out DIR]
"""
import argparse
import csv
import math
import os
import statistics


def read_csv(p):
    with open(p, newline="") as f:
        return list(csv.DictReader(f))


def load_traj(p):
    out = {}
    for line in open(p):
        s = line.split()
        if len(s) < 8:
            continue
        out[round(float(s[0]), 3)] = tuple(map(float, s[1:8]))
    return out


def med(rows, key):
    return statistics.median(float(r[key]) for r in rows)


def pct(vals, q):
    v = sorted(vals)
    return v[int(q * (len(v) - 1))]


def traj_stats(p):
    d = load_traj(p)
    nan = sum(1 for v in d.values() if any(math.isnan(x) or math.isinf(x) for x in v))
    ts = sorted(d)
    L = sum(math.dist(d[ts[i]][0:3], d[ts[i - 1]][0:3]) for i in range(1, len(ts)))
    return len(d), nan, L


def classify(rows, traj_path, fd_ok, base_iter, base_reject, base_runtime):
    n, nan, L = traj_stats(traj_path)
    it = med(rows, "lm_iterations")
    rr = med(rows, "reject_rate")
    ptd = [float(r["photo_step_t_mm"]) for r in rows]
    prd = [float(r["photo_step_r_deg"]) for r in rows]
    fails = []
    warns = []
    if nan > 0 or L > 300:
        fails.append("trajectory NaN/diverged")
    if not fd_ok:
        fails.append("FD Jacobian gate not met")
    if pct(ptd, 0.99) > 50:
        warns.append("photo step translation P99>50mm")
    if max(ptd) > 200:
        fails.append("photo step translation single>200mm")
    if pct(prd, 0.99) > 0.2:
        warns.append("photo step rotation P99>0.2deg")
    if max(prd) > 1.0:
        fails.append("photo step rotation single>1.0deg")
    if rr > 2 * base_reject:
        warns.append("reject rate >2x C0")
    if rr > 4 * base_reject:
        fails.append("reject rate >4x C0")
    if it > 1.5 * base_iter:
        warns.append("LM iterations >1.5x C0")
    if it > 2 * base_iter:
        fails.append("LM iterations >2x C0")
    if warns and not fails:
        return "MARGINAL", warns, fails
    if fails:
        return "UNSAFE", warns, fails
    return "SAFE", warns, fails


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir", help="results/photo_safety/<seq>")
    args = ap.parse_args()
    D = args.dir
    out = args.dir

    # ---------- shadow summary ----------
    shadow_rows = read_csv(os.path.join(D, "shadow", "photo.csv"))
    wm = [r for r in shadow_rows if int(r["valid_matches"]) > 0]
    fd = [r for r in shadow_rows if r["fd_done"] == "1"]
    lam = [float(r["lambda_ref"]) for r in wm if float(r["lambda_ref"]) > 0]
    s = {
        "frames": len(shadow_rows),
        "frames_with_matches": len(wm),
        "candidates_mean": statistics.mean(float(r["candidates"]) for r in shadow_rows),
        "matches_mean": statistics.mean(float(r["valid_matches"]) for r in wm),
        "matches_p50": statistics.median(float(r["valid_matches"]) for r in wm),
        "immature_mean": statistics.mean(float(r["immature_matches"]) for r in shadow_rows),
        "match_ratio": statistics.median(float(r["match_ratio"]) for r in wm),
        "r_p50": statistics.median(float(r["r_abs_p50"]) for r in wm),
        "r_p90": statistics.median(float(r["r_abs_p90"]) for r in wm),
        "r_p95": statistics.median(float(r["r_abs_p95"]) for r in wm),
        "r_signed_med": statistics.median(float(r["r_signed_median"]) for r in wm),
        "grad_p50": statistics.median(float(r["grad_p50"]) for r in wm),
        "grad_p90": statistics.median(float(r["grad_p90"]) for r in wm),
        "grad_p95": statistics.median(float(r["grad_p95"]) for r in wm),
        "J_p50": statistics.median(float(r["J_norm_p50"]) for r in wm),
        "J_p90": statistics.median(float(r["J_norm_p90"]) for r in wm),
        "J_p99": statistics.median(float(r["J_norm_p99"]) for r in wm),
        "J_max": statistics.median(float(r["J_norm_max"]) for r in wm),
        "Jdof_p90": [statistics.median(float(r[f"Jdof_p90_{d}"]) for r in wm) for d in
                     ("rx", "ry", "rz", "tx", "ty", "tz")],
        "H_geo": statistics.median(float(r["H_geo_fro"]) for r in wm),
        "H_photo": statistics.median(float(r["H_photo_fro"]) for r in wm),
        "R_H_unscaled": statistics.median(float(r["R_H_unscaled"]) for r in wm),
        "b_geo": statistics.median(float(r["b_geo_norm"]) for r in wm),
        "b_photo": statistics.median(float(r["b_photo_norm"]) for r in wm),
        "R_b_unscaled": statistics.median(float(r["R_b_unscaled"]) for r in wm),
        "lambda_ref": statistics.median(lam) if lam else 0.0,
        "fd_samples": int(fd[-1]["fd_samples"]) if fd else 0,
        "fd_median": float(fd[-1]["fd_median_rel_err"]) if fd else -1,
        "fd_p95": float(fd[-1]["fd_p95_rel_err"]) if fd else -1,
    }

    with open(os.path.join(out, "shadow_summary.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(s.keys())
        w.writerow(list(s.values()))

    with open(os.path.join(out, "shadow_summary.md"), "w") as f:
        f.write("# Shadow photometric diagnostics (C0)\n\n")
        f.write(f"- frames: {s['frames']} (with matches: {s['frames_with_matches']})\n")
        f.write(f"- candidates/frame mean: {s['candidates_mean']:.0f}\n")
        f.write(f"- valid matches/frame mean: {s['matches_mean']:.0f} (P50 {s['matches_p50']:.0f})\n")
        f.write(f"- immature/frame mean: {s['immature_mean']:.0f}\n")
        f.write(f"- match ratio: {s['match_ratio']:.3f}\n")
        f.write(f"- residual abs P50/P90/P95: {s['r_p50']:.1f}/{s['r_p90']:.1f}/{s['r_p95']:.1f} "
                f"(signed median {s['r_signed_med']:.1f})\n")
        f.write(f"- gradient P50/P90/P95: {s['grad_p50']:.1f}/{s['grad_p90']:.1f}/{s['grad_p95']:.1f}\n")
        f.write(f"- J norm P50/P90/P99/max: {s['J_p50']:.1f}/{s['J_p90']:.1f}/{s['J_p99']:.1f}/{s['J_max']:.1f}\n")
        dofs = ("rx", "ry", "rz", "tx", "ty", "tz")
        f.write("- Jdof P90: " + ", ".join(f"{d}={v:.1f}" for d, v in zip(dofs, s["Jdof_p90"])) + "\n")
        f.write(f"- H_geo fro: {s['H_geo']:.4g}, H_photo fro: {s['H_photo']:.4g}, "
                f"R_H unscaled: {s['R_H_unscaled']:.4g}\n")
        f.write(f"- b_geo: {s['b_geo']:.4g}, b_photo: {s['b_photo']:.4g}, "
                f"R_b unscaled: {s['R_b_unscaled']:.4g}\n")
        f.write(f"- lambda_ref (median): {s['lambda_ref']:.6g}\n")
        f.write(f"- FD check: samples={s['fd_samples']}, median rel err={s['fd_median']:.4g}, "
                f"P95={s['fd_p95']:.4g} "
                f"({'PASS' if s['fd_median'] < 1e-3 and s['fd_p95'] < 1e-2 else 'FAIL'})\n")

    # ---------- lambda safety summary ----------
    base_rows = read_csv(os.path.join(D, "lambda_0", "photo.csv"))
    base_iter = med(base_rows, "lm_iterations")
    base_reject = med(base_rows, "reject_rate")
    fd_ok = s["fd_median"] < 1e-3 and s["fd_p95"] < 1e-2

    rows_out = []
    md_lines = ["| Group | lambda | scaled H/H | stable | iter | reject | step_t P50/P90/P99/max | "
                "step_r P50/P90/P99/max | runtime | classification |"]
    md_lines.append("|---|---|---|---|---|---|---|---|---|---|")
    for name, desc in (("lambda_0", "C0/L0"), ("lambda_01ref", "L1"),
                       ("lambda_03ref", "L2"), ("lambda_1ref", "L3")):
        rows = read_csv(os.path.join(D, name, "photo.csv"))
        traj_path = os.path.join(D, name, "trajectory.txt")
        # The FD Jacobian gate only applies when the photometric term is active
        # (L1..L3); C0/L0 has no photometric contribution.
        cls, warns, fails = classify(rows, traj_path, fd_ok if name != "lambda_0" else True,
                                     base_iter, base_reject, 0.0)
        n, nan, L = traj_stats(traj_path)
        it = med(rows, "lm_iterations")
        rr = med(rows, "reject_rate")
        ptd = [float(r["photo_step_t_mm"]) for r in rows]
        prd = [float(r["photo_step_r_deg"]) for r in rows]
        lam = rows[0]["photometric_scale"]
        Hratio = med(rows, "R_H_scaled")
        rt = 0.0
        timing = os.path.join(D, name, "timing.txt")
        if os.path.exists(timing):
            for line in open(timing):
                if "Elapsed" in line:
                    try:
                        rt = float(line.split("(")[-1].split(")")[0].split(":")[-1])
                    except Exception:
                        rt = 0.0
        rec = {"group": desc, "lambda": lam, "Hphoto/Hgeo": Hratio, "stable": int(nan == 0 and L < 300),
               "path_m": L, "iter": it, "reject": rr,
               "step_t_p50": pct(ptd, 0.5), "step_t_p90": pct(ptd, 0.9),
               "step_t_p99": pct(ptd, 0.99), "step_t_max": max(ptd),
               "step_r_p50": pct(prd, 0.5), "step_r_p90": pct(prd, 0.9),
               "step_r_p99": pct(prd, 0.99), "step_r_max": max(prd),
               "runtime_s": rt, "classification": cls, "warns": ";".join(warns), "fails": ";".join(fails)}
        rows_out.append(rec)
        md_lines.append(f"| {desc} | {float(lam):.6g} | {float(Hratio):.4g} | "
                        f"{'yes' if rec['stable'] else 'NO'} | "
                        f"{it:.1f} | {rr:.3f} | {rec['step_t_p50']:.3f}/{rec['step_t_p90']:.3f}/"
                        f"{rec['step_t_p99']:.3f}/{rec['step_t_max']:.3f} | "
                        f"{rec['step_r_p50']:.5f}/{rec['step_r_p90']:.5f}/{rec['step_r_p99']:.5f}/"
                        f"{rec['step_r_max']:.5f} | {rt:.1f} | {cls} |")
    with open(os.path.join(out, "lambda_safety_summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows_out[0].keys()))
        w.writeheader()
        w.writerows(rows_out)
    with open(os.path.join(out, "lambda_safety_summary.md"), "w") as f:
        f.write("\n".join(md_lines) + "\n")


if __name__ == "__main__":
    main()
