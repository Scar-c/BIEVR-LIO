#!/usr/bin/env python3
"""Round-12 post-fix Shield analysis: GEODE evaluation, C1 determinism gate,
old-vs-new comparison and trend classification.

Usage: analyze_round12_shields.py [root] [geode_dir]
"""
import math
import os
import shutil
import subprocess
import sys

import numpy as np

ROOT = sys.argv[1] if len(sys.argv) > 1 else "/home/lc/algorithm_versa/src/BIEVR-LIO/results/round12_shield_rerun"
GEODE = sys.argv[2] if len(sys.argv) > 2 else "/tmp/opencode/GEODE_dataset"
GT_DIR = "/home/lc/algorithm_versa/bag/GEODE"

SEQS = [("shield1", "Shield_tunnel1.txt", 0.256, 0.220),
        ("shield4", "Shield_tunnel4.txt", 0.275, 0.245),
        ("shield5", "Shield_tunnel5.txt", 0.146, 0.219)]
OLD_C1 = {"shield1": 0.4166, "shield4": 1.7772, "shield5": 2.0367}
ORIG_S5 = 2.04

def pct(v, q):
    v = sorted(v)
    if not v: return 0.0
    return v[int(q * (len(v) - 1))]

def gamma2leica(raw_dir):
    work = "/tmp/opencode/geode_r12"
    os.makedirs(f"{work}/raw", exist_ok=True)
    os.makedirs(f"{work}/eval", exist_ok=True)
    for f in os.listdir(f"{work}/raw"):
        os.remove(f"{work}/raw/{f}")
    shutil.copy(os.path.join(raw_dir, "trajectory.tum"), f"{work}/raw/trajectory.txt")
    src = open(f"{GEODE}/script/gamma2GT_leica.py").read()
    src = src.replace('raw_folder = "./shield_tunnel1"', f'raw_folder = "{work}/raw"')
    src = src.replace('output_folder = "./tran2body"', f'output_folder = "{work}/eval"')
    open("/tmp/opencode/gamma2GT_leica_io.py", "w").write(src)
    subprocess.run(["python3", "/tmp/opencode/gamma2GT_leica_io.py"], capture_output=True)
    return f"{work}/eval/trajectory.txt"

def eval_ape(traj, gt):
    out = subprocess.run(["evo_ape", "tum", traj, gt, "-va", "--t_max_diff", "0.1",
                          "--t_offset", "0", "--no_warnings"], capture_output=True, text=True).stdout
    ape = {}
    for line in out.splitlines():
        p = line.split()
        if len(p) == 2 and p[0] in ("rmse", "mean", "median", "max"):
            try: ape[p[0]] = float(p[1])
            except ValueError: pass
    return ape, out

def load_traj(fn):
    rows = {}
    for line in open(fn):
        p = line.split()
        if len(p) < 8: continue
        try:
            rows[float(p[0])] = ([float(p[1]), float(p[2]), float(p[3])],
                                 [float(p[4]), float(p[5]), float(p[6]), float(p[7])])
        except ValueError: continue
    return rows

def pairwise(a, b):
    common = sorted(set(a) & set(b))
    dt, dr = [], []
    for t in common:
        dt.append(math.dist(a[t][0], b[t][0]))
        qa, qb = a[t][1], b[t][1]
        # SO(3) relative AngleAxis via quaternion dot
        dot = abs(qa[0]*qb[0] + qa[1]*qb[1] + qa[2]*qb[2] + qa[3]*qb[3])
        ang = 2.0 * math.degrees(math.acos(min(1.0, dot)))
        dr.append(ang)
    return dt, dr

def main():
    summary = []
    for seq, gt_name, paper_b, paper_c in SEQS:
        gt = f"{GT_DIR}/{gt_name}"
        print(f"===== {seq} =====")
        row = {"seq": seq, "paper_b": paper_b, "paper_c": paper_c,
               "old_c1": OLD_C1[seq], "old_status": "PRE_FIX / NOT_AUTHORITATIVE"}
        for mode in ["B0", "C0", "C1_A", "C1_B"]:
            d = f"{ROOT}/{seq}/{mode}"
            if not os.path.exists(f"{d}/trajectory.tum"):
                print(f"  {mode}: MISSING")
                row[mode] = None
                continue
            traj_g = gamma2leica(d)
            ape, out = eval_ape(traj_g, gt)
            open(f"{d}/evaluation.txt", "w").write(out)
            traj = load_traj(f"{d}/trajectory.tum")
            ts = sorted(traj)
            path = sum(math.dist(traj[ts[i]][0], traj[ts[i-1]][0]) for i in range(1, len(ts)))
            row[mode] = ape
            row[f"{mode}_path"] = path
            row[f"{mode}_sha"] = os.popen(f"sha256sum {d}/trajectory.tum").read().split()[0]
            print(f"  {mode}: APE rmse={ape.get('rmse'):.4f} mean={ape.get('mean'):.4f} "
                  f"median={ape.get('median'):.4f} max={ape.get('max'):.4f} path={path:.1f}m")
            print(f"    SHA: {row[f'{mode}_sha'][:16]}...")
        # C1 determinism gate
        if row.get("C1_A") and row.get("C1_B"):
            ta, tb = load_traj(f"{ROOT}/{seq}/C1_A/trajectory.tum"), load_traj(f"{ROOT}/{seq}/C1_B/trajectory.tum")
            dt, dr = pairwise(ta, tb)
            sha_eq = row["C1_A_sha"] == row["C1_B_sha"]
            photo_a = os.popen(f"sha256sum {ROOT}/{seq}/C1_A/photo.csv").read().split()[0]
            photo_b = os.popen(f"sha256sum {ROOT}/{seq}/C1_B/photo.csv").read().split()[0]
            ape_diff = abs(row["C1_A"]["rmse"] - row["C1_B"]["rmse"])
            gate = (sha_eq and photo_a == photo_b) or (
                ape_diff <= 0.01 and pct(dt, .99) <= 0.01 and max(dt) <= 0.05 and
                pct(dr, .99) <= 0.02 and max(dr) <= 0.10)
            row["c1_gate"] = "PASS" if gate else "FAIL"
            print(f"  C1 determinism: traj SHA equal={sha_eq} photo SHA equal={photo_a==photo_b} "
                  f"APE diff={ape_diff:.4f} trans P99/max={pct(dt,.99):.4f}/{max(dt):.4f} "
                  f"rot P99/max={pct(dr,.99):.4f}/{max(dr):.4f} -> {row['c1_gate']}")
            # trend vs B0
            b0 = row.get("B0")
            c1 = row.get("C1_A")
            if b0 and c1:
                if b0["rmse"] > 2.0:
                    cls = ("B0_DIVERGED_C1_STABLE" if c1.get("max", 0) < 5.0 else
                           "B0_DIVERGED_C1_DIVERGED")
                else:
                    r = c1["rmse"] / b0["rmse"]
                    cls = ("CLEAR_IMPROVEMENT" if r <= 0.90 else
                           "MILD_IMPROVEMENT" if r <= 0.98 else
                           "NEUTRAL" if r < 1.02 else
                           "MILD_WORSE" if r < 1.10 else "CLEAR_WORSE")
                row["trend"] = cls
                print(f"  Trend (C1 vs B0): {cls}")
            # C0 side effect
            if row.get("C0") and b0 and b0["rmse"] < 2.0:
                d = abs(row["C0"]["rmse"] - b0["rmse"]) / b0["rmse"]
                se = "PIPELINE_NEUTRAL" if d <= 0.02 else "PIPELINE_SIDE_EFFECT" if d <= 0.10 else "PIPELINE_LARGE_SIDE_EFFECT"
                row["c0_side"] = se
                print(f"  C0 vs B0: delta {d*100:.1f}% -> {se}")
        summary.append(row)
    print("\n===== summary table =====")
    print(f"{'seq':8} {'B0':>9} {'C0':>9} {'C1_A':>9} {'C1_B':>9} {'gate':>4} {'trend':>28}")
    for r in summary:
        f = lambda k: f"{r[k]['rmse']:.4f}" if r.get(k) else "-"
        print(f"{r['seq']:8} {f('B0'):>9} {f('C0'):>9} {f('C1_A'):>9} {f('C1_B'):>9} "
              f"{r.get('c1_gate','-'):>4} {r.get('trend','-'):>28}")

if __name__ == "__main__":
    main()