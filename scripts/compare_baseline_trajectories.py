#!/usr/bin/env python3
"""Round-10.5 Phase B: compare two TUM trajectories (common timestamps).

Usage: compare_baseline_trajectories.py <a.tum> <b.tum> [checkpoint_step_s]
Prints pairwise translation/rotation diffs and checkpoint positions/dists."""
import math, sys
import numpy as np

def q2R(q):
    qx, qy, qz, qw = q
    return np.array([[1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                     [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
                     [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]])

def load(fn):
    rows = {}
    for line in open(fn):
        p = line.split()
        if len(p) < 8: continue
        try:
            t = float(p[0])
            rows[t] = (np.array([float(p[1]), float(p[2]), float(p[3])]),
                       [float(p[4]), float(p[5]), float(p[6]), float(p[7])])
        except ValueError: continue
    return rows

def pct(v, q):
    v = sorted(v)
    return v[int(q*(len(v)-1))]

def main():
    fa, fb = sys.argv[1], sys.argv[2]
    step = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0
    a, b = load(fa), load(fb)
    common = sorted(set(a) & set(b))
    dt, dr = [], []
    for t in common:
        dt.append(np.linalg.norm(a[t][0]-b[t][0]))
        R = q2R(a[t][1]).T @ q2R(b[t][1])
        cos = max(-1.0, min(1.0, (np.trace(R)-1)/2))
        dr.append(math.degrees(math.acos(cos)))
    print(f"matched={len(common)}")
    print(f"trans mean/P50/P90/P99/max = {np.mean(dt):.4f}/{pct(dt,.5):.4f}/{pct(dt,.9):.4f}/{pct(dt,.99):.4f}/{max(dt):.4f} m")
    print(f"rot   mean/P50/P90/P99/max = {np.mean(dr):.4f}/{pct(dr,.5):.4f}/{pct(dr,.9):.4f}/{pct(dr,.99):.4f}/{max(dr):.4f} deg")
    t0 = min(common)
    a0, b0 = a[t0][0], b[t0][0]
    print("checkpoints (rel t | A dist | B dist):")
    for rel in np.arange(0, (max(common)-t0)+1, step):
        t = min(common, key=lambda x: abs(x-(t0+rel)))
        print(f"  {t-t0:6.1f}s | {np.linalg.norm(a[t][0]-a0):9.2f} | {np.linalg.norm(b[t][0]-b0):9.2f}")

if __name__ == "__main__":
    main()
