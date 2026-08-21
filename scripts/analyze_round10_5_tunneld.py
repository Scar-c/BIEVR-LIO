#!/usr/bin/env python3
"""Round-10.5 Phase A: TunnelD C1 determinism analysis."""
import math, os, subprocess, sys
import numpy as np

T_PRISM = np.array([-0.006253, 0.011775, 0.10825])

def q2R(q):
    qx, qy, qz, qw = q
    return np.array([[1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                     [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
                     [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]])

def load_traj(path):
    rows = {}
    for line in open(path):
        p = line.split()
        if len(p) < 8: continue
        try:
            t = float(p[0]); xyz = np.array([float(p[1]), float(p[2]), float(p[3])])
            q = [float(p[4]), float(p[5]), float(p[6]), float(p[7])]
        except ValueError: continue
        rows[t] = (xyz, q)
    return rows

def pct(v, q):
    v = sorted(v)
    if not v: return 0.0
    return v[int(q*(len(v)-1))]

def so3_angle(R):
    cos = max(-1.0, min(1.0, (np.trace(R) - 1.0)/2.0))
    return math.acos(cos)

def main():
    root, gt = sys.argv[1], sys.argv[2]
    runs = ["run1", "run2", "run3"]
    data = {}
    for name in runs:
        rows = load_traj(os.path.join(root, name, "trajectory.tum"))
        prism = os.path.join(root, name, "trajectory_prism.tum")
        with open(prism, "w") as f:
            for t, (xyz, q) in rows.items():
                p = xyz + q2R(q) @ T_PRISM
                f.write(f"{t:.6f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n")
        ts = sorted(rows)
        path = sum(np.linalg.norm(rows[ts[i]][0]-rows[ts[i-1]][0]) for i in range(1, len(ts)))
        x0 = rows[ts[0]][0]
        maxd = max(np.linalg.norm(rows[t][0]-x0) for t in ts)
        maxstep = max(np.linalg.norm(rows[ts[i]][0]-rows[ts[i-1]][0]) for i in range(1, len(ts)))
        maxrot = 0.0
        for i in range(1, len(ts)):
            R = q2R(rows[ts[i]][1]) @ q2R(rows[ts[i-1]][1]).T
            maxrot = max(maxrot, so3_angle(R))
        # evo APE
        out = subprocess.run(["evo_ape", "tum", prism, gt, "-a", "--pose_relation", "trans_part",
                              "--t_max_diff", "0.1", "--no_warnings"],
                             capture_output=True, text=True).stdout
        ape = {}
        for line in out.splitlines():
            parts = line.split()
            if len(parts) == 2 and parts[0] in ("rmse", "mean", "median", "max", "min"):
                try: ape[parts[0]] = float(parts[1])
                except ValueError: pass
        # photo csv
        ph = os.path.join(root, name, "photo.csv")
        photo = {}
        if os.path.exists(ph):
            import csv as _csv
            pr = list(_csv.DictReader(open(ph)))
            def F(k):
                return [float(r[k]) for r in pr if r[k] not in ("", "nan")]
            photo = {"matches": pct(F("valid_matches"), .5), "r_p50": pct(F("r_abs_p50"), .5),
                     "r_p90": pct(F("r_abs_p90"), .5), "R_H_p50": pct(F("R_H_scaled"), .5),
                     "R_H_p99": pct(F("R_H_scaled"), .99), "iter": pct(F("lm_iterations"), .5),
                     "reject": pct(F("reject_rate"), .5)}
        data[name] = {"rows": rows, "poses": len(rows), "t0": ts[0], "t1": ts[-1], "path": path, "maxd": maxd,
                      "maxstep": maxstep, "maxrot": maxrot, "ape": ape, "photo": photo}
        print(f"== {name} ==")
        print(f"  poses={len(rows)} t=[{ts[0]:.2f},{ts[-1]:.2f}] path={path:.1f}m maxd={maxd:.1f}m maxstep={maxstep:.3f}m maxrot={maxrot*180/math.pi:.3f}deg")
        print(f"  APE rmse/mean/median/max = {ape.get('rmse'):.4f}/{ape.get('mean'):.4f}/{ape.get('median'):.4f}/{ape.get('max'):.4f}")
        if photo:
            print(f"  photo matches={photo['matches']:.0f} r={photo['r_p50']:.1f}/{photo['r_p90']:.1f} R_H={photo['R_H_p50']:.4f}/{photo['R_H_p99']:.4f} iter={photo['iter']:.1f} reject={photo['reject']:.3f}")

    # pairwise
    print("\n== pairwise ==")
    for a, b in [("run1","run2"), ("run1","run3"), ("run2","run3")]:
        ra, rb = data[a]["rows"], data[b]["rows"]
        common = sorted(set(ra) & set(rb))
        dt, dr = [], []
        for t in common:
            dt.append(np.linalg.norm(ra[t][0]-rb[t][0]))
            R = q2R(rb[t][1]) @ q2R(ra[t][1]).T
            dr.append(so3_angle(R)*180.0/math.pi)
        print(f"  {a} vs {b}: matched={len(common)}")
        print(f"    trans mean/P50/P90/P99/max = {np.mean(dt):.4f}/{pct(dt,.5):.4f}/{pct(dt,.9):.4f}/{pct(dt,.99):.4f}/{max(dt):.4f} m")
        print(f"    rot   mean/P50/P90/P99/max = {np.mean(dr):.4f}/{pct(dr,.5):.4f}/{pct(dr,.9):.4f}/{pct(dr,.99):.4f}/{max(dr):.4f} deg")

    apes = [data[r]["ape"]["rmse"] for r in runs]
    print("\nAPE spread: min=%.4f max=%.4f max-min=%.4f" % (min(apes), max(apes), max(apes)-min(apes)))

if __name__ == "__main__":
    main()
