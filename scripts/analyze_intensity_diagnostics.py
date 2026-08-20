#!/usr/bin/env python3
"""Analyze the per-frame intensity preprocessing diagnostics exported by BIEVR.

Reads, for one or more result directories (each produced by one round-4 run):
  diagnostics.csv            per-frame scalar intensity preprocessing stats
  filtered_histogram.csv     per-frame 256-bin filtered intensity histogram

Prints global statistics, writes a summary table (markdown), a 256-bin filtered
histogram CSV and (if matplotlib is available) a histogram PNG. Compares
multiple windows when several directories are given.

Usage:
  python3 scripts/analyze_intensity_diagnostics.py \
      results/intensity_preprocess/avia_shield1/global \
      results/intensity_preprocess/avia_shield1/w41x7 ... \
      [--out results/intensity_preprocess/avia_shield1/summary]
"""
import argparse
import csv
import os
import statistics
import sys


def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def pct(hist, q):
    """Quantile q in [0,100] of a 256-bin histogram (bin centers 0.5..255.5)."""
    total = sum(hist)
    if total == 0:
        return 0.0
    target = q / 100.0 * total
    acc = 0
    for i, c in enumerate(hist):
        acc += c
        if acc >= target:
            return i + 0.5
    return 256.0


def find_histogram_csv(d):
    for name in ("filtered_histogram.csv", "diagnostics_histogram.csv"):
        p = os.path.join(d, name)
        if os.path.exists(p):
            return p
    for name in sorted(os.listdir(d)):
        if name.endswith("_histogram.csv"):
            return os.path.join(d, name)
    return None


def analyze_dir(d):
    rows = read_csv(os.path.join(d, "diagnostics.csv"))
    hist_csv = find_histogram_csv(d)
    if not rows or hist_csv is None:
        return None
    hist_rows = read_csv(hist_csv)

    # Global filtered histogram (sum across frames) -> exact global percentiles.
    hist = [0] * 256
    for hr in hist_rows:
        for i in range(256):
            hist[i] += int(hr.get(str(i), hr.get("v%d" % i, 0)) or 0)
    total_points = sum(int(r["input_points"]) for r in rows)

    def wmed(key):
        vals = [float(r[key]) for r in rows]
        wts = [float(r["input_points"]) for r in rows]
        sw = sum(wts)
        return sum(v * w for v, w in zip(vals, wts)) / sw if sw else 0.0

    return {
        "frames": len(rows),
        "total_points": total_points,
        "elev_min": min(float(r["elev_min_deg"]) for r in rows),
        "elev_max": max(float(r["elev_max_deg"]) for r in rows),
        "elev_p01": wmed("elev_p01_deg"),
        "elev_p99": wmed("elev_p99_deg"),
        "vclamp_mean": statistics.mean(float(r["vertical_clamped_ratio"]) for r in rows),
        "vclamp_max": max(float(r["vertical_clamped_ratio"]) for r in rows),
        "collision_ratio": sum(int(r["collisions"]) for r in rows) / total_points if total_points else 0.0,
        "filtered_p01": pct(hist, 1.0),
        "filtered_p05": pct(hist, 5.0),
        "filtered_p50": pct(hist, 50.0),
        "filtered_p95": pct(hist, 95.0),
        "filtered_p99": pct(hist, 99.0),
        "filtered_mean": wmed("filtered_mean"),
        "sat0": hist[0],
        "sat255": hist[255],
        "sat0_ratio": hist[0] / total_points if total_points else 0.0,
        "sat255_ratio": hist[255] / total_points if total_points else 0.0,
        "raw_min": min(float(r["raw_min"]) for r in rows),
        "raw_max": max(float(r["raw_max"]) for r in rows),
        "raw_p50": wmed("raw_p50"),
        "raw_p95": wmed("raw_p95"),
        "raw_p99": wmed("raw_p99"),
        "preproc_ms": statistics.mean(float(r["intensity_preprocess_ms"]) for r in rows),
        "preproc_ms_p95": sorted(float(r["intensity_preprocess_ms"]) for r in rows)[
            int(0.95 * len(rows))
        ],
        "map_voxels": max(int(r["intensity_map_voxels"]) for r in rows),
        "hist": hist,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--out", default="", help="output prefix for summary.md / hist CSV / PNG")
    args = ap.parse_args()

    stats = []
    for d in args.dirs:
        s = analyze_dir(d)
        if s is None:
            print(f"WARN: no diagnostics.csv in {d}", file=sys.stderr)
            continue
        s["name"] = os.path.basename(os.path.normpath(d))
        stats.append(s)
        hist = s["hist"]
        hist_csv = os.path.join(d, "filtered_intensity_histogram.csv")
        with open(hist_csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["bin_low", "count"])
            for i, c in enumerate(hist):
                w.writerow([i, c])
        print(f"{s['name']}: frames={s['frames']} pts={s['total_points']} "
              f"elev=[{s['elev_min']:.1f},{s['elev_max']:.1f}]deg P1/P99="
              f"[{s['elev_p01']:.1f},{s['elev_p99']:.1f}] "
              f"vclamp mean/max={s['vclamp_mean']*100:.3f}/{s['vclamp_max']*100:.3f}% "
              f"collision={s['collision_ratio']*100:.2f}% "
              f"filtered P1/P50/P99={s['filtered_p01']:.1f}/{s['filtered_p50']:.1f}/"
              f"{s['filtered_p99']:.1f} sat@0={s['sat0_ratio']*100:.1f}% "
              f"sat@255={s['sat255_ratio']*100:.1f}% preproc={s['preproc_ms']:.2f}ms")

    if not stats:
        sys.exit(1)

    out = args.out
    if out:
        os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
        with open(out + ".md", "w") as f:
            f.write("| Config | Window | Vert clamp mean/max | Collision ratio | "
                    "Filtered P1/P50/P99 | Sat@0 | Sat@255 | Preproc ms | Map tex |\n")
            f.write("|---|---|---|---|---|---|---|---|---|\n")
            for s in stats:
                f.write(f"| {s['name']} | | {s['vclamp_mean']*100:.3f}/{s['vclamp_max']*100:.3f}% | "
                        f"{s['collision_ratio']*100:.2f}% | {s['filtered_p01']:.1f}/"
                        f"{s['filtered_p50']:.1f}/{s['filtered_p99']:.1f} | "
                        f"{s['sat0_ratio']*100:.1f}% | {s['sat255_ratio']*100:.1f}% | "
                        f"{s['preproc_ms']:.2f} | |\n")
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots()
            for s in stats:
                ax.plot(range(256), s["hist"], label=s["name"])
            ax.set_xlabel("filtered intensity")
            ax.set_ylabel("count")
            ax.legend()
            fig.tight_layout()
            fig.savefig(out + ".png", dpi=150)
            print(f"wrote {out}.png")
        except Exception as e:  # noqa: BLE001 - optional plotting
            print(f"matplotlib not available, skipping PNG ({e})", file=sys.stderr)


if __name__ == "__main__":
    main()
