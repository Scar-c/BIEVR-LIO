#!/usr/bin/env bash
# Round-9 Phase A: Shield1 corrected-production photometric revalidation.
#
# Uses the previously-validated GEODE Shield_tunnel1 99 s stationary-start
# prefix (results/intensity_preprocess/avia_shield1/segment_init.bag). Starts
# from the bag start; NEVER from a mid-motion crop.
#
# Runs:
#   S-C0 : COIN pipeline, photo OFF, shadow diagnostics ON (serial vs
#          production H/b/cost parity + robust shadow lambda scan)
#   S-L1 : lambda = 0.001, photo ON
#   S-L2 : lambda = 0.003, photo ON   (only if S-L1 is stable; gate below)
#
# Usage:
#   scripts/run_round9_phase_a.sh
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
BAG="$REPO/results/intensity_preprocess/avia_shield1/segment_init.bag"
SENSOR_CFG="$REPO/config/sensor_configs/gamma.yaml"
AVIA_CFG="$REPO/config/params_coin_bievr_avia.yaml"
RESULTS="$REPO/results/photo_parity_round9/shield1"
WS_DEVEL="${WS_DEVEL:-/home/lc/algorithm_versa/devel}"

source /opt/ros/noetic/setup.bash
source "$WS_DEVEL/setup.bash"

mkdir -p "$RESULTS"
: > "$RESULTS/dataset_info.txt"
: > "$RESULTS/gt_info.txt"
: > "$RESULTS/evaluator_info.txt"
echo "bag: $BAG" > "$RESULTS/dataset_info.txt"
timeout 20 rosbag info "$BAG" 2>/dev/null | grep -iE "duration|topics" >> "$RESULTS/dataset_info.txt" || true
echo "gt: $REPO/bag/GEODE/Shield_tunnel1.txt" > "$RESULTS/gt_info.txt"

run_coin() {
  local name="$1"; shift
  local dir="$RESULTS/$name"
  mkdir -p "$dir"
  sed -e "s#^  photo_diagnostics_path: .*#  photo_diagnostics_path: \"$dir/photo.csv\"#" \
      -e "s#^  robust_shadow_scan_path: .*#  robust_shadow_scan_path: \"$dir/robust_shadow_scan.csv\"#" \
      -e "s#^  trajectory_path: .*#  trajectory_path: \"$dir/trajectory.txt\"#" \
      -e "s/^  timing: .*/  timing: True/" \
      -e "s/^  dashboard: .*/  dashboard: False/" \
      "$@" "$AVIA_CFG" > "$dir/config_used.yaml"
  echo "=== $name ==="
  /usr/bin/time -v \
    rosrun bievr_lio_ros process_bag \
      --params_file "$dir/config_used.yaml" \
      --sensor_config_file "$SENSOR_CFG" --bag "$BAG" \
      > "$dir/stdout.log" 2> "$dir/stderr.log" || true
  grep -E "Maximum resident|Elapsed" "$dir/stderr.log" > "$dir/timing.txt" || true
  if [ -f "$dir/photo.csv" ]; then
    echo "  photo rows: $(($(wc -l < "$dir/photo.csv") - 1))"
  fi
}

# Stability gate for the corrected S-L1: no NaN/Inf, bounded trajectory.
stable() {
  local traj="$1"
  python3 - "$traj" <<'EOF'
import sys, math
p = sys.argv[1]
rows = []
for line in open(p):
    parts = line.split()
    if len(parts) < 4: continue
    try:
        t, x, y, z = float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])
    except ValueError:
        continue
    if any(map(lambda v: math.isnan(v) or math.isinf(v), (x, y, z))):
        print("FAIL NaN"); sys.exit(1)
    rows.append((x, y, z))
if not rows:
    print("FAIL empty"); sys.exit(1)
maxdist = 0.0; maxstep = 0.0
x0, y0, z0 = rows[0]
px, py, pz = rows[0]
for x, y, z in rows:
    maxdist = max(maxdist, math.hypot(x - x0, y - y0))
    maxstep = max(maxstep, math.hypot(x - px, y - py))
    px, py, pz = x, y, z
if maxdist > 200.0 or maxstep > 5.0:
    print(f"FAIL maxdist={maxdist:.1f} maxstep={maxstep:.2f}")
    sys.exit(1)
print(f"OK maxdist={maxdist:.2f} maxstep={maxstep:.3f}")
sys.exit(0)
EOF
}

# S-C0: shadow (photo OFF, shadow diagnostics + robust shadow scan ON).
run_coin C0 \
  -e "s/^    shadow_diagnostics: .*/    shadow_diagnostics: True/"

# S-L1: corrected production lambda = 0.001.
run_coin L1_0p001 \
  -e "s/^    enabled: False # Round 4-7.*/    enabled: True/" \
  -e "s/^    photometric_scale: .*/    photometric_scale: 0.001/" \
  -e "s/^    shadow_diagnostics: .*/    shadow_diagnostics: False/"

# Gate: only run S-L2 if S-L1 is stable (round-9 section 31).
if stable "$RESULTS/L1_0p001/trajectory.txt"; then
  echo "S-L1 stable -> running S-L2"
  run_coin L2_0p003 \
    -e "s/^    enabled: False # Round 4-7.*/    enabled: True/" \
    -e "s/^    photometric_scale: .*/    photometric_scale: 0.003/" \
    -e "s/^    shadow_diagnostics: .*/    shadow_diagnostics: False/"
else
  echo "S-L1 NOT stable -> STOP (no L2, no FlatSurfaces)"
  exit 1
fi

echo "round-9 phase A done -> $RESULTS"
