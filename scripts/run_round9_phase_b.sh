#!/usr/bin/env bash
# Round-9 Phase B: FlatSurfacesS corrected-production photometric revalidation.
#
# Runs ONLY after Phase A gates: production parity PASS + Shield1 L1 stable
# (and Shield1 L2 stable -> F-L2 executed). Dataset fixed: FlatSurfacesS
# (Flat_Surfaces_Smooth, Livox Avia, gamma, 82 s) from t0.
#
# Runs:
#   F-C0 : COIN pipeline, photo OFF, shadow diagnostics ON (parity + shadow scan)
#   F-L1 : lambda = 0.001, photo ON
#   F-L2 : lambda = 0.003, photo ON
#
# Usage:
#   scripts/run_round9_phase_b.sh
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
BAG="/home/lc/algorithm_versa/bag/ENWIDE/flat_surfaces_smooth.bag"
GT="/home/lc/algorithm_versa/bag/ENWIDE/flat_surfaces_smooth.txt"
SENSOR_CFG="$REPO/config/sensor_configs/gamma.yaml"
AVIA_CFG="$REPO/config/params_coin_bievr_avia.yaml"
RESULTS="$REPO/results/photo_parity_round9/flatsurfaces"
WS_DEVEL="${WS_DEVEL:-/home/lc/algorithm_versa/devel}"

source /opt/ros/noetic/setup.bash
source "$WS_DEVEL/setup.bash"

mkdir -p "$RESULTS"
echo "bag: $BAG" > "$RESULTS/dataset_info.txt"
timeout 20 rosbag info "$BAG" 2>/dev/null | grep -iE "duration|topics" >> "$RESULTS/dataset_info.txt" || true
echo "gt: $GT" > "$RESULTS/gt_info.txt"

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

# F-C0: shadow (photo OFF, shadow diagnostics + robust shadow scan ON).
run_coin C0 \
  -e "s/^    shadow_diagnostics: .*/    shadow_diagnostics: True/"

# F-L1: corrected production lambda = 0.001.
run_coin L1_0p001 \
  -e "s/^    enabled: False # Round 4-7.*/    enabled: True/" \
  -e "s/^    photometric_scale: .*/    photometric_scale: 0.001/" \
  -e "s/^    shadow_diagnostics: .*/    shadow_diagnostics: False/"

# F-L2: corrected production lambda = 0.003 (Shield1 L2 was stable -> run).
run_coin L2_0p003 \
  -e "s/^    enabled: False # Round 4-7.*/    enabled: True/" \
  -e "s/^    photometric_scale: .*/    photometric_scale: 0.003/" \
  -e "s/^    shadow_diagnostics: .*/    shadow_diagnostics: False/"

echo "round-9 phase B done -> $RESULTS"
