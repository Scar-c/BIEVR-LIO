#!/usr/bin/env bash
# Round-10-A: GEODE / Livox Avia fixed-parameter cross-sequence generalization.
#
# Fixed params (no per-sequence tuning): lambda=0.001, Avia preprocessing frozen
# (77.2 FOV, 1024x64, 41x7 window ref, 140, raw 1.0), map/sampling frozen.
#
# For each of Shield1/4/5 (FULL bag from t0, gamma device):
#   B0 : Original BIEVR (intensity OFF)
#   C0 : COIN pipeline, photo OFF, shadow diagnostics ON
#   C1 : COIN pipeline, photo ON, lambda = 0.001
#
# Usage:
#   scripts/run_round10_avia.sh
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
SENSOR_CFG="$REPO/config/sensor_configs/gamma.yaml"
AVIA_CFG="$REPO/config/params_coin_bievr_avia.yaml"
PARAMS_CFG="$REPO/config/params.yaml"
ROOT="$REPO/results/round10/avia"
BAG_ROOT="/home/lc/algorithm_versa/bag/GEODE"
WS_DEVEL="${WS_DEVEL:-/home/lc/algorithm_versa/devel}"

source /opt/ros/noetic/setup.bash
source "$WS_DEVEL/setup.bash"

run_b0() {
  local seq="$1" bag="$2"
  local dir="$ROOT/$seq/B0"
  mkdir -p "$dir"
  sed -e "s#^  trajectory_path: .*#  trajectory_path: \"$dir/trajectory.txt\"#" \
      -e "s/^  timing: .*/  timing: True/" \
      -e "s/^  dashboard: .*/  dashboard: False/" \
      "$PARAMS_CFG" > "$dir/config_used.yaml"
  echo "=== $seq B0 ==="
  /usr/bin/time -v rosrun bievr_lio_ros process_bag \
    --params_file "$dir/config_used.yaml" --sensor_config_file "$SENSOR_CFG" --bag "$bag" \
    > "$dir/stdout.log" 2> "$dir/stderr.log" || true
  grep -E "Maximum resident|Elapsed" "$dir/stderr.log" > "$dir/timing.txt" || true
}

run_coin() {
  local seq="$1" bag="$2" name="$3"
  local dir="$ROOT/$seq/$name"; shift 3
  mkdir -p "$dir"
  sed -e "s#^  photo_diagnostics_path: .*#  photo_diagnostics_path: \"$dir/photo.csv\"#" \
      -e "s#^  robust_shadow_scan_path: .*#  robust_shadow_scan_path: \"$dir/robust_shadow_scan.csv\"#" \
      -e "s#^  trajectory_path: .*#  trajectory_path: \"$dir/trajectory.txt\"#" \
      -e "s/^  timing: .*/  timing: True/" \
      -e "s/^  dashboard: .*/  dashboard: False/" \
      "$@" "$AVIA_CFG" > "$dir/config_used.yaml"
  echo "=== $seq $name ==="
  /usr/bin/time -v rosrun bievr_lio_ros process_bag \
    --params_file "$dir/config_used.yaml" --sensor_config_file "$SENSOR_CFG" --bag "$bag" \
    > "$dir/stdout.log" 2> "$dir/stderr.log" || true
  grep -E "Maximum resident|Elapsed" "$dir/stderr.log" > "$dir/timing.txt" || true
  if [ -f "$dir/photo.csv" ]; then echo "  photo rows: $(($(wc -l < "$dir/photo.csv") - 1))"; fi
}

for seq_bag in "Shield_tunnel1_gamma.bag shield1" "Shield_tunnel4_gamma.bag shield4" "Shield_tunnel5_gamma.bag shield5"; do
  set -- $seq_bag
  BAG="$BAG_ROOT/$1"
  SEQ="$2"
  mkdir -p "$ROOT/$SEQ"
  timeout 20 rosbag info "$BAG" 2>/dev/null | grep -iE "duration|topics" > "$ROOT/$SEQ/dataset_info.txt" || true
  run_b0 "$SEQ" "$BAG"
  run_coin "$SEQ" "$BAG" C0 -e "s/^    shadow_diagnostics: .*/    shadow_diagnostics: True/"
  run_coin "$SEQ" "$BAG" C1_0p001 \
    -e "s/^    enabled: False # Round 4-7.*/    enabled: True/" \
    -e "s/^    photometric_scale: .*/    photometric_scale: 0.001/" \
    -e "s/^    shadow_diagnostics: .*/    shadow_diagnostics: False/"
done

echo "round-10-A done -> $ROOT"
