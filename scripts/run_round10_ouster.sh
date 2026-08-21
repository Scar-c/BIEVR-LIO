#!/usr/bin/env bash
# Round-10-B: ENWIDE TunnelD Ouster reproduction runner.
#
# O-B0 : Original BIEVR (intensity OFF), sensor_config=enwide
# O-C0 : COIN-BIEVR Ouster preprocessing/map/sampling ON, photo OFF (shadow)
# O-C1 : COIN-BIEVR Ouster, photo ON, lambda=0.001 (PROVISIONAL_FIXED_LAMBDA)
#
# Usage:
#   scripts/run_round10_ouster.sh [B0|C0|C1]   (default: all)
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
BAG="/home/lc/algorithm_versa/bag/ENWIDE/2023-08-08-17-50-31-tunnel_d.bag"
GT="/home/lc/algorithm_versa/bag/ENWIDE/2023-08-08-17-50-31-tunnel_d_gt.txt"
SENSOR_CFG="$REPO/config/sensor_configs/enwide.yaml"
OUSTER_CFG="$REPO/config/params_coin_bievr_ouster_enwide.yaml"
PARAMS_CFG="$REPO/config/params.yaml"
ROOT="$REPO/results/round10/ouster/tunneld"
WS_DEVEL="${WS_DEVEL:-/home/lc/algorithm_versa/devel}"
ONLY="${1:-all}"

source /opt/ros/noetic/setup.bash
source "$WS_DEVEL/setup.bash"
mkdir -p "$ROOT"
[ -f "$ROOT/dataset_info.txt" ] || timeout 20 rosbag info "$BAG" 2>/dev/null | grep -iE "duration|topics|start|end" > "$ROOT/dataset_info.txt" || true
[ -f "$ROOT/gt_info.txt" ] || { echo "gt: $GT" > "$ROOT/gt_info.txt"; ls -la "$GT" 2>/dev/null >> "$ROOT/gt_info.txt" || echo "gt file NOT FOUND" >> "$ROOT/gt_info.txt"; }

run_b0() {
  local dir="$ROOT/B0"; mkdir -p "$dir"
  sed -e "s#^  trajectory_path: .*#  trajectory_path: \"$dir/trajectory.txt\"#" \
      -e "s/^  timing: .*/  timing: True/" -e "s/^  dashboard: .*/  dashboard: False/" \
      "$PARAMS_CFG" > "$dir/config_used.yaml"
  echo "=== O-B0 ==="
  /usr/bin/time -v rosrun bievr_lio_ros process_bag __name:=bievr_ob0 \
    --params_file "$dir/config_used.yaml" --sensor_config_file "$SENSOR_CFG" --bag "$BAG" \
    > "$dir/stdout.log" 2> "$dir/stderr.log" || true
  grep -E "Maximum resident|Elapsed" "$dir/stderr.log" > "$dir/timing.txt" || true
  echo "  traj lines: $(wc -l < "$dir/trajectory.txt" 2>/dev/null)"
}

run_coin() {
  local name="$1"; shift
  local dir="$ROOT/$name"; mkdir -p "$dir"
  sed -e "s#^    ouster_metadata_path: .*#    ouster_metadata_path: \"$REPO/config/ouster/enwide_metadata.yaml\"#" \
      -e "s#^  intensity_diagnostics_path: .*#  intensity_diagnostics_path: \"$dir/diag.csv\"#" \
      -e "s#^  photo_diagnostics_path: .*#  photo_diagnostics_path: \"$dir/photo.csv\"#" \
      -e "s#^  robust_shadow_scan_path: .*#  robust_shadow_scan_path: \"$dir/robust_shadow_scan.csv\"#" \
      -e "s#^  trajectory_path: .*#  trajectory_path: \"$dir/trajectory.txt\"#" \
      -e "s/^  timing: .*/  timing: True/" -e "s/^  dashboard: .*/  dashboard: False/" \
      "$@" "$OUSTER_CFG" > "$dir/config_used.yaml"
  echo "=== $name ==="
  /usr/bin/time -v rosrun bievr_lio_ros process_bag __name:=bievr_ocoin \
    --params_file "$dir/config_used.yaml" --sensor_config_file "$SENSOR_CFG" --bag "$BAG" \
    > "$dir/stdout.log" 2> "$dir/stderr.log" || true
  grep -E "Maximum resident|Elapsed" "$dir/stderr.log" > "$dir/timing.txt" || true
  if [ -f "$dir/photo.csv" ]; then echo "  photo rows: $(($(wc -l < "$dir/photo.csv") - 1))"; fi
}

if [ "$ONLY" = "B0" ] || [ "$ONLY" = "all" ]; then run_b0; fi
if [ "$ONLY" = "C0" ] || [ "$ONLY" = "all" ]; then run_coin C0 -e "s/^    shadow_diagnostics: .*/    shadow_diagnostics: True/"; fi
if [ "$ONLY" = "C1" ] || [ "$ONLY" = "all" ]; then run_coin C1_0p001 \
  -e "s/^    enabled: False # Round 10:.*/    enabled: True/" \
  -e "s/^    shadow_diagnostics: .*/    shadow_diagnostics: False/"; fi

echo "round-10-B done -> $ROOT"
