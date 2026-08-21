#!/usr/bin/env bash
# Round-12: GEODE Shield post-vector<bool>-fix re-benchmark.
#
# Runs B0 / C0 / C1-A / C1-B for Shield1/4/5 on the CURRENT HEAD (Round11 fix
# present), strictly one estimator at a time, each writing its own independent
# output directory from launch. C1 is duplicated (A/B) to gate the parallel
# determinism of the Avia photometric path.
#
# Usage:
#   scripts/run_round12_shields_post_vectorbool.sh
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
SENSOR_CFG="$REPO/config/sensor_configs/gamma.yaml"
AVIA_CFG="$REPO/config/params_coin_bievr_avia.yaml"
PARAMS_CFG="$REPO/config/params.yaml"
ROOT="$REPO/results/round12_shield_rerun"
BAG_ROOT="/home/lc/algorithm_versa/bag/GEODE"
WS_DEVEL="/home/lc/algorithm_versa/devel"
BIN="$WS_DEVEL/.private/bievr_lio_ros/lib/bievr_lio_ros/process_bag"

source /opt/ros/noetic/setup.bash
source "$WS_DEVEL/setup.bash"

mkdir -p "$ROOT"
echo "HEAD: $(git -C "$REPO" rev-parse HEAD)" > "$ROOT/run_header.txt"
echo "binary SHA256: $(sha256sum "$BIN" | cut -d' ' -f1)" >> "$ROOT/run_header.txt"

wait_barrier() {
  for _ in $(seq 1 120); do
    pgrep -x process_bag > /dev/null || return 0
    sleep 1
  done
  echo "STILL RUNNING process_bag; abort" >&2
  exit 1
}

run_one() {
  local seq="$1" bag="$2" name="$3" cfg="$4"
  local dir="$ROOT/$seq/$name"
  mkdir -p "$dir"
  sed -e "s#^  trajectory_path: .*#  trajectory_path: \"$dir/trajectory.tum\"#" \
      -e "s#^  photo_diagnostics_path: .*#  photo_diagnostics_path: \"$dir/photo.csv\"#" \
      -e "s#^  intensity_diagnostics_path: .*#  intensity_diagnostics_path: \"$dir/intensity.csv\"#" \
      -e "s#^  robust_shadow_scan_path: .*#  robust_shadow_scan_path: \"$dir/robust_shadow_scan.csv\"#" \
      -e "s/^  timing: .*/  timing: True/" -e "s/^  dashboard: .*/  dashboard: False/" \
      "$cfg" > "$dir/config_used.yaml"
  {
    echo "=== $seq $name : $(date) ==="; echo "hostname: $(hostname)"
    echo "HEAD: $(git -C "$REPO" rev-parse HEAD)"
    echo "binary: $(sha256sum "$BIN" | cut -d' ' -f1)"
    echo "bag: $bag"; echo "--- load ---"; uptime; echo "--- mem ---"; free -h
    echo "--- process_bag ---"; pgrep -a -x process_bag || echo none
  } > "$dir/environment_before.txt"
  wait_barrier
  echo "=== $seq $name ==="
  /usr/bin/time -v rosrun bievr_lio_ros process_bag __name:=bievr_r12 \
    --params_file "$dir/config_used.yaml" --sensor_config_file "$SENSOR_CFG" --bag "$bag" \
    > "$dir/stdout.log" 2> "$dir/stderr.log" || true
  wait_barrier
  grep -E "Elapsed|Maximum resident" "$dir/stderr.log" > "$dir/timing.txt" || true
  {
    echo "trajectory: $(sha256sum "$dir/trajectory.tum" 2>/dev/null | cut -d' ' -f1)"
    echo "photo:      $(sha256sum "$dir/photo.csv" 2>/dev/null | cut -d' ' -f1)"
    echo "config norm:$(grep -vE 'trajectory_path|photo_diagnostics_path|intensity_diagnostics_path|robust_shadow_scan_path' "$dir/config_used.yaml" | sha256sum | cut -d' ' -f1)"
  } > "$dir/hashes.txt"
  echo "  $(wc -l < "$dir/trajectory.tum") poses"
}

for seq_bag in "Shield_tunnel1_gamma.bag shield1" "Shield_tunnel4_gamma.bag shield4" "Shield_tunnel5_gamma.bag shield5"; do
  set -- $seq_bag
  BAG="$BAG_ROOT/$1"
  SEQ="$2"
  run_one "$SEQ" "$BAG" B0 "$PARAMS_CFG"
  run_one "$SEQ" "$BAG" C0 \
    <(sed "s/^    shadow_diagnostics: .*/    shadow_diagnostics: True/" "$AVIA_CFG")
  run_one "$SEQ" "$BAG" C1_A \
    <(sed -e "s/^    enabled: False # Round 4-7.*/    enabled: True/" \
           -e "s/^    photometric_scale: .*/    photometric_scale: 0.001/" \
           -e "s/^    shadow_diagnostics: .*/    shadow_diagnostics: False/" "$AVIA_CFG")
  run_one "$SEQ" "$BAG" C1_B \
    <(sed -e "s/^    enabled: False # Round 4-7.*/    enabled: True/" \
           -e "s/^    photometric_scale: .*/    photometric_scale: 0.001/" \
           -e "s/^    shadow_diagnostics: .*/    shadow_diagnostics: False/" "$AVIA_CFG")
done

echo "round-12 done -> $ROOT"