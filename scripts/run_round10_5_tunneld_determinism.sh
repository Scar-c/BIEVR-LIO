#!/usr/bin/env bash
# Round-10.5 Phase A: TunnelD C1 determinism audit (3x strictly sequential).
#
# Uses ONE fixed config snapshot (results/round10_5_audit/tunneld/config_used.yaml,
# SHA256 recorded) writing to results/round10_5_audit/tunneld/out/; after each run
# the outputs are snapshotted into run1/run2/run3 (with hashes) before the next
# run overwrites them, so the config is byte-identical across all runs.
#
# One estimator run at a time; nothing else running. Environment recorded before
# each run.
#
# Usage:
#   scripts/run_round10_5_tunneld_determinism.sh [--single-thread]
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
BAG="/home/lc/algorithm_versa/bag/ENWIDE/2023-08-08-17-50-31-tunnel_d.bag"
SENSOR_CFG="$REPO/config/sensor_configs/enwide.yaml"
ROOT="$REPO/results/round10_5_audit/tunneld"
CFG="$ROOT/config_used.yaml"
OUT="$ROOT/out"
MODE="${1:-multi}"
WS_DEVEL="/home/lc/algorithm_versa/devel"

source /opt/ros/noetic/setup.bash
source "$WS_DEVEL/setup.bash"

mkdir -p "$OUT"
echo "config SHA256: $(sha256sum "$CFG" | cut -d' ' -f1)"

# Optional single-thread mode: reuse the same config but force max_num_threads=1
# (the config option is a clean, minimal thread cap; algorithm math unchanged).
run_one() {
  local name="$1" cfg="$CFG"
  local dir="$ROOT/$name"
  mkdir -p "$dir"
  if [ "$MODE" = "single" ]; then
    cfg="$ROOT/config_single.yaml"
    sed "s/^max_num_threads: .*/max_num_threads: 1/" "$CFG" > "$cfg"
    echo "single-thread config SHA256: $(sha256sum "$cfg" | cut -d' ' -f1)"
  fi

  # Record environment before the run.
  {
    echo "=== $name : $(date) ==="
    echo "hostname: $(hostname)"
    echo "nproc: $(nproc)"
    echo "--- uptime/load ---"; uptime
    echo "--- memory ---"; free -h
    echo "--- running bievr/rosbag/process_bag ---"
    ps aux | grep -E "bievr|process_bag|rosbag" | grep -v grep || echo "(none)"
  } > "$dir/environment_before.txt"

  # Ensure no residual estimator process.
  if pgrep -f "process_bag" > /dev/null; then
    echo "ABORT: process_bag already running"; exit 1
  fi

  echo "=== $name ==="
  /usr/bin/time -v rosrun bievr_lio_ros process_bag __name:=bievr_r105 \
    --params_file "$cfg" --sensor_config_file "$SENSOR_CFG" --bag "$BAG" \
    > "$OUT/stdout.log" 2> "$OUT/stderr.log" || true
  grep -E "Elapsed" "$OUT/stderr.log" > "$dir/timing.txt" || true

  # Snapshot outputs before the next run overwrites them.
  cp "$OUT/trajectory.tum" "$dir/trajectory.tum" 2>/dev/null || true
  cp "$OUT/photo.csv" "$dir/photo.csv" 2>/dev/null || true
  cp "$cfg" "$dir/config_used.yaml"
  cp "$OUT/stdout.log" "$dir/stdout.log" 2>/dev/null || true
  cp "$OUT/stderr.log" "$dir/stderr.log" 2>/dev/null || true
  echo "trajectory SHA256: $(sha256sum "$dir/trajectory.tum" | cut -d' ' -f1)"
  echo "photo CSV   SHA256: $(sha256sum "$dir/photo.csv" | cut -d' ' -f1)"
  echo "poses: $(wc -l < "$dir/trajectory.tum")"
}

run_one run1
run_one run2
run_one run3

echo "round-10.5 Phase A done ($MODE) -> $ROOT"
