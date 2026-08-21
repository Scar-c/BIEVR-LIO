#!/usr/bin/env bash
# Round-10.5 Phase B: Shield5 original-SHA BIEVR baseline run.
#
# Uses the ORIGINAL public BIEVR executable (worktree at the SHA below, separate
# build) with the original config; only the trajectory_path I/O line is added to
# the params copy (diff proven 1 line, algorithm identical).
#
# Usage:
#   scripts/run_round10_5_shield5_original.sh <original_worktree> <original_devel>
set -euo pipefail
ORIG="${1:?original worktree path}"
DEVEL="${2:?original devel setup path}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
BAG="/home/lc/algorithm_versa/bag/GEODE/Shield_tunnel5_gamma.bag"
DIR="$REPO/results/round10_5_audit/shield5/original_21121698"
source /opt/ros/noetic/setup.bash
source "$DEVEL/setup.bash"
mkdir -p "$DIR"
sed "s#^  trajectory_path: \"\" #  trajectory_path: \"$DIR/trajectory.txt\" #" \
    "$ORIG/config/params.yaml" > /tmp/opencode/params_original_traj.yaml
diff "$ORIG/config/params.yaml" /tmp/opencode/params_original_traj.yaml | head -5 || true
{
  echo "=== O-B0 : $(date) ==="; echo "hostname: $(hostname)"; echo "nproc: $(nproc)"
  echo "--- uptime ---"; uptime; echo "--- memory ---"; free -h
  echo "--- running ---"; ps aux | grep -E "bievr|process_bag|rosbag" | grep -v grep || echo "(none)"
} > "$DIR/environment_before.txt"
echo "=== O-B0 (original 21121698) ==="
/usr/bin/time -v rosrun bievr_lio_ros process_bag __name:=bievr_ob0orig \
  --params_file /tmp/opencode/params_original_traj.yaml \
  --sensor_config_file "$ORIG/config/sensor_configs/gamma.yaml" --bag "$BAG" \
  > "$DIR/stdout.log" 2> "$DIR/stderr.log" || true
grep -E "Elapsed" "$DIR/stderr.log" > "$DIR/timing.txt" || true
echo "trajectory lines: $(wc -l < "$DIR/trajectory.txt")"
