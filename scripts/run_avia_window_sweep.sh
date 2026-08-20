#!/usr/bin/env bash
# Round-4 Livox Avia brightness-window sweep on a short GEODE segment.
#
# For each window it:
#   1. generates a config based on config/params_coin_bievr_avia.yaml with the
#      window override and per-run diagnostics/trajectory paths;
#   2. runs the BIEVR-LIO process_bag node on the short segment (geometry pose
#      only: intensity.optimization.enabled=false);
#   3. saves config_used.yaml / stdout.log / timing.txt / diagnostics.csv /
#      filtered_histogram.csv into results/intensity_preprocess/<seq>/<window>/.
#
# Usage:
#   scripts/run_avia_window_sweep.sh <segment.bag> <seq_name>
set -euo pipefail

BAG="${1:?segment bag path required}"
SEQ="${2:?sequence name required (e.g. avia_shield1)}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
BASE_CFG="$REPO/config/params_coin_bievr_avia.yaml"
SENSOR_CFG="$REPO/config/sensor_configs/gamma.yaml"
RESULTS="$REPO/results/intensity_preprocess/$SEQ"
WS_DEVEL="${WS_DEVEL:-/home/lc/algorithm_versa/devel}"

source /opt/ros/noetic/setup.bash
source "$WS_DEVEL/setup.bash"

declare -A WINDOWS=(
  [global]="0 0"
  [w41x7]="41 7"
  [w81x15]="81 15"
  [w161x31]="161 31"
)

mkdir -p "$RESULTS"
for name in "${!WINDOWS[@]}"; do
  read -r wu wv <<<"${WINDOWS[$name]}"
  d="$RESULTS/$name"
  mkdir -p "$d"
  cfg="$d/config_used.yaml"
  # Generate the window config.
  sed -e "s/^    brightness_window_u: .*/    brightness_window_u: $wu/" \
      -e "s/^    brightness_window_v: .*/    brightness_window_v: $wv/" \
      -e "s#^  intensity_diagnostics_path: .*#  intensity_diagnostics_path: \"$d/diagnostics.csv\"#" \
      -e "s#^  trajectory_path: .*#  trajectory_path: \"$d/trajectory.txt\"#" \
      -e "s/^  timing: .*/  timing: True/" \
      -e "s/^  dashboard: .*/  dashboard: False/" \
      "$BASE_CFG" > "$cfg"
  echo "=== window $name (${wu}x${wv}) ==="
  /usr/bin/time -v \
    rosrun bievr_lio_ros process_bag \
      --params_file "$cfg" \
      --sensor_config_file "$SENSOR_CFG" \
      --bag "$BAG" > "$d/stdout.log" 2> "$d/stderr.log" || true
  grep -E "Maximum resident|Elapsed" "$d/stderr.log" > "$d/timing.txt" || true
  if [ -f "$d/diagnostics.csv" ]; then
    echo "  diagnostics rows: $(($(wc -l < "$d/diagnostics.csv") - 1))"
  else
    echo "  WARNING: diagnostics.csv not produced"
  fi
done
echo "sweep done -> $RESULTS"
