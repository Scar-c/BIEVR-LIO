#!/usr/bin/env bash
# Round-8 FlatSurfacesS photometric-effectiveness runner.
#
# DO NOT start from a mid-sequence crop.
# DO NOT sweep lambda.
# DO NOT tune preprocessing.
#
# Runs the four fixed groups on the FULL bag from t0:
#   B0  : Original BIEVR (params + gamma sensor config, intensity OFF)
#   C0  : COIN-BIEVR pipeline, photo optimization OFF
#   L1  : COIN-BIEVR, photo optimization ON, lambda = 0.001
#   L2  : COIN-BIEVR, photo optimization ON, lambda = 0.003
#
# Usage:
#   scripts/run_flatsurfaces_round8.sh <flatsurfaces.bag> [--label mylabel]
set -euo pipefail

BAG="${1:?FlatSurfacesS bag path required}"
LABEL="${2:-flatsurfaces_round8}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
SENSOR_CFG="$REPO/config/sensor_configs/gamma.yaml"
AVIA_CFG="$REPO/config/params_coin_bievr_avia.yaml"
PARAMS_CFG="$REPO/config/params.yaml"
RESULTS="$REPO/results/$LABEL"
WS_DEVEL="${WS_DEVEL:-/home/lc/algorithm_versa/devel}"

source /opt/ros/noetic/setup.bash
source "$WS_DEVEL/setup.bash"

mkdir -p "$RESULTS"
# Dataset / GT / evaluator info stubs (filled by the analysis script).
: > "$RESULTS/dataset_info.txt"
: > "$RESULTS/gt_info.txt"
: > "$RESULTS/evaluator_info.txt"

run_coin() {
  local name="$1"; shift
  local dir="$RESULTS/$name"
  mkdir -p "$dir"
  # args: sed expressions applied to the Avia base config
  sed -e "s#^  photo_diagnostics_path: .*#  photo_diagnostics_path: \"$dir/photo.csv\"#" \
      -e "s#^  trajectory_path: .*#  trajectory_path: \"$dir/trajectory.tum\"#" \
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
}

# B0: true BIEVR baseline.
mkdir -p "$RESULTS/B0"
sed -e "s#^  trajectory_path: .*#  trajectory_path: \"$RESULTS/B0/trajectory.tum\"#" \
    -e "s/^  timing: .*/  timing: True/" \
    -e "s/^  dashboard: .*/  dashboard: False/" \
    "$PARAMS_CFG" > "$RESULTS/B0/config_used.yaml"
echo "=== B0 ==="
/usr/bin/time -v rosrun bievr_lio_ros process_bag \
  --params_file "$RESULTS/B0/config_used.yaml" \
  --sensor_config_file "$SENSOR_CFG" --bag "$BAG" \
  > "$RESULTS/B0/stdout.log" 2> "$RESULTS/B0/stderr.log" || true
grep -E "Maximum resident|Elapsed" "$RESULTS/B0/stderr.log" > "$RESULTS/B0/timing.txt" || true

# C0 / L1 / L2: COIN pipeline (same bootstrap/preprocessing/map/sampling; only
# photometric enabled + lambda differ).
run_coin C0 -e "s/^    shadow_diagnostics: .*/    shadow_diagnostics: True/"
run_coin L1_0p001 \
  -e "s/^    enabled: False # Round 4-7.*/    enabled: True/" \
  -e "s/^    photometric_scale: .*/    photometric_scale: 0.001/" \
  -e "s/^    shadow_diagnostics: .*/    shadow_diagnostics: False/"
run_coin L2_0p003 \
  -e "s/^    enabled: False # Round 4-7.*/    enabled: True/" \
  -e "s/^    photometric_scale: .*/    photometric_scale: 0.003/" \
  -e "s/^    shadow_diagnostics: .*/    shadow_diagnostics: False/"

echo "round-8 done -> $RESULTS"
