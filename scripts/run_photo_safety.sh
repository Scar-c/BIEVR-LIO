#!/usr/bin/env bash
# Round-5 photometric safety validation on a short GEODE/Avia segment.
#
# Runs:
#   shadow     : C0 shadow photometric diagnostics (photo residual/J/H/b computed
#                but never merged; includes the real-map finite-difference check).
#   lambda_0   : photometric residual with weight 0 (L0).
#   lambda_01ref / lambda_03ref / lambda_1ref : lambda sweep around lambda_ref
#                (0.1 / 0.3 / 1.0 x lambda_ref) where lambda_ref makes the photo
#                Hessian ~10% of the geometry Hessian.
#
# Usage:
#   scripts/run_photo_safety.sh <segment.bag> <seq_name>
set -euo pipefail

BAG="${1:?segment bag path required}"
SEQ="${2:?sequence name required (e.g. avia_shield1)}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
BASE_CFG="$REPO/config/params_coin_bievr_avia.yaml"
SENSOR_CFG="$REPO/config/sensor_configs/gamma.yaml"
RESULTS="$REPO/results/photo_safety/$SEQ"
WS_DEVEL="${WS_DEVEL:-/home/lc/algorithm_versa/devel}"

source /opt/ros/noetic/setup.bash
source "$WS_DEVEL/setup.bash"

run_one() {
  local dir="$1"; shift
  mkdir -p "$dir"
  local cfg="$dir/config_used.yaml"
  # args: pairs of sed expressions applied to the base config
  sed -e "s#^  intensity_diagnostics_path: .*#  intensity_diagnostics_path: \"$dir/diag.csv\"#" \
      -e "s#^  photo_diagnostics_path: .*#  photo_diagnostics_path: \"$dir/photo.csv\"#" \
      -e "s#^  trajectory_path: .*#  trajectory_path: \"$dir/trajectory.txt\"#" \
      -e "s/^  timing: .*/  timing: True/" \
      -e "s/^  dashboard: .*/  dashboard: False/" \
      "$@" "$BASE_CFG" > "$cfg"
  echo "=== $(basename "$dir") ==="
  /usr/bin/time -v \
    rosrun bievr_lio_ros process_bag \
      --params_file "$cfg" --sensor_config_file "$SENSOR_CFG" --bag "$BAG" \
      > "$dir/stdout.log" 2> "$dir/stderr.log" || true
  grep -E "Maximum resident|Elapsed" "$dir/stderr.log" > "$dir/timing.txt" || true
  if [ -f "$dir/photo.csv" ]; then
    echo "  photo rows: $(($(wc -l < "$dir/photo.csv") - 1))"
  else
    echo "  WARNING: photo.csv not produced"
  fi
}

mkdir -p "$RESULTS"

# 1. C0 shadow photometric diagnostics (photo OFF, shadow ON, FD check ON).
run_one "$RESULTS/shadow" \
  -e "s/^    enabled: False # Round 4\/5.*/    enabled: False/" \
  -e "s/^    shadow_diagnostics: .*/    shadow_diagnostics: True/"

# 2. lambda_ref = median of per-frame lambda_ref over the shadow run (>0).
LAM_REF=$(python3 - <<EOF
import csv
rows = list(csv.DictReader(open("$RESULTS/shadow/photo.csv")))
vals = [float(r["lambda_ref"]) for r in rows if float(r["lambda_ref"]) > 0]
print(f"{sorted(vals)[len(vals)//2]:.6g}" if vals else "0")
EOF
)
echo "lambda_ref = $LAM_REF"
if [ -z "$LAM_REF" ] || [ "$LAM_REF" = "0" ] || [ "$LAM_REF" = "nan" ]; then
  echo "ABNORMAL SCALING: lambda_ref unavailable; stopping C-lambda sweep."
  exit 1
fi

# 3. Lambda sweep (all on the SAME segment).
run_one "$RESULTS/lambda_0" \
  -e "s/^    enabled: False # Round 4\/5.*/    enabled: True/" \
  -e "s/^    photometric_scale: .*/    photometric_scale: 0/" \
  -e "s/^    shadow_diagnostics: .*/    shadow_diagnostics: False/"

L1=$(python3 -c "print($LAM_REF*0.1)")
L2=$(python3 -c "print($LAM_REF*0.3)")
L3=$(python3 -c "print($LAM_REF*1.0)")

run_one "$RESULTS/lambda_01ref" \
  -e "s/^    enabled: False # Round 4\/5.*/    enabled: True/" \
  -e "s/^    photometric_scale: .*/    photometric_scale: $L1/" \
  -e "s/^    shadow_diagnostics: .*/    shadow_diagnostics: False/"

run_one "$RESULTS/lambda_03ref" \
  -e "s/^    enabled: False # Round 4\/5.*/    enabled: True/" \
  -e "s/^    photometric_scale: .*/    photometric_scale: $L2/" \
  -e "s/^    shadow_diagnostics: .*/    shadow_diagnostics: False/"

run_one "$RESULTS/lambda_1ref" \
  -e "s/^    enabled: False # Round 4\/5.*/    enabled: True/" \
  -e "s/^    photometric_scale: .*/    photometric_scale: $L3/" \
  -e "s/^    shadow_diagnostics: .*/    shadow_diagnostics: False/"

echo "photo safety sweep done -> $RESULTS"
