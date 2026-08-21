#!/usr/bin/env bash
# Round-10-A: GEODE official evaluation chain (gamma device -> Leica frame -> evo_ape).
#
#   raw trajectory (TUM, gamma device frame)
#     -> GEODE official gamma2GT_leica.py  (extrinsic untouched; I/O path only)
#     -> Leica-frame trajectory
#     -> evo_ape tum <traj> <gt> -va --t_max_diff 0.1 --t_offset 0   (rmse.py semantics)
#
# Only trajectory.txt is fed to gamma2GT (the official script processes every
# *.txt in its raw folder, so other files would break it).
#
# Usage:
#   scripts/geode_evaluate.sh <sequence> <group> <gt.txt>
set -euo pipefail

SEQ="$1"; GROUP="$2"; GT="$3"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
GEODE="/tmp/opencode/GEODE_dataset"
RAW_DIR="$REPO/results/round10/avia/$SEQ/$GROUP"
WORK="$(mktemp -d /tmp/geode_${SEQ}_${GROUP}.XXXXXX)"
RAW_IN="$WORK/raw"
EVAL_OUT="$WORK/eval"
mkdir -p "$RAW_IN" "$EVAL_OUT"
cp "$RAW_DIR/trajectory.txt" "$RAW_IN/"

# gamma2GT_leica.py is the official script; only the raw/output folder paths are
# overridden (extrinsic / transform order untouched).
python3 - "$GEODE/script/gamma2GT_leica.py" "$RAW_IN" "$EVAL_OUT" <<'EOF'
import sys
src = open(sys.argv[1]).read()
src = src.replace('raw_folder = "./shield_tunnel1"', f'raw_folder = "{sys.argv[2]}"')
src = src.replace('output_folder = "./tran2body"', f'output_folder = "{sys.argv[3]}"')
open("/tmp/opencode/gamma2GT_leica_io.py", "w").write(src)
EOF
python3 /tmp/opencode/gamma2GT_leica_io.py > /dev/null 2>&1 || { echo "gamma2GT FAILED"; exit 1; }

EVO="$(command -v evo_ape || echo evo_ape)"
"$EVO" tum "$EVAL_OUT/trajectory.txt" "$GT" -va --t_max_diff 0.1 --t_offset 0 --no_warnings 2>&1 | grep -E "max|mean|median|min|rmse|sse|std"
rm -rf "$WORK"
