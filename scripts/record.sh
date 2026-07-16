#!/usr/bin/env bash
set -Eeuo pipefail

# Run the interactive dual-arm recorder in the foreground. This script is kept
# separate from scripts/start.sh so restarting the control stack never creates a second
# recorder or steals keyboard control from the recorder terminal.

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

die() {
  printf '[record] ERROR: %s\n' "$*" >&2
  exit 1
}

if [[ "${RECORDER_DRY_RUN:-0}" != "1" ]] \
  && pgrep -f '(^|[[:space:]/])scripts/record_unitree_json\.py([[:space:]]|$)' >/dev/null 2>&1; then
  die "a recorder is already running; exit it with q before starting another"
fi

ISAAC_CONDA_ENV="${ISAAC_CONDA_ENV:-isaacsim}"
if [[ -z "${ISAAC_PYTHON:-}" ]]; then
  command -v conda >/dev/null 2>&1 || die "conda not found; set ISAAC_PYTHON explicitly"
  CONDA_BASE="$(conda info --base)"
  # shellcheck source=/dev/null
  source "$CONDA_BASE/etc/profile.d/conda.sh"
  conda activate "$ISAAC_CONDA_ENV"
  ISAAC_PYTHON="$(command -v python)"
fi
[[ -x "$ISAAC_PYTHON" ]] || die "ISAAC_PYTHON is not executable: $ISAAC_PYTHON"

GATEWAY_ENDPOINT="${GATEWAY_ENDPOINT:-tcp://127.0.0.1:5790}"
TASK_NAME="${TASK_NAME:-pick_place_redblock_dual}"
TASK_DIR="${TASK_DIR:-}"
OUTPUT_ROOT="${OUTPUT_ROOT:-datasets/stage1_records}"
FPS="${FPS:-30}"
EPISODES="${EPISODES:-10}"
IMAGE_SIZE="${IMAGE_SIZE:-640x480}"
MAX_FRAMES="${MAX_FRAMES:-0}"
RESET_TIMEOUT_SEC="${RESET_TIMEOUT_SEC:-90}"
RESET_ON_SAVE="${RESET_ON_SAVE:-1}"

if [[ -n "$TASK_DIR" ]]; then
  TASK_ARGS=(--task-dir "$TASK_DIR")
else
  TASK_ARGS=(--task-name "$TASK_NAME")
fi

COMMAND=(
  "$ISAAC_PYTHON"
  "$REPO_ROOT/scripts/record_unitree_json.py"
  --gateway-endpoint "$GATEWAY_ENDPOINT"
  "${TASK_ARGS[@]}"
  --output-root "$OUTPUT_ROOT"
  --fps "$FPS"
  --episodes "$EPISODES"
  --image-size "$IMAGE_SIZE"
  --max-frames "$MAX_FRAMES"
  --reset-timeout-sec "$RESET_TIMEOUT_SEC"
)
if [[ "$RESET_ON_SAVE" == "1" || "$RESET_ON_SAVE" == "true" ]]; then
  COMMAND+=(--reset-on-save)
fi
COMMAND+=("$@")

printf '[record] task=%s episodes=%s fps=%s output=%s\n' \
  "${TASK_DIR:-$TASK_NAME}" "$EPISODES" "$FPS" "$OUTPUT_ROOT"
printf '[record] keys: s=start/resume, e=pause/save, d=discard, r=reset, q=quit\n'

if [[ "${RECORDER_DRY_RUN:-0}" == "1" ]]; then
  printf '[record] command:'
  printf ' %q' "${COMMAND[@]}"
  printf '\n'
  exit 0
fi

exec "${COMMAND[@]}"
