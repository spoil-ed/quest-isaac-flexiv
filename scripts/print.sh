#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

if [[ -n "${PRINT_PYTHON:-}" ]]; then
  PYTHON="$PRINT_PYTHON"
elif [[ -n "${ISAAC_PYTHON:-}" ]]; then
  PYTHON="$ISAAC_PYTHON"
elif command -v conda >/dev/null 2>&1; then
  ISAAC_CONDA_ENV="${ISAAC_CONDA_ENV:-isaacsim}"
  CONDA_BASE="$(conda info --base)"
  # shellcheck source=/dev/null
  source "$CONDA_BASE/etc/profile.d/conda.sh"
  conda activate "$ISAAC_CONDA_ENV"
  PYTHON="$(command -v python)"
else
  PYTHON="$(command -v python3)"
fi

if ! "$PYTHON" -c 'import matplotlib' >/dev/null 2>&1; then
  printf '[print] ERROR: Python has no matplotlib: %s\n' "$PYTHON" >&2
  printf '[print] set PRINT_PYTHON or activate the isaacsim conda environment\n' >&2
  exit 1
fi

TORQUE_MONITOR_HOST="${TORQUE_MONITOR_HOST:-127.0.0.1}"
TORQUE_MONITOR_PORT="${TORQUE_MONITOR_PORT:-57685}"
TORQUE_MONITOR_WINDOW_SEC="${TORQUE_MONITOR_WINDOW_SEC:-30}"
TORQUE_MONITOR_REFRESH_HZ="${TORQUE_MONITOR_REFRESH_HZ:-10}"

[[ "$TORQUE_MONITOR_PORT" =~ ^[0-9]+$ ]] \
  && (( TORQUE_MONITOR_PORT > 0 && TORQUE_MONITOR_PORT <= 65535 )) \
  || {
    printf '[print] ERROR: TORQUE_MONITOR_PORT must be between 1 and 65535\n' >&2
    exit 2
  }

# Old versions bound the plot and terminal monitor to the same UDP port, which
# made them compete for unicast packets. Stop only stale repo monitor processes
# before launching the forwarding-based pair below.
for stale_pattern in '[/]print_dual_arm_state.py' '[/]plot_dual_arm_torque.py'; do
  while read -r stale_pid; do
    [[ -n "$stale_pid" && "$stale_pid" != "$$" ]] || continue
    kill "$stale_pid" 2>/dev/null || true
  done < <(pgrep -f -- "$stale_pattern" 2>/dev/null || true)
done

PLOT_PID=""
cleanup() {
  if [[ -n "$PLOT_PID" ]] && kill -0 "$PLOT_PID" 2>/dev/null; then
    kill "$PLOT_PID" 2>/dev/null || true
    wait "$PLOT_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

"$PYTHON" "$REPO_ROOT/scripts/plot_dual_arm_torque.py" \
  --host "$TORQUE_MONITOR_HOST" \
  --port "$TORQUE_MONITOR_PORT" \
  --window-sec "$TORQUE_MONITOR_WINDOW_SEC" \
  --refresh-hz "$TORQUE_MONITOR_REFRESH_HZ" &
PLOT_PID=$!
sleep 0.5
if ! kill -0 "$PLOT_PID" 2>/dev/null; then
  wait "$PLOT_PID" || true
  printf '[print] ERROR: q/dq/torque monitor failed to open; check DISPLAY and matplotlib\n' >&2
  exit 1
fi
printf '[print] q/dq/torque monitor started (pid=%s, udp=%s:%s)\n' \
  "$PLOT_PID" "$TORQUE_MONITOR_HOST" "$TORQUE_MONITOR_PORT"

"$PYTHON" "$REPO_ROOT/scripts/print_dual_arm_state.py" \
  --forward-host "$TORQUE_MONITOR_HOST" \
  --forward-port "$TORQUE_MONITOR_PORT" \
  "$@"
