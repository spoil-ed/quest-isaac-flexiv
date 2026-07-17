#!/usr/bin/env bash
set -Eeuo pipefail

# Start the control stack, Web dashboard, and Web-controlled recorder as one
# collection session. The lower-level start.sh remains available when an
# existing foreground recorder must be preserved during control-stack restart.

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

die() {
  printf '[start-all] ERROR: %s\n' "$*" >&2
  exit 1
}

info() {
  printf '[start-all] %s\n' "$*"
}

first_matching_pid() {
  pgrep -f "$1" 2>/dev/null | head -n 1 || true
}

wait_for_tcp_listener() {
  local description="$1" port="$2" timeout_sec="$3"
  local deadline=$((SECONDS + timeout_sec))
  while ! ss -ltnH | awk -v port=":$port" '$4 ~ port "$" {found=1} END {exit !found}'; do
    (( SECONDS >= deadline )) && die "timed out waiting for $description on TCP port $port"
    sleep 1
  done
}

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
command -v pgrep >/dev/null 2>&1 || die "required command not found: pgrep"
command -v setsid >/dev/null 2>&1 || die "required command not found: setsid"
command -v ss >/dev/null 2>&1 || die "required command not found: ss"

WEB_HOST="${WEB_HOST:-0.0.0.0}"
WEB_PORT="${WEB_PORT:-8080}"
RECORDER_CONTROL_PORT="${RECORDER_CONTROL_PORT:-57687}"
RECORDER_STATUS_PORT="${RECORDER_STATUS_PORT:-57688}"

for value_name in WEB_PORT RECORDER_CONTROL_PORT RECORDER_STATUS_PORT; do
  value="${!value_name}"
  [[ "$value" =~ ^[0-9]+$ ]] && (( value > 0 && value <= 65535 )) \
    || die "$value_name must be between 1 and 65535"
done

# A Web-controlled recorder can be stopped cleanly through its command socket,
# which saves a non-empty interrupted episode. Refuse to steal an unrelated
# interactive recorder because it may own unsaved collection data.
while read -r recorder_pid; do
  [[ -n "$recorder_pid" ]] || continue
  command_line="$(tr '\0' ' ' <"/proc/$recorder_pid/cmdline" 2>/dev/null || true)"
  if [[ "$command_line" != *" --web-control-port "* ]]; then
    die "interactive recorder pid=$recorder_pid is running; exit it with q before start_all.sh"
  fi
  info "stopping previous Web recorder pid=$recorder_pid"
  "$ISAAC_PYTHON" -c \
    'import json,socket,sys; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.sendto(json.dumps({"command":"quit"}).encode(),("127.0.0.1",int(sys.argv[1])))' \
    "$RECORDER_CONTROL_PORT"
  for _attempt in {1..50}; do
    kill -0 "$recorder_pid" 2>/dev/null || break
    sleep 0.1
  done
  kill -0 "$recorder_pid" 2>/dev/null && die "Web recorder pid=$recorder_pid did not stop cleanly"
done < <(pgrep -f '(^|[[:space:]/])scripts/record_unitree_json\.py([[:space:]]|$)' 2>/dev/null || true)

old_dashboard_pid="$(first_matching_pid 'web_control_dashboard\.py([[:space:]]|$)')"
if [[ -n "$old_dashboard_pid" ]]; then
  info "stopping previous Web dashboard pid=$old_dashboard_pid"
  kill "$old_dashboard_pid" 2>/dev/null || true
  wait_deadline=$((SECONDS + 5))
  while kill -0 "$old_dashboard_pid" 2>/dev/null; do
    (( SECONDS >= wait_deadline )) && break
    sleep 0.1
  done
  kill -KILL "$old_dashboard_pid" 2>/dev/null || true
fi

# The Web dashboard replaces both legacy monitor processes and owns the arm
# state UDP port. They contain no collection state and are safe to stop.
for monitor_pattern in 'print_dual_arm_state\.py([[:space:]]|$)' 'plot_dual_arm_torque\.py([[:space:]]|$)'; do
  while read -r monitor_pid; do
    [[ -n "$monitor_pid" ]] || continue
    info "stopping legacy monitor pid=$monitor_pid"
    kill "$monitor_pid" 2>/dev/null || true
  done < <(pgrep -f "$monitor_pattern" 2>/dev/null || true)
done

info "starting control stack"
ISAAC_PYTHON="$ISAAC_PYTHON" "$REPO_ROOT/scripts/start.sh" "$@"

mkdir -p "$REPO_ROOT/logs"
stamp="$(date +%Y%m%d_%H%M%S)"
dashboard_stdout="$REPO_ROOT/logs/web_control_dashboard_${stamp}.stdout.log"
dashboard_stderr="$REPO_ROOT/logs/web_control_dashboard_${stamp}.stderr.log"
setsid "$ISAAC_PYTHON" "$REPO_ROOT/scripts/web_control_dashboard.py" \
  --host "$WEB_HOST" \
  --port "$WEB_PORT" \
  --arm-state-host 127.0.0.1 \
  --arm-state-port 57684 \
  --recorder-command-host 127.0.0.1 \
  --recorder-command-port "$RECORDER_CONTROL_PORT" \
  --recorder-status-host 127.0.0.1 \
  --recorder-status-port "$RECORDER_STATUS_PORT" \
  >"$dashboard_stdout" 2>"$dashboard_stderr" </dev/null &
dashboard_pid=$!
wait_for_tcp_listener "Web dashboard" "$WEB_PORT" 10

recorder_stdout="$REPO_ROOT/logs/web_recorder_${stamp}.stdout.log"
recorder_stderr="$REPO_ROOT/logs/web_recorder_${stamp}.stderr.log"
setsid env ISAAC_PYTHON="$ISAAC_PYTHON" \
  "$REPO_ROOT/scripts/record.sh" \
  --web-control-host 127.0.0.1 \
  --web-control-port "$RECORDER_CONTROL_PORT" \
  --web-status-host 127.0.0.1 \
  --web-status-port "$RECORDER_STATUS_PORT" \
  >"$recorder_stdout" 2>"$recorder_stderr" </dev/null &
recorder_pid=$!
sleep 1
kill -0 "$dashboard_pid" 2>/dev/null || die "Web dashboard exited; see $dashboard_stderr"
kill -0 "$recorder_pid" 2>/dev/null || die "Web recorder exited; see $recorder_stderr"

if [[ -z "${HOST_IP:-}" ]]; then
  route_info="$(ip -4 route get 1.1.1.1 2>/dev/null || true)"
  HOST_IP="$(awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}' <<<"$route_info")"
fi
HOST_IP="${HOST_IP:-127.0.0.1}"

info "READY: http://$HOST_IP:$WEB_PORT"
info "dashboard pid=$dashboard_pid logs=$dashboard_stdout"
info "recorder pid=$recorder_pid logs=$recorder_stdout"
info "Quest: https://$HOST_IP:8012/?ws=wss://$HOST_IP:8012"
