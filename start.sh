#!/usr/bin/env bash
set -Eeuo pipefail

# Cold-restart the verified dual-arm Quest stack, excluding the recorder.
# Repository-owned paths are derived from this file. External runtimes can be
# overridden with environment variables; see usage() below.

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_ROOT"

usage() {
  cat <<'EOF'
Usage: ./start.sh

Starts everything needed for dual-arm Quest teleoperation except the recorder:
  Docker left Studio, host right Studio, RobotControlApp, FlexivSimulation,
  gateway, DRDK RobotPair, Isaac Sim GUI, and the dual-hand Quest publisher.

Every run first stops the previous control stack and the Docker left Studio
container, while leaving any recorder process untouched. It then starts a clean
control stack without creating another recorder, so code and configuration
changes always take effect.

Optional environment variables:
  STUDIO_ROOT              Host Elements Studio root.
                           Default: ../elements_studio/FlexivElementsStudio
  ISAAC_PYTHON             Isaac Sim Python executable.
                           Default: Python from conda env "isaacsim"
  QUEST_PYTHON             Python containing TeleVuer/Quest dependencies.
                           Default: .venv-quest/bin/python, or conda env "tv"
  ISAAC_CONDA_ENV          Isaac conda environment name (default: isaacsim)
  QUEST_CONDA_ENV          Quest conda environment name (default: tv)
  HOST_IP                  IPv4 reachable by Quest (default: route source IP)
  LEFT_ROBOT_SERIAL        Docker/left alias (default: Rizon4-qSaFLh)
  RIGHT_ROBOT_SERIAL       Host/right alias (default: Rizon4-I0LIRN)
  SCENE_CONFIG             Scene YAML relative to the repository, or absolute.
  FLEXIV_CAPSH             Optional capsh executable for RobotControlApp.
  FLEXIV_SHM_ROOT          Host Studio shared-memory root (default: /dev/shm)
  STARTUP_TIMEOUT_SEC      Runtime readiness timeout (default: 120)
  FLEXIV_STUDIO_VNC_PORT   Docker Studio VNC port (default: 5902)
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ $# -ne 0 ]]; then
  usage >&2
  exit 2
fi

die() {
  printf '[start] ERROR: %s\n' "$*" >&2
  exit 1
}

info() {
  printf '[start] %s\n' "$*"
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "required command not found: $1"
}

resolve_from_repo() {
  local value="$1"
  if [[ "$value" = /* ]]; then
    printf '%s\n' "$value"
  else
    printf '%s/%s\n' "$REPO_ROOT" "$value"
  fi
}

first_matching_pid() {
  local pattern="$1"
  pgrep -f "$pattern" 2>/dev/null | head -n 1 || true
}

start_detached() {
  local label="$1"
  local pattern="$2"
  shift 2

  local existing_pid
  existing_pid="$(first_matching_pid "$pattern")"
  if [[ -n "$existing_pid" ]]; then
    info "$label already running (pid=$existing_pid)"
    return 0
  fi

  mkdir -p "$REPO_ROOT/logs"
  local stamp stdout_log stderr_log pid
  stamp="$(date +%Y%m%d_%H%M%S)"
  stdout_log="$REPO_ROOT/logs/${label}_${stamp}.stdout.log"
  stderr_log="$REPO_ROOT/logs/${label}_${stamp}.stderr.log"
  setsid "$@" >"$stdout_log" 2>"$stderr_log" </dev/null &
  pid=$!
  info "$label started (pid=$pid)"
  info "$label stdout=$stdout_log"
  info "$label stderr=$stderr_log"
}

wait_for_file_pattern() {
  local description="$1"
  local pattern="$2"
  local timeout_sec="$3"
  local deadline=$((SECONDS + timeout_sec))
  while ! compgen -G "$pattern" >/dev/null; do
    (( SECONDS >= deadline )) && die "timed out waiting for $description: $pattern"
    sleep 1
  done
}

require_command docker
require_command pgrep
require_command setsid

ISAAC_CONDA_ENV="${ISAAC_CONDA_ENV:-isaacsim}"
QUEST_CONDA_ENV="${QUEST_CONDA_ENV:-tv}"
STARTUP_TIMEOUT_SEC="${STARTUP_TIMEOUT_SEC:-120}"
[[ "$STARTUP_TIMEOUT_SEC" =~ ^[0-9]+$ ]] || die "STARTUP_TIMEOUT_SEC must be an integer"

if [[ -z "${ISAAC_PYTHON:-}" ]]; then
  require_command conda
  CONDA_BASE="$(conda info --base)"
  # shellcheck source=/dev/null
  source "$CONDA_BASE/etc/profile.d/conda.sh"
  conda activate "$ISAAC_CONDA_ENV"
  ISAAC_PYTHON="$(command -v python)"
fi
[[ -x "$ISAAC_PYTHON" ]] || die "ISAAC_PYTHON is not executable: $ISAAC_PYTHON"

if [[ -z "${QUEST_PYTHON:-}" ]]; then
  if [[ -x "$REPO_ROOT/.venv-quest/bin/python" ]]; then
    QUEST_PYTHON="$REPO_ROOT/.venv-quest/bin/python"
  else
    require_command conda
    QUEST_PYTHON="$(conda run -n "$QUEST_CONDA_ENV" python -c 'import sys; print(sys.executable)' 2>/dev/null)"
  fi
fi
[[ -x "$QUEST_PYTHON" ]] || die "QUEST_PYTHON is not executable: $QUEST_PYTHON"

STUDIO_ROOT="${STUDIO_ROOT:-$REPO_ROOT/../elements_studio/FlexivElementsStudio}"
[[ -d "$STUDIO_ROOT" ]] || die "STUDIO_ROOT does not exist: $STUDIO_ROOT"

LEFT_ROBOT_SERIAL="${LEFT_ROBOT_SERIAL:-Rizon4-qSaFLh}"
RIGHT_ROBOT_SERIAL="${RIGHT_ROBOT_SERIAL:-Rizon4-I0LIRN}"
SCENE_CONFIG="$(resolve_from_repo "${SCENE_CONFIG:-standalone_examples/api/isaacsim.robot.manipulators/flexiv_quest/app_config.yaml}")"
[[ -f "$SCENE_CONFIG" ]] || die "SCENE_CONFIG does not exist: $SCENE_CONFIG"

if [[ -z "${HOST_IP:-}" ]]; then
  require_command ip
  ROUTE_INFO="$(ip -4 route get 1.1.1.1 2>/dev/null || true)"
  HOST_IP="$(awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}' <<<"$ROUTE_INFO")"
fi
[[ -n "$HOST_IP" ]] || die "could not detect HOST_IP; export HOST_IP explicitly"

FLEXIV_STUDIO_VNC_PORT="${FLEXIV_STUDIO_VNC_PORT:-5902}"
FLEXIV_ARM_SERIAL="${FLEXIV_ARM_SERIAL:-A02L-00-M6-${LEFT_ROBOT_SERIAL#Rizon4-}}"
RIGHT_FLEXIV_ARM_SERIAL="${RIGHT_FLEXIV_ARM_SERIAL:-A02L-00-M6-${RIGHT_ROBOT_SERIAL#Rizon4-}}"
FLEXIV_SHM_ROOT="${FLEXIV_SHM_ROOT:-/dev/shm}"
export FLEXIV_ARM_SERIAL FLEXIV_STUDIO_VNC_PORT

info "repository=$REPO_ROOT"
info "scene=$SCENE_CONFIG"
info "left=$LEFT_ROBOT_SERIAL (Docker), right=$RIGHT_ROBOT_SERIAL (host)"
info "Quest host=https://$HOST_IP:8012"

info "stopping previous host control stack; recorder is preserved"
"$ISAAC_PYTHON" "$REPO_ROOT/scripts/stop_flexiv_stack.py" --timeout 12

info "clearing stale host shared memory for $RIGHT_FLEXIV_ARM_SERIAL"
RIGHT_SHM_SERIAL="${RIGHT_FLEXIV_ARM_SERIAL//-/_}"
for SHM_PREFIX in \
  flexiv_arm_command \
  flexiv_arm_state \
  flexiv_simulation_motion_bar \
  flexiv_simulation_rca_motion_bar \
  flexiv_simulation_tool_payload; do
  rm -f -- "$FLEXIV_SHM_ROOT/${SHM_PREFIX}_${RIGHT_SHM_SERIAL}"
done

info "stopping previous Docker left Studio"
docker compose -f "$REPO_ROOT/docker/flexiv-studio/compose.yaml" down --remove-orphans

info "starting clean Docker left Studio"
docker compose -f "$REPO_ROOT/docker/flexiv-studio/compose.yaml" up -d
CONTAINER_ID="$(docker compose -f "$REPO_ROOT/docker/flexiv-studio/compose.yaml" ps -q studio-left)"
[[ -n "$CONTAINER_ID" ]] || die "Docker left Studio container was not created"

DOCKER_DEADLINE=$((SECONDS + STARTUP_TIMEOUT_SEC))
while true; do
  DOCKER_HEALTH="$(docker inspect --format '{{if .State.Health}}{{.State.Health.Status}}{{else}}{{.State.Status}}{{end}}' "$CONTAINER_ID")"
  if [[ "$DOCKER_HEALTH" == "healthy" || "$DOCKER_HEALTH" == "running" ]]; then
    break
  fi
  [[ "$DOCKER_HEALTH" == "unhealthy" || "$DOCKER_HEALTH" == "exited" ]] \
    && die "Docker left Studio state is $DOCKER_HEALTH; inspect with: docker logs flexiv-studio-left"
  (( SECONDS >= DOCKER_DEADLINE )) \
    && die "timed out waiting for Docker left Studio (last state: $DOCKER_HEALTH)"
  sleep 1
done
info "Docker left Studio is $DOCKER_HEALTH; GUI=127.0.0.1:$FLEXIV_STUDIO_VNC_PORT"

info "starting clean host Elements Studio UI"
"$ISAAC_PYTHON" "$REPO_ROOT/scripts/start_elements_studio_ui.py" --studio-root "$STUDIO_ROOT"

wait_for_file_pattern \
  "host Studio generated robot URDF" \
  "$STUDIO_ROOT/user_data_ui/simDir/simulator0/user_data/settings/generated_robot*_abs_path.urdf" \
  "$STARTUP_TIMEOUT_SEC"
wait_for_file_pattern \
  "host Studio arm driver parameters" \
  "$STUDIO_ROOT/user_data_ui/simDir/simulator0/*/arm_driver_param.xml" \
  "$STARTUP_TIMEOUT_SEC"

ROBOT_CONTROL_ARGS=(--studio-root "$STUDIO_ROOT")
if [[ -n "${FLEXIV_CAPSH:-}" ]]; then
  ROBOT_CONTROL_ARGS+=(--capsh "$FLEXIV_CAPSH")
elif [[ -x "$REPO_ROOT/.deps/runtime-tools/capsh" ]]; then
  ROBOT_CONTROL_ARGS+=(--capsh "$REPO_ROOT/.deps/runtime-tools/capsh")
fi
"$ISAAC_PYTHON" "$REPO_ROOT/scripts/start_robot_control_app.py" "${ROBOT_CONTROL_ARGS[@]}"
"$ISAAC_PYTHON" "$REPO_ROOT/scripts/start_flexiv_simulation.py" --studio-root "$STUDIO_ROOT"

start_detached \
  "data_gateway" \
  '(^|[[:space:]/])scripts/start_data_gateway\.py([[:space:]]|$)' \
  "$ISAAC_PYTHON" "$REPO_ROOT/scripts/start_data_gateway.py" \
  --backend bridge \
  --sample-endpoint tcp://127.0.0.1:5790 \
  --bridge-endpoint tcp://127.0.0.1:5791 \
  --fps 30 \
  --image-size 640x480 \
  --camera-keys color_0

# DRDK discovery and Isaac/SimPlugin must overlap: FlexivSimulation cannot
# advance until Isaac starts supplying the 2 kHz physics state stream.
INDEPENDENT_RDK_PID="$(first_matching_pid '(^|[[:space:]/])scripts/rdk_target_streamer\.py([[:space:]]|$)')"
if [[ -n "$INDEPENDENT_RDK_PID" ]]; then
  die "independent RDK streamer is running (pid=$INDEPENDENT_RDK_PID); stop it before starting DRDK RobotPair"
fi
if [[ -n "$(first_matching_pid '(^|[[:space:]/])scripts/drdk_target_streamer\.py([[:space:]]|$)')" ]]; then
  info "DRDK RobotPair already running"
else
  "$ISAAC_PYTHON" "$REPO_ROOT/scripts/start_drdk_target_streamer.py" \
    --python "$ISAAC_PYTHON" \
    --scene-config "$SCENE_CONFIG" \
    --left-serial-number "$LEFT_ROBOT_SERIAL" \
    --right-serial-number "$RIGHT_ROBOT_SERIAL" \
    --left-port 57680 \
    --right-port 57681 \
    --left-status-port 57682 \
    --right-status-port 57683 \
    --connect-timeout-sec 120 \
    --max-linear-speed-m-s 0.5 \
    --max-angular-speed-rad-s 0.75
fi

ISAAC_DUAL_PID="$(first_matching_pid 'dual_follow_with_studio\.py([[:space:]]|$)')"
if [[ -n "$ISAAC_DUAL_PID" ]]; then
  ISAAC_DUAL_COMMAND="$(tr '\0' ' ' <"/proc/$ISAAC_DUAL_PID/cmdline" 2>/dev/null || true)"
  if [[ " $ISAAC_DUAL_COMMAND " != *" --quest-relative-orientation-mode relative "* ]]; then
    die "Isaac dual-arm app pid=$ISAAC_DUAL_PID is not using relative Quest orientation; stop the old stack before restarting"
  fi
  info "Isaac dual-arm app already running (pid=$ISAAC_DUAL_PID, relative position/orientation)"
else
  "$ISAAC_PYTHON" "$REPO_ROOT/scripts/start_dual_isaac_follow.py" \
    --isaac-python "$ISAAC_PYTHON" \
    --scene-config "$SCENE_CONFIG" \
    --left-serial-number "$LEFT_ROBOT_SERIAL" \
    --right-serial-number "$RIGHT_ROBOT_SERIAL" \
    --no-manual-play \
    --no-gpu-dynamics \
    --physics-hz 2000 \
    --render-hz 30 \
    --enable-quest-target-udp \
    --quest-target-udp-port 57679 \
    --quest-relative-orientation-mode relative \
    --quest-position-scale 1.0 \
    --quest-position-deadband-m 0.0 \
    --left-target-pose-udp-port 57680 \
    --right-target-pose-udp-port 57681 \
    --left-rdk-status-udp-port 57682 \
    --right-rdk-status-udp-port 57683 \
    --target-pose-publish-hz 30 \
    --max-linear-speed-m-s 0.5 \
    --max-angular-speed-rad-s 0.75 \
    --gateway-endpoint tcp://127.0.0.1:5791 \
    --gateway-fps 30 \
    --gateway-jpeg-quality 90 \
    --state-monitor-udp-host 127.0.0.1 \
    --state-monitor-udp-port 57684 \
    --state-monitor-hz 10 \
    --coordinated-reset \
    --reset-settle-sec 2 \
    --reset-timeout-sec 90
fi

start_detached \
  "quest_target_publisher_dual" \
  'rizon4_quest_target_publisher\.py([[:space:]]|$)' \
  "$QUEST_PYTHON" "$REPO_ROOT/scripts/rizon4_quest_target_publisher.py" \
  --host-ip "$HOST_IP" \
  --udp-host 127.0.0.1 \
  --udp-port 57679 \
  --side both \
  --left-serial-number "$LEFT_ROBOT_SERIAL" \
  --right-serial-number "$RIGHT_ROBOT_SERIAL" \
  --enable-button squeeze \
  --gripper-button trigger \
  --axis-map=-z,-x,y \
  --position-delta-scale 1 \
  --position-deadband 0 \
  --rate-hz 30 \
  --log-hz 2

info "startup commands completed"
info "Quest URL: https://$HOST_IP:8012/?ws=wss://$HOST_IP:8012"
info "Docker Studio GUI: vncviewer 127.0.0.1:$FLEXIV_STUDIO_VNC_PORT"
info "recorder was left untouched and no new recorder was started"
