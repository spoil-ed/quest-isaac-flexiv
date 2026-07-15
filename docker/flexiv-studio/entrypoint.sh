#!/usr/bin/env bash
set -euo pipefail

readonly STUDIO_ROOT="/opt/flexiv-studio"
readonly DISPLAY_NUMBER="${DISPLAY_NUMBER:-1}"
readonly VNC_PORT="${VNC_PORT:-5900}"
readonly SCREEN_GEOMETRY="${SCREEN_GEOMETRY:-1920x1080x24}"
readonly LOG_ROOT="${FLEXIV_CONTAINER_LOG_ROOT:-/tmp/flexiv-studio}"
readonly ROBOT_CONTROL_PROCESS_NAME="flexiv-docker-robot-control"
readonly SIMULATION_PROCESS_NAME="flexiv-docker-simulation"

export DISPLAY=":${DISPLAY_NUMBER}"
export HOME="${HOME:-/tmp/flexiv-home}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/flexiv-runtime}"
export LD_LIBRARY_PATH="${STUDIO_ROOT}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PATH="${STUDIO_ROOT}/bin:${PATH}"
export QT_QPA_PLATFORM_PLUGIN_PATH="${STUDIO_ROOT}/plugins"
export QTWEBENGINE_DISABLE_SANDBOX=1
export QT_X11_NO_MITSHM=1

require_executable() {
    if [[ ! -x "${STUDIO_ROOT}/$1" ]]; then
        printf 'Missing executable: %s/%s\n' "${STUDIO_ROOT}" "$1" >&2
        exit 2
    fi
}

is_running() {
    pgrep -f "(^|/)${1}([[:space:]]|$)" >/dev/null 2>&1
}

stop_children() {
    local pid
    for pid in $(jobs -pr); do
        kill "${pid}" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}

require_executable FlexivElementsStudio
mkdir -p "${HOME}" "${LOG_ROOT}" "${XDG_RUNTIME_DIR}"
chmod 0700 "${XDG_RUNTIME_DIR}"
trap stop_children EXIT INT TERM

Xvfb "${DISPLAY}" -screen 0 "${SCREEN_GEOMETRY}" -ac +extension GLX +render -noreset \
    >"${LOG_ROOT}/xvfb.log" 2>&1 &

for _ in $(seq 1 50); do
    if xdpyinfo -display "${DISPLAY}" >/dev/null 2>&1; then
        break
    fi
    sleep 0.2
done
if ! xdpyinfo -display "${DISPLAY}" >/dev/null 2>&1; then
    printf 'Xvfb did not become ready; see %s/xvfb.log\n' "${LOG_ROOT}" >&2
    exit 3
fi

openbox-session >"${LOG_ROOT}/openbox.log" 2>&1 &
x11vnc -display "${DISPLAY}" -forever -shared -nopw -rfbport "${VNC_PORT}" \
    >"${LOG_ROOT}/x11vnc.log" 2>&1 &

cd "${STUDIO_ROOT}"
./FlexivElementsStudio -p ubuntu_pc >"${LOG_ROOT}/studio.log" 2>&1 &

if [[ "${FLEXIV_AUTO_START_RUNTIME:-1}" == "1" ]]; then
    simulator_parent="${STUDIO_ROOT}/user_data_ui/simDir"
    serial="${FLEXIV_ARM_SERIAL:-}"
    if [[ -z "${serial}" ]]; then
        # Prefer the newest simulator. This preserves a simulator created or
        # selected in the Studio UI without committing a machine-specific ID.
        param_path="$(find "${simulator_parent}" -mindepth 3 -maxdepth 3 \
            -name arm_driver_param.xml -printf '%T@ %p\n' \
            | sort -nr | head -1 | cut -d' ' -f2-)"
        serial="$(basename "$(dirname "${param_path}")")"
    else
        param_path="$(find "${simulator_parent}" -mindepth 3 -maxdepth 3 \
            -path "*/${serial}/arm_driver_param.xml" -print -quit)"
    fi
    if [[ -z "${serial}" || -z "${param_path}" ]]; then
        printf 'Could not discover FLEXIV_ARM_SERIAL under %s\n' "${simulator_parent}" >&2
        exit 4
    fi
    simulator_root="$(dirname "$(dirname "${param_path}")")"

    config_path="specs/robots/Flexiv${serial%%-*}/flexivCfg.xml"
    # RobotControlApp unlocks specs_enc into specs during its own cold start, so
    # the config path is intentionally allowed to be absent at this point.
    if ! is_running "${ROBOT_CONTROL_PROCESS_NAME}"; then
        user_data_path="${simulator_root#${STUDIO_ROOT}/}/user_data/"
        # Compose runs as the host UID so runtime files keep their ownership.
        # A file-capability capsh wrapper passes only SYS_NICE to the vendor
        # controller, allowing its real-time threads without running it as root.
        /sbin/capsh \
            --caps=cap_sys_nice+eip \
            --addamb=cap_sys_nice \
            -- -c \
            'export LD_LIBRARY_PATH="$PWD/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"; export QT_QPA_PLATFORM_PLUGIN_PATH="$PWD/plugins"; export PATH="$PWD/bin:$PATH"; exec -a flexiv-docker-robot-control "$@"' \
            flexiv-robot-control \
            ./RobotControlApp \
            -u "./${user_data_path}" \
            -c "${config_path}" \
            -m MotionBarSimulation \
            -s "${serial}" \
            -x "${FLEXIV_CONTROL_BOX_SERIAL:-CX01-02-P1-00034}" \
            -n -g >"${LOG_ROOT}/robot-control.log" 2>&1 &
    fi

    settings_root="${simulator_root}/user_data/settings"
    robot_urdf="$(find "${settings_root}" -maxdepth 1 -name 'generated_robot*_abs_path.urdf' -print -quit)"
    robot_srdf="$(find "${settings_root}" -maxdepth 1 -name 'generated_robot*_abs_path.srdf' -print -quit)"
    scene_urdf="${settings_root}/user_scene_abs_path.urdf"
    # The vendor launcher passes user_scene_abs_path.urdf even when no custom
    # scene has been generated; FlexivSimulation treats that input as optional.
    for path in "${robot_urdf}" "${robot_srdf}" "${param_path}"; do
        if [[ ! -f "${path}" ]]; then
            printf 'Missing FlexivSimulation input: %s\n' "${path}" >&2
            exit 5
        fi
    done

    if ! is_running "${SIMULATION_PROCESS_NAME}"; then
        # Elements Studio uses host-wide `pkill -f ./FlexivSimulation` when a
        # simulated robot is selected. Give the Docker process a distinct
        # argv[0] so a host Studio reconnect cannot terminate the other arm.
        bash -c 'exec -a flexiv-docker-simulation ./FlexivSimulation "$@"' \
            flexiv-docker-simulation \
            --robot_urdf "${robot_urdf#${STUDIO_ROOT}/}" \
            --robot_srdf "${robot_srdf#${STUDIO_ROOT}/}" \
            --scene_urdf "${scene_urdf#${STUDIO_ROOT}/}" \
            --param "${param_path#${STUDIO_ROOT}/}" \
            --group_state "${FLEXIV_GROUP_STATE:-home}" \
            >"${LOG_ROOT}/simulation.log" 2>&1 &
    fi
fi

printf 'Flexiv container Studio is available at VNC port %s (display %s).\n' "${VNC_PORT}" "${DISPLAY}"
printf 'Container logs: %s\n' "${LOG_ROOT}"

while true; do
    if ! is_running FlexivElementsStudio; then
        printf 'FlexivElementsStudio exited; see %s/studio.log\n' "${LOG_ROOT}" >&2
        exit 6
    fi
    if [[ "${FLEXIV_AUTO_START_RUNTIME:-1}" == "1" ]]; then
        if ! is_running "${ROBOT_CONTROL_PROCESS_NAME}"; then
            printf 'RobotControlApp exited; restarting the container runtime. See %s/robot-control.log\n' \
                "${LOG_ROOT}" >&2
            exit 7
        fi
        if ! is_running "${SIMULATION_PROCESS_NAME}"; then
            printf 'FlexivSimulation exited; restarting the container runtime. See %s/simulation.log\n' \
                "${LOG_ROOT}" >&2
            exit 8
        fi
    fi
    sleep 2
done
