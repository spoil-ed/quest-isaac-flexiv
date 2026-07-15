#!/usr/bin/env bash
set -euo pipefail

display_number="${DISPLAY_NUMBER:-1}"
vnc_port="${VNC_PORT:-5900}"

xdpyinfo -display ":${display_number}" >/dev/null 2>&1
pgrep -f '(^|/)FlexivElementsStudio([[:space:]]|$)' >/dev/null
bash -c "exec 3<>/dev/tcp/127.0.0.1/${vnc_port}"

if [[ "${FLEXIV_AUTO_START_RUNTIME:-1}" == "1" ]]; then
    pgrep -f '^flexiv-docker-robot-control([[:space:]]|$)' >/dev/null
    pgrep -f '^flexiv-docker-simulation([[:space:]]|$)' >/dev/null
    external_config="$(find /opt/flexiv-studio/user_data_ui/simDir \
        -path '*/user_data/settings/robotExtInterfaceCfg.xml' -print -quit)"
    test -n "${external_config}"
    grep -q '<enable>1</enable>' "${external_config}"
    grep -q '<interface_config_file>externalEthernetConfig.xml</interface_config_file>' \
        "${external_config}"
fi
