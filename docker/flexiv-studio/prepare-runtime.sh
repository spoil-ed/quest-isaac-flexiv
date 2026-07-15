#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
repo_root="$(cd -- "${script_dir}/../.." && pwd -P)"
studio_root="${STUDIO_ROOT:-}"
output="${repo_root}/.deps/docker_studio/FlexivElementsStudio"
target_suffix="WE7ssd"
source_simulator="simulator0"
force=0

usage() {
    cat <<'EOF'
Usage: prepare-runtime.sh --studio-root PATH [options]

Copy an Elements Studio installation into the ignored .deps directory
and assign the isolated simulator a different six-character serial suffix.

Options:
  --studio-root PATH    Source FlexivElementsStudio directory (or STUDIO_ROOT).
  --output PATH         Runtime copy destination.
  --source-simulator ID Source simulator directory (default: simulator0).
  --target-suffix ID    Six alphanumeric characters (default: WE7ssd).
  --force               Replace an existing runtime copy.
EOF
}

while (($#)); do
    case "$1" in
        --studio-root)
            studio_root="$2"
            shift 2
            ;;
        --output)
            output="$2"
            shift 2
            ;;
        --target-suffix)
            target_suffix="$2"
            shift 2
            ;;
        --source-simulator)
            source_simulator="$2"
            shift 2
            ;;
        --force)
            force=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            printf 'Unknown argument: %s\n' "$1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [[ -z "${studio_root}" ]]; then
    printf '%s\n' 'Pass --studio-root or set STUDIO_ROOT.' >&2
    exit 2
fi
if [[ ! "${target_suffix}" =~ ^[[:alnum:]]{6}$ ]]; then
    printf 'Invalid target suffix %q: expected six alphanumeric characters.\n' "${target_suffix}" >&2
    exit 2
fi
if [[ ! "${source_simulator}" =~ ^simulator[[:digit:]]+$ ]]; then
    printf 'Invalid source simulator %q: expected simulatorN.\n' "${source_simulator}" >&2
    exit 2
fi
studio_root="$(cd -- "${studio_root}" && pwd -P)"
if [[ ! -x "${studio_root}/FlexivElementsStudio" ]]; then
    printf 'Not an Elements Studio installation: %s\n' "${studio_root}" >&2
    exit 2
fi

source_param="$(find "${studio_root}/user_data_ui/simDir/${source_simulator}" \
    -mindepth 2 -maxdepth 2 -name arm_driver_param.xml -print -quit)"
if [[ -z "${source_param}" ]]; then
    printf 'No simulator arm_driver_param.xml found under %s\n' "${studio_root}" >&2
    exit 3
fi
source_serial="$(basename "$(dirname "${source_param}")")"
source_suffix="${source_serial##*-}"
target_serial="${source_serial%-*}-${target_suffix}"

if [[ -e "${output}" && "${force}" != "1" ]]; then
    printf 'Runtime destination already exists: %s (use --force to replace it)\n' "${output}" >&2
    exit 4
fi

mkdir -p "$(dirname "${output}")"
temporary="${output}.tmp.$$"
rm -rf "${temporary}"
trap 'rm -rf "${temporary}"' EXIT
mkdir -p "${temporary}"

rsync -a \
    --exclude '.ubuntu24-equiv/' \
    --exclude 'FlexivElements.core' \
    --exclude 'log/' \
    --exclude 'tests/' \
    "${studio_root}/" "${temporary}/"

# One container serves one arm. Remove other simulator definitions so the GUI
# and entrypoint cannot select a serial that belongs to the host runtime.
find "${temporary}/user_data_ui/simDir" \
    -mindepth 1 -maxdepth 1 -type d ! -name "${source_simulator}" \
    -exec rm -rf -- {} +

python3 - "${temporary}" "${source_suffix}" "${target_suffix}" <<'PY'
from pathlib import Path
import re
import sys

root = Path(sys.argv[1])
old = sys.argv[2]
new = sys.argv[3]

for path in root.rglob("*"):
    if not path.is_file() or path.stat().st_size > 16 * 1024 * 1024:
        continue
    try:
        data = path.read_text(encoding="utf-8")
    except (UnicodeDecodeError, OSError):
        continue
    replaced = data.replace(old, new)
    if replaced != data:
        path.write_text(replaced, encoding="utf-8")

for path in sorted(root.rglob("*"), key=lambda item: len(item.parts), reverse=True):
    if old in path.name:
        path.rename(path.with_name(path.name.replace(old, new)))

# Match the host Studio RDK connection scheme: External Interface enabled over
# Ethernet. A newly created simulator can otherwise retain External IO disabled,
# in which case RobotControlApp runs but never exposes the robot to host RDK.
external_configs = list(
    root.glob(
        "user_data_ui/simDir/simulator*/user_data/settings/robotExtInterfaceCfg.xml"
    )
)
if len(external_configs) != 1:
    raise SystemExit(
        f"expected one robotExtInterfaceCfg.xml, found {len(external_configs)}"
    )
external_config = external_configs[0]
data = external_config.read_text(encoding="utf-8")
data, enable_count = re.subn(r"<enable>[^<]*</enable>", "<enable>1</enable>", data)
data, file_count = re.subn(
    r"<interface_config_file>[^<]*</interface_config_file>",
    "<interface_config_file>externalEthernetConfig.xml</interface_config_file>",
    data,
)
if enable_count != 1 or file_count != 1:
    raise SystemExit(f"unexpected External Interface schema in {external_config}")
external_config.write_text(data, encoding="utf-8")
PY

printf '%s\n' "Rizon4-${target_suffix}" >"${temporary}/.prepared-rdk-serial"
rm -rf "${output}"
mv "${temporary}" "${output}"
trap - EXIT

if [[ "${source_suffix}" != "${target_suffix}" ]] && \
    rg -a -l --glob '!*.core' "${source_suffix}" "${output}/user_data_ui" >/dev/null; then
    printf 'Serial rewrite left old suffix %s in the runtime copy.\n' "${source_suffix}" >&2
    exit 5
fi

printf 'Prepared Studio runtime: %s\n' "${output}"
printf 'Source simulator: %s (RDK alias Rizon4-%s)\n' "${source_serial}" "${source_suffix}"
printf 'Container simulator: %s (RDK alias Rizon4-%s)\n' "${target_serial}" "${target_suffix}"
