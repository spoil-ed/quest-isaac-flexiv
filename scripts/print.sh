#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

if [[ -n "${PRINT_PYTHON:-}" ]]; then
  PYTHON="$PRINT_PYTHON"
elif [[ -n "${ISAAC_PYTHON:-}" ]]; then
  PYTHON="$ISAAC_PYTHON"
else
  PYTHON="$(command -v python3)"
fi

exec "$PYTHON" "$REPO_ROOT/scripts/print_dual_arm_state.py" "$@"
