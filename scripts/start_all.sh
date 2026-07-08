#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE="${1:-navigation}"
ENV_FILE="${ROOT}/deploy/env.local"

if [[ -f "${ENV_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${ENV_FILE}"
fi

export SMART_CANE_ROOT="${SMART_CANE_ROOT:-$ROOT}"
export PYTHONUNBUFFERED=1

ROS_SETUP="${SMART_CANE_ROS_SETUP:-/opt/tros/humble/setup.bash}"
if [[ -f "${ROS_SETUP}" ]]; then
  # shellcheck disable=SC1090
  source "${ROS_SETUP}"
fi

if [[ -f "${ROOT}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT}/install/setup.bash"
fi

mkdir -p "${ROOT}/logs" "${ROOT}/run"
echo "$$" > "${ROOT}/run/start_all.pid"

exec python3 "${ROOT}/bringup/smart_cane_supervisor.py" \
  --profile "${PROFILE}" \
  --config "config/modules.yaml"
