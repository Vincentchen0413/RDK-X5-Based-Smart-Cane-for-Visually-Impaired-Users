#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if systemctl is-active --quiet smart-cane.service 2>/dev/null; then
  sudo systemctl stop smart-cane.service
  exit 0
fi

PID_FILE="${ROOT}/run/start_all.pid"
if [[ -f "${PID_FILE}" ]]; then
  PID="$(cat "${PID_FILE}")"
  kill -TERM "${PID}" 2>/dev/null || true
  rm -f "${PID_FILE}"
else
  pkill -TERM -f "smart_cane_supervisor.py" 2>/dev/null || true
fi
