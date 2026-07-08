#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OPENVINS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PID_FILE="${OPENVINS_ROOT}/run/openvins_backend.pids"

stop_pid() {
  local pid="$1"

  if [[ ! "${pid}" =~ ^[0-9]+$ ]]; then
    return 0
  fi

  if ! kill -0 "${pid}" 2>/dev/null; then
    return 0
  fi

  kill -INT "${pid}" 2>/dev/null || true

  for _ in $(seq 1 30); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      return 0
    fi
    sleep 0.2
  done

  kill -TERM "${pid}" 2>/dev/null || true
  sleep 1

  if kill -0 "${pid}" 2>/dev/null; then
    kill -KILL "${pid}" 2>/dev/null || true
  fi
}

if [[ -f "${PID_FILE}" ]]; then
  # 逆序停止：健康监控、桥接节点、OpenVINS估计器。
  mapfile -t pids < "${PID_FILE}"

  for (( index=${#pids[@]}-1; index>=0; index-- )); do
    stop_pid "${pids[index]}"
  done

  rm -f "${PID_FILE}"
fi

# 仅清理本目录启动的特征进程，避免误杀其他RTAB-Map节点。
pkill -f "${OPENVINS_ROOT}/nodes/openvins_health_monitor.py" 2>/dev/null || true
pkill -f "${OPENVINS_ROOT}/nodes/openvins_to_rtabmap_odom.py" 2>/dev/null || true

echo "[OK] V7 OpenVINS backend stopped."
