#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OPENVINS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

source "${SCRIPT_DIR}/openvins_env.sh"

if [[ -f "${SLAM_NAV_ROOT}/slam_env.sh" ]]; then
  # shellcheck disable=SC1090
  source "${SLAM_NAV_ROOT}/slam_env.sh"
fi

if [[ -f "${OPENVINS_WS}/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${OPENVINS_WS}/install/setup.bash"
else
  echo "[WARN] OpenVINS setup not found: ${OPENVINS_WS}/install/setup.bash"
  echo "[WARN] Continue only if ov_msckf is already available in the current ROS environment."
fi

LOG_DIR="${OPENVINS_ROOT}/logs"
RUN_DIR="${OPENVINS_ROOT}/run"
PID_FILE="${RUN_DIR}/openvins_backend.pids"

mkdir -p "${LOG_DIR}" "${RUN_DIR}"
: > "${PID_FILE}"

cleanup_on_error() {
  local exit_code=$?
  if (( exit_code != 0 )); then
    echo "[ERROR] OpenVINS backend startup failed; cleaning started processes."
    bash "${SCRIPT_DIR}/stop_openvins_backend.sh" || true
  fi
  exit "${exit_code}"
}
trap cleanup_on_error EXIT

topic_exists() {
  local topic="$1"
  ros2 topic list 2>/dev/null | grep -Fxq "${topic}"
}

wait_for_topic_message() {
  local topic="$1"
  local type="$2"
  local timeout_seconds="$3"

  echo "[WAIT] ${topic} (${type}), timeout=${timeout_seconds}s"

  if timeout "${timeout_seconds}" \
      ros2 topic echo "${topic}" "${type}" --once \
      >/dev/null 2>&1; then
    echo "[OK] ${topic}"
    return 0
  fi

  echo "[ERROR] No message from ${topic}"
  return 1
}

wait_for_tf() {
  local parent="$1"
  local child="$2"
  local timeout_seconds="$3"

  echo "[WAIT] TF ${parent} -> ${child}, timeout=${timeout_seconds}s"

  if timeout "${timeout_seconds}" \
      ros2 run tf2_ros tf2_echo "${parent}" "${child}" \
      >/dev/null 2>&1; then
    echo "[OK] TF ${parent} -> ${child}"
    return 0
  fi

  echo "[ERROR] TF unavailable: ${parent} -> ${child}"
  return 1
}

echo "============================================================"
echo " Starting V7 OpenVINS backend"
echo " Root: ${OPENVINS_ROOT}"
echo "============================================================"

wait_for_topic_message \
  "${OPENVINS_IMU_TOPIC}" \
  sensor_msgs/msg/Imu \
  15

wait_for_topic_message \
  "${OPENVINS_LEFT_TOPIC}" \
  sensor_msgs/msg/Image \
  15

wait_for_topic_message \
  "${OPENVINS_RIGHT_TOPIC}" \
  sensor_msgs/msg/Image \
  15

echo "[STEP] Start OpenVINS estimator."
(
  set +u
  eval "exec ${OPENVINS_START_COMMAND}"
) >"${LOG_DIR}/openvins_estimator.log" 2>&1 &
OPENVINS_PID=$!
echo "${OPENVINS_PID}" >> "${PID_FILE}"

sleep 2
if ! kill -0 "${OPENVINS_PID}" 2>/dev/null; then
  tail -n 100 "${LOG_DIR}/openvins_estimator.log" || true
  echo "[ERROR] OpenVINS estimator exited early."
  exit 1
fi

echo "[STEP] Start OpenVINS -> RTAB-Map odometry bridge."
python3 "${OPENVINS_ROOT}/nodes/openvins_to_rtabmap_odom.py" \
  --ros-args \
  --params-file "${OPENVINS_BRIDGE_CONFIG}" \
  >"${LOG_DIR}/openvins_bridge.log" 2>&1 &
BRIDGE_PID=$!
echo "${BRIDGE_PID}" >> "${PID_FILE}"

echo "[STEP] Start health monitor."
python3 "${OPENVINS_ROOT}/nodes/openvins_health_monitor.py" \
  >"${LOG_DIR}/openvins_health.log" 2>&1 &
HEALTH_PID=$!
echo "${HEALTH_PID}" >> "${PID_FILE}"

wait_for_topic_message \
  "${OPENVINS_RAW_ODOM_TOPIC}" \
  nav_msgs/msg/Odometry \
  60

wait_for_topic_message \
  "${OPENVINS_OUTPUT_ODOM_TOPIC}" \
  nav_msgs/msg/Odometry \
  15

wait_for_tf \
  "${OPENVINS_ODOM_FRAME}" \
  "${OPENVINS_BASE_FRAME}" \
  15

echo "[OK] V7 OpenVINS backend started."
echo "[INFO] PID file: ${PID_FILE}"
echo "[INFO] Logs: ${LOG_DIR}"

trap - EXIT
