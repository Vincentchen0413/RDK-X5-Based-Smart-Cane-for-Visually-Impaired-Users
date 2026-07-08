#!/usr/bin/env bash

# 将本段逻辑合并到现有start_mapping_slam_v4.sh中。
# 它不是独立完整的建图脚本，只体现替换rgbd_odometry的核心位置。

ODOM_BACKEND="${ODOM_BACKEND:-openvins}"
OPENVINS_ROOT="${OPENVINS_ROOT:-/home/sunrise/smart_cane_ros/slam_nav/openvins}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
ODOM_FRAME="${ODOM_FRAME:-odom}"
BASE_FRAME="${BASE_FRAME:-base_link}"

start_odom_backend() {
  case "${ODOM_BACKEND}" in
    openvins)
      echo "[INFO] Starting OpenVINS odometry backend."
      bash "${OPENVINS_ROOT}/scripts/start_openvins_backend.sh"
      ;;

    rgbd)
      echo "[INFO] Starting legacy rgbd_odometry backend."
      echo "[INFO] Put the original ros2 run rtabmap_odom rgbd_odometry command here."
      ;;

    *)
      echo "[ERROR] Unsupported ODOM_BACKEND=${ODOM_BACKEND}"
      return 1
      ;;
  esac
}

stop_odom_backend() {
  case "${ODOM_BACKEND}" in
    openvins)
      bash "${OPENVINS_ROOT}/scripts/stop_openvins_backend.sh"
      ;;
    rgbd)
      echo "[INFO] Stop the original rgbd_odometry process here."
      ;;
  esac
}

# 建图脚本中的推荐顺序：
#
# 1. 启动双目相机与StereoNet
# 2. 启动RGB-D同步节点
# 3. 启动现有base_link -> camera静态TF
# 4. start_odom_backend
# 5. 等待/odom
# 6. 等待odom -> base_link
# 7. 启动RTAB-Map mapping，并继续-r /odom:=/odom
#
# 结束建图时：
#
# 1. 先SIGINT停止RTAB-Map并等待数据库保存完成
# 2. 再stop_odom_backend
# 3. 最后停止同步节点、StereoNet和相机
