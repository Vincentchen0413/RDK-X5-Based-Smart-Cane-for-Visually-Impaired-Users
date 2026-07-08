#!/usr/bin/env bash

# 当前V7环境
export SMART_CANE_ROOT="${SMART_CANE_ROOT:-/home/sunrise/smart_cane_ros}"
export SLAM_NAV_ROOT="${SLAM_NAV_ROOT:-${SMART_CANE_ROOT}/slam_nav}"
export OPENVINS_V7_ROOT="${OPENVINS_V7_ROOT:-${SLAM_NAV_ROOT}/openvins}"

# OpenVINS ROS2工作空间。按实际安装路径修改。
export OPENVINS_WS="${OPENVINS_WS:-/home/sunrise/openvins_ws}"

# 主话题
export OPENVINS_IMU_TOPIC="${OPENVINS_IMU_TOPIC:-/imu_data}"
export OPENVINS_LEFT_TOPIC="${OPENVINS_LEFT_TOPIC:-/StereoNetNode/rectify_left_image}"
export OPENVINS_RIGHT_TOPIC="${OPENVINS_RIGHT_TOPIC:-/StereoNetNode/rectify_right_image}"
export OPENVINS_RAW_ODOM_TOPIC="${OPENVINS_RAW_ODOM_TOPIC:-/openvins/odom_raw}"
export OPENVINS_OUTPUT_ODOM_TOPIC="${OPENVINS_OUTPUT_ODOM_TOPIC:-/odom}"

# 主坐标系
export OPENVINS_ODOM_FRAME="${OPENVINS_ODOM_FRAME:-odom}"
export OPENVINS_BASE_FRAME="${OPENVINS_BASE_FRAME:-base_link}"

# 配置
export OPENVINS_ESTIMATOR_CONFIG="${OPENVINS_ESTIMATOR_CONFIG:-${OPENVINS_V7_ROOT}/config/estimator_config.yaml}"
export OPENVINS_BRIDGE_CONFIG="${OPENVINS_BRIDGE_CONFIG:-${OPENVINS_V7_ROOT}/config/openvins_bridge.yaml}"

# OpenVINS实际启动命令。
# 如果你的版本不是这个命令，请在运行前覆盖该环境变量。
export OPENVINS_START_COMMAND="${OPENVINS_START_COMMAND:-ros2 run ov_msckf run_subscribe_msckf --ros-args -p config_path:=${OPENVINS_ESTIMATOR_CONFIG}}"
