#!/usr/bin/env bash

# 将本段逻辑合并到现有start_localization_verify_v4.sh中。
# RTAB-Map正式定位仍负责map -> odom，OpenVINS负责odom -> base_link。

OPENVINS_ROOT="${OPENVINS_ROOT:-/home/sunrise/smart_cane_ros/slam_nav/openvins}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
ODOM_FRAME="${ODOM_FRAME:-odom}"
BASE_FRAME="${BASE_FRAME:-base_link}"

start_local_vio() {
  bash "${OPENVINS_ROOT}/scripts/start_openvins_backend.sh"
}

stop_local_vio() {
  bash "${OPENVINS_ROOT}/scripts/stop_openvins_backend.sh"
}

# 正式定位推荐顺序：
#
# 1. 启动双目相机、StereoNet和IMU驱动
# 2. 启动RGB-D同步节点
# 3. 启动现有base_link -> camera静态TF
# 4. start_local_vio
# 5. 等待/odom和odom -> base_link
# 6. 启动map_server发布静态/map
# 7. 启动RTAB-Map localization
# 8. RTAB-Map订阅/odom并发布map -> odom
# 9. 等待/info有效定位事件
# 10. 验证map -> base_link
#
# 注意：
# OpenVINS每次启动的odom局部原点可以不同。
# RTAB-Map通过视觉重定位重新计算map -> odom，
# 最终仍能得到正确的map -> base_link。
