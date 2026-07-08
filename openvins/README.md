# V7 + OpenVINS 融合示例目录

该目录用于把 OpenVINS 接入当前智能导盲杖 V7 架构，核心设计是：

```text
双目图像 + IMU
        ↓
     OpenVINS
        ↓
局部 VIO：odom → imu_link
        ↓
openvins_to_rtabmap_odom.py
        ↓
/odom + odom → base_link
        ↓
RTAB-Map 建图/重定位
        ↓
map → odom
        ↓
map → base_link
        ↓
地点标记、A*规划、实时导航
```

## 重要说明

本目录是“核心融合代码模板”，不是已经针对你的相机、IMU和 OpenVINS 版本完成标定的成品。

在实机运行前必须替换：

1. `config/kalibr_imu_chain.yaml` 中的 IMU 频率和噪声参数；
2. `config/kalibr_imucam_chain.yaml` 中的相机内参、双目外参、相机—IMU外参和时间偏移；
3. `config/openvins_bridge.yaml` 中的 `T_imu_base`；
4. `scripts/openvins_env.sh` 中 OpenVINS 工作空间路径；
5. OpenVINS 实际输入/输出话题名称；
6. `scripts/start_openvins_backend.sh` 中 OpenVINS 的实际启动命令。

## 文件说明

```text
openvins/
├── README.md
├── config/
│   ├── estimator_config.yaml
│   ├── kalibr_imu_chain.yaml
│   ├── kalibr_imucam_chain.yaml
│   └── openvins_bridge.yaml
├── launch/
│   └── openvins_v7.launch.py
├── nodes/
│   ├── openvins_to_rtabmap_odom.py
│   └── openvins_health_monitor.py
├── scripts/
│   ├── openvins_env.sh
│   ├── start_openvins_backend.sh
│   └── stop_openvins_backend.sh
└── integration/
    ├── mapping_backend_patch.sh
    ├── localization_backend_patch.sh
    └── navigation_health_patch.py
```

## 推荐安装位置

```bash
/home/sunrise/smart_cane_ros/slam_nav/openvins
```

复制后：

```bash
chmod +x /home/sunrise/smart_cane_ros/slam_nav/openvins/nodes/*.py
chmod +x /home/sunrise/smart_cane_ros/slam_nav/openvins/scripts/*.sh
chmod +x /home/sunrise/smart_cane_ros/slam_nav/openvins/integration/*.sh
```

## 最小启动方式

先确认相机、右目图像和 IMU 已经发布，再执行：

```bash
source /home/sunrise/smart_cane_ros/slam_nav/slam_env.sh
bash /home/sunrise/smart_cane_ros/slam_nav/openvins/scripts/start_openvins_backend.sh
```

检查：

```bash
ros2 topic hz /imu_data
ros2 topic hz /StereoNetNode/rectify_left_image
ros2 topic hz /StereoNetNode/rectify_right_image
ros2 topic hz /openvins/odom_raw
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic echo /openvins/healthy --once
```

停止：

```bash
bash /home/sunrise/smart_cane_ros/slam_nav/openvins/scripts/stop_openvins_backend.sh
```

## 与现有 V7 的接法

当前 V7 的 `rgbd_odometry` 应被 OpenVINS 替换，而不是同时发布同一条 TF。

建图与正式定位脚本中：

- 保留 RGB-D 同步节点；
- 保留 RTAB-Map；
- 删除或禁用 `rtabmap_odom rgbd_odometry`；
- 启动本目录中的 OpenVINS 后端；
- 等待 `/odom`；
- 等待 `odom -> base_link`；
- 再启动 RTAB-Map。

RTAB-Map 继续订阅：

```text
/odom
```

下游继续查询：

```text
map -> base_link
```

所以地点标记、A*规划和导航主体不需要因为 OpenVINS 而重写。

## TF 唯一发布原则

主 TF 树推荐为：

```text
map
 └── odom
      └── base_link
           ├── imu_link
           └── slam_camera_optical_frame
```

发布权：

- `map -> odom`：只由 RTAB-Map 发布；
- `odom -> base_link`：只由 `openvins_to_rtabmap_odom.py` 发布；
- `base_link -> imu_link`：静态 TF 或 URDF；
- `base_link -> slam_camera_optical_frame`：现有 V7 静态 TF。

不要让 OpenVINS、适配节点和 `rgbd_odometry` 同时发布 `odom -> base_link`。

## 关于二维输出

桥接节点默认把 OpenVINS 六自由度位姿压成：

```text
x、y、yaw
z = 0
roll = 0
pitch = 0
```

这与当前 V7 的二维建图、地点标记和 A* 导航接口一致。

原始六自由度 OpenVINS 输出仍保留在：

```text
/openvins/odom_raw
```

## 故障策略

初期不建议在运行中自动切换 OpenVINS 和 `rgbd_odometry`，因为两个局部坐标系的原点不同，直接切换会导致 `/odom` 跳变。

更稳妥的策略是：

- 正常运行只启用 OpenVINS；
- OpenVINS失效时暂停导航并提示用户；
- `rgbd_odometry`仅作为独立测试后端；
- 后续若做自动切换，需要增加坐标系对齐层。
