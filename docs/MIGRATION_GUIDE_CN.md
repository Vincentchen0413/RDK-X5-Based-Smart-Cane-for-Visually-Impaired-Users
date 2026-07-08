# 现有工程迁移指南

## 阶段一：不改算法，只补融合层

1. 在现有根目录增加 `bringup`、`config`、`deploy`、`docs`、`adapters` 和 `tests`。
2. 保留 `detection`、`fall`、`openvins`、`slam_nav`、`viz`、`voice` 原目录。
3. 修改 `config/modules.yaml` 中的启动命令，使每个模块可单独运行。
4. 让语音识别只发布“意图”，不直接执行导航脚本。
5. 让所有播报统一发送到 `/smart_cane/audio/request`。
6. 用 `location_cache_node.py` 统一维护 `last_position.json`。
7. 用 `fall_file_adapter.py` 兼容当前跌倒历史文件。

这一阶段改动最小，适合竞赛提交前整理。

## 阶段二：把关键模块改为 ROS 节点

优先改造顺序：

1. 板端目标检测运行节点；
2. 语音 ASR/TTS 话题接口；
3. 跌倒事件直接发布 ROS 消息；
4. 地标管理服务；
5. 导航 action；
6. 云端报警重试队列。

## 阶段三：标准 ROS 2 工作空间

最终可重构为：

```text
src/
├── smart_cane_interfaces
├── smart_cane_bringup
├── smart_cane_perception
├── smart_cane_navigation
├── smart_cane_voice
├── smart_cane_safety
└── smart_cane_viz
```

每个包包含 `package.xml`、`setup.py`/`CMakeLists.txt`、launch、config 和测试。竞赛当前阶段没有必要一次性完成全部重构。
