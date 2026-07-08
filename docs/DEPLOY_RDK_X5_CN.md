# RDK X5 部署与开机自启

## 1. 部署目录

建议将仓库放在固定路径，例如：

```text
/home/sunrise/smart_cane_ros
```

不建议把运行目录放在 Windows 挂载盘、U 盘或经常变化的路径。

## 2. 用户权限

运行用户需要访问相机、音频和串口/IMU：

```bash
sudo usermod -aG video,audio,dialout $USER
```

修改后重新登录。

## 3. 环境配置

```bash
cp deploy/env.example deploy/env.local
nano deploy/env.local
```

确认 `SMART_CANE_ROS_SETUP` 指向设备实际的 ROS/TROS 环境脚本。

## 4. 启动前验证

```bash
python3 scripts/preflight_check.py
bash scripts/start_all.sh navigation
```

先手动运行成功，再安装 systemd。

## 5. 安装服务

```bash
sudo bash deploy/install_autostart.sh
sudo systemctl start smart-cane.service
sudo systemctl status smart-cane.service
```

实时查看日志：

```bash
journalctl -u smart-cane.service -f
tail -f logs/systemd.log
```

## 6. 常见问题

### 开机后相机不存在

设备驱动可能晚于服务启动。可在服务中增加设备依赖，或在 `start_all.sh` 中等待 `/dev/video*`。

### ROS 节点找不到

检查 `deploy/env.local` 中的 ROS 环境脚本，以及工作空间 `install/setup.bash` 是否已生成。

### 服务反复重启

查看 `logs/*.log`，确认模型、地图、数据库、相机标定和脚本路径。

### 开机时没有网络

核心功能不应依赖网络。天气和监护人推送应使用超时、缓存和指数退避重试。
