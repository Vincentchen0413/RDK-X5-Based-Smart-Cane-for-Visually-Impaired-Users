#!/usr/bin/env python3
"""Publish basic host health for degraded-mode decisions."""

import json
import os
import shutil
import time

import psutil
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HealthMonitor(Node):
    def __init__(self):
        super().__init__("health_monitor")
        self.pub = self.create_publisher(
            String, "/smart_cane/system/health", 10
        )
        self.timer = self.create_timer(2.0, self.publish_health)

    def publish_health(self):
        disk = shutil.disk_usage(os.environ.get("SMART_CANE_ROOT", "/"))
        payload = {
            "timestamp_unix": time.time(),
            "cpu_percent": psutil.cpu_percent(interval=None),
            "memory_percent": psutil.virtual_memory().percent,
            "disk_percent": round(100.0 * disk.used / max(disk.total, 1), 1),
            "load_1min": os.getloadavg()[0] if hasattr(os, "getloadavg") else None,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = HealthMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
