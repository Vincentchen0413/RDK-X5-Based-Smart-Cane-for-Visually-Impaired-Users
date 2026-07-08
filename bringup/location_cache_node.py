#!/usr/bin/env python3
"""Cache the latest fused pose for navigation and emergency reporting."""

from __future__ import annotations

import json
import math
import os
import tempfile
import time
from pathlib import Path

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class LocationCacheNode(Node):
    def __init__(self):
        super().__init__("location_cache_node")
        root = Path(
            os.environ.get("SMART_CANE_ROOT", Path(__file__).resolve().parents[1])
        )
        cfg = yaml.safe_load(
            (root / "config/system.yaml").read_text(encoding="utf-8")
        )
        self.output_path = root / cfg["paths"]["location_cache"]
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        odom_topic = cfg["ros"].get("odom_topic", "/rtabmap/odom")
        location_topic = cfg["ros"].get(
            "location_topic", "/smart_cane/location/current"
        )

        self.pub = self.create_publisher(String, location_topic, 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 20)

    def atomic_write(self, payload: dict):
        fd, tmp = tempfile.mkstemp(
            prefix=self.output_path.name + ".",
            dir=str(self.output_path.parent),
            text=True,
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(payload, f, ensure_ascii=False, indent=2)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self.output_path)
        finally:
            if os.path.exists(tmp):
                os.unlink(tmp)

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        payload = {
            "timestamp_unix": time.time(),
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "x": p.x,
            "y": p.y,
            "z": p.z,
            "yaw": yaw_from_quaternion(q.x, q.y, q.z, q.w),
            "source": "fused_odom",
        }
        self.atomic_write(payload)
        out = String()
        out.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = LocationCacheNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
