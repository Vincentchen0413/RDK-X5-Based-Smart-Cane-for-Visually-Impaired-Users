#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
OpenVINS输入/输出健康监控示例。

输入：
    IMU
    左目图像
    右目图像
    OpenVINS原始Odometry
    桥接后的/odom

输出：
    /openvins/system_healthy
    /openvins/health_reason

该节点不参与坐标计算，只负责给启动脚本和导航状态机提供诊断信号。
"""

from __future__ import annotations

import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Bool, String


class OpenVinsHealthMonitor(Node):
    def __init__(self) -> None:
        super().__init__("openvins_health_monitor")

        self.declare_parameter("imu_topic", "/imu_data")
        self.declare_parameter("left_topic", "/StereoNetNode/rectify_left_image")
        self.declare_parameter("right_topic", "/StereoNetNode/rectify_right_image")
        self.declare_parameter("raw_odom_topic", "/openvins/odom_raw")
        self.declare_parameter("output_odom_topic", "/odom")

        self.declare_parameter("imu_timeout", 0.5)
        self.declare_parameter("camera_timeout", 1.0)
        self.declare_parameter("raw_odom_timeout", 1.0)
        self.declare_parameter("output_odom_timeout", 1.0)

        self.imu_timeout = float(self.get_parameter("imu_timeout").value)
        self.camera_timeout = float(self.get_parameter("camera_timeout").value)
        self.raw_odom_timeout = float(self.get_parameter("raw_odom_timeout").value)
        self.output_odom_timeout = float(self.get_parameter("output_odom_timeout").value)

        self.last_times = {
            "imu": 0.0,
            "left": 0.0,
            "right": 0.0,
            "raw_odom": 0.0,
            "output_odom": 0.0,
        }

        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            lambda _: self.mark("imu"),
            50,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("left_topic").value),
            lambda _: self.mark("left"),
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("right_topic").value),
            lambda _: self.mark("right"),
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("raw_odom_topic").value),
            lambda _: self.mark("raw_odom"),
            20,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("output_odom_topic").value),
            lambda _: self.mark("output_odom"),
            20,
        )

        self.health_pub = self.create_publisher(Bool, "/openvins/system_healthy", 10)
        self.reason_pub = self.create_publisher(String, "/openvins/health_reason", 10)

        self.create_timer(0.2, self.evaluate)

    def mark(self, name: str) -> None:
        self.last_times[name] = time.monotonic()

    def evaluate(self) -> None:
        now = time.monotonic()

        checks = [
            ("IMU数据中断", "imu", self.imu_timeout),
            ("左目图像中断", "left", self.camera_timeout),
            ("右目图像中断", "right", self.camera_timeout),
            ("OpenVINS原始里程计中断", "raw_odom", self.raw_odom_timeout),
            ("桥接后的/odom中断", "output_odom", self.output_odom_timeout),
        ]

        reason = "正常"
        healthy = True

        for message, key, timeout_value in checks:
            last = self.last_times[key]
            if last <= 0.0 or now - last > timeout_value:
                healthy = False
                reason = message
                break

        health_msg = Bool()
        health_msg.data = healthy
        self.health_pub.publish(health_msg)

        reason_msg = String()
        reason_msg.data = reason
        self.reason_pub.publish(reason_msg)


def main() -> None:
    rclpy.init()
    node = OpenVinsHealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
