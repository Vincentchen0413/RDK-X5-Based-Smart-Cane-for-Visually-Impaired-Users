#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
OpenVINS -> 当前V7 RTAB-Map接口适配节点。

输入：
    /openvins/odom_raw
        OpenVINS输出的IMU六自由度Odometry。

输出：
    /odom
        供RTAB-Map和V7导航使用的base_link里程计。

    TF: odom -> base_link
        主局部里程计TF，只允许本节点发布。

    /openvins/healthy
        OpenVINS输出是否持续、有效。

核心关系：
    T_odom_base = T_odom_imu @ T_imu_base

说明：
    这是融合结构示例。正式工程应进一步严格处理：
    1. OpenVINS具体坐标方向；
    2. 速度的刚体外参变换；
    3. SE(3)协方差伴随变换；
    4. 时间同步和延迟补偿；
    5. OpenVINS重置事件。
"""

from __future__ import annotations

import math
import time
from typing import Iterable, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


def quaternion_to_rotation(q: Quaternion) -> np.ndarray:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(3, dtype=float)

    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rotation_to_quaternion(rotation: np.ndarray) -> Quaternion:
    trace = float(np.trace(rotation))

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s

    result = Quaternion()
    result.x = float(x)
    result.y = float(y)
    result.z = float(z)
    result.w = float(w)
    return result


def rpy_to_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def make_transform(xyz: Iterable[float], rpy_deg: Iterable[float]) -> np.ndarray:
    xyz_list = [float(v) for v in xyz]
    rpy = [math.radians(float(v)) for v in rpy_deg]

    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rpy_to_rotation(*rpy)
    transform[:3, 3] = np.asarray(xyz_list, dtype=float)
    return transform


def yaw_from_rotation(rotation: np.ndarray) -> float:
    return math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))


def yaw_rotation(yaw: float) -> np.ndarray:
    return rpy_to_rotation(0.0, 0.0, yaw)


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


class OpenVinsOdomBridge(Node):
    def __init__(self) -> None:
        super().__init__("openvins_to_rtabmap_odom")

        self.declare_parameter("input_topic", "/openvins/odom_raw")
        self.declare_parameter("output_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("imu_to_base_xyz", [0.0, 0.0, -0.70])
        self.declare_parameter("imu_to_base_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("planar_output", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("timeout_seconds", 1.0)
        self.declare_parameter("max_linear_jump_m", 1.0)
        self.declare_parameter("max_yaw_jump_deg", 60.0)
        self.declare_parameter("zero_first_pose", False)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.planar_output = bool(self.get_parameter("planar_output").value)
        self.publish_tf_enabled = bool(self.get_parameter("publish_tf").value)
        self.timeout_seconds = float(self.get_parameter("timeout_seconds").value)
        self.max_linear_jump_m = float(self.get_parameter("max_linear_jump_m").value)
        self.max_yaw_jump = math.radians(float(self.get_parameter("max_yaw_jump_deg").value))
        self.zero_first_pose = bool(self.get_parameter("zero_first_pose").value)

        self.t_imu_base = make_transform(
            self.get_parameter("imu_to_base_xyz").value,
            self.get_parameter("imu_to_base_rpy_deg").value,
        )

        self.odom_publisher = self.create_publisher(Odometry, self.output_topic, 20)
        self.health_publisher = self.create_publisher(Bool, "/openvins/healthy", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Odometry, self.input_topic, self.on_openvins_odom, 50)
        self.create_timer(0.2, self.publish_health)

        self.last_receive_monotonic = 0.0
        self.last_stamp_ns = -1
        self.initialized = False
        self.last_output_transform: Optional[np.ndarray] = None
        self.first_pose_inverse: Optional[np.ndarray] = None

        self.get_logger().info(
            f"OpenVINS bridge: {self.input_topic} -> {self.output_topic}, "
            f"TF {self.odom_frame}->{self.base_frame}, planar={self.planar_output}"
        )

    def on_openvins_odom(self, msg: Odometry) -> None:
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if stamp_ns <= self.last_stamp_ns:
            self.get_logger().warning("Rejected non-increasing OpenVINS timestamp.")
            return

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(float(v)) for v in values):
            self.get_logger().warning("Rejected non-finite OpenVINS pose.")
            return

        t_odom_imu = np.eye(4, dtype=float)
        t_odom_imu[:3, :3] = quaternion_to_rotation(q)
        t_odom_imu[:3, 3] = np.array([p.x, p.y, p.z], dtype=float)

        # 核心融合接口转换
        t_odom_base = t_odom_imu @ self.t_imu_base

        if self.zero_first_pose:
            if self.first_pose_inverse is None:
                self.first_pose_inverse = np.linalg.inv(t_odom_base)
            t_odom_base = self.first_pose_inverse @ t_odom_base

        if self.planar_output:
            yaw = yaw_from_rotation(t_odom_base[:3, :3])
            t_odom_base[2, 3] = 0.0
            t_odom_base[:3, :3] = yaw_rotation(yaw)

        if not self.is_jump_reasonable(t_odom_base):
            self.get_logger().warning("Rejected an implausible OpenVINS pose jump.")
            return

        output = Odometry()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = self.odom_frame
        output.child_frame_id = self.base_frame

        output.pose.pose.position.x = float(t_odom_base[0, 3])
        output.pose.pose.position.y = float(t_odom_base[1, 3])
        output.pose.pose.position.z = float(t_odom_base[2, 3])
        output.pose.pose.orientation = rotation_to_quaternion(t_odom_base[:3, :3])

        # 这里只是保留输入协方差。正式版本应做SE(3)伴随变换。
        output.pose.covariance = list(msg.pose.covariance)

        if self.planar_output:
            output.twist.twist.linear.x = float(msg.twist.twist.linear.x)
            output.twist.twist.linear.y = float(msg.twist.twist.linear.y)
            output.twist.twist.angular.z = float(msg.twist.twist.angular.z)
            output.twist.covariance = list(msg.twist.covariance)
        else:
            output.twist = msg.twist

        self.odom_publisher.publish(output)

        if self.publish_tf_enabled:
            transform = TransformStamped()
            transform.header.stamp = msg.header.stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = float(t_odom_base[0, 3])
            transform.transform.translation.y = float(t_odom_base[1, 3])
            transform.transform.translation.z = float(t_odom_base[2, 3])
            transform.transform.rotation = output.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

        self.last_output_transform = t_odom_base.copy()
        self.last_stamp_ns = stamp_ns
        self.last_receive_monotonic = time.monotonic()
        self.initialized = True

    def is_jump_reasonable(self, current: np.ndarray) -> bool:
        if self.last_output_transform is None:
            return True

        previous = self.last_output_transform
        translation_jump = float(np.linalg.norm(current[:3, 3] - previous[:3, 3]))

        previous_yaw = yaw_from_rotation(previous[:3, :3])
        current_yaw = yaw_from_rotation(current[:3, :3])
        yaw_jump = abs(wrap_angle(current_yaw - previous_yaw))

        return (
            translation_jump <= self.max_linear_jump_m
            and yaw_jump <= self.max_yaw_jump
        )

    def publish_health(self) -> None:
        healthy = (
            self.initialized
            and time.monotonic() - self.last_receive_monotonic <= self.timeout_seconds
        )
        msg = Bool()
        msg.data = bool(healthy)
        self.health_publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = OpenVinsOdomBridge()
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
