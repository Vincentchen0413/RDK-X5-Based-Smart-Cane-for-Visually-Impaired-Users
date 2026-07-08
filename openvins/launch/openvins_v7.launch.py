#!/usr/bin/env python3

"""
OpenVINS V7示例Launch文件。

注意：
1. OpenVINS不同版本的package、executable和参数名可能不同；
2. 本文件默认ov_msckf可通过run_subscribe_msckf启动；
3. 桥接节点和健康监控直接以Python进程运行，不要求额外编译ROS包。
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_ROOT = "/home/sunrise/smart_cane_ros/slam_nav/openvins"


def generate_launch_description() -> LaunchDescription:
    root_arg = DeclareLaunchArgument(
        "openvins_root",
        default_value=DEFAULT_ROOT,
    )
    config_arg = DeclareLaunchArgument(
        "config_path",
        default_value=f"{DEFAULT_ROOT}/config/estimator_config.yaml",
    )
    bridge_config_arg = DeclareLaunchArgument(
        "bridge_config_path",
        default_value=f"{DEFAULT_ROOT}/config/openvins_bridge.yaml",
    )

    openvins_root = LaunchConfiguration("openvins_root")
    config_path = LaunchConfiguration("config_path")
    bridge_config_path = LaunchConfiguration("bridge_config_path")

    # 这里的OpenVINS启动形式需要按实际版本核对。
    openvins_node = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        name="openvins_msckf",
        output="screen",
        parameters=[{"config_path": config_path}],
        remappings=[
            ("/imu0", "/imu_data"),
            ("/camera0/image_raw", "/StereoNetNode/rectify_left_image"),
            ("/camera1/image_raw", "/StereoNetNode/rectify_right_image"),
            ("/ov_msckf/odomimu", "/openvins/odom_raw"),
        ],
    )

    bridge = ExecuteProcess(
        cmd=[
            "python3",
            [openvins_root, "/nodes/openvins_to_rtabmap_odom.py"],
            "--ros-args",
            "--params-file",
            bridge_config_path,
        ],
        output="screen",
    )

    health_monitor = ExecuteProcess(
        cmd=[
            "python3",
            [openvins_root, "/nodes/openvins_health_monitor.py"],
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            root_arg,
            config_arg,
            bridge_config_arg,
            openvins_node,
            bridge,
            health_monitor,
        ]
    )
