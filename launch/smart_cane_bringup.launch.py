#!/usr/bin/env python3
"""ROS 2 launch entry that delegates legacy process control to the supervisor."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    root = Path(
        os.environ.get("SMART_CANE_ROOT", Path(__file__).resolve().parents[1])
    )
    profile = LaunchConfiguration("profile")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="navigation",
                description="navigation or mapping",
            ),
            ExecuteProcess(
                cmd=[
                    "python3",
                    str(root / "bringup/smart_cane_supervisor.py"),
                    "--profile",
                    profile,
                ],
                cwd=str(root),
                output="screen",
            ),
        ]
    )
