#!/usr/bin/env python3
"""System state machine and command dispatcher.

The current implementation calls the existing shell/Python scripts. In a fully
ROS-native version, these actions can be replaced by lifecycle services.
"""

from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path
from typing import Dict, List

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String


class SystemStateManager(Node):
    VALID_STATES = {
        "BOOTING",
        "IDLE",
        "MAPPING",
        "LOCALIZING",
        "NAVIGATING",
        "EMERGENCY",
        "DEGRADED",
    }

    def __init__(self):
        super().__init__("system_state_manager")
        self.root = Path(
            os.environ.get("SMART_CANE_ROOT", Path(__file__).resolve().parents[1])
        )
        cfg = yaml.safe_load(
            (self.root / "config/system.yaml").read_text(encoding="utf-8")
        )
        self.commands: Dict[str, List[str]] = cfg["commands"]
        self.state = "IDLE"
        self.child_processes: Dict[str, subprocess.Popen] = {}

        self.state_pub = self.create_publisher(
            String, "/smart_cane/system/state", 10
        )
        self.audio_pub = self.create_publisher(
            String, "/smart_cane/audio/request", 10
        )
        self.create_subscription(
            String, "/smart_cane/system/request", self.on_request, 20
        )
        self.create_subscription(
            String, "/smart_cane/fall/event", self.on_fall, 20
        )
        self.publish_state("startup")

    def publish_state(self, reason: str):
        msg = String()
        msg.data = json.dumps(
            {"state": self.state, "reason": reason}, ensure_ascii=False
        )
        self.state_pub.publish(msg)

    def set_state(self, state: str, reason: str):
        if state not in self.VALID_STATES:
            raise ValueError(state)
        self.state = state
        self.publish_state(reason)

    def speak(self, text: str, priority: int = 50):
        msg = String()
        msg.data = json.dumps(
            {
                "source": "state_manager",
                "text": text,
                "priority": priority,
                "interruptible": priority < 90,
            },
            ensure_ascii=False,
        )
        self.audio_pub.publish(msg)

    def run_command(self, key: str, extra: List[str] | None = None):
        command = [str(x) for x in self.commands[key]]
        if extra:
            command.extend(extra)
        self.get_logger().info("run: " + " ".join(command))
        proc = subprocess.Popen(command, cwd=self.root, start_new_session=True)
        self.child_processes[key] = proc
        return proc

    def stop_if_running(self, key: str):
        proc = self.child_processes.get(key)
        if proc and proc.poll() is None:
            proc.terminate()

    def on_request(self, msg: String):
        try:
            data = json.loads(msg.data)
            action = data["action"]
        except Exception as exc:
            self.get_logger().warning(f"invalid request: {exc}")
            return

        try:
            if action == "start_mapping":
                self.stop_if_running("start_localization")
                self.run_command("start_mapping")
                self.set_state("MAPPING", "voice_request")
                self.speak("开始室内建图")
            elif action == "stop_mapping":
                self.run_command("stop_mapping")
                self.set_state("IDLE", "mapping_saved")
                self.speak("建图结束，地图已保存")
            elif action == "navigate":
                place = str(data.get("place", "")).strip()
                self.run_command("navigate_to_landmark", [place])
                self.set_state("NAVIGATING", f"goal:{place}")
                self.speak(f"开始前往{place}")
            elif action == "mark_landmark":
                place = str(data.get("place", "")).strip()
                self.run_command("mark_landmark", [place])
                self.speak(f"已记录当前位置为{place}")
            elif action == "cancel_navigation":
                self.stop_if_running("navigate_to_landmark")
                self.set_state("LOCALIZING", "navigation_cancelled")
                self.speak("导航已取消")
            elif action == "system_status":
                self.speak(f"系统当前处于{self.state}状态")
            elif action in {"query_time", "query_date", "query_weather"}:
                # Let the existing voice node answer these local/cloud queries.
                forward = String()
                forward.data = json.dumps(
                    {"action": action, "source": "state_manager"},
                    ensure_ascii=False,
                )
                # Reuse a dedicated topic in the real project.
                self.get_logger().info(forward.data)
        except Exception as exc:
            self.get_logger().error(f"action {action} failed: {exc}")
            self.set_state("DEGRADED", f"command_failed:{action}")
            self.speak("相关功能启动失败，请检查系统", priority=75)

    def on_fall(self, msg: String):
        self.set_state("EMERGENCY", "fall_detected")
        self.speak("检测到可能跌倒，正在发送报警", priority=100)


def main():
    rclpy.init()
    node = SystemStateManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
