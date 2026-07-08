#!/usr/bin/env python3
"""Fuse perception, navigation, fall and battery events into safe outputs."""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


CLASS_TEXT = {
    "traffic_light": "前方检测到交通信号灯",
    "puddle": "前方可能有水洼，请绕行",
    "manhole_cover": "前方有井盖，请注意脚下",
    "slippery_area": "前方地面湿滑，请减速",
    "pedestrian_prohibited": "前方禁止行人通行",
    "crosswalk": "前方检测到人行横道",
}


class SafetyEventRouter(Node):
    def __init__(self):
        super().__init__("safety_event_router")
        self.audio_pub = self.create_publisher(
            String, "/smart_cane/audio/request", 20
        )
        self.emergency_pub = self.create_publisher(
            String, "/smart_cane/emergency/upload", 20
        )
        self.create_subscription(
            String, "/smart_cane/perception/event", self.on_perception, 20
        )
        self.create_subscription(
            String, "/smart_cane/fall/event", self.on_fall, 20
        )
        self.create_subscription(
            String, "/smart_cane/navigation/instruction", self.on_navigation, 20
        )
        self.create_subscription(
            String, "/smart_cane/power/event", self.on_power, 20
        )

    def request_audio(
        self,
        text: str,
        priority: int,
        dedupe_key: str,
        cooldown_sec: float,
        interruptible: bool = True,
    ):
        msg = String()
        msg.data = json.dumps(
            {
                "source": "safety_router",
                "text": text,
                "priority": priority,
                "dedupe_key": dedupe_key,
                "cooldown_sec": cooldown_sec,
                "interruptible": interruptible,
            },
            ensure_ascii=False,
        )
        self.audio_pub.publish(msg)

    def on_perception(self, msg: String):
        try:
            event = json.loads(msg.data)
            cls = str(event["class_name"])
            confidence = float(event.get("confidence", 0.0))
            distance = float(event.get("distance_m", -1.0))
        except Exception as exc:
            self.get_logger().warning(f"bad perception event: {exc}")
            return

        if confidence < 0.45:
            return

        text = CLASS_TEXT.get(cls, f"前方检测到{cls}")
        if 0 < distance < 2.0:
            text += f"，距离约{distance:.1f}米"
        priority = 80 if cls in {"puddle", "slippery_area", "pedestrian_prohibited"} else 40
        self.request_audio(text, priority, cls, 4.0)

    def on_navigation(self, msg: String):
        try:
            event = json.loads(msg.data)
            text = str(event["text"])
            kind = str(event.get("kind", "instruction"))
        except Exception:
            return
        self.request_audio(text, 60, f"nav:{kind}:{text}", 0.5)

    def on_power(self, msg: String):
        try:
            event = json.loads(msg.data)
            percent = int(event["percent"])
        except Exception:
            return
        if percent <= 15:
            self.request_audio(
                f"电量仅剩百分之{percent}，请及时充电",
                70,
                "low_battery",
                120.0,
            )

    def on_fall(self, msg: String):
        try:
            event = json.loads(msg.data)
        except Exception:
            event = {"raw": msg.data}

        event["routed_at_unix"] = time.time()
        out = String()
        out.data = json.dumps(event, ensure_ascii=False)
        self.emergency_pub.publish(out)
        self.request_audio(
            "检测到可能跌倒，正在联系监护人",
            100,
            "fall_emergency",
            15.0,
            interruptible=False,
        )


def main():
    rclpy.init()
    node = SafetyEventRouter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
