#!/usr/bin/env python3
"""Translate offline ASR intents into stable system requests.

Input example on /smart_cane/voice/intent:
{"intent":"navigate","slots":{"place":"电梯"},"confidence":0.93}
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoiceCommandRouter(Node):
    def __init__(self):
        super().__init__("voice_command_router")
        self.request_pub = self.create_publisher(
            String, "/smart_cane/system/request", 10
        )
        self.audio_pub = self.create_publisher(
            String, "/smart_cane/audio/request", 10
        )
        self.create_subscription(
            String, "/smart_cane/voice/intent", self.on_intent, 10
        )

    def publish_request(self, action: str, **kwargs):
        msg = String()
        msg.data = json.dumps(
            {"source": "voice", "action": action, **kwargs},
            ensure_ascii=False,
        )
        self.request_pub.publish(msg)

    def speak(self, text: str, priority: int = 20):
        msg = String()
        msg.data = json.dumps(
            {
                "source": "voice_router",
                "text": text,
                "priority": priority,
                "interruptible": True,
            },
            ensure_ascii=False,
        )
        self.audio_pub.publish(msg)

    def on_intent(self, msg: String):
        try:
            data = json.loads(msg.data)
            intent = data.get("intent", "")
            slots = data.get("slots", {})
            confidence = float(data.get("confidence", 1.0))
        except Exception as exc:
            self.get_logger().warning(f"invalid voice intent: {exc}")
            return

        if confidence < 0.45:
            self.speak("没有听清，请再说一遍")
            return

        if intent == "navigate":
            place = str(slots.get("place", "")).strip()
            if not place:
                self.speak("请说出目的地名称")
                return
            self.publish_request("navigate", place=place)
        elif intent == "mark_landmark":
            place = str(slots.get("place", "")).strip()
            if not place:
                self.speak("请说出地点名称")
                return
            self.publish_request("mark_landmark", place=place)
        elif intent in {
            "start_mapping",
            "stop_mapping",
            "cancel_navigation",
            "system_status",
            "query_time",
            "query_date",
            "query_weather",
        }:
            self.publish_request(intent)
        else:
            self.speak("暂不支持这个指令")


def main():
    rclpy.init()
    node = VoiceCommandRouter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
