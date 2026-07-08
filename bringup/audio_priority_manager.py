#!/usr/bin/env python3
"""Single speech output gate with priority, deduplication and cooldown."""

from __future__ import annotations

import heapq
import json
import time
from dataclasses import dataclass, field
from typing import Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


@dataclass(order=True)
class QueueItem:
    sort_key: tuple
    text: str = field(compare=False)
    priority: int = field(compare=False)
    interruptible: bool = field(compare=False)
    key: str = field(compare=False)


class AudioPriorityManager(Node):
    def __init__(self):
        super().__init__("audio_priority_manager")
        self.queue: List[QueueItem] = []
        self.sequence = 0
        self.last_spoken: Dict[str, float] = {}
        self.busy_until = 0.0
        self.speak_pub = self.create_publisher(
            String, "/smart_cane/voice/speak", 10
        )
        self.create_subscription(
            String, "/smart_cane/audio/request", self.on_request, 50
        )
        self.create_timer(0.1, self.tick)

    def on_request(self, msg: String):
        try:
            data = json.loads(msg.data)
            text = str(data["text"]).strip()
            priority = int(data.get("priority", 20))
            interruptible = bool(data.get("interruptible", True))
            key = str(data.get("dedupe_key", text))
            cooldown = float(data.get("cooldown_sec", 1.5))
        except Exception as exc:
            self.get_logger().warning(f"invalid audio request: {exc}")
            return

        now = time.monotonic()
        if now - self.last_spoken.get(key, -1e9) < cooldown:
            return

        self.sequence += 1
        item = QueueItem(
            sort_key=(-priority, self.sequence),
            text=text,
            priority=priority,
            interruptible=interruptible,
            key=key,
        )
        heapq.heappush(self.queue, item)

        # Emergency prompts may preempt the estimated current playback.
        if priority >= 90:
            self.busy_until = now

    def tick(self):
        now = time.monotonic()
        if now < self.busy_until or not self.queue:
            return

        item = heapq.heappop(self.queue)
        out = String()
        out.data = json.dumps(
            {
                "text": item.text,
                "priority": item.priority,
                "interruptible": item.interruptible,
            },
            ensure_ascii=False,
        )
        self.speak_pub.publish(out)
        self.last_spoken[item.key] = now

        # Replace with a playback-finished acknowledgement in the final system.
        estimated_seconds = max(1.2, len(item.text) * 0.18)
        self.busy_until = now + estimated_seconds


def main():
    rclpy.init()
    node = AudioPriorityManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
