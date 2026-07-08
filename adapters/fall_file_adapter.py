#!/usr/bin/env python3
"""Bridge legacy fall_history.json updates to /smart_cane/fall/event.

The adapter supports a JSON list, an object containing an "events" list, or a
single event object. Adjust normalize_events() if the existing fall.py uses a
different schema.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, List

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String


def normalize_events(data: Any) -> List[Dict[str, Any]]:
    if isinstance(data, list):
        return [x for x in data if isinstance(x, dict)]
    if isinstance(data, dict) and isinstance(data.get("events"), list):
        return [x for x in data["events"] if isinstance(x, dict)]
    if isinstance(data, dict):
        return [data]
    return []


class FallFileAdapter(Node):
    def __init__(self):
        super().__init__("fall_file_adapter")
        root = Path(
            os.environ.get("SMART_CANE_ROOT", Path(__file__).resolve().parents[1])
        )
        cfg = yaml.safe_load(
            (root / "config/system.yaml").read_text(encoding="utf-8")
        )
        self.path = root / cfg["paths"]["fall_history"]
        self.pub = self.create_publisher(
            String, "/smart_cane/fall/event", 20
        )
        self.last_signature = None
        self.timer = self.create_timer(0.5, self.poll)

    def poll(self):
        if not self.path.exists():
            return
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            events = normalize_events(data)
            if not events:
                return
            latest = events[-1]
            signature = json.dumps(latest, sort_keys=True, ensure_ascii=False)
            if signature == self.last_signature:
                return
            self.last_signature = signature

            out = dict(latest)
            out.setdefault("source", "fall_file_adapter")
            out.setdefault("event_type", "fall")
            msg = String()
            msg.data = json.dumps(out, ensure_ascii=False)
            self.pub.publish(msg)
        except Exception as exc:
            self.get_logger().warning(f"cannot read {self.path}: {exc}")


def main():
    rclpy.init()
    node = FallFileAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
