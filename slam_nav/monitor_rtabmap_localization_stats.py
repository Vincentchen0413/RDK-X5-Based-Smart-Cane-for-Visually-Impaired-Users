#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import time
from typing import Iterable

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rtabmap_msgs.msg import Info


def top_pairs(keys: Iterable[int], values: Iterable[float], count: int = 3):
    pairs = []
    for key, value in zip(keys, values):
        value = float(value)
        if math.isfinite(value):
            pairs.append((int(key), value))
    pairs.sort(key=lambda item: item[1], reverse=True)
    return pairs[:count]


class Monitor(Node):
    def __init__(self, topic: str, duration: float) -> None:
        super().__init__("monitor_rtabmap_localization_stats")
        self.deadline = time.monotonic() + duration
        self.received = 0
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Info, topic, self.callback, qos)

    def callback(self, msg: Info) -> None:
        self.received += 1
        stats = dict(zip(msg.stats_keys, msg.stats_values))
        interesting = {}
        tokens = (
            "highest_hypothesis",
            "hypothesis",
            "visual_inlier",
            "visual_match",
            "loop/",
            "retriev",
            "likelihood",
            "keypoint",
            "feature",
            "word",
            "signature",
        )
        for key, value in stats.items():
            lower = key.lower()
            if any(token in lower for token in tokens):
                interesting[key] = float(value)

        print(
            f"\n[INFO #{self.received}] ref={msg.ref_id} "
            f"loop={msg.loop_closure_id} "
            f"proximity={msg.proximity_detection_id} "
            f"landmark={msg.landmark_id} "
            f"wm={len(msg.wm_state)}"
        )
        print(f"posterior_top={top_pairs(msg.posterior_keys, msg.posterior_values)}")
        print(f"likelihood_top={top_pairs(msg.likelihood_keys, msg.likelihood_values)}")
        print(
            f"raw_likelihood_top="
            f"{top_pairs(msg.raw_likelihood_keys, msg.raw_likelihood_values)}"
        )
        if interesting:
            for key in sorted(interesting):
                print(f"{key}={interesting[key]:.9g}")
        else:
            print("selected_stats: none")

        if (
            msg.loop_closure_id > 0
            or msg.proximity_detection_id > 0
            or msg.landmark_id > 0
        ):
            print("[LOCALIZED] Accepted localization constraint detected.")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/info")
    parser.add_argument("--duration", type=float, default=60.0)
    args = parser.parse_args()

    rclpy.init()
    node = Monitor(args.topic, args.duration)
    try:
        while rclpy.ok() and time.monotonic() < node.deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        count = node.received
        node.destroy_node()
        rclpy.shutdown()

    if count == 0:
        print("[ERROR] No Info messages received.")
        return 2
    print(f"\n[DONE] Received {count} Info messages.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
