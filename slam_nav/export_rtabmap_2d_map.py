#!/usr/bin/env python3
"""Headless export of RTAB-Map OccupancyGrid to PGM + YAML.

Expected workflow:
1. Start RTAB-Map with the exact database that will later be used for localization.
2. Ensure RTAB-Map's /map output is available (the user's current test remaps it to
   /rtabmap_localization_map).
3. Run this script. It requests a global optimized map through PublishMap, waits
   for nav_msgs/OccupancyGrid, and writes a Nav2-compatible PGM/YAML pair.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Optional

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rtabmap_msgs.srv import PublishMap


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class MapExporter(Node):
    def __init__(self, topic: str, service: str) -> None:
        super().__init__("rtabmap_headless_map_exporter")
        self.topic = topic
        self.service_name = service
        self.map_msg: Optional[OccupancyGrid] = None

        reliable_latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        best_effort_live = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub_reliable = self.create_subscription(
            OccupancyGrid, topic, self._on_map, reliable_latched
        )
        self.sub_best_effort = self.create_subscription(
            OccupancyGrid, topic, self._on_map, best_effort_live
        )
        self.client = self.create_client(PublishMap, service)

    def _on_map(self, msg: OccupancyGrid) -> None:
        if self.map_msg is None:
            self.map_msg = msg
            self.get_logger().info(
                f"Received {self.topic}: {msg.info.width}x{msg.info.height}, "
                f"resolution={msg.info.resolution}"
            )

    def request_map(self, wait_seconds: float) -> None:
        if not self.client.wait_for_service(timeout_sec=wait_seconds):
            raise RuntimeError(
                f"Service {self.service_name} was not found within "
                f"{wait_seconds:.1f} seconds."
            )

        request = PublishMap.Request()
        request.global_map = True
        request.optimized = True
        request.graph_only = False

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=wait_seconds)
        if not future.done():
            raise RuntimeError(
                f"Service call to {self.service_name} timed out."
            )
        if future.exception() is not None:
            raise RuntimeError(
                f"Service call to {self.service_name} failed: "
                f"{future.exception()}"
            )
        self.get_logger().info(
            "Requested global optimized occupancy map from RTAB-Map."
        )

    def wait_for_map(self, timeout_seconds: float) -> OccupancyGrid:
        deadline = time.monotonic() + timeout_seconds
        while (
            rclpy.ok()
            and self.map_msg is None
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.2)

        if self.map_msg is None:
            raise RuntimeError(
                f"No OccupancyGrid was received on {self.topic} within "
                f"{timeout_seconds:.1f} seconds."
            )
        return self.map_msg


def occupancy_to_pixel(
    occupancy: int,
    free_threshold_percent: int,
    occupied_threshold_percent: int,
) -> int:
    if occupancy < 0:
        return 205
    if occupancy >= occupied_threshold_percent:
        return 0
    if occupancy <= free_threshold_percent:
        return 254
    return 205


def save_map(
    msg: OccupancyGrid,
    output_base: Path,
    free_threshold: float,
    occupied_threshold: float,
) -> tuple[Path, Path, Path]:
    width = int(msg.info.width)
    height = int(msg.info.height)
    resolution = float(msg.info.resolution)

    if width <= 0 or height <= 0:
        raise ValueError(f"Invalid map size: {width}x{height}")
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError(f"Invalid map resolution: {resolution}")
    if len(msg.data) != width * height:
        raise ValueError(
            f"OccupancyGrid data length is {len(msg.data)}, "
            f"expected {width * height}."
        )
    if not (0.0 <= free_threshold < occupied_threshold <= 1.0):
        raise ValueError(
            "Thresholds must satisfy "
            "0 <= free_threshold < occupied_threshold <= 1."
        )

    output_base = output_base.expanduser().resolve()
    output_base.parent.mkdir(parents=True, exist_ok=True)
    pgm_path = output_base.with_suffix(".pgm")
    yaml_path = output_base.with_suffix(".yaml")
    metadata_path = output_base.with_name(
        output_base.name + "_metadata.json"
    )

    free_percent = int(round(free_threshold * 100.0))
    occupied_percent = int(round(occupied_threshold * 100.0))

    pixels = bytearray()
    # OccupancyGrid starts at map cell (0, 0), while a PGM starts at the
    # upper-left pixel. Reverse row order so the YAML origin remains correct.
    for y in range(height - 1, -1, -1):
        row_start = y * width
        for x in range(width):
            pixels.append(
                occupancy_to_pixel(
                    int(msg.data[row_start + x]),
                    free_percent,
                    occupied_percent,
                )
            )

    with pgm_path.open("wb") as stream:
        stream.write(f"P5\n# RTAB-Map OccupancyGrid export\n{width} {height}\n255\n".encode("ascii"))
        stream.write(pixels)

    origin = msg.info.origin
    yaw = yaw_from_quaternion(
        float(origin.orientation.x),
        float(origin.orientation.y),
        float(origin.orientation.z),
        float(origin.orientation.w),
    )

    yaml_path.write_text(
        (
            f"image: {pgm_path.name}\n"
            "mode: trinary\n"
            f"resolution: {resolution:.12g}\n"
            f"origin: [{origin.position.x:.12g}, "
            f"{origin.position.y:.12g}, {yaw:.12g}]\n"
            "negate: 0\n"
            f"occupied_thresh: {occupied_threshold:.12g}\n"
            f"free_thresh: {free_threshold:.12g}\n"
        ),
        encoding="utf-8",
    )

    known = sum(1 for value in msg.data if int(value) >= 0)
    occupied = sum(
        1
        for value in msg.data
        if int(value) >= occupied_percent
    )
    free = sum(
        1
        for value in msg.data
        if 0 <= int(value) <= free_percent
    )
    metadata = {
        "topic": msg.header.frame_id,
        "frame_id": msg.header.frame_id,
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin": [
            float(origin.position.x),
            float(origin.position.y),
            yaw,
        ],
        "free_threshold": free_threshold,
        "occupied_threshold": occupied_threshold,
        "known_cells": known,
        "free_cells": free,
        "occupied_cells": occupied,
        "unknown_cells": width * height - known,
        "pgm": str(pgm_path),
        "yaml": str(yaml_path),
    }
    metadata_path.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    return pgm_path, yaml_path, metadata_path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--topic",
        default="/rtabmap_localization_map",
        help="OccupancyGrid topic to export.",
    )
    parser.add_argument(
        "--service",
        default="/rtabmap/publish_map",
        help="RTAB-Map PublishMap service.",
    )
    parser.add_argument(
        "--output-base",
        required=True,
        help="Output path without extension.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=120.0,
        help="Service/topic wait timeout in seconds.",
    )
    parser.add_argument(
        "--free-threshold",
        type=float,
        default=0.25,
    )
    parser.add_argument(
        "--occupied-threshold",
        type=float,
        default=0.65,
    )
    parser.add_argument(
        "--skip-service",
        action="store_true",
        help="Only wait for the map topic; do not call PublishMap.",
    )
    args = parser.parse_args()

    rclpy.init()
    node = MapExporter(args.topic, args.service)
    try:
        if not args.skip_service:
            node.request_map(args.timeout)
        msg = node.wait_for_map(args.timeout)
        pgm_path, yaml_path, metadata_path = save_map(
            msg,
            Path(args.output_base),
            args.free_threshold,
            args.occupied_threshold,
        )
        print("[OK] Headless RTAB-Map 2D map export completed.")
        print(f"[PGM]  {pgm_path}")
        print(f"[YAML] {yaml_path}")
        print(f"[META] {metadata_path}")
        print(
            f"[MAP]  {msg.info.width}x{msg.info.height}, "
            f"resolution={msg.info.resolution}"
        )
        return 0
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
