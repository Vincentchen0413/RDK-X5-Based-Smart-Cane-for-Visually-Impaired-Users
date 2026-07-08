#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import signal
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from PIL import Image, ImageDraw
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


class LocalizationPathRecorder(Node):
    def __init__(
        self,
        yaml_path: Path,
        output_base: Path,
        map_frame: str,
        base_frame: str,
        sample_hz: float,
    ) -> None:
        super().__init__("localization_path_recorder")
        self.yaml_path = yaml_path
        self.output_base = output_base
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.sample_period = 1.0 / sample_hz

        config = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        image_path = Path(str(config["image"]))
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path

        self.image = Image.open(image_path).convert("RGB")
        self.resolution = float(config["resolution"])
        origin = config["origin"]
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        self.origin_yaw = float(origin[2])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.localization_events = 0
        self.localized = False
        self.records: list[tuple[float, float, float, float]] = []
        self.last_record = None
        self.start_monotonic = time.monotonic()
        self.last_wait_log = 0.0
        self.finished = False

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/rtabmap/localization_pose",
            self.on_localization,
            10,
        )
        self.create_timer(self.sample_period, self.sample)

        print("[WAIT] Waiting for /rtabmap/localization_pose...", flush=True)
        print(
            "[INFO] Recording starts only after a real visual localization.",
            flush=True,
        )
        print("[INFO] Press Ctrl+C at the end to save CSV and PNG.", flush=True)

    def on_localization(self, msg: PoseWithCovarianceStamped) -> None:
        self.localization_events += 1
        if not self.localized:
            self.localized = True
            p = msg.pose.pose.position
            print(
                f"[LOCALIZED] first event: x={p.x:.3f}, y={p.y:.3f}, "
                f"frame={msg.header.frame_id}",
                flush=True,
            )
        else:
            print(
                f"[LOCALIZED] correction event #{self.localization_events}",
                flush=True,
            )

    def sample(self) -> None:
        if not self.localized:
            now = time.monotonic()
            if now - self.last_wait_log > 5.0:
                print("[WAIT] No visual localization event yet.", flush=True)
                self.last_wait_log = now
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            return

        t = transform.transform.translation
        yaw = yaw_from_quaternion(transform.transform.rotation)
        elapsed = time.monotonic() - self.start_monotonic

        if self.last_record is not None:
            _, px, py, pyaw = self.last_record
            distance = math.hypot(t.x - px, t.y - py)
            angle = abs(wrap_angle(yaw - pyaw))
            if distance < 0.02 and angle < math.radians(2.0):
                return

        record = (elapsed, float(t.x), float(t.y), float(yaw))
        self.records.append(record)
        self.last_record = record
        print(
            f"map pose: x={t.x:7.3f} y={t.y:7.3f} "
            f"yaw={math.degrees(yaw):7.1f}°",
            flush=True,
        )

    def map_to_pixel(self, x: float, y: float) -> tuple[float, float]:
        dx = x - self.origin_x
        dy = y - self.origin_y
        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)

        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy

        col = local_x / self.resolution
        grid_y = local_y / self.resolution
        row = (self.image.height - 1) - grid_y
        return col, row

    def save(self) -> None:
        if self.finished:
            return
        self.finished = True

        self.output_base.parent.mkdir(parents=True, exist_ok=True)
        csv_path = self.output_base.with_suffix(".csv")
        png_path = self.output_base.with_suffix(".png")

        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                ["elapsed_s", "map_x_m", "map_y_m", "yaw_deg"]
            )
            for elapsed, x, y, yaw in self.records:
                writer.writerow(
                    [
                        f"{elapsed:.6f}",
                        f"{x:.6f}",
                        f"{y:.6f}",
                        f"{math.degrees(yaw):.3f}",
                    ]
                )

        scale = 6
        canvas = self.image.resize(
            (self.image.width * scale, self.image.height * scale),
            Image.Resampling.NEAREST,
        )
        draw = ImageDraw.Draw(canvas)

        points = []
        for _, x, y, _ in self.records:
            col, row = self.map_to_pixel(x, y)
            points.append((col * scale, row * scale))

        if len(points) >= 2:
            draw.line(points, fill=(0, 90, 255), width=5)
        if points:
            radius = 10
            sx, sy = points[0]
            ex, ey = points[-1]
            draw.ellipse(
                (sx - radius, sy - radius, sx + radius, sy + radius),
                outline=(255, 0, 0),
                width=5,
            )
            draw.ellipse(
                (ex - radius, ey - radius, ex + radius, ey + radius),
                outline=(0, 200, 0),
                width=5,
            )
            draw.text((sx + 12, sy - 18), "START", fill=(255, 0, 0))
            draw.text((ex + 12, ey - 18), "END", fill=(0, 170, 0))

        canvas.save(png_path)
        print(f"[OK] CSV: {csv_path}", flush=True)
        print(f"[OK] PNG: {png_path}", flush=True)
        print(
            "[LEGEND] Red circle=START, blue line=localized path, "
            "green circle=END",
            flush=True,
        )
        print(
            f"[INFO] samples={len(self.records)}, "
            f"localization_events={self.localization_events}",
            flush=True,
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--yaml", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--sample-hz", type=float, default=5.0)
    args = parser.parse_args()

    yaml_path = Path(args.yaml).expanduser().resolve()
    output_base = Path(args.output).expanduser().resolve()
    if not yaml_path.is_file():
        raise SystemExit(f"YAML not found: {yaml_path}")

    rclpy.init()
    node = LocalizationPathRecorder(
        yaml_path,
        output_base,
        args.map_frame,
        args.base_frame,
        args.sample_hz,
    )

    def stop_handler(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop_handler)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
