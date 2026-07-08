#!/usr/bin/env python3
"""Command-line landmark manager for an RTAB-Map/Nav2 map.

Typical usage while formal localization is running:
  python3 landmark_cli.py mark 起点
  python3 landmark_cli.py mark 转角
  python3 landmark_cli.py list

A landmark is saved only after:
- either a new accepted RTAB-Map localization event is observed on /info,
  or a long-running supervisor explicitly confirms a recent localization;
- map -> base_link is fresh and stable for a short sampling window;
- the saved point is on a free cell of the matching YAML/PGM map.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
import statistics
import sys
import tempfile
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from PIL import Image
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from rtabmap_msgs.msg import Info
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


DEFAULT_BASE = Path(
    os.environ.get(
        "SLAM_NAV_BASE",
        "/home/sunrise/smart_cane_ros/slam_nav",
    )
)
DEFAULT_MAP_DIR = DEFAULT_BASE / "blind_cane_maps"
DEFAULT_YAML = DEFAULT_MAP_DIR / "map1.yaml"
DEFAULT_DB = DEFAULT_MAP_DIR / "map1_recovered.db"
DEFAULT_FILE = DEFAULT_MAP_DIR / "map1_landmarks.json"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def circular_mean(values: list[float]) -> float:
    sine = sum(math.sin(value) for value in values)
    cosine = sum(math.cos(value) for value in values)
    return math.atan2(sine, cosine)


def circular_std(values: list[float]) -> float:
    if not values:
        return float("inf")
    sine = sum(math.sin(value) for value in values) / len(values)
    cosine = sum(math.cos(value) for value in values) / len(values)
    resultant = min(1.0, max(1e-12, math.hypot(sine, cosine)))
    return math.sqrt(max(0.0, -2.0 * math.log(resultant)))


def atomic_write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        backup = path.with_suffix(path.suffix + ".bak")
        shutil.copy2(path, backup)

    fd, temp_name = tempfile.mkstemp(
        prefix=path.name + ".",
        suffix=".tmp",
        dir=str(path.parent),
    )
    temp_path = Path(temp_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_path, path)
    finally:
        if temp_path.exists():
            temp_path.unlink(missing_ok=True)


def load_map(yaml_path: Path) -> dict:
    data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    image_path = Path(str(data["image"]))
    if not image_path.is_absolute():
        image_path = (yaml_path.parent / image_path).resolve()

    image = Image.open(image_path).convert("L")
    origin = [float(value) for value in data["origin"]]
    if len(origin) != 3:
        raise ValueError(f"Invalid map origin: {origin}")

    return {
        "yaml_path": yaml_path.resolve(),
        "image_path": image_path,
        "image": image,
        "resolution": float(data["resolution"]),
        "origin": origin,
        "negate": int(data.get("negate", 0)),
        "occupied_thresh": float(data.get("occupied_thresh", 0.65)),
        "free_thresh": float(data.get("free_thresh", 0.25)),
        "width": image.width,
        "height": image.height,
        "yaml_sha256": sha256_file(yaml_path),
        "image_sha256": sha256_file(image_path),
    }


def world_to_pixel(
    x: float,
    y: float,
    resolution: float,
    origin: list[float],
    image_height: int,
) -> tuple[int, int]:
    origin_x, origin_y, origin_yaw = origin
    dx = x - origin_x
    dy = y - origin_y
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    map_x = cosine * dx + sine * dy
    map_y = -sine * dx + cosine * dy
    pixel_x = int(math.floor(map_x / resolution))
    pixel_y = int(math.floor(image_height - 1 - map_y / resolution))
    return pixel_x, pixel_y


def pixel_to_world(
    pixel_x: int,
    pixel_y: int,
    resolution: float,
    origin: list[float],
    image_height: int,
) -> tuple[float, float]:
    origin_x, origin_y, origin_yaw = origin
    map_x = (pixel_x + 0.5) * resolution
    map_y = (image_height - 1 - pixel_y + 0.5) * resolution
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    world_x = origin_x + cosine * map_x - sine * map_y
    world_y = origin_y + sine * map_x + cosine * map_y
    return world_x, world_y


def classify_pixel(value: int, map_data: dict) -> str:
    if map_data["negate"]:
        occupancy = value / 255.0
    else:
        occupancy = (255.0 - value) / 255.0

    if occupancy > map_data["occupied_thresh"]:
        return "occupied"
    if occupancy < map_data["free_thresh"]:
        return "free"
    return "unknown"


def point_cell_state(x: float, y: float, map_data: dict) -> tuple[str, int, int]:
    pixel_x, pixel_y = world_to_pixel(
        x,
        y,
        map_data["resolution"],
        map_data["origin"],
        map_data["height"],
    )
    if not (
        0 <= pixel_x < map_data["width"]
        and 0 <= pixel_y < map_data["height"]
    ):
        return "outside", pixel_x, pixel_y
    value = int(map_data["image"].getpixel((pixel_x, pixel_y)))
    return classify_pixel(value, map_data), pixel_x, pixel_y



def map_local_coordinates(
    x: float,
    y: float,
    origin: list[float],
) -> tuple[float, float]:
    """Convert world coordinates into the unrotated map-local coordinate system."""
    origin_x, origin_y, origin_yaw = origin
    dx = x - origin_x
    dy = y - origin_y
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    map_x = cosine * dx + sine * dy
    map_y = -sine * dx + cosine * dy
    return map_x, map_y


def outside_distance_m(x: float, y: float, map_data: dict) -> float:
    """Euclidean distance from a world point to the map rectangle; 0 when inside."""
    map_x, map_y = map_local_coordinates(x, y, map_data["origin"])
    width_m = map_data["width"] * map_data["resolution"]
    height_m = map_data["height"] * map_data["resolution"]
    dx = max(0.0, -map_x, map_x - width_m)
    dy = max(0.0, -map_y, map_y - height_m)
    return math.hypot(dx, dy)


def map_bounds_text(map_data: dict) -> str:
    """Human-readable axis-aligned bounds for the common zero-yaw map case."""
    origin_x, origin_y, origin_yaw = map_data["origin"]
    width_m = map_data["width"] * map_data["resolution"]
    height_m = map_data["height"] * map_data["resolution"]
    if abs(origin_yaw) < 1e-6:
        return (
            f"x=[{origin_x:.3f},{origin_x + width_m:.3f}], "
            f"y=[{origin_y:.3f},{origin_y + height_m:.3f}]"
        )
    return (
        f"map_local_x=[0,{width_m:.3f}], "
        f"map_local_y=[0,{height_m:.3f}], yaw={origin_yaw:.3f}rad"
    )


def nearest_free_cell(
    center_x: int,
    center_y: int,
    max_radius_cells: int,
    map_data: dict,
) -> Optional[tuple[int, int, float]]:
    best: Optional[tuple[int, int, float]] = None
    for radius in range(max_radius_cells + 1):
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if max(abs(dx), abs(dy)) != radius:
                    continue
                pixel_x = center_x + dx
                pixel_y = center_y + dy
                if not (
                    0 <= pixel_x < map_data["width"]
                    and 0 <= pixel_y < map_data["height"]
                ):
                    continue
                value = int(map_data["image"].getpixel((pixel_x, pixel_y)))
                if classify_pixel(value, map_data) != "free":
                    continue
                distance = math.hypot(dx, dy) * map_data["resolution"]
                if best is None or distance < best[2]:
                    best = (pixel_x, pixel_y, distance)
        if best is not None:
            return best
    return None


def make_empty_database(
    map_name: str,
    map_db: Path,
    map_data: dict,
) -> dict:
    return {
        "schema_version": 1,
        "map": {
            "name": map_name,
            "frame_id": "map",
            "db": str(map_db.resolve()),
            "yaml": str(map_data["yaml_path"]),
            "image": str(map_data["image_path"]),
            "resolution": map_data["resolution"],
            "origin": map_data["origin"],
            "width": map_data["width"],
            "height": map_data["height"],
            "yaml_sha256": map_data["yaml_sha256"],
            "image_sha256": map_data["image_sha256"],
        },
        "places": {},
    }


def load_database(
    path: Path,
    map_name: str,
    map_db: Path,
    map_data: dict,
    force_map_mismatch: bool,
) -> dict:
    if not path.exists():
        return make_empty_database(map_name, map_db, map_data)

    payload = json.loads(path.read_text(encoding="utf-8"))
    stored_map = payload.get("map", {})
    mismatch = (
        stored_map.get("yaml_sha256") != map_data["yaml_sha256"]
        or stored_map.get("image_sha256") != map_data["image_sha256"]
    )
    if mismatch and not force_map_mismatch:
        raise RuntimeError(
            "The landmark file belongs to a different map image/YAML. "
            "Use the matching map or pass --force-map-mismatch only after "
            "manually verifying coordinate compatibility."
        )
    payload.setdefault("places", {})
    return payload


class PoseSampler(Node):
    def __init__(
        self,
        info_topic: str,
        map_frame: str,
        base_frame: str,
        max_tf_age: float,
    ) -> None:
        super().__init__("landmark_cli_pose_sampler")
        self.info_topic = info_topic
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.max_tf_age = max(0.0, float(max_tf_age))
        self.last_localization_event: Optional[float] = None
        self.last_ids = (0, 0, 0)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Info, info_topic, self._info_callback, qos)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )
        self.last_tf_error = ""
        self.last_tf_age: Optional[float] = None

    def _info_callback(self, msg: Info) -> None:
        ids = (
            int(msg.loop_closure_id),
            int(msg.proximity_detection_id),
            int(msg.landmark_id),
        )
        if any(value > 0 for value in ids):
            self.last_localization_event = time.monotonic()
            self.last_ids = ids

    def wait_for_localization(self, timeout_seconds: float) -> None:
        deadline = time.monotonic() + timeout_seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.last_localization_event is not None:
                print(
                    "[LOCALIZED] "
                    f"loop={self.last_ids[0]}, "
                    f"proximity={self.last_ids[1]}, "
                    f"landmark={self.last_ids[2]}"
                )
                return
        raise RuntimeError(
            f"No accepted localization event was observed on "
            f"{self.info_topic} within {timeout_seconds:.0f}s. "
            "Keep formal localization running and move slowly in a mapped area."
        )

    def _latest_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
            )

            stamp = transform.header.stamp
            stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            self.last_tf_age = None
            if stamp_ns > 0:
                now_ns = int(self.get_clock().now().nanoseconds)
                age = (now_ns - stamp_ns) / 1_000_000_000.0
                self.last_tf_age = age
                if self.max_tf_age > 0.0 and (
                    age > self.max_tf_age or age < -1.0
                ):
                    self.last_tf_error = (
                        f"Latest TF is stale or clock-mismatched: "
                        f"age={age:.3f}s, limit={self.max_tf_age:.3f}s"
                    )
                    return None

            self.last_tf_error = ""
            return transform
        except TransformException as exc:
            self.last_tf_error = str(exc)
            return None

    def latest_pose(self) -> Optional[tuple[float, float, float, float]]:
        transform = self._latest_transform()
        if transform is None:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            float(translation.x),
            float(translation.y),
            float(translation.z),
            yaw_from_quaternion(
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            ),
        )

    def wait_for_pose_in_map(
        self,
        map_data: dict,
        timeout_seconds: float,
        consecutive_required: int = 3,
    ) -> None:
        """Require the current map->base_link pose to be inside the static map.

        This rejects a valid-looking TF that belongs to an odometry continuation
        outside the exported PGM, or a wrong visual localization solution.
        """
        deadline = time.monotonic() + timeout_seconds
        consecutive = 0
        last_detail = "No TF pose received"
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            pose = self.latest_pose()
            if pose is None:
                continue
            x, y, _, _ = pose
            state, pixel_x, pixel_y = point_cell_state(x, y, map_data)
            distance = outside_distance_m(x, y, map_data)
            last_detail = (
                f"x={x:.3f}, y={y:.3f}, pixel=({pixel_x},{pixel_y}), "
                f"state={state}, outside_distance={distance:.3f}m, "
                f"bounds={map_bounds_text(map_data)}"
            )
            if state != "outside":
                consecutive += 1
                if consecutive >= max(1, consecutive_required):
                    print(f"[MAP CHECK] Pose is inside static map: {last_detail}")
                    return
            else:
                consecutive = 0
            time.sleep(0.05)

        raise RuntimeError(
            "Current localization pose is outside static map bounds or never "
            f"became valid within {timeout_seconds:.1f}s. {last_detail}. "
            "Return to an area covered by the exported map. If this physical "
            "place was walked during mapping, re-localize or rebuild/export a "
            "larger map instead of snapping the goal."
        )

    def wait_for_tf(self, timeout_seconds: float) -> None:
        deadline = time.monotonic() + timeout_seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._latest_transform() is not None:
                print(
                    f"[TF] {self.map_frame} -> {self.base_frame} is ready."
                )
                return
        detail = f" Last TF error: {self.last_tf_error}" if self.last_tf_error else ""
        raise RuntimeError(
            f"TF {self.map_frame} -> {self.base_frame} was not available "
            f"within {timeout_seconds:.0f}s.{detail}"
        )

    def collect_samples(
        self,
        sample_seconds: float,
        sample_rate: float,
        min_samples: int,
        collection_timeout: float,
    ) -> tuple[list[tuple[float, float, float, float]], int]:
        if self.last_localization_event is None:
            raise RuntimeError("No localization event is available.")

        period = 1.0 / sample_rate
        start = time.monotonic()
        minimum_deadline = start + sample_seconds
        final_deadline = start + max(sample_seconds, collection_timeout)
        next_sample = start
        samples: list[tuple[float, float, float, float]] = []
        skipped = 0

        while rclpy.ok():
            now = time.monotonic()
            if (
                now >= minimum_deadline
                and len(samples) >= min_samples
            ):
                break
            if now >= final_deadline:
                break

            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.monotonic()
            if now < next_sample:
                continue
            next_sample += period

            transform = self._latest_transform()
            if transform is None:
                skipped += 1
                continue

            translation = transform.transform.translation
            rotation = transform.transform.rotation
            yaw = yaw_from_quaternion(
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            )
            samples.append(
                (
                    float(translation.x),
                    float(translation.y),
                    float(translation.z),
                    yaw,
                )
            )

        return samples, skipped


def summarize_samples(
    samples: list[tuple[float, float, float, float]],
) -> dict:
    if not samples:
        raise RuntimeError("No TF samples were collected.")

    xs = [sample[0] for sample in samples]
    ys = [sample[1] for sample in samples]
    zs = [sample[2] for sample in samples]
    yaws = [sample[3] for sample in samples]

    x = statistics.median(xs)
    y = statistics.median(ys)
    z = statistics.median(zs)
    yaw = circular_mean(yaws)

    radial_errors = [
        math.hypot(sample_x - x, sample_y - y)
        for sample_x, sample_y in zip(xs, ys)
    ]
    position_rms = math.sqrt(
        sum(error * error for error in radial_errors) / len(radial_errors)
    )
    position_max = max(radial_errors)
    yaw_std = circular_std(yaws)

    return {
        "x": x,
        "y": y,
        "z": z,
        "yaw": yaw,
        "position_rms": position_rms,
        "position_max": position_max,
        "yaw_std": yaw_std,
        "sample_count": len(samples),
    }


def command_mark(args: argparse.Namespace) -> int:
    yaml_path = Path(args.yaml).expanduser().resolve()
    map_db = Path(args.db).expanduser().resolve()
    landmark_file = Path(args.file).expanduser().resolve()

    if not yaml_path.is_file():
        raise RuntimeError(f"Map YAML does not exist: {yaml_path}")
    if not map_db.is_file():
        raise RuntimeError(f"Map database does not exist: {map_db}")

    map_data = load_map(yaml_path)
    database = load_database(
        landmark_file,
        args.map_name,
        map_db,
        map_data,
        args.force_map_mismatch,
    )

    if args.name in database["places"] and not args.overwrite:
        raise RuntimeError(
            f'Place "{args.name}" already exists. '
            "Pass --overwrite to replace it."
        )

    rclpy.init()
    node = PoseSampler(
        info_topic=args.info_topic,
        map_frame=args.map_frame,
        base_frame=args.base_frame,
        max_tf_age=args.max_tf_age,
    )
    try:
        if args.skip_new_localization_wait:
            # A long-running supervisor (the unified voice node) has already
            # confirmed that an accepted /info event is recent. We still
            # require a fresh TF and stable multi-sample pose below.
            node.last_localization_event = time.monotonic()
            print(
                "[LOCALIZATION] A recent accepted localization was confirmed "
                "by the supervising process; skipping the duplicate /info wait."
            )
        else:
            print(
                f'[WAIT] Move slowly at "{args.name}" until a new accepted '
                "visual localization is observed, then hold still."
            )
            node.wait_for_localization(args.localization_timeout)
        print(
            f"[WAIT] Waiting for TF {args.map_frame} -> "
            f"{args.base_frame}..."
        )
        node.wait_for_tf(args.tf_timeout)
        print("[WAIT] Checking whether the current pose is inside the static map...")
        node.wait_for_pose_in_map(
            map_data,
            timeout_seconds=args.map_valid_timeout,
        )
        print(
            f"[SAMPLE] Collecting stable {args.map_frame} -> "
            f"{args.base_frame} poses for at least "
            f"{args.sample_seconds:.1f}s..."
        )
        samples, skipped_tf = node.collect_samples(
            sample_seconds=args.sample_seconds,
            sample_rate=args.sample_rate,
            min_samples=args.min_samples,
            collection_timeout=args.collection_timeout,
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if len(samples) < args.min_samples:
        detail = (
            f" Last TF error: {node.last_tf_error}"
            if node.last_tf_error
            else ""
        )
        raise RuntimeError(
            f"Only {len(samples)} TF samples were collected "
            f"(skipped={skipped_tf}); at least "
            f"{args.min_samples} are required.{detail}"
        )

    summary = summarize_samples(samples)
    if summary["position_rms"] > args.max_position_rms:
        raise RuntimeError(
            f"Position is not stable: RMS={summary['position_rms']:.3f}m, "
            f"limit={args.max_position_rms:.3f}m."
        )
    if math.degrees(summary["yaw_std"]) > args.max_yaw_std_deg:
        raise RuntimeError(
            f"Yaw is not stable: std={math.degrees(summary['yaw_std']):.2f}°, "
            f"limit={args.max_yaw_std_deg:.2f}°."
        )

    measured_x = float(summary["x"])
    measured_y = float(summary["y"])
    saved_x = measured_x
    saved_y = measured_y

    state, pixel_x, pixel_y = point_cell_state(
        measured_x,
        measured_y,
        map_data,
    )
    print(
        f"[MEASURED] x={measured_x:.3f}, y={measured_y:.3f}, "
        f"pixel=({pixel_x},{pixel_y}), state={state}, "
        f"map_pixels={map_data['width']}x{map_data['height']}"
    )
    snapped = False
    snap_distance = 0.0

    if state != "free":
        radius_cells = int(
            math.ceil(args.free_search_radius / map_data["resolution"])
        )
        nearest = nearest_free_cell(
            pixel_x,
            pixel_y,
            radius_cells,
            map_data,
        )
        nearest_text = "none"
        if nearest is not None:
            nearest_text = f"{nearest[2]:.3f}m"

        if not args.snap_to_free:
            if state == "outside":
                raise RuntimeError(
                    f'The measured point is "outside" the static map. '
                    f"x={measured_x:.3f}, y={measured_y:.3f}, "
                    f"pixel=({pixel_x},{pixel_y}), "
                    f"outside_distance={outside_distance_m(measured_x, measured_y, map_data):.3f}m, "
                    f"bounds={map_bounds_text(map_data)}. "
                    "Return to an area covered by the map, or rebuild/export a "
                    "larger map if this destination should be navigable."
                )
            raise RuntimeError(
                f'The measured point is "{state}", not free. '
                f"Nearest free cell within "
                f"{args.free_search_radius:.2f}m: {nearest_text}. "
                "Move toward the middle of the free corridor and mark again, "
                "or pass --snap-to-free for a controlled test."
            )
        if nearest is None:
            raise RuntimeError(
                "No free cell was found within the requested snap radius."
            )

        pixel_x, pixel_y, snap_distance = nearest
        saved_x, saved_y = pixel_to_world(
            pixel_x,
            pixel_y,
            map_data["resolution"],
            map_data["origin"],
            map_data["height"],
        )
        state = "free"
        snapped = True

    now = datetime.now().astimezone().isoformat(timespec="seconds")
    existing = database["places"].get(args.name)
    created_at = (
        existing.get("created_at")
        if isinstance(existing, dict)
        else now
    )

    database["places"][args.name] = {
        "name": args.name,
        "frame_id": args.map_frame,
        "x": saved_x,
        "y": saved_y,
        "z": 0.0,
        "measured_z": float(summary["z"]),
        "yaw": float(summary["yaw"]),
        "yaw_deg": math.degrees(float(summary["yaw"])),
        "measured_x": measured_x,
        "measured_y": measured_y,
        "cell_state": state,
        "pixel_x": pixel_x,
        "pixel_y": pixel_y,
        "snapped_to_free": snapped,
        "snap_distance_m": snap_distance,
        "sample_count": int(summary["sample_count"]),
        "position_rms_m": float(summary["position_rms"]),
        "position_max_error_m": float(summary["position_max"]),
        "yaw_std_deg": math.degrees(float(summary["yaw_std"])),
        "last_localization_ids": {
            "loop_closure_id": int(node.last_ids[0]),
            "proximity_detection_id": int(node.last_ids[1]),
            "landmark_id": int(node.last_ids[2]),
        },
        "created_at": created_at,
        "updated_at": now,
    }

    atomic_write_json(landmark_file, database)

    print(f'[OK] Saved place "{args.name}".')
    print(
        f"[POSE] x={saved_x:.3f}, y={saved_y:.3f}, "
        f"yaw={math.degrees(float(summary['yaw'])):.1f}°"
    )
    print(
        f"[STABILITY] samples={summary['sample_count']}, "
        f"skipped_tf={skipped_tf}, "
        f"position_rms={summary['position_rms']:.3f}m, "
        f"yaw_std={math.degrees(summary['yaw_std']):.2f}°"
    )
    if snapped:
        print(
            f"[SNAP] Moved the saved goal by {snap_distance:.3f}m "
            "to the nearest free map cell."
        )
    print(f"[FILE] {landmark_file}")
    return 0



def command_mark_pose(args: argparse.Namespace) -> int:
    """Save an explicitly sampled map-frame pose without creating a TF listener.

    This mode is used by the long-running unified voice node. Its persistent
    TF buffer has already observed map->odom and odom->base_link over time,
    avoiding the empty-cache extrapolation failure seen when every mark command
    starts a brand-new TransformListener.
    """
    yaml_path = Path(args.yaml).expanduser().resolve()
    map_db = Path(args.db).expanduser().resolve()
    landmark_file = Path(args.file).expanduser().resolve()

    if not yaml_path.is_file():
        raise RuntimeError(f"Map YAML does not exist: {yaml_path}")
    if not map_db.is_file():
        raise RuntimeError(f"Map database does not exist: {map_db}")

    map_data = load_map(yaml_path)
    database = load_database(
        landmark_file,
        args.map_name,
        map_db,
        map_data,
        args.force_map_mismatch,
    )

    if args.name in database["places"] and not args.overwrite:
        raise RuntimeError(
            f'Place "{args.name}" already exists. '
            "Pass --overwrite to replace it."
        )

    summary = {
        "x": float(args.x),
        "y": float(args.y),
        "z": float(args.z),
        "yaw": float(args.yaw_rad),
        "position_rms": float(args.position_rms),
        "position_max": float(args.position_max_error),
        "yaw_std": math.radians(float(args.yaw_std_deg)),
        "sample_count": int(args.sample_count),
    }

    if summary["sample_count"] < 1:
        raise RuntimeError("sample_count must be positive")
    if summary["position_rms"] > args.max_position_rms:
        raise RuntimeError(
            f"Position is not stable: RMS={summary['position_rms']:.3f}m, "
            f"limit={args.max_position_rms:.3f}m."
        )
    if float(args.yaw_std_deg) > args.max_yaw_std_deg:
        raise RuntimeError(
            f"Yaw is not stable: std={float(args.yaw_std_deg):.2f}°, "
            f"limit={args.max_yaw_std_deg:.2f}°."
        )

    measured_x = float(summary["x"])
    measured_y = float(summary["y"])
    saved_x = measured_x
    saved_y = measured_y

    state, pixel_x, pixel_y = point_cell_state(
        measured_x,
        measured_y,
        map_data,
    )
    print(
        f"[MEASURED] x={measured_x:.3f}, y={measured_y:.3f}, "
        f"pixel=({pixel_x},{pixel_y}), state={state}, "
        f"map_pixels={map_data['width']}x{map_data['height']}"
    )
    snapped = False
    snap_distance = 0.0

    if state != "free":
        radius_cells = int(
            math.ceil(args.free_search_radius / map_data["resolution"])
        )
        nearest = nearest_free_cell(
            pixel_x,
            pixel_y,
            radius_cells,
            map_data,
        )
        nearest_text = "none"
        if nearest is not None:
            nearest_text = f"{nearest[2]:.3f}m"

        if not args.snap_to_free:
            if state == "outside":
                raise RuntimeError(
                    f'The measured point is "outside" the static map. '
                    f"x={measured_x:.3f}, y={measured_y:.3f}, "
                    f"pixel=({pixel_x},{pixel_y}), "
                    f"outside_distance={outside_distance_m(measured_x, measured_y, map_data):.3f}m, "
                    f"bounds={map_bounds_text(map_data)}. "
                    "Return to an area covered by the map, or rebuild/export a "
                    "larger map if this destination should be navigable."
                )
            raise RuntimeError(
                f'The measured point is "{state}", not free. '
                f"Nearest free cell within "
                f"{args.free_search_radius:.2f}m: {nearest_text}. "
                "Move toward the middle of the free corridor and mark again, "
                "or pass --snap-to-free for a controlled test."
            )
        if nearest is None:
            raise RuntimeError(
                "No free cell was found within the requested snap radius."
            )

        pixel_x, pixel_y, snap_distance = nearest
        saved_x, saved_y = pixel_to_world(
            pixel_x,
            pixel_y,
            map_data["resolution"],
            map_data["origin"],
            map_data["height"],
        )
        state = "free"
        snapped = True

    now = datetime.now().astimezone().isoformat(timespec="seconds")
    existing = database["places"].get(args.name)
    created_at = (
        existing.get("created_at")
        if isinstance(existing, dict)
        else now
    )

    database["places"][args.name] = {
        "name": args.name,
        "frame_id": args.map_frame,
        "x": saved_x,
        "y": saved_y,
        "z": 0.0,
        "measured_z": float(summary["z"]),
        "yaw": float(summary["yaw"]),
        "yaw_deg": math.degrees(float(summary["yaw"])),
        "measured_x": measured_x,
        "measured_y": measured_y,
        "cell_state": state,
        "pixel_x": pixel_x,
        "pixel_y": pixel_y,
        "snapped_to_free": snapped,
        "snap_distance_m": snap_distance,
        "sample_count": int(summary["sample_count"]),
        "position_rms_m": float(summary["position_rms"]),
        "position_max_error_m": float(summary["position_max"]),
        "yaw_std_deg": float(args.yaw_std_deg),
        "pose_source": str(args.pose_source),
        "last_localization_ids": {
            "loop_closure_id": int(args.loop_id),
            "proximity_detection_id": int(args.proximity_id),
            "landmark_id": int(args.localization_landmark_id),
        },
        "created_at": created_at,
        "updated_at": now,
    }

    atomic_write_json(landmark_file, database)

    print(f'[OK] Saved place "{args.name}".')
    print(
        f"[POSE] x={saved_x:.3f}, y={saved_y:.3f}, "
        f"yaw={math.degrees(float(summary['yaw'])):.1f}°"
    )
    print(
        f"[STABILITY] samples={summary['sample_count']}, "
        f"position_rms={summary['position_rms']:.3f}m, "
        f"yaw_std={float(args.yaw_std_deg):.2f}°, "
        f"source={args.pose_source}"
    )
    if snapped:
        print(
            f"[SNAP] Moved the saved goal by {snap_distance:.3f}m "
            "to the nearest free map cell."
        )
    print(f"[FILE] {landmark_file}")
    return 0

def command_list(args: argparse.Namespace) -> int:
    path = Path(args.file).expanduser().resolve()
    if not path.exists():
        print(f"[INFO] No landmark file yet: {path}")
        return 0
    payload = json.loads(path.read_text(encoding="utf-8"))
    places = payload.get("places", {})
    if not places:
        print("[INFO] No places saved.")
        return 0

    print("name\tx(m)\ty(m)\tyaw(deg)\tupdated_at")
    for name in sorted(places):
        place = places[name]
        print(
            f"{name}\t{float(place['x']):.3f}\t"
            f"{float(place['y']):.3f}\t"
            f"{float(place.get('yaw_deg', 0.0)):.1f}\t"
            f"{place.get('updated_at', '')}"
        )
    print(f"[COUNT] {len(places)}")
    print(f"[FILE] {path}")
    return 0


def command_show(args: argparse.Namespace) -> int:
    path = Path(args.file).expanduser().resolve()
    payload = json.loads(path.read_text(encoding="utf-8"))
    place = payload.get("places", {}).get(args.name)
    if place is None:
        raise RuntimeError(f'Unknown place: "{args.name}"')
    print(json.dumps(place, ensure_ascii=False, indent=2))
    return 0


def command_delete(args: argparse.Namespace) -> int:
    path = Path(args.file).expanduser().resolve()
    payload = json.loads(path.read_text(encoding="utf-8"))
    places = payload.get("places", {})
    if args.name not in places:
        raise RuntimeError(f'Unknown place: "{args.name}"')
    del places[args.name]
    atomic_write_json(path, payload)
    print(f'[OK] Deleted place "{args.name}".')
    print(f"[FILE] {path}")
    return 0


def command_validate(args: argparse.Namespace) -> int:
    path = Path(args.file).expanduser().resolve()
    yaml_path = Path(args.yaml).expanduser().resolve()
    payload = json.loads(path.read_text(encoding="utf-8"))
    map_data = load_map(yaml_path)

    failures = 0
    for name, place in sorted(payload.get("places", {}).items()):
        state, pixel_x, pixel_y = point_cell_state(
            float(place["x"]),
            float(place["y"]),
            map_data,
        )
        print(
            f"{name}: state={state}, pixel=({pixel_x},{pixel_y}), "
            f"x={float(place['x']):.3f}, y={float(place['y']):.3f}"
        )
        if state != "free":
            failures += 1
    print(f"[RESULT] failures={failures}")
    return 1 if failures else 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--yaml", default=str(DEFAULT_YAML))
    parser.add_argument("--db", default=str(DEFAULT_DB))
    parser.add_argument("--file", default=str(DEFAULT_FILE))
    parser.add_argument("--map-name", default="map1")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--info-topic", default="/info")
    parser.add_argument("--force-map-mismatch", action="store_true")

    subparsers = parser.add_subparsers(dest="command", required=True)

    mark = subparsers.add_parser("mark")
    mark.add_argument("name")
    mark.add_argument("--overwrite", action="store_true")
    mark.add_argument("--localization-timeout", type=float, default=60.0)
    mark.add_argument(
        "--skip-new-localization-wait",
        action="store_true",
        help=(
            "Skip the duplicate /info event wait only when a long-running "
            "supervisor has already confirmed a recent accepted localization. "
            "Fresh/stable TF and free-cell checks are still enforced."
        ),
    )
    mark.add_argument(
        "--max-tf-age",
        type=float,
        default=0.0,
        help=(
            "Optional absolute TF-age limit in seconds. The RTAB-Map chain may "
            "publish transforms several seconds behind wall time, so the "
            "default disables this fragile check. Recent /info, map bounds and "
            "stable multi-sample pose checks remain active."
        ),
    )
    mark.add_argument(
        "--map-valid-timeout",
        type=float,
        default=5.0,
        help="Wait this long for map->base_link to enter the static map bounds.",
    )
    mark.add_argument("--sample-seconds", type=float, default=3.0)
    mark.add_argument("--sample-rate", type=float, default=10.0)
    mark.add_argument("--min-samples", type=int, default=15)
    mark.add_argument("--tf-timeout", type=float, default=20.0)
    mark.add_argument("--collection-timeout", type=float, default=12.0)
    mark.add_argument("--max-position-rms", type=float, default=0.08)
    mark.add_argument("--max-yaw-std-deg", type=float, default=8.0)
    mark.add_argument("--free-search-radius", type=float, default=0.40)
    mark.add_argument("--snap-to-free", action="store_true")
    mark.set_defaults(func=command_mark)

    mark_pose = subparsers.add_parser("mark-pose")
    mark_pose.add_argument("name")
    mark_pose.add_argument("--overwrite", action="store_true")
    mark_pose.add_argument("--x", type=float, required=True)
    mark_pose.add_argument("--y", type=float, required=True)
    mark_pose.add_argument("--z", type=float, default=0.0)
    mark_pose.add_argument("--yaw-rad", type=float, required=True)
    mark_pose.add_argument("--sample-count", type=int, required=True)
    mark_pose.add_argument("--position-rms", type=float, required=True)
    mark_pose.add_argument("--position-max-error", type=float, required=True)
    mark_pose.add_argument("--yaw-std-deg", type=float, required=True)
    mark_pose.add_argument("--pose-source", default="persistent_tf")
    mark_pose.add_argument("--loop-id", type=int, default=0)
    mark_pose.add_argument("--proximity-id", type=int, default=0)
    mark_pose.add_argument("--localization-landmark-id", type=int, default=0)
    mark_pose.add_argument("--max-position-rms", type=float, default=0.08)
    mark_pose.add_argument("--max-yaw-std-deg", type=float, default=8.0)
    mark_pose.add_argument("--free-search-radius", type=float, default=0.40)
    mark_pose.add_argument("--snap-to-free", action="store_true")
    mark_pose.set_defaults(func=command_mark_pose)

    list_parser = subparsers.add_parser("list")
    list_parser.set_defaults(func=command_list)

    show = subparsers.add_parser("show")
    show.add_argument("name")
    show.set_defaults(func=command_show)

    delete = subparsers.add_parser("delete")
    delete.add_argument("name")
    delete.set_defaults(func=command_delete)

    validate = subparsers.add_parser("validate")
    validate.set_defaults(func=command_validate)

    return parser


def main() -> int:
    try:
        sys.stdout.reconfigure(line_buffering=True)
        sys.stderr.reconfigure(line_buffering=True)
    except Exception:
        pass

    parser = build_parser()
    args = parser.parse_args()
    try:
        return int(args.func(args))
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
