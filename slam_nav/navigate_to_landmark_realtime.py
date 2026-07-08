#!/usr/bin/env python3
"""Real-time route guidance from the live RTAB-Map pose to a named place.

Navigation TF health logic revision: v7-delay-tolerant.

This node uses:
- /info (rtabmap_msgs/msg/Info) to confirm recent visual localization;
- /odom plus split TF map->odom and odom->base_link for the live pose;
- delay-tolerant TF health checks based on transform progress, not absolute sensor latency;
- a stable multi-sample navigation start pose instead of one instantaneous TF;
- a static Nav2 YAML/PGM map and landmark JSON;
- plan_landmarks_astar_v3.py for inflated-grid A* planning;
- CSV/JSON diagnostics for topic freshness, TF links and pose stability.

It provides command-line guidance:
- initial forward distance;
- turn prompt near each corner;
- forward prompt after completing the turn;
- arrival detection;
- off-route detection and automatic re-planning.

This is route guidance on a static map. It does NOT detect dynamic obstacles.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import Info
from tf2_ros import Buffer, TransformException, TransformListener

try:
    import plan_landmarks_astar_v3 as planner
except ImportError as exc:
    raise SystemExit(
        "[ERROR] Cannot import plan_landmarks_astar_v3.py. "
        "Install this script in the same directory as "
        "plan_landmarks_astar_v3.py."
    ) from exc


DEFAULT_BASE = Path(
    os.environ.get(
        "SLAM_NAV_BASE",
        "/home/sunrise/smart_cane_ros/slam_nav",
    )
)
DEFAULT_MAP_DIR = DEFAULT_BASE / "blind_cane_maps"
DEFAULT_YAML = DEFAULT_MAP_DIR / "map1.yaml"
DEFAULT_LANDMARKS = DEFAULT_MAP_DIR / "map1_landmarks.json"
DEFAULT_OUTPUT_DIR = DEFAULT_MAP_DIR / "plans" / "realtime"


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def path_cumulative_lengths(
    points: list[tuple[float, float]],
) -> list[float]:
    cumulative = [0.0]
    for index in range(1, len(points)):
        cumulative.append(
            cumulative[-1]
            + math.hypot(
                points[index][0] - points[index - 1][0],
                points[index][1] - points[index - 1][1],
            )
        )
    return cumulative


def project_to_polyline(
    point: tuple[float, float],
    path: list[tuple[float, float]],
    cumulative: list[float],
) -> dict:
    if not path:
        raise ValueError("Cannot project onto an empty path.")
    if len(path) == 1:
        distance = math.hypot(point[0] - path[0][0], point[1] - path[0][1])
        return {
            "distance": distance,
            "s": 0.0,
            "nearest": path[0],
            "segment_index": 0,
        }

    best = {
        "distance": float("inf"),
        "s": 0.0,
        "nearest": path[0],
        "segment_index": 0,
    }
    px, py = point

    for index in range(len(path) - 1):
        ax, ay = path[index]
        bx, by = path[index + 1]
        dx = bx - ax
        dy = by - ay
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-12:
            fraction = 0.0
        else:
            fraction = ((px - ax) * dx + (py - ay) * dy) / length_sq
            fraction = min(1.0, max(0.0, fraction))

        nearest_x = ax + fraction * dx
        nearest_y = ay + fraction * dy
        distance = math.hypot(px - nearest_x, py - nearest_y)
        if distance < best["distance"]:
            segment_length = math.sqrt(length_sq)
            best = {
                "distance": distance,
                "s": cumulative[index] + fraction * segment_length,
                "nearest": (nearest_x, nearest_y),
                "segment_index": index,
            }

    return best


def collapse_collinear_legs(
    points: list[tuple[float, float]],
    turn_threshold_deg: float,
) -> list[dict]:
    if len(points) < 2:
        return []

    threshold = math.radians(turn_threshold_deg)
    raw: list[dict] = []
    for index in range(len(points) - 1):
        start = points[index]
        end = points[index + 1]
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length <= 1e-6:
            continue
        raw.append(
            {
                "start": start,
                "end": end,
                "length": length,
                "heading": math.atan2(dy, dx),
            }
        )

    if not raw:
        return []

    merged = [raw[0].copy()]
    for leg in raw[1:]:
        delta = abs(wrap_angle(leg["heading"] - merged[-1]["heading"]))
        if delta < threshold:
            previous = merged[-1]
            previous["end"] = leg["end"]
            previous["length"] += leg["length"]
            dx = previous["end"][0] - previous["start"][0]
            dy = previous["end"][1] - previous["start"][1]
            previous["heading"] = math.atan2(dy, dx)
        else:
            merged.append(leg.copy())
    return merged


def build_turn_events(
    simple_world: list[tuple[float, float]],
    raw_world: list[tuple[float, float]],
    raw_cumulative: list[float],
    turn_threshold_deg: float,
    merge_short_forward_m: float,
    max_merged_turn_deg: float,
    angle_rounding_deg: float,
) -> list[dict]:
    legs = collapse_collinear_legs(simple_world, turn_threshold_deg)
    if len(legs) < 2:
        return []

    threshold = math.radians(turn_threshold_deg)
    max_merged = math.radians(max_merged_turn_deg)
    events: list[dict] = []
    index = 1

    while index < len(legs):
        previous = legs[index - 1]
        current = legs[index]
        first_delta = wrap_angle(current["heading"] - previous["heading"])
        if abs(first_delta) < threshold:
            index += 1
            continue

        event_point = previous["end"]
        event_projection = project_to_polyline(
            event_point,
            raw_world,
            raw_cumulative,
        )
        absorbed_short = 0.0
        target_heading = current["heading"]
        total_delta = first_delta

        if index + 1 < len(legs):
            next_leg = legs[index + 1]
            second_delta = wrap_angle(
                next_leg["heading"] - current["heading"]
            )
            same_direction = first_delta * second_delta > 0.0
            combined_delta = wrap_angle(
                next_leg["heading"] - previous["heading"]
            )
            if (
                current["length"] <= merge_short_forward_m
                and abs(second_delta) >= threshold
                and same_direction
                and abs(combined_delta) <= max_merged
            ):
                absorbed_short = current["length"]
                target_heading = next_leg["heading"]
                total_delta = combined_delta
                index += 1

        direction = "left" if total_delta > 0.0 else "right"
        display_angle = planner.round_half_up(
            abs(math.degrees(total_delta)),
            angle_rounding_deg,
        )
        events.append(
            {
                "s": float(event_projection["s"]),
                "point": event_point,
                "direction": direction,
                "angle_deg": abs(math.degrees(total_delta)),
                "display_angle_deg": display_angle,
                "target_heading": target_heading,
                "absorbed_short_forward_m": absorbed_short,
                "announced": False,
                "completed": False,
            }
        )
        index += 1

    total_length = raw_cumulative[-1]
    for event_index, event in enumerate(events):
        next_s = (
            events[event_index + 1]["s"]
            if event_index + 1 < len(events)
            else total_length
        )
        event["post_forward_distance_m"] = max(
            0.0,
            next_s - event["s"] - event["absorbed_short_forward_m"],
        )
    return events


class NavigationDiagnostics:
    def __init__(self, root_dir: Path, goal_name: str) -> None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_goal = planner.sanitize_name(goal_name)
        self.session_dir = (root_dir / f"{stamp}_{safe_goal}").resolve()
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.startup_checks = self.session_dir / "startup_checks.csv"
        self.startup_samples = self.session_dir / "startup_pose_samples.csv"
        self.runtime = self.session_dir / "runtime_pose.csv"
        self.events = self.session_dir / "localization_events.csv"
        self.summary = self.session_dir / "summary.json"
        self._write_header(
            self.startup_checks,
            [
                "wall_time", "monotonic", "info_age_s", "odom_msg_age_s",
                "localization_age_s", "map_odom_available",
                "map_odom_tf_age_s", "odom_base_available",
                "odom_base_tf_age_s", "odom_base_tf_progress_age_s",
                "odom_header_age_s", "direct_map_base_available",
                "direct_tf_age_s", "x", "y", "z", "yaw_deg",
                "direct_split_xy_error_m", "reasons",
            ],
        )
        self._write_header(
            self.startup_samples,
            [
                "wall_time", "monotonic", "x", "y", "z", "yaw_deg",
                "map_odom_tf_age_s", "odom_base_tf_age_s",
                "odom_base_tf_progress_age_s", "odom_header_age_s",
                "info_age_s", "odom_msg_age_s", "localization_age_s",
            ],
        )
        self._write_header(
            self.runtime,
            [
                "wall_time", "monotonic", "x", "y", "z", "yaw_deg",
                "info_age_s", "odom_msg_age_s", "localization_age_s",
                "map_odom_tf_age_s", "odom_base_tf_age_s",
                "odom_base_tf_progress_age_s", "odom_header_age_s",
                "off_route_m", "goal_distance_m", "event",
            ],
        )
        self._write_header(
            self.events,
            ["wall_time", "monotonic", "loop_id", "proximity_id", "landmark_id"],
        )

    @staticmethod
    def _write_header(path: Path, fields: list[str]) -> None:
        with path.open("w", newline="", encoding="utf-8") as stream:
            csv.writer(stream).writerow(fields)

    @staticmethod
    def _append(path: Path, row: list[object]) -> None:
        with path.open("a", newline="", encoding="utf-8") as stream:
            csv.writer(stream).writerow(row)

    @staticmethod
    def _fmt(value: object) -> object:
        if value is None:
            return ""
        if isinstance(value, float):
            if not math.isfinite(value):
                return "inf"
            return f"{value:.6f}"
        return value

    def log_localization_event(self, now: float, ids: tuple[int, int, int]) -> None:
        self._append(
            self.events,
            [datetime.now().isoformat(timespec="milliseconds"), now, *ids],
        )

    def log_startup_check(self, now: float, status: dict, reasons: list[str]) -> None:
        pose = status.get("pose") or {}
        self._append(
            self.startup_checks,
            [
                datetime.now().isoformat(timespec="milliseconds"), now,
                self._fmt(status.get("info_age")),
                self._fmt(status.get("odom_msg_age")),
                self._fmt(status.get("localization_age")),
                int(bool(status.get("map_odom_available"))),
                self._fmt(status.get("map_odom_tf_age")),
                int(bool(status.get("odom_base_available"))),
                self._fmt(status.get("odom_base_tf_age")),
                self._fmt(status.get("odom_base_tf_progress_age")),
                self._fmt(status.get("odom_header_age")),
                int(bool(status.get("direct_available"))),
                self._fmt(status.get("direct_tf_age")),
                self._fmt(pose.get("x")), self._fmt(pose.get("y")),
                self._fmt(pose.get("z")), self._fmt(pose.get("yaw_deg")),
                self._fmt(status.get("direct_split_xy_error")),
                " | ".join(reasons),
            ],
        )

    def log_startup_sample(self, now: float, sample: dict, status: dict) -> None:
        self._append(
            self.startup_samples,
            [
                datetime.now().isoformat(timespec="milliseconds"), now,
                self._fmt(sample.get("x")), self._fmt(sample.get("y")),
                self._fmt(sample.get("z")), self._fmt(sample.get("yaw_deg")),
                self._fmt(status.get("map_odom_tf_age")),
                self._fmt(status.get("odom_base_tf_age")),
                self._fmt(status.get("odom_base_tf_progress_age")),
                self._fmt(status.get("odom_header_age")),
                self._fmt(status.get("info_age")),
                self._fmt(status.get("odom_msg_age")),
                self._fmt(status.get("localization_age")),
            ],
        )

    def log_runtime(
        self,
        now: float,
        pose: Optional[dict],
        status: dict,
        off_route: Optional[float],
        goal_distance: Optional[float],
        event: str = "",
    ) -> None:
        pose = pose or {}
        self._append(
            self.runtime,
            [
                datetime.now().isoformat(timespec="milliseconds"), now,
                self._fmt(pose.get("x")), self._fmt(pose.get("y")),
                self._fmt(pose.get("z")), self._fmt(pose.get("yaw_deg")),
                self._fmt(status.get("info_age")),
                self._fmt(status.get("odom_msg_age")),
                self._fmt(status.get("localization_age")),
                self._fmt(status.get("map_odom_tf_age")),
                self._fmt(status.get("odom_base_tf_age")),
                self._fmt(status.get("odom_base_tf_progress_age")),
                self._fmt(status.get("odom_header_age")),
                self._fmt(off_route), self._fmt(goal_distance), event,
            ],
        )

    def write_summary(self, payload: dict) -> None:
        self.summary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    yaw = yaw_from_quaternion(x, y, z, w)
    return roll, pitch, yaw


def circular_mean(values: list[float]) -> float:
    return math.atan2(
        sum(math.sin(value) for value in values),
        sum(math.cos(value) for value in values),
    )


def circular_std(values: list[float]) -> float:
    if not values:
        return float("inf")
    sine = sum(math.sin(value) for value in values) / len(values)
    cosine = sum(math.cos(value) for value in values) / len(values)
    resultant = min(1.0, max(1e-12, math.hypot(sine, cosine)))
    return math.sqrt(max(0.0, -2.0 * math.log(resultant)))


class RealtimeNavigator(Node):
    def __init__(
        self,
        info_topic: str,
        odom_topic: str,
        map_frame: str,
        odom_frame: str,
        base_frame: str,
        diagnostics: NavigationDiagnostics,
    ) -> None:
        super().__init__("realtime_landmark_navigator")
        self.info_topic = info_topic
        self.odom_topic = odom_topic
        self.map_frame = map_frame
        self.odom_frame = odom_frame
        self.base_frame = base_frame
        self.diagnostics = diagnostics
        self.last_info_message_at: Optional[float] = None
        self.last_odom_message_at: Optional[float] = None
        self.last_odom_header_stamp: Optional[float] = None
        self.last_odom_tf_stamp: Optional[float] = None
        self.last_odom_tf_progress_at: Optional[float] = None
        self.last_localization_event: Optional[float] = None
        self.last_ids = (0, 0, 0)

        info_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Info, info_topic, self._on_info, info_qos)
        self.create_subscription(Odometry, odom_topic, self._on_odom, odom_qos)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=120.0))
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )

    def _on_info(self, msg: Info) -> None:
        now = time.monotonic()
        self.last_info_message_at = now
        ids = (
            int(msg.loop_closure_id),
            int(msg.proximity_detection_id),
            int(msg.landmark_id),
        )
        if any(value > 0 for value in ids):
            self.last_localization_event = now
            self.last_ids = ids
            self.diagnostics.log_localization_event(now, ids)

    def _on_odom(self, msg: Odometry) -> None:
        self.last_odom_message_at = time.monotonic()
        stamp = msg.header.stamp
        self.last_odom_header_stamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9

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
            f"{self.info_topic} within {timeout_seconds:.0f}s."
        )

    @staticmethod
    def _transform_stamp_sec(transform) -> Optional[float]:
        stamp_sec = (
            float(transform.header.stamp.sec)
            + float(transform.header.stamp.nanosec) * 1e-9
        )
        return stamp_sec if stamp_sec > 0.0 else None

    def _transform_age(self, transform) -> Optional[float]:
        stamp_sec = self._transform_stamp_sec(transform)
        if stamp_sec is None:
            return None
        ros_now_sec = float(self.get_clock().now().nanoseconds) * 1e-9
        return max(0.0, ros_now_sec - stamp_sec)

    def _ros_stamp_age(self, stamp_sec: Optional[float]) -> Optional[float]:
        if stamp_sec is None or stamp_sec <= 0.0:
            return None
        ros_now_sec = float(self.get_clock().now().nanoseconds) * 1e-9
        return max(0.0, ros_now_sec - stamp_sec)

    def split_pose(self, timeout_seconds: float = 0.15) -> dict:
        map_to_odom = self.tf_buffer.lookup_transform(
            self.map_frame,
            self.odom_frame,
            Time(),
            timeout=Duration(seconds=timeout_seconds),
        )
        odom_to_base = self.tf_buffer.lookup_transform(
            self.odom_frame,
            self.base_frame,
            Time(),
            timeout=Duration(seconds=timeout_seconds),
        )
        mt = map_to_odom.transform.translation
        mq = map_to_odom.transform.rotation
        ot = odom_to_base.transform.translation
        oq = odom_to_base.transform.rotation
        _, _, map_odom_yaw = quaternion_to_rpy(
            float(mq.x), float(mq.y), float(mq.z), float(mq.w)
        )
        odom_roll, odom_pitch, odom_base_yaw = quaternion_to_rpy(
            float(oq.x), float(oq.y), float(oq.z), float(oq.w)
        )
        cosine = math.cos(map_odom_yaw)
        sine = math.sin(map_odom_yaw)
        x = float(mt.x) + cosine * float(ot.x) - sine * float(ot.y)
        y = float(mt.y) + sine * float(ot.x) + cosine * float(ot.y)
        z = float(mt.z) + float(ot.z)
        yaw = wrap_angle(map_odom_yaw + odom_base_yaw)
        return {
            "x": x,
            "y": y,
            "z": z,
            "yaw": yaw,
            "yaw_deg": math.degrees(yaw),
            "odom_roll_deg": math.degrees(odom_roll),
            "odom_pitch_deg": math.degrees(odom_pitch),
            "map_odom_tf_age": self._transform_age(map_to_odom),
            "odom_base_tf_age": self._transform_age(odom_to_base),
            "map_odom_tf_stamp": self._transform_stamp_sec(map_to_odom),
            "odom_base_tf_stamp": self._transform_stamp_sec(odom_to_base),
            "source": "split_map_odom_plus_odom_base",
        }

    def direct_pose(self, timeout_seconds: float = 0.05) -> dict:
        transform = self.tf_buffer.lookup_transform(
            self.map_frame,
            self.base_frame,
            Time(),
            timeout=Duration(seconds=timeout_seconds),
        )
        t = transform.transform.translation
        q = transform.transform.rotation
        roll, pitch, yaw = quaternion_to_rpy(
            float(q.x), float(q.y), float(q.z), float(q.w)
        )
        return {
            "x": float(t.x), "y": float(t.y), "z": float(t.z),
            "yaw": yaw, "yaw_deg": math.degrees(yaw),
            "roll_deg": math.degrees(roll),
            "pitch_deg": math.degrees(pitch),
            "tf_age": self._transform_age(transform),
        }

    def pose_status(self, timeout_seconds: float = 0.12) -> dict:
        now = time.monotonic()
        status = {
            "info_age": float("inf") if self.last_info_message_at is None else now - self.last_info_message_at,
            "odom_msg_age": float("inf") if self.last_odom_message_at is None else now - self.last_odom_message_at,
            "localization_age": float("inf") if self.last_localization_event is None else now - self.last_localization_event,
            "map_odom_available": False,
            "odom_base_available": False,
            "direct_available": False,
            "map_odom_tf_age": None,
            "odom_base_tf_age": None,
            "odom_base_tf_progress_age": float("inf"),
            "odom_header_age": self._ros_stamp_age(self.last_odom_header_stamp),
            "direct_tf_age": None,
            "direct_split_xy_error": None,
            "pose": None,
            "errors": [],
        }
        try:
            pose = self.split_pose(timeout_seconds=timeout_seconds)
            status["pose"] = pose
            status["map_odom_available"] = True
            status["odom_base_available"] = True
            status["map_odom_tf_age"] = pose.get("map_odom_tf_age")
            status["odom_base_tf_age"] = pose.get("odom_base_tf_age")
            odom_tf_stamp = pose.get("odom_base_tf_stamp")
            if odom_tf_stamp is not None:
                if (
                    self.last_odom_tf_stamp is None
                    or abs(odom_tf_stamp - self.last_odom_tf_stamp) > 1e-6
                ):
                    self.last_odom_tf_stamp = odom_tf_stamp
                    self.last_odom_tf_progress_at = now
            status["odom_base_tf_progress_age"] = (
                float("inf")
                if self.last_odom_tf_progress_at is None
                else now - self.last_odom_tf_progress_at
            )
        except TransformException as exc:
            text = str(exc)
            status["errors"].append(text)
            # Probe each link separately so diagnostics identify the missing side.
            try:
                t = self.tf_buffer.lookup_transform(
                    self.map_frame, self.odom_frame, Time(),
                    timeout=Duration(seconds=0.03),
                )
                status["map_odom_available"] = True
                status["map_odom_tf_age"] = self._transform_age(t)
            except TransformException:
                pass
            try:
                t = self.tf_buffer.lookup_transform(
                    self.odom_frame, self.base_frame, Time(),
                    timeout=Duration(seconds=0.03),
                )
                status["odom_base_available"] = True
                status["odom_base_tf_age"] = self._transform_age(t)
                odom_tf_stamp = self._transform_stamp_sec(t)
                if odom_tf_stamp is not None:
                    if (
                        self.last_odom_tf_stamp is None
                        or abs(odom_tf_stamp - self.last_odom_tf_stamp) > 1e-6
                    ):
                        self.last_odom_tf_stamp = odom_tf_stamp
                        self.last_odom_tf_progress_at = now
                status["odom_base_tf_progress_age"] = (
                    float("inf")
                    if self.last_odom_tf_progress_at is None
                    else now - self.last_odom_tf_progress_at
                )
            except TransformException:
                pass

        try:
            direct = self.direct_pose(timeout_seconds=0.03)
            status["direct_available"] = True
            status["direct_tf_age"] = direct.get("tf_age")
            status["direct_pose"] = direct
            split = status.get("pose")
            if split is not None:
                status["direct_split_xy_error"] = math.hypot(
                    direct["x"] - split["x"], direct["y"] - split["y"]
                )
        except TransformException:
            pass
        return status

    @staticmethod
    def readiness_reasons(status: dict, args: argparse.Namespace, *, startup: bool) -> list[str]:
        reasons: list[str] = []
        if status["info_age"] > args.max_info_age:
            reasons.append(f"/info过期({status['info_age']:.1f}s>{args.max_info_age:.1f}s)")
        if status["odom_msg_age"] > args.max_odom_age:
            reasons.append(f"{args.odom_topic}过期({status['odom_msg_age']:.1f}s>{args.max_odom_age:.1f}s)")
        if startup and status["localization_age"] > args.max_localization_age:
            reasons.append(
                f"视觉定位确认过期({status['localization_age']:.1f}s>"
                f"{args.max_localization_age:.1f}s)"
            )
        if not status["map_odom_available"]:
            reasons.append(f"TF {args.map_frame}->{args.odom_frame}不可用")
        if not status["odom_base_available"]:
            reasons.append(f"TF {args.odom_frame}->{args.base_frame}不可用")
        # The TF timestamp can legitimately lag wall time by several seconds
        # because RGB-D odometry is computed from delayed image frames.  A fresh
        # /odom callback plus an advancing TF stamp is a better health signal
        # than the absolute TF age.  The old max_odom_tf_age value is retained
        # only as a diagnostic threshold and is not a hard stop.
        odom_tf_progress_age = status.get("odom_base_tf_progress_age")
        if (
            odom_tf_progress_age is not None
            and math.isfinite(odom_tf_progress_age)
            and odom_tf_progress_age > args.max_odom_tf_stall
        ):
            reasons.append(
                f"TF {args.odom_frame}->{args.base_frame}时间戳停止推进("
                f"{odom_tf_progress_age:.1f}s>{args.max_odom_tf_stall:.1f}s)"
            )
        # map->odom is allowed to be sparse. Its age is logged but is not used
        # as a hard stop because a stable global correction can remain valid.
        return reasons

    def collect_stable_pose(self, args: argparse.Namespace) -> tuple[dict, dict]:
        deadline = time.monotonic() + args.pose_ready_timeout
        next_report = 0.0
        attempt = 0
        last_failure = "没有获得可用坐标"
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            status = self.pose_status()
            reasons = self.readiness_reasons(status, args, startup=True)
            if now >= next_report:
                self.diagnostics.log_startup_check(now, status, reasons)
                pose = status.get("pose") or {}
                print(
                    "[POSE CHECK] "
                    f"info_age={status['info_age']:.2f}s, "
                    f"odom_msg_age={status['odom_msg_age']:.2f}s, "
                    f"localization_age={status['localization_age']:.2f}s, "
                    f"map_odom={'yes' if status['map_odom_available'] else 'no'}, "
                    f"map_odom_tf_age={status.get('map_odom_tf_age')}, "
                    f"odom_base={'yes' if status['odom_base_available'] else 'no'}, "
                    f"odom_base_tf_age={status.get('odom_base_tf_age')}, "
                    f"odom_base_tf_progress_age={status.get('odom_base_tf_progress_age')}, "
                    f"odom_header_age={status.get('odom_header_age')}, "
                    f"x={pose.get('x')}, y={pose.get('y')}, z={pose.get('z')}, "
                    f"reason={'none' if not reasons else '; '.join(reasons)}"
                )
                next_report = now + args.diagnostic_interval
            if reasons:
                last_failure = "; ".join(reasons)
                continue

            attempt += 1
            print(
                f"[POSE SAMPLE] attempt={attempt}, keep the cane still for "
                f"{args.pose_sample_seconds:.1f}s."
            )
            samples: list[dict] = []
            sample_end = time.monotonic() + args.pose_sample_seconds
            next_sample = time.monotonic()
            period = 1.0 / args.pose_sample_rate
            while rclpy.ok() and time.monotonic() < sample_end:
                rclpy.spin_once(self, timeout_sec=0.01)
                now = time.monotonic()
                if now < next_sample:
                    continue
                next_sample += period
                sample_status = self.pose_status()
                sample_reasons = self.readiness_reasons(sample_status, args, startup=True)
                sample = sample_status.get("pose")
                if sample is None or sample_reasons:
                    last_failure = "; ".join(sample_reasons) or "采样期间TF不可用"
                    continue
                samples.append(sample)
                self.diagnostics.log_startup_sample(now, sample, sample_status)

            if len(samples) < args.pose_min_samples:
                last_failure = (
                    f"稳定采样不足: {len(samples)} < {args.pose_min_samples}"
                )
                print(f"[POSE UNSTABLE] {last_failure}")
                continue

            xs = [sample["x"] for sample in samples]
            ys = [sample["y"] for sample in samples]
            zs = [sample["z"] for sample in samples]
            yaws = [sample["yaw"] for sample in samples]
            x = statistics.median(xs)
            y = statistics.median(ys)
            z = statistics.median(zs)
            yaw = circular_mean(yaws)
            errors = [math.hypot(px - x, py - y) for px, py in zip(xs, ys)]
            rms = math.sqrt(sum(error * error for error in errors) / len(errors))
            max_error = max(errors)
            yaw_std_deg = math.degrees(circular_std(yaws))
            result = {
                "x": x, "y": y, "z": z, "yaw": yaw,
                "yaw_deg": math.degrees(yaw),
                "sample_count": len(samples),
                "position_rms": rms,
                "position_max_error": max_error,
                "yaw_std_deg": yaw_std_deg,
                "source": "stable_split_tf_samples",
            }
            print(
                "[POSE STABILITY] "
                f"samples={len(samples)}, x={x:.3f}, y={y:.3f}, z={z:.3f}, "
                f"yaw={math.degrees(yaw):.1f}deg, rms={rms:.3f}m, "
                f"max={max_error:.3f}m, yaw_std={yaw_std_deg:.2f}deg"
            )
            median_roll = statistics.median(
                [sample.get("odom_roll_deg", 0.0) for sample in samples]
            )
            median_pitch = statistics.median(
                [sample.get("odom_pitch_deg", 0.0) for sample in samples]
            )
            if abs(z) > args.pose_warn_abs_z:
                print(
                    f"[POSE WARNING] abs(z)={abs(z):.3f}m exceeds "
                    f"{args.pose_warn_abs_z:.3f}m; inspect vertical drift and frames."
                )
            if max(abs(median_roll), abs(median_pitch)) > args.pose_warn_tilt_deg:
                print(
                    f"[POSE WARNING] odom roll/pitch=({median_roll:.1f},"
                    f"{median_pitch:.1f})deg exceeds {args.pose_warn_tilt_deg:.1f}deg."
                )
            if rms > args.pose_max_position_rms:
                last_failure = (
                    f"坐标RMS不稳定: {rms:.3f}m > "
                    f"{args.pose_max_position_rms:.3f}m"
                )
                print(f"[POSE UNSTABLE] {last_failure}")
                continue
            if max_error > args.pose_max_position_error:
                last_failure = (
                    f"坐标最大误差过大: {max_error:.3f}m > "
                    f"{args.pose_max_position_error:.3f}m"
                )
                print(f"[POSE UNSTABLE] {last_failure}")
                continue
            if yaw_std_deg > args.pose_max_yaw_std_deg:
                last_failure = (
                    f"朝向不稳定: {yaw_std_deg:.2f}deg > "
                    f"{args.pose_max_yaw_std_deg:.2f}deg"
                )
                print(f"[POSE UNSTABLE] {last_failure}")
                continue
            return result, {
                "attempt": attempt,
                "samples": len(samples),
                "position_rms": rms,
                "position_max_error": max_error,
                "yaw_std_deg": yaw_std_deg,
            }

        raise RuntimeError(
            f"No stable navigation start pose within {args.pose_ready_timeout:.0f}s: "
            f"{last_failure}"
        )


def build_plan(
    *,
    current_pose: dict,
    goal_place: dict,
    map_data: dict,
    args: argparse.Namespace,
) -> dict:
    blocked = planner.build_blocked_grid(map_data, args.allow_unknown)
    raw_start = planner.world_to_pixel(
        current_pose["x"],
        current_pose["y"],
        map_data["resolution"],
        map_data["origin"],
        map_data["height"],
    )
    raw_goal = planner.world_to_pixel(
        float(goal_place["x"]),
        float(goal_place["y"]),
        map_data["resolution"],
        map_data["origin"],
        map_data["height"],
    )

    plan = planner.plan_with_inflation(
        base_blocked=blocked,
        raw_start=raw_start,
        raw_goal=raw_goal,
        resolution=map_data["resolution"],
        requested_radius=args.inflation_radius,
        minimum_radius=args.min_inflation_radius,
        radius_step=args.inflation_step,
        snap_radius=args.snap_radius,
        auto_relax=args.auto_relax,
    )

    raw_path = plan["raw_path"]
    simple_path = planner.simplify_path(raw_path, plan["inflated"])
    raw_world = [
        planner.pixel_to_world(
            pixel_x,
            pixel_y,
            map_data["resolution"],
            map_data["origin"],
            map_data["height"],
        )
        for pixel_x, pixel_y in raw_path
    ]
    simple_world = [
        planner.pixel_to_world(
            pixel_x,
            pixel_y,
            map_data["resolution"],
            map_data["origin"],
            map_data["height"],
        )
        for pixel_x, pixel_y in simple_path
    ]
    cumulative = path_cumulative_lengths(raw_world)
    events = build_turn_events(
        simple_world=simple_world,
        raw_world=raw_world,
        raw_cumulative=cumulative,
        turn_threshold_deg=args.turn_threshold_deg,
        merge_short_forward_m=args.merge_short_forward_m,
        max_merged_turn_deg=args.max_merged_turn_deg,
        angle_rounding_deg=args.instruction_angle_rounding_deg,
    )

    plan.update(
        {
            "raw_world": raw_world,
            "simple_world": simple_world,
            "cumulative": cumulative,
            "turn_events": events,
            "total_length_m": cumulative[-1] if cumulative else 0.0,
        }
    )
    return plan


def save_plan_snapshot(
    *,
    plan: dict,
    current_pose: dict,
    goal_name: str,
    goal_place: dict,
    map_data: dict,
    output_dir: Path,
    plan_index: int,
    reason: str,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = f"plan_{plan_index:02d}"
    csv_path = output_dir / f"{stem}.csv"
    json_path = output_dir / f"{stem}.json"
    png_path = output_dir / f"{stem}.png"

    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["index", "x_m", "y_m"])
        for index, (x, y) in enumerate(plan["raw_world"]):
            writer.writerow([index, f"{x:.6f}", f"{y:.6f}"])

    payload = {
        "created_at": datetime.now().astimezone().isoformat(
            timespec="seconds"
        ),
        "reason": reason,
        "goal_name": goal_name,
        "start_pose": current_pose,
        "goal_place": goal_place,
        "path_length_m": plan["total_length_m"],
        "selected_inflation_radius_m": plan["selected_radius_m"],
        "turn_events": [
            {
                key: value
                for key, value in event.items()
                if key not in {"announced", "completed"}
            }
            for event in plan["turn_events"]
        ],
        "raw_path": [
            {"x": x, "y": y}
            for x, y in plan["raw_world"]
        ],
    }
    json_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    planner.configure_matplotlib_font()
    figure, axes = plt.subplots(figsize=(10, 10))
    axes.imshow(
        map_data["image"],
        origin="upper",
        interpolation="nearest",
    )
    axes.plot(
        [point[0] for point in plan["raw_path"]],
        [point[1] for point in plan["raw_path"]],
        linewidth=2.0,
        label="Planned path",
    )
    axes.plot(
        plan["start"][0],
        plan["start"][1],
        marker="o",
        markersize=9,
        linestyle="None",
        label="当前位置",
    )
    axes.plot(
        plan["goal"][0],
        plan["goal"][1],
        marker="s",
        markersize=9,
        linestyle="None",
        label=goal_name,
    )
    axes.set_title(
        f"实时导航：当前位置 → {goal_name} "
        f"({plan['total_length_m']:.2f} m)"
    )
    axes.set_axis_off()
    axes.legend(loc="best")
    figure.tight_layout()
    figure.savefig(png_path, dpi=160, bbox_inches="tight")
    plt.close(figure)

    latest_json = output_dir / "latest_plan.json"
    latest_json.write_text(
        json_path.read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    print(
        f"[PLAN FILES] {csv_path.name}, "
        f"{json_path.name}, {png_path.name}"
    )


def prompt_initial(plan: dict, goal_name: str) -> None:
    events = plan["turn_events"]
    if events:
        distance = max(0.0, events[0]["s"])
        if distance >= 0.20:
            print(f"[PROMPT] 请沿路线前进约{distance:.1f}米。")
        else:
            event = events[0]
            direction = "左转" if event["direction"] == "left" else "右转"
            print(
                f"[PROMPT] 请{direction}约"
                f"{event['display_angle_deg']:.0f}度。"
            )
    else:
        print(
            f"[PROMPT] 请沿路线前进约"
            f"{plan['total_length_m']:.1f}米，到达{goal_name}。"
        )


def run_navigation(
    *,
    node: RealtimeNavigator,
    initial_pose: dict,
    diagnostics: NavigationDiagnostics,
    goal_name: str,
    goal_place: dict,
    map_data: dict,
    args: argparse.Namespace,
) -> int:
    current_pose = dict(initial_pose)
    plan_index = 1
    plan = build_plan(
        current_pose=current_pose,
        goal_place=goal_place,
        map_data=map_data,
        args=args,
    )
    output_dir = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir
        else (DEFAULT_OUTPUT_DIR / planner.sanitize_name(goal_name)).resolve()
    )
    save_plan_snapshot(
        plan=plan,
        current_pose=current_pose,
        goal_name=goal_name,
        goal_place=goal_place,
        map_data=map_data,
        output_dir=output_dir,
        plan_index=plan_index,
        reason="initial",
    )

    print(
        f"[PLAN] goal={goal_name}, "
        f"length={plan['total_length_m']:.2f}m, "
        f"turns={len(plan['turn_events'])}, "
        f"inflation={plan['selected_radius_m']:.2f}m"
    )
    prompt_initial(plan, goal_name)

    next_turn_index = 0
    offroute_since: Optional[float] = None
    last_replan_time = -float("inf")
    last_status_time = 0.0
    localization_pause_announced = False
    localization_fault_since: Optional[float] = None
    localization_fault_key = ""
    localization_recovery_since: Optional[float] = None
    localization_stale_warning_announced = False
    replan_count = 0

    period = 1.0 / args.rate
    next_cycle = time.monotonic()

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
        now = time.monotonic()
        if now < next_cycle:
            continue
        next_cycle = now + period

        status = node.pose_status()
        pause_reasons = node.readiness_reasons(status, args, startup=False)
        pose = status.get("pose")

        if pause_reasons:
            reason_key = "|".join(reason.split("(", 1)[0] for reason in pause_reasons)
            diagnostics.log_runtime(
                now, pose, status, None, None, "degraded:" + reason_key
            )
            if localization_fault_since is None or reason_key != localization_fault_key:
                localization_fault_since = now
                localization_fault_key = reason_key
                localization_recovery_since = None
                print(f"[NAV DEGRADED] {reason_key}; waiting for confirmation.")
            fault_duration = now - localization_fault_since
            if (
                not localization_pause_announced
                and fault_duration >= args.pause_confirm_seconds
            ):
                localization_pause_announced = True
                reason = "、".join(pause_reasons)
                print(
                    f"[NAV PAUSED] sustained_for={fault_duration:.1f}s, "
                    f"reason={reason}"
                )
                if args.voice_localization_state:
                    print(
                        f"[PROMPT] 导航定位暂时不可用，{reason}，"
                        "请停止前进。"
                    )
            continue

        localization_fault_since = None
        localization_fault_key = ""
        if localization_pause_announced:
            if localization_recovery_since is None:
                localization_recovery_since = now
                print("[NAV RECOVERY] conditions are healthy; confirming stability.")
                continue
            if now - localization_recovery_since < args.recovery_confirm_seconds:
                diagnostics.log_runtime(
                    now, pose, status, None, None, "recovering"
                )
                continue
            print(
                f"[NAV RECOVERED] healthy_for="
                f"{now - localization_recovery_since:.1f}s; navigation resumed."
            )
            if args.voice_localization_state:
                print("[PROMPT] 导航定位已恢复。")
            localization_pause_announced = False
            localization_recovery_since = None
        else:
            localization_recovery_since = None

        assert pose is not None
        localization_age = status["localization_age"]
        if localization_age > args.max_localization_age:
            if not localization_stale_warning_announced:
                print(
                    "[NAV WARNING] no recent non-zero visual localization event; "
                    "odometry and TF are still healthy, so navigation continues."
                )
                if args.voice_localization_state:
                    print(
                        "[PROMPT] 较长时间没有新的视觉定位确认，"
                        "当前导航继续，请在安全位置缓慢转动盲杖。"
                    )
                localization_stale_warning_announced = True
        elif localization_stale_warning_announced:
            print("[NAV INFO] visual localization confirmation was refreshed.")
            if args.voice_localization_state:
                print("[PROMPT] 视觉定位确认已更新。")
            localization_stale_warning_announced = False

        point = (pose["x"], pose["y"])
        projection = project_to_polyline(
            point,
            plan["raw_world"],
            plan["cumulative"],
        )
        path_s = float(projection["s"])
        offroute_distance = float(projection["distance"])
        goal_distance = math.hypot(
            pose["x"] - float(goal_place["x"]),
            pose["y"] - float(goal_place["y"]),
        )
        diagnostics.log_runtime(
            now, pose, status, offroute_distance, goal_distance, "tracking"
        )

        if (
            goal_distance <= args.arrival_radius
            and path_s >= plan["total_length_m"] - args.arrival_path_margin
        ):
            print(
                f"[PROMPT] 已到达{goal_name}。"
            )
            print(
                f"[ARRIVED] goal_distance={goal_distance:.2f}m, "
                f"replans={replan_count}"
            )
            return 0

        if offroute_distance > args.offroute_distance:
            if offroute_since is None:
                offroute_since = now
            offroute_duration = now - offroute_since
            if (
                offroute_duration >= args.offroute_duration
                and now - last_replan_time >= args.replan_cooldown
            ):
                if replan_count >= args.max_replans:
                    print(
                        "[ERROR] Maximum automatic re-plan count reached."
                    )
                    return 3

                print(
                    f"[PROMPT] 检测到偏离路线约"
                    f"{offroute_distance:.2f}米，正在重新规划。"
                )
                try:
                    new_plan = build_plan(
                        current_pose=pose,
                        goal_place=goal_place,
                        map_data=map_data,
                        args=args,
                    )
                except Exception as exc:
                    print(f"[REPLAN ERROR] {exc}")
                    offroute_since = now
                    last_replan_time = now
                    continue

                plan = new_plan
                plan_index += 1
                replan_count += 1
                next_turn_index = 0
                offroute_since = None
                last_replan_time = now
                save_plan_snapshot(
                    plan=plan,
                    current_pose=pose,
                    goal_name=goal_name,
                    goal_place=goal_place,
                    map_data=map_data,
                    output_dir=output_dir,
                    plan_index=plan_index,
                    reason="off_route",
                )
                print(
                    f"[REPLAN] count={replan_count}, "
                    f"new_length={plan['total_length_m']:.2f}m"
                )
                prompt_initial(plan, goal_name)
                continue
        else:
            offroute_since = None

        events = plan["turn_events"]
        if next_turn_index < len(events):
            event = events[next_turn_index]
            remaining_to_turn = event["s"] - path_s
            direction_text = (
                "左转" if event["direction"] == "left" else "右转"
            )

            if (
                not event["announced"]
                and remaining_to_turn <= args.turn_approach_distance
                and remaining_to_turn >= -args.turn_pass_margin
            ):
                print(
                    f"[PROMPT] 前方转角，请{direction_text}约"
                    f"{event['display_angle_deg']:.0f}度。"
                )
                event["announced"] = True

            if event["announced"] and not event["completed"]:
                heading_error = abs(
                    wrap_angle(
                        pose["yaw"] - event["target_heading"]
                    )
                )
                if (
                    heading_error
                    <= math.radians(args.turn_complete_tolerance_deg)
                    and path_s >= event["s"] - args.turn_pass_margin
                ):
                    event["completed"] = True
                    next_turn_index += 1
                    forward_distance = event[
                        "post_forward_distance_m"
                    ]
                    if forward_distance >= 0.20:
                        print(
                            f"[PROMPT] 转向完成，请继续前进约"
                            f"{forward_distance:.1f}米。"
                        )
                    else:
                        print("[PROMPT] 转向完成，请继续沿路线前进。")

        if now - last_status_time >= args.status_interval:
            next_action = "到达目标"
            remaining = max(
                0.0,
                plan["total_length_m"] - path_s,
            )
            if next_turn_index < len(plan["turn_events"]):
                next_event = plan["turn_events"][next_turn_index]
                next_action = (
                    f"{'左转' if next_event['direction'] == 'left' else '右转'}"
                    f"{next_event['display_angle_deg']:.0f}度"
                )
                remaining = max(0.0, next_event["s"] - path_s)

            print(
                f"[STATUS] x={pose['x']:.2f}, y={pose['y']:.2f}, z={pose['z']:.2f}, "
                f"off_route={offroute_distance:.2f}m, "
                f"next={next_action}, distance={remaining:.1f}m, "
                f"goal={goal_distance:.1f}m, "
                f"info_age={status['info_age']:.2f}s, "
                f"odom_age={status['odom_msg_age']:.2f}s, "
                f"loc_age={status['localization_age']:.2f}s, "
                f"map_odom_tf_age={status.get('map_odom_tf_age')}, "
                f"odom_base_tf_age={status.get('odom_base_tf_age')}, "
                f"odom_base_tf_progress_age={status.get('odom_base_tf_progress_age')}, "
                f"odom_header_age={status.get('odom_header_age')}"
            )
            last_status_time = now


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("goal_name")
    parser.add_argument("--yaml", default=str(DEFAULT_YAML))
    parser.add_argument("--landmarks", default=str(DEFAULT_LANDMARKS))
    parser.add_argument("--output-dir")
    parser.add_argument("--info-topic", default="/info")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument(
        "--diagnostic-root",
        default=str(DEFAULT_BASE / "logs" / "navigation_pose"),
    )

    parser.add_argument("--localization-timeout", type=float, default=90.0)
    parser.add_argument("--pose-ready-timeout", type=float, default=60.0)
    parser.add_argument("--pose-sample-seconds", type=float, default=2.0)
    parser.add_argument("--pose-sample-rate", type=float, default=10.0)
    parser.add_argument("--pose-min-samples", type=int, default=12)
    parser.add_argument("--pose-max-position-rms", type=float, default=0.15)
    parser.add_argument("--pose-max-position-error", type=float, default=0.30)
    parser.add_argument("--pose-max-yaw-std-deg", type=float, default=15.0)
    parser.add_argument("--pose-warn-abs-z", type=float, default=1.0)
    parser.add_argument("--pose-warn-tilt-deg", type=float, default=20.0)
    parser.add_argument("--diagnostic-interval", type=float, default=1.0)
    parser.add_argument(
        "--max-localization-age",
        type=float,
        default=60.0,
        help=(
            "最近一次非零视觉定位确认超过该时间时只播报警告；"
            "只要/info、/odom和TF仍新鲜，导航不会立即暂停。"
        ),
    )
    parser.add_argument(
        "--max-info-age",
        type=float,
        default=5.0,
        help="/info超过该时间没有新消息时暂停导航。",
    )
    parser.add_argument(
        "--max-odom-age",
        type=float,
        default=3.0,
        help="/odom超过该时间没有新消息时暂停导航。",
    )
    parser.add_argument(
        "--max-odom-tf-age",
        type=float,
        default=3.0,
        help=(
            "仅用于诊断TF相对墙钟的处理延迟，不再作为暂停条件。"
            "RGB-D里程计的TF时间戳可能天然落后数秒。"
        ),
    )
    parser.add_argument(
        "--max-odom-tf-stall",
        type=float,
        default=8.0,
        help=(
            "odom到base_link的TF时间戳连续不推进超过该时长才暂停。"
            "这比检查绝对TF年龄更能区分处理延迟和真正中断。"
        ),
    )
    parser.add_argument(
        "--pause-confirm-seconds",
        type=float,
        default=4.0,
        help="运行中故障持续超过该时长才进入暂停状态。",
    )
    parser.add_argument(
        "--recovery-confirm-seconds",
        type=float,
        default=1.5,
        help="故障恢复后连续健康达到该时长才恢复导航。",
    )
    parser.add_argument(
        "--voice-localization-state",
        action="store_true",
        help=(
            "播报持续定位故障和恢复状态。默认关闭，避免USB TTS队列被"
            "短暂状态变化占满；终端和CSV诊断仍完整保留。"
        ),
    )
    parser.add_argument(
        "--max-tf-age",
        type=float,
        default=None,
        help="兼容旧命令；提供时等同于--max-odom-tf-age。",
    )
    parser.add_argument("--rate", type=float, default=5.0)

    parser.add_argument("--inflation-radius", type=float, default=0.15)
    parser.add_argument("--min-inflation-radius", type=float, default=0.05)
    parser.add_argument("--inflation-step", type=float, default=0.05)
    parser.add_argument("--auto-relax", action="store_true")
    parser.add_argument("--snap-radius", type=float, default=0.30)
    parser.add_argument("--allow-unknown", action="store_true")

    parser.add_argument("--turn-threshold-deg", type=float, default=20.0)
    parser.add_argument("--merge-short-forward-m", type=float, default=0.35)
    parser.add_argument(
        "--instruction-angle-rounding-deg",
        type=float,
        default=10.0,
    )
    parser.add_argument("--max-merged-turn-deg", type=float, default=135.0)

    parser.add_argument("--turn-approach-distance", type=float, default=0.45)
    parser.add_argument(
        "--turn-complete-tolerance-deg",
        type=float,
        default=22.0,
    )
    parser.add_argument("--turn-pass-margin", type=float, default=0.30)

    parser.add_argument("--arrival-radius", type=float, default=0.45)
    parser.add_argument("--arrival-path-margin", type=float, default=0.80)

    parser.add_argument("--offroute-distance", type=float, default=0.60)
    parser.add_argument("--offroute-duration", type=float, default=2.0)
    parser.add_argument("--replan-cooldown", type=float, default=5.0)
    parser.add_argument("--max-replans", type=int, default=10)
    parser.add_argument("--status-interval", type=float, default=2.0)

    parser.add_argument("--force-map-mismatch", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.max_tf_age is not None:
        args.max_odom_tf_age = float(args.max_tf_age)
    if args.rate <= 0.0:
        print("[ERROR] --rate must be positive.", file=sys.stderr)
        return 2
    if args.max_odom_tf_stall <= 0.0:
        print("[ERROR] --max-odom-tf-stall must be positive.", file=sys.stderr)
        return 2
    if args.pause_confirm_seconds < 0.0 or args.recovery_confirm_seconds < 0.0:
        print("[ERROR] pause/recovery confirmation times cannot be negative.", file=sys.stderr)
        return 2

    yaml_path = Path(args.yaml).expanduser().resolve()
    landmark_path = Path(args.landmarks).expanduser().resolve()
    if not yaml_path.is_file():
        print(f"[ERROR] Missing map YAML: {yaml_path}", file=sys.stderr)
        return 2
    if not landmark_path.is_file():
        print(
            f"[ERROR] Missing landmark file: {landmark_path}",
            file=sys.stderr,
        )
        return 2

    try:
        map_data = planner.load_map(yaml_path)
        landmark_db = json.loads(
            landmark_path.read_text(encoding="utf-8")
        )
        stored_map = landmark_db.get("map", {})
        mismatch = (
            stored_map.get("yaml_sha256") != map_data["yaml_sha256"]
            or stored_map.get("image_sha256") != map_data["image_sha256"]
        )
        if mismatch and not args.force_map_mismatch:
            raise RuntimeError(
                "The landmark file does not match this YAML/PGM map."
            )

        places = landmark_db.get("places", {})
        if args.goal_name not in places:
            available = "、".join(sorted(places)) or "无"
            raise RuntimeError(
                f'Unknown goal place: "{args.goal_name}". '
                f"Available places: {available}"
            )
        goal_place = places[args.goal_name]

        diagnostic_root = Path(args.diagnostic_root).expanduser().resolve()
        diagnostics = NavigationDiagnostics(diagnostic_root, args.goal_name)
        print(f"[DIAGNOSTICS] {diagnostics.session_dir}")

        rclpy.init()
        node = RealtimeNavigator(
            info_topic=args.info_topic,
            odom_topic=args.odom_topic,
            map_frame=args.map_frame,
            odom_frame=args.odom_frame,
            base_frame=args.base_frame,
            diagnostics=diagnostics,
        )
        try:
            print(
                f'[WAIT] Preparing real-time navigation to '
                f'"{args.goal_name}".'
            )
            print(
                "[ACTION] Keep formal localization running. "
                "Move slowly until a non-zero localization event appears."
            )
            node.wait_for_localization(args.localization_timeout)
            print(
                "[ACTION] Visual localization was accepted. Keep the cane still "
                "while the navigation start pose is sampled."
            )
            initial_pose, stability = node.collect_stable_pose(args)
            print(
                f"[TF] stable {args.map_frame} -> {args.base_frame}: "
                f"x={initial_pose['x']:.3f}, y={initial_pose['y']:.3f}, "
                f"z={initial_pose['z']:.3f}, yaw={initial_pose['yaw_deg']:.1f}deg"
            )
            diagnostics.write_summary(
                {
                    "goal": args.goal_name,
                    "started_at": datetime.now().isoformat(),
                    "frames": {
                        "map": args.map_frame,
                        "odom": args.odom_frame,
                        "base": args.base_frame,
                    },
                    "topics": {"info": args.info_topic, "odom": args.odom_topic},
                    "initial_pose": initial_pose,
                    "stability": stability,
                    "status": "navigation_started",
                }
            )

            result = run_navigation(
                node=node,
                initial_pose=initial_pose,
                diagnostics=diagnostics,
                goal_name=args.goal_name,
                goal_place=goal_place,
                map_data=map_data,
                args=args,
            )
            diagnostics.write_summary(
                {
                    "goal": args.goal_name,
                    "finished_at": datetime.now().isoformat(),
                    "initial_pose": initial_pose,
                    "stability": stability,
                    "return_code": result,
                    "status": "finished",
                }
            )
            return result
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    except KeyboardInterrupt:
        print("\n[STOP] Real-time navigation stopped by user.")
        return 130
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
