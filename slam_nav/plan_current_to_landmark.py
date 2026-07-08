#!/usr/bin/env python3
"""Plan a static global path from the live RTAB-Map pose to a named place.

Required runtime:
- formal RTAB-Map localization is running;
- /info publishes rtabmap_msgs/msg/Info;
- TF map -> base_link is available;
- the matching map YAML/PGM and landmark JSON exist.

This tool takes one stable snapshot of the current pose, runs the same
inflated-grid A* planner used by plan_landmarks_astar_v3.py, and writes:
- CSV path poses;
- JSON plan metadata and high-level instructions;
- PNG map overlay.

It does not continuously follow the path and does not re-plan while walking.
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
DEFAULT_PLAN_DIR = DEFAULT_MAP_DIR / "plans"


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


def summarize_pose_samples(
    samples: list[tuple[float, float, float, float]],
) -> dict:
    if not samples:
        raise RuntimeError("No TF pose samples were collected.")

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
        sum(error * error for error in radial_errors)
        / len(radial_errors)
    )

    return {
        "x": x,
        "y": y,
        "z": z,
        "yaw": yaw,
        "yaw_deg": math.degrees(yaw),
        "position_rms_m": position_rms,
        "position_max_error_m": max(radial_errors),
        "yaw_std_deg": math.degrees(circular_std(yaws)),
        "sample_count": len(samples),
    }


class LivePoseSampler(Node):
    def __init__(
        self,
        info_topic: str,
        map_frame: str,
        base_frame: str,
    ) -> None:
        super().__init__("plan_current_to_landmark_pose_sampler")
        self.info_topic = info_topic
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.last_localization_event: Optional[float] = None
        self.last_ids = (0, 0, 0)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Info, info_topic, self._on_info, qos)

    def _on_info(self, msg: Info) -> None:
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

    def wait_for_tf(self, timeout_seconds: float) -> None:
        deadline = time.monotonic() + timeout_seconds
        last_error = ""
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.base_frame,
                    Time(),
                    timeout=Duration(seconds=0.2),
                )
                print(
                    f"[TF] {self.map_frame} -> "
                    f"{self.base_frame} is ready."
                )
                return
            except TransformException as exc:
                last_error = str(exc)
                time.sleep(0.05)

        raise RuntimeError(
            f"TF {self.map_frame} -> {self.base_frame} did not become "
            f"available within {timeout_seconds:.0f}s. "
            f"Last error: {last_error}"
        )

    def collect_stable_pose(
        self,
        sample_seconds: float,
        sample_rate: float,
        collection_timeout: float,
        max_localization_age: float,
        min_samples: int,
    ) -> list[tuple[float, float, float, float]]:
        period = 1.0 / sample_rate
        collection_deadline = time.monotonic() + collection_timeout
        active_since: Optional[float] = None
        next_sample = time.monotonic()
        samples: list[tuple[float, float, float, float]] = []
        last_error = ""

        while rclpy.ok() and time.monotonic() < collection_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.monotonic()

            if (
                self.last_localization_event is None
                or now - self.last_localization_event
                > max_localization_age
            ):
                raise RuntimeError(
                    "The accepted localization event became too old before "
                    "the current pose could be sampled."
                )

            if now < next_sample:
                continue
            next_sample += period

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.base_frame,
                    Time(),
                    timeout=Duration(seconds=0.15),
                )
            except TransformException as exc:
                last_error = str(exc)
                continue

            if active_since is None:
                active_since = now

            translation = transform.transform.translation
            rotation = transform.transform.rotation
            samples.append(
                (
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
            )

            if (
                active_since is not None
                and now - active_since >= sample_seconds
                and len(samples) >= min_samples
            ):
                return samples

        raise RuntimeError(
            f"Only {len(samples)} TF samples were collected; "
            f"at least {min_samples} are required. "
            f"Last TF error: {last_error or 'none'}"
        )


def validate_landmark_map(
    landmark_db: dict,
    map_data: dict,
    force_map_mismatch: bool,
) -> None:
    stored_map = landmark_db.get("map", {})
    mismatch = (
        stored_map.get("yaml_sha256") != map_data["yaml_sha256"]
        or stored_map.get("image_sha256") != map_data["image_sha256"]
    )
    if mismatch and not force_map_mismatch:
        raise RuntimeError(
            "The landmark file does not match this YAML/PGM map."
        )


def save_plan(
    *,
    args: argparse.Namespace,
    map_data: dict,
    landmark_path: Path,
    goal_place: dict,
    current_pose: dict,
    localization_ids: tuple[int, int, int],
    plan: dict,
) -> tuple[Path, Path, Path]:
    selected_inflation_radius = float(plan["selected_radius_m"])
    inflated = plan["inflated"]
    start_match = plan["start_match"]
    goal_match = plan["goal_match"]
    start = plan["start"]
    goal = plan["goal"]
    raw_path = plan["raw_path"]
    simple_path = planner.simplify_path(raw_path, inflated)

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

    total_length = planner.path_length(raw_world)
    detailed_instructions = planner.build_instructions(
        simple_world,
        float(current_pose["yaw"]),
        args.turn_threshold_deg,
    )
    if args.no_merge_short_turns:
        instructions = detailed_instructions
        instruction_merges: list[dict] = []
    else:
        instructions, instruction_merges = (
            planner.merge_short_same_direction_turns(
                detailed_instructions,
                max_short_forward_m=args.merge_short_forward_m,
                angle_rounding_deg=args.instruction_angle_rounding_deg,
                max_merged_turn_deg=args.max_merged_turn_deg,
            )
        )

    if args.output_base:
        output_base = Path(args.output_base).expanduser().resolve()
    else:
        output_base = (
            DEFAULT_PLAN_DIR
            / (
                "当前位置_to_"
                + planner.sanitize_name(args.goal_name)
            )
        ).resolve()
    output_base.parent.mkdir(parents=True, exist_ok=True)

    csv_path = output_base.with_suffix(".csv")
    json_path = output_base.with_suffix(".json")
    png_path = output_base.with_suffix(".png")

    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "index",
                "x_m",
                "y_m",
                "yaw_deg",
                "pixel_x",
                "pixel_y",
            ]
        )
        for index, ((pixel_x, pixel_y), (x, y)) in enumerate(
            zip(raw_path, raw_world)
        ):
            if index + 1 < len(raw_world):
                next_x, next_y = raw_world[index + 1]
                yaw = math.atan2(next_y - y, next_x - x)
            else:
                yaw = float(goal_place.get("yaw", 0.0))
            writer.writerow(
                [
                    index,
                    f"{x:.6f}",
                    f"{y:.6f}",
                    f"{math.degrees(yaw):.3f}",
                    pixel_x,
                    pixel_y,
                ]
            )

    summary = {
        "schema_version": 1,
        "created_at": datetime.now().astimezone().isoformat(
            timespec="seconds"
        ),
        "map_yaml": str(map_data["yaml_path"]),
        "landmarks": str(landmark_path),
        "start_name": "当前位置",
        "goal_name": args.goal_name,
        "start_pose_source": "TF map->base_link after accepted /info event",
        "start_pose": {
            "frame_id": args.map_frame,
            **current_pose,
            "last_localization_ids": {
                "loop_closure_id": int(localization_ids[0]),
                "proximity_detection_id": int(localization_ids[1]),
                "landmark_id": int(localization_ids[2]),
            },
        },
        "goal_landmark": goal_place,
        "planner": {
            "type": "live_pose_static_grid_astar",
            "allow_unknown": bool(args.allow_unknown),
            "requested_inflation_radius_m": args.inflation_radius,
            "selected_inflation_radius_m": selected_inflation_radius,
            "inflation_attempts": plan["attempts"],
            "snap_radius_m": args.snap_radius,
            "continuous_replanning": False,
            "dynamic_obstacle_avoidance": False,
        },
        "start_snap_distance_m": (
            start_match[2] * map_data["resolution"]
        ),
        "goal_snap_distance_m": (
            goal_match[2] * map_data["resolution"]
        ),
        "path_length_m": total_length,
        "raw_pose_count": len(raw_world),
        "simplified_pose_count": len(simple_world),
        "raw_path": [
            {"x": x, "y": y}
            for x, y in raw_world
        ],
        "simplified_path": [
            {"x": x, "y": y}
            for x, y in simple_world
        ],
        "instruction_postprocess": {
            "merge_short_turns": not args.no_merge_short_turns,
            "max_short_forward_m": args.merge_short_forward_m,
            "angle_rounding_deg": args.instruction_angle_rounding_deg,
            "max_merged_turn_deg": args.max_merged_turn_deg,
            "merge_count": len(instruction_merges),
        },
        "detailed_instructions": detailed_instructions,
        "instructions": instructions,
    }
    json_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    selected_font = planner.configure_matplotlib_font()
    figure, axes = plt.subplots(figsize=(10, 10))
    axes.imshow(
        map_data["image"],
        origin="upper",
        interpolation="nearest",
    )
    path_x = [point[0] for point in raw_path]
    path_y = [point[1] for point in raw_path]
    axes.plot(path_x, path_y, linewidth=2.0, label="Planned path")
    axes.plot(
        start[0],
        start[1],
        marker="o",
        markersize=9,
        linestyle="None",
        label="当前位置",
    )
    axes.plot(
        goal[0],
        goal[1],
        marker="s",
        markersize=9,
        linestyle="None",
        label=args.goal_name,
    )
    axes.set_title(
        f"当前位置 → {args.goal_name} ({total_length:.2f} m)"
    )
    axes.set_axis_off()
    axes.legend(loc="best")
    figure.tight_layout()
    figure.savefig(png_path, dpi=160, bbox_inches="tight")
    plt.close(figure)

    print(
        f'[OK] Planned "当前位置" -> "{args.goal_name}".'
    )
    print(
        f"[CURRENT] x={current_pose['x']:.3f}, "
        f"y={current_pose['y']:.3f}, "
        f"yaw={current_pose['yaw_deg']:.1f}°"
    )
    print(
        f"[STABILITY] samples={current_pose['sample_count']}, "
        f"position_rms={current_pose['position_rms_m']:.3f}m, "
        f"yaw_std={current_pose['yaw_std_deg']:.2f}°"
    )
    print(
        f"[PATH] length={total_length:.2f}m, "
        f"raw_poses={len(raw_world)}, "
        f"simplified_poses={len(simple_world)}"
    )

    if selected_inflation_radius + 1e-9 < args.inflation_radius:
        print(
            f"[WARN] Requested inflation {args.inflation_radius:.2f}m "
            f"was not connected; using "
            f"{selected_inflation_radius:.2f}m."
        )
    print(
        f"[CLEARANCE] selected_inflation_radius="
        f"{selected_inflation_radius:.2f}m"
    )

    start_snap_m = start_match[2] * map_data["resolution"]
    goal_snap_m = goal_match[2] * map_data["resolution"]
    if start_snap_m > 0:
        print(
            f"[SNAP] Current start moved {start_snap_m:.2f}m "
            "to the nearest inflated-free cell."
        )
    if goal_snap_m > 0:
        print(
            f"[SNAP] Goal moved {goal_snap_m:.2f}m "
            "to the nearest inflated-free cell."
        )

    if instruction_merges:
        for merge_index, merge in enumerate(
            instruction_merges,
            start=1,
        ):
            components = " + ".join(
                f"{angle:.0f}°"
                for angle in merge["component_turn_angles_deg"]
            )
            print(
                f"[MERGE {merge_index}] {components} -> "
                f"{merge['text']}"
            )

    for index, instruction in enumerate(instructions, start=1):
        print(f"[INSTRUCTION {index}] {instruction['text']}")

    print(f"[FONT] {selected_font}")
    print(f"[CSV]  {csv_path}")
    print(f"[JSON] {json_path}")
    print(f"[PNG]  {png_path}")
    print(
        "[NOTE] This is a one-shot static plan. It is not yet tracking "
        "your progress or re-planning while walking."
    )
    return csv_path, json_path, png_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("goal_name")
    parser.add_argument("--yaml", default=str(DEFAULT_YAML))
    parser.add_argument("--landmarks", default=str(DEFAULT_LANDMARKS))
    parser.add_argument("--output-base")
    parser.add_argument("--info-topic", default="/info")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")

    parser.add_argument("--localization-timeout", type=float, default=60.0)
    parser.add_argument("--tf-timeout", type=float, default=30.0)
    parser.add_argument("--sample-seconds", type=float, default=3.0)
    parser.add_argument("--sample-rate", type=float, default=10.0)
    parser.add_argument("--collection-timeout", type=float, default=15.0)
    parser.add_argument("--min-samples", type=int, default=15)
    parser.add_argument("--max-localization-age", type=float, default=15.0)
    parser.add_argument("--max-position-rms", type=float, default=0.08)
    parser.add_argument("--max-yaw-std-deg", type=float, default=8.0)

    parser.add_argument("--inflation-radius", type=float, default=0.15)
    parser.add_argument("--min-inflation-radius", type=float, default=0.05)
    parser.add_argument("--inflation-step", type=float, default=0.05)
    parser.add_argument(
        "--auto-relax",
        action="store_true",
        help=(
            "Allow the planner to reduce inflation radius if the requested "
            "clearance disconnects the map. Disabled by default."
        ),
    )
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
    parser.add_argument("--no-merge-short-turns", action="store_true")
    parser.add_argument("--force-map-mismatch", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()

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
        validate_landmark_map(
            landmark_db,
            map_data,
            args.force_map_mismatch,
        )
        places = landmark_db.get("places", {})
        if args.goal_name not in places:
            available = "、".join(sorted(places)) or "无"
            raise RuntimeError(
                f'Unknown goal place: "{args.goal_name}". '
                f"Available places: {available}"
            )
        goal_place = places[args.goal_name]

        rclpy.init()
        node = LivePoseSampler(
            info_topic=args.info_topic,
            map_frame=args.map_frame,
            base_frame=args.base_frame,
        )
        try:
            print(
                f'[WAIT] Planning from the live pose to '
                f'"{args.goal_name}".'
            )
            print(
                "[ACTION] Keep formal localization running. "
                "Move slowly until a non-zero localization event appears, "
                "then stand still."
            )
            node.wait_for_localization(args.localization_timeout)
            node.wait_for_tf(args.tf_timeout)
            print(
                f"[SAMPLE] Stand still for about "
                f"{args.sample_seconds:.1f}s..."
            )
            samples = node.collect_stable_pose(
                sample_seconds=args.sample_seconds,
                sample_rate=args.sample_rate,
                collection_timeout=args.collection_timeout,
                max_localization_age=args.max_localization_age,
                min_samples=args.min_samples,
            )
            localization_ids = node.last_ids
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

        current_pose = summarize_pose_samples(samples)
        if current_pose["position_rms_m"] > args.max_position_rms:
            raise RuntimeError(
                f"Current position is not stable: "
                f"RMS={current_pose['position_rms_m']:.3f}m, "
                f"limit={args.max_position_rms:.3f}m."
            )
        if current_pose["yaw_std_deg"] > args.max_yaw_std_deg:
            raise RuntimeError(
                f"Current yaw is not stable: "
                f"std={current_pose['yaw_std_deg']:.2f}°, "
                f"limit={args.max_yaw_std_deg:.2f}°."
            )

        blocked = planner.build_blocked_grid(
            map_data,
            args.allow_unknown,
        )
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

        save_plan(
            args=args,
            map_data=map_data,
            landmark_path=landmark_path,
            goal_place=goal_place,
            current_pose=current_pose,
            localization_ids=localization_ids,
            plan=plan,
        )
        return 0

    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
