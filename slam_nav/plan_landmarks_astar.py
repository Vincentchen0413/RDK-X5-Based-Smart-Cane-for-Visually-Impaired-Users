#!/usr/bin/env python3
"""Static named-place path planner for a Nav2 PGM/YAML map.

This first-stage planner is independent of voice and independent of Nav2
planner_server. It reads two saved places, plans an inflated-grid A* path,
and writes CSV, JSON and a map overlay PNG.

It is intended to validate:
- landmark coordinates;
- map free-space connectivity;
- start/goal selection;
- path geometry.

Dynamic obstacle avoidance is not included.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import heapq
import json
import math
import os
import re
import sys
from pathlib import Path
from typing import Iterable, Optional

import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
from PIL import Image
import yaml


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


def configure_matplotlib_font() -> str:
    """Select an installed CJK-capable font to avoid missing-glyph warnings."""
    preferred = (
        "Noto Sans CJK SC",
        "Noto Sans CJK TC",
        "WenQuanYi Zen Hei",
        "AR PL UMing CN",
        "AR PL SungtiL GB",
        "SimHei",
    )
    available = {font.name for font in font_manager.fontManager.ttflist}
    for family in preferred:
        if family in available:
            plt.rcParams["font.sans-serif"] = [family, "DejaVu Sans"]
            plt.rcParams["axes.unicode_minus"] = False
            return family
    return "DejaVu Sans"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sanitize_name(value: str) -> str:
    cleaned = re.sub(r"[^\w\u4e00-\u9fff-]+", "_", value, flags=re.UNICODE)
    return cleaned.strip("_") or "place"


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def load_map(yaml_path: Path) -> dict:
    data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    image_path = Path(str(data["image"]))
    if not image_path.is_absolute():
        image_path = (yaml_path.parent / image_path).resolve()

    image = Image.open(image_path).convert("L")
    array = np.asarray(image, dtype=np.uint8)
    origin = [float(value) for value in data["origin"]]

    return {
        "yaml_path": yaml_path.resolve(),
        "image_path": image_path,
        "image": image,
        "array": array,
        "resolution": float(data["resolution"]),
        "origin": origin,
        "mode": str(data.get("mode", "trinary")).lower(),
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


def build_blocked_grid(map_data: dict, allow_unknown: bool) -> np.ndarray:
    values = map_data["array"].astype(np.float32)

    # The exporter used for this project writes an exact trinary PGM:
    # 0=occupied, 205=unknown, 254=free. Preserve that meaning explicitly
    # instead of letting a permissive YAML free threshold turn 205 into free.
    unique_values = set(int(value) for value in np.unique(values))
    exact_project_trinary = (
        map_data.get("mode", "trinary") == "trinary"
        and unique_values.issubset({0, 205, 254, 255})
    )
    if exact_project_trinary:
        occupied = values <= 5
        free = values >= 250
        unknown = ~(occupied | free)
    else:
        if map_data["negate"]:
            occupancy = values / 255.0
        else:
            occupancy = (255.0 - values) / 255.0
        occupied = occupancy > map_data["occupied_thresh"]
        free = occupancy < map_data["free_thresh"]
        unknown = ~(occupied | free)

    return occupied | (unknown & (not allow_unknown))


def inflate_grid(blocked: np.ndarray, radius_cells: int) -> np.ndarray:
    if radius_cells <= 0:
        return blocked.copy()

    height, width = blocked.shape
    inflated = blocked.copy()
    offsets: list[tuple[int, int]] = []
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy <= radius_cells * radius_cells:
                offsets.append((dx, dy))

    obstacle_y, obstacle_x = np.nonzero(blocked)
    for center_x, center_y in zip(obstacle_x.tolist(), obstacle_y.tolist()):
        for dx, dy in offsets:
            x = center_x + dx
            y = center_y + dy
            if 0 <= x < width and 0 <= y < height:
                inflated[y, x] = True
    return inflated


def nearest_free(
    point: tuple[int, int],
    blocked: np.ndarray,
    max_radius_cells: int,
) -> Optional[tuple[int, int, float]]:
    center_x, center_y = point
    height, width = blocked.shape
    if (
        0 <= center_x < width
        and 0 <= center_y < height
        and not blocked[center_y, center_x]
    ):
        return center_x, center_y, 0.0

    best: Optional[tuple[int, int, float]] = None
    for radius in range(1, max_radius_cells + 1):
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if max(abs(dx), abs(dy)) != radius:
                    continue
                x = center_x + dx
                y = center_y + dy
                if not (0 <= x < width and 0 <= y < height):
                    continue
                if blocked[y, x]:
                    continue
                distance = math.hypot(dx, dy)
                if best is None or distance < best[2]:
                    best = (x, y, distance)
        if best is not None:
            return best
    return None


def candidate_inflation_radii(
    requested: float,
    minimum: float,
    step: float,
    auto_relax: bool,
) -> list[float]:
    if requested < 0.0 or minimum < 0.0 or step <= 0.0:
        raise ValueError("Inflation radii must be non-negative and step positive.")
    if not auto_relax:
        return [requested]

    radii = [requested]
    current = requested - step
    while current >= minimum - 1e-9:
        radii.append(max(minimum, current))
        current -= step
    if minimum not in radii:
        radii.append(minimum)

    unique: list[float] = []
    for radius in radii:
        rounded = round(radius, 6)
        if all(abs(rounded - existing) > 1e-9 for existing in unique):
            unique.append(rounded)
    return unique


def plan_with_inflation(
    base_blocked: np.ndarray,
    raw_start: tuple[int, int],
    raw_goal: tuple[int, int],
    resolution: float,
    requested_radius: float,
    minimum_radius: float,
    radius_step: float,
    snap_radius: float,
    auto_relax: bool,
):
    snap_cells = int(math.ceil(snap_radius / resolution))
    attempts = []

    for radius in candidate_inflation_radii(
        requested_radius,
        minimum_radius,
        radius_step,
        auto_relax,
    ):
        inflation_cells = int(math.ceil(radius / resolution))
        inflated = inflate_grid(base_blocked, inflation_cells)
        start_match = nearest_free(raw_start, inflated, snap_cells)
        goal_match = nearest_free(raw_goal, inflated, snap_cells)

        attempt = {
            "radius_m": radius,
            "inflation_cells": inflation_cells,
            "start_found": start_match is not None,
            "goal_found": goal_match is not None,
            "path_found": False,
        }

        if start_match is None or goal_match is None:
            attempts.append(attempt)
            continue

        start = (start_match[0], start_match[1])
        goal = (goal_match[0], goal_match[1])
        try:
            raw_path = astar(inflated, start, goal)
        except RuntimeError:
            attempts.append(attempt)
            continue

        attempt["path_found"] = True
        attempts.append(attempt)
        return {
            "selected_radius_m": radius,
            "inflation_cells": inflation_cells,
            "inflated": inflated,
            "start_match": start_match,
            "goal_match": goal_match,
            "start": start,
            "goal": goal,
            "raw_path": raw_path,
            "attempts": attempts,
        }

    details = ", ".join(
        (
            f"{attempt['radius_m']:.2f}m:"
            f"{'path' if attempt['path_found'] else 'no-path'}"
        )
        for attempt in attempts
    )
    raise RuntimeError(
        "No path exists between the selected places for the tested "
        f"inflation radii ({details}). Try a smaller minimum radius only "
        "for diagnosis, or clean the occupancy map bottleneck."
    )


NEIGHBORS = (
    (-1, 0, 1.0),
    (1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (-1, -1, math.sqrt(2.0)),
    (-1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)),
    (1, 1, math.sqrt(2.0)),
)


def astar(
    blocked: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> list[tuple[int, int]]:
    height, width = blocked.shape

    def heuristic(node: tuple[int, int]) -> float:
        return math.hypot(goal[0] - node[0], goal[1] - node[1])

    open_heap: list[tuple[float, float, int, int]] = []
    heapq.heappush(open_heap, (heuristic(start), 0.0, start[0], start[1]))
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score: dict[tuple[int, int], float] = {start: 0.0}
    closed: set[tuple[int, int]] = set()

    while open_heap:
        _, current_g, current_x, current_y = heapq.heappop(open_heap)
        current = (current_x, current_y)
        if current in closed:
            continue
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        closed.add(current)

        for dx, dy, move_cost in NEIGHBORS:
            next_x = current_x + dx
            next_y = current_y + dy
            if not (0 <= next_x < width and 0 <= next_y < height):
                continue
            if blocked[next_y, next_x]:
                continue

            if dx != 0 and dy != 0:
                if (
                    blocked[current_y, next_x]
                    or blocked[next_y, current_x]
                ):
                    continue

            neighbor = (next_x, next_y)
            tentative_g = current_g + move_cost
            if tentative_g >= g_score.get(neighbor, float("inf")):
                continue

            came_from[neighbor] = current
            g_score[neighbor] = tentative_g
            heapq.heappush(
                open_heap,
                (
                    tentative_g + heuristic(neighbor),
                    tentative_g,
                    next_x,
                    next_y,
                ),
            )

    raise RuntimeError("No path exists between the selected places.")


def bresenham(
    start: tuple[int, int],
    end: tuple[int, int],
) -> Iterable[tuple[int, int]]:
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    step_x = 1 if x0 < x1 else -1
    step_y = 1 if y0 < y1 else -1
    error = dx + dy

    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        twice_error = 2 * error
        if twice_error >= dy:
            error += dy
            x0 += step_x
        if twice_error <= dx:
            error += dx
            y0 += step_y


def line_is_free(
    start: tuple[int, int],
    end: tuple[int, int],
    blocked: np.ndarray,
) -> bool:
    height, width = blocked.shape
    for x, y in bresenham(start, end):
        if not (0 <= x < width and 0 <= y < height):
            return False
        if blocked[y, x]:
            return False
    return True


def simplify_path(
    path: list[tuple[int, int]],
    blocked: np.ndarray,
) -> list[tuple[int, int]]:
    if len(path) <= 2:
        return path[:]

    simplified = [path[0]]
    anchor_index = 0
    probe_index = 2

    while probe_index < len(path):
        if line_is_free(path[anchor_index], path[probe_index], blocked):
            probe_index += 1
            continue
        simplified.append(path[probe_index - 1])
        anchor_index = probe_index - 1
        probe_index = anchor_index + 2

    simplified.append(path[-1])
    return simplified


def path_length(points: list[tuple[float, float]]) -> float:
    return sum(
        math.hypot(
            points[index][0] - points[index - 1][0],
            points[index][1] - points[index - 1][1],
        )
        for index in range(1, len(points))
    )


def build_instructions(
    points: list[tuple[float, float]],
    start_yaw: float,
    turn_threshold_deg: float,
) -> list[dict]:
    if len(points) < 2:
        return []

    raw_segments: list[dict] = []
    for index in range(1, len(points)):
        dx = points[index][0] - points[index - 1][0]
        dy = points[index][1] - points[index - 1][1]
        distance = math.hypot(dx, dy)
        if distance <= 1e-6:
            continue
        raw_segments.append(
            {
                "heading": math.atan2(dy, dx),
                "distance": distance,
            }
        )

    if not raw_segments:
        return []

    merged: list[dict] = [raw_segments[0].copy()]
    threshold = math.radians(turn_threshold_deg)
    for segment in raw_segments[1:]:
        delta = abs(
            wrap_angle(segment["heading"] - merged[-1]["heading"])
        )
        if delta < threshold:
            total = merged[-1]["distance"] + segment["distance"]
            x_component = (
                math.cos(merged[-1]["heading"])
                * merged[-1]["distance"]
                + math.cos(segment["heading"])
                * segment["distance"]
            )
            y_component = (
                math.sin(merged[-1]["heading"])
                * merged[-1]["distance"]
                + math.sin(segment["heading"])
                * segment["distance"]
            )
            merged[-1]["heading"] = math.atan2(y_component, x_component)
            merged[-1]["distance"] = total
        else:
            merged.append(segment.copy())

    instructions: list[dict] = []
    previous_heading = start_yaw
    for segment in merged:
        turn = wrap_angle(segment["heading"] - previous_heading)
        turn_deg = math.degrees(turn)
        if abs(turn_deg) >= turn_threshold_deg:
            instructions.append(
                {
                    "type": "turn",
                    "direction": "left" if turn_deg > 0 else "right",
                    "angle_deg": abs(turn_deg),
                    "text": (
                        f"{'左转' if turn_deg > 0 else '右转'}"
                        f"约{abs(turn_deg):.0f}度"
                    ),
                }
            )
        instructions.append(
            {
                "type": "forward",
                "distance_m": segment["distance"],
                "text": f"前进约{segment['distance']:.2f}米",
            }
        )
        previous_heading = segment["heading"]

    return instructions


def round_half_up(value: float, step: float) -> float:
    """Round positive values to the nearest step, with exact halves rounded up."""
    if step <= 0.0:
        return value
    return math.floor(value / step + 0.5) * step


def merge_short_same_direction_turns(
    instructions: list[dict],
    max_short_forward_m: float,
    angle_rounding_deg: float,
    max_merged_turn_deg: float,
) -> tuple[list[dict], list[dict]]:
    """Merge turn-forward(short)-turn patterns when both turns are same direction.

    The underlying A* path and CSV are not changed. Only the high-level spoken
    instruction list is simplified. Example:
      左转48° -> 前进0.28m -> 左转37°
    becomes:
      左转约90°
    """
    if max_short_forward_m < 0.0:
        raise ValueError("max_short_forward_m must be non-negative")
    if max_merged_turn_deg <= 0.0:
        raise ValueError("max_merged_turn_deg must be positive")

    merged: list[dict] = []
    merge_records: list[dict] = []
    index = 0

    while index < len(instructions):
        current = instructions[index]
        if current.get("type") != "turn":
            merged.append(current)
            index += 1
            continue

        direction = str(current.get("direction"))
        component_angles = [float(current.get("angle_deg", 0.0))]
        skipped_forward_distances: list[float] = []
        scan = index

        while scan + 2 < len(instructions):
            middle = instructions[scan + 1]
            following = instructions[scan + 2]
            if middle.get("type") != "forward":
                break
            if following.get("type") != "turn":
                break
            if str(following.get("direction")) != direction:
                break

            short_distance = float(middle.get("distance_m", float("inf")))
            next_angle = float(following.get("angle_deg", 0.0))
            proposed_total = sum(component_angles) + next_angle

            if short_distance > max_short_forward_m:
                break
            if proposed_total > max_merged_turn_deg:
                break

            skipped_forward_distances.append(short_distance)
            component_angles.append(next_angle)
            scan += 2

        if len(component_angles) == 1:
            merged.append(current)
            index += 1
            continue

        total_angle = sum(component_angles)
        display_angle = round_half_up(total_angle, angle_rounding_deg)
        direction_text = "左转" if direction == "left" else "右转"
        combined = {
            "type": "turn",
            "direction": direction,
            "angle_deg": total_angle,
            "display_angle_deg": display_angle,
            "text": f"{direction_text}约{display_angle:.0f}度",
            "merged": True,
            "component_turn_angles_deg": component_angles,
            "absorbed_short_forward_distances_m": skipped_forward_distances,
            "absorbed_short_forward_total_m": sum(skipped_forward_distances),
        }
        merged.append(combined)
        merge_records.append(combined.copy())
        index = scan + 1

    return merged, merge_records


def _main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("start_name")
    parser.add_argument("goal_name")
    parser.add_argument("--yaml", default=str(DEFAULT_YAML))
    parser.add_argument("--landmarks", default=str(DEFAULT_LANDMARKS))
    parser.add_argument("--output-base")
    parser.add_argument("--inflation-radius", type=float, default=0.20)
    parser.add_argument("--min-inflation-radius", type=float, default=0.05)
    parser.add_argument("--inflation-step", type=float, default=0.05)
    parser.add_argument("--no-auto-relax", action="store_true")
    parser.add_argument("--snap-radius", type=float, default=0.30)
    parser.add_argument("--allow-unknown", action="store_true")
    parser.add_argument("--turn-threshold-deg", type=float, default=20.0)
    parser.add_argument(
        "--merge-short-forward-m",
        type=float,
        default=0.35,
        help=(
            "Merge same-direction turn/short-forward/turn instruction "
            "patterns when the middle forward distance is at most this value."
        ),
    )
    parser.add_argument(
        "--instruction-angle-rounding-deg",
        type=float,
        default=10.0,
        help="Round merged spoken turn angles to this many degrees.",
    )
    parser.add_argument(
        "--max-merged-turn-deg",
        type=float,
        default=135.0,
        help="Do not merge a same-direction turn sequence beyond this angle.",
    )
    parser.add_argument(
        "--no-merge-short-turns",
        action="store_true",
        help="Keep the original detailed turn/short-forward/turn instructions.",
    )
    parser.add_argument("--force-map-mismatch", action="store_true")
    args = parser.parse_args()

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

    map_data = load_map(yaml_path)
    landmark_db = json.loads(landmark_path.read_text(encoding="utf-8"))
    stored_map = landmark_db.get("map", {})
    mismatch = (
        stored_map.get("yaml_sha256") != map_data["yaml_sha256"]
        or stored_map.get("image_sha256") != map_data["image_sha256"]
    )
    if mismatch and not args.force_map_mismatch:
        print(
            "[ERROR] The landmark file does not match this YAML/PGM map.",
            file=sys.stderr,
        )
        return 2

    places = landmark_db.get("places", {})
    if args.start_name not in places:
        print(
            f'[ERROR] Unknown start place: "{args.start_name}"',
            file=sys.stderr,
        )
        return 2
    if args.goal_name not in places:
        print(
            f'[ERROR] Unknown goal place: "{args.goal_name}"',
            file=sys.stderr,
        )
        return 2

    start_place = places[args.start_name]
    goal_place = places[args.goal_name]

    blocked = build_blocked_grid(map_data, args.allow_unknown)

    raw_start = world_to_pixel(
        float(start_place["x"]),
        float(start_place["y"]),
        map_data["resolution"],
        map_data["origin"],
        map_data["height"],
    )
    raw_goal = world_to_pixel(
        float(goal_place["x"]),
        float(goal_place["y"]),
        map_data["resolution"],
        map_data["origin"],
        map_data["height"],
    )

    plan = plan_with_inflation(
        base_blocked=blocked,
        raw_start=raw_start,
        raw_goal=raw_goal,
        resolution=map_data["resolution"],
        requested_radius=args.inflation_radius,
        minimum_radius=args.min_inflation_radius,
        radius_step=args.inflation_step,
        snap_radius=args.snap_radius,
        auto_relax=not args.no_auto_relax,
    )
    selected_inflation_radius = float(plan["selected_radius_m"])
    inflation_cells = int(plan["inflation_cells"])
    inflated = plan["inflated"]
    start_match = plan["start_match"]
    goal_match = plan["goal_match"]
    start = plan["start"]
    goal = plan["goal"]
    raw_path = plan["raw_path"]
    simple_path = simplify_path(raw_path, inflated)

    raw_world = [
        pixel_to_world(
            x,
            y,
            map_data["resolution"],
            map_data["origin"],
            map_data["height"],
        )
        for x, y in raw_path
    ]
    simple_world = [
        pixel_to_world(
            x,
            y,
            map_data["resolution"],
            map_data["origin"],
            map_data["height"],
        )
        for x, y in simple_path
    ]

    total_length = path_length(raw_world)
    detailed_instructions = build_instructions(
        simple_world,
        float(start_place.get("yaw", 0.0)),
        args.turn_threshold_deg,
    )
    if args.no_merge_short_turns:
        instructions = detailed_instructions
        instruction_merges: list[dict] = []
    else:
        instructions, instruction_merges = merge_short_same_direction_turns(
            detailed_instructions,
            max_short_forward_m=args.merge_short_forward_m,
            angle_rounding_deg=args.instruction_angle_rounding_deg,
            max_merged_turn_deg=args.max_merged_turn_deg,
        )

    if args.output_base:
        output_base = Path(args.output_base).expanduser().resolve()
    else:
        output_base = (
            DEFAULT_PLAN_DIR
            / (
                sanitize_name(args.start_name)
                + "_to_"
                + sanitize_name(args.goal_name)
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
        "map_yaml": str(yaml_path),
        "landmarks": str(landmark_path),
        "start_name": args.start_name,
        "goal_name": args.goal_name,
        "start_landmark": start_place,
        "goal_landmark": goal_place,
        "planner": {
            "type": "static_grid_astar",
            "allow_unknown": bool(args.allow_unknown),
            "requested_inflation_radius_m": args.inflation_radius,
            "selected_inflation_radius_m": selected_inflation_radius,
            "inflation_attempts": plan["attempts"],
            "snap_radius_m": args.snap_radius,
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

    selected_font = configure_matplotlib_font()
    figure, axes = plt.subplots(figsize=(10, 10))
    axes.imshow(map_data["image"], origin="upper", interpolation="nearest")
    path_x = [point[0] for point in raw_path]
    path_y = [point[1] for point in raw_path]
    axes.plot(path_x, path_y, linewidth=2.0, label="Planned path")
    axes.plot(
        start[0],
        start[1],
        marker="o",
        markersize=9,
        linestyle="None",
        label=args.start_name,
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
        f"{args.start_name} → {args.goal_name} "
        f"({total_length:.2f} m)"
    )
    axes.set_axis_off()
    axes.legend(loc="best")
    figure.tight_layout()
    figure.savefig(png_path, dpi=160, bbox_inches="tight")
    plt.close(figure)

    print(
        f'[OK] Planned "{args.start_name}" -> "{args.goal_name}".'
    )
    print(
        f"[PATH] length={total_length:.2f}m, "
        f"raw_poses={len(raw_world)}, "
        f"simplified_poses={len(simple_world)}"
    )
    if selected_inflation_radius + 1e-9 < args.inflation_radius:
        print(
            f"[WARN] Requested inflation {args.inflation_radius:.2f}m "
            f"disconnects the free-space graph; using "
            f"{selected_inflation_radius:.2f}m for this diagnostic plan."
        )
    print(
        f"[CLEARANCE] selected_inflation_radius="
        f"{selected_inflation_radius:.2f}m "
        f"(requested={args.inflation_radius:.2f}m)"
    )
    if start_match[2] > 0:
        print(
            f"[SNAP] start moved "
            f"{start_match[2] * map_data['resolution']:.2f}m "
            "to an inflated-free cell."
        )
    if goal_match[2] > 0:
        print(
            f"[SNAP] goal moved "
            f"{goal_match[2] * map_data['resolution']:.2f}m "
            "to an inflated-free cell."
        )

    if instruction_merges:
        for merge_index, merge in enumerate(instruction_merges, start=1):
            components = " + ".join(
                f"{angle:.0f}°"
                for angle in merge["component_turn_angles_deg"]
            )
            short_total = merge["absorbed_short_forward_total_m"]
            print(
                f"[MERGE {merge_index}] {components} with "
                f"{short_total:.2f}m short connector -> "
                f"{merge['text']}"
            )

    for index, instruction in enumerate(instructions, start=1):
        print(f"[INSTRUCTION {index}] {instruction['text']}")

    print(f"[FONT] {selected_font}")

    print(f"[CSV]  {csv_path}")
    print(f"[JSON] {json_path}")
    print(f"[PNG]  {png_path}")
    return 0


def main() -> int:
    try:
        return _main()
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
