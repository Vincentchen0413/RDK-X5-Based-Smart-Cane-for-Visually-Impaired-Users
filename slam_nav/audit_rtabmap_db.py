#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sqlite3
from pathlib import Path


def scalar(con: sqlite3.Connection, query: str):
    row = con.execute(query).fetchone()
    return None if row is None else row[0]


def audit(path: Path) -> None:
    print(f"========== {path}")
    with sqlite3.connect(str(path)) as con:
        checks = [
            ("quick_check", "PRAGMA quick_check"),
            ("nodes", "SELECT COUNT(*) FROM Node"),
            ("map_ids", "SELECT COUNT(DISTINCT map_id) FROM Node"),
            ("data_rows", "SELECT COUNT(*) FROM Data"),
            ("images", "SELECT COUNT(*) FROM Data WHERE length(image)>0"),
            ("depths", "SELECT COUNT(*) FROM Data WHERE length(depth)>0"),
            ("calibrations", "SELECT COUNT(*) FROM Data WHERE length(calibration)>0"),
            ("local_grids", """
                SELECT COUNT(*) FROM Data
                WHERE length(ground_cells)>0
                   OR length(obstacle_cells)>0
                   OR length(empty_cells)>0
            """),
            ("features", "SELECT COUNT(*) FROM Feature"),
            ("words", "SELECT COUNT(*) FROM Word"),
            ("missing_graph_words", """
                SELECT COUNT(*) FROM (
                  SELECT DISTINCT f.word_id
                  FROM Feature f
                  JOIN Node n ON n.id=f.node_id
                  LEFT JOIN Word w ON w.id=f.word_id
                  WHERE f.word_id>0
                    AND COALESCE(n.weight,0)>=0
                    AND w.id IS NULL
                )
            """),
            ("links_total", "SELECT COUNT(*) FROM Link"),
            ("links_neighbor", "SELECT COUNT(*) FROM Link WHERE type=0"),
            ("links_global_loop", "SELECT COUNT(*) FROM Link WHERE type=1"),
            ("links_local_space", "SELECT COUNT(*) FROM Link WHERE type=2"),
            ("links_local_time", "SELECT COUNT(*) FROM Link WHERE type=3"),
        ]
        results = {}
        for label, query in checks:
            try:
                value = scalar(con, query)
                results[label] = value
                print(f"{label:20s}: {value}")
            except sqlite3.Error as exc:
                print(f"{label:20s}: ERROR {exc}")

        try:
            row = con.execute(
                """
                SELECT version,
                       length(opt_map),
                       opt_map_x_min,
                       opt_map_y_min,
                       opt_map_resolution,
                       length(opt_ids),
                       length(opt_poses)
                FROM Admin LIMIT 1
                """
            ).fetchone()
            print(f"{'admin':20s}: {row}")
            if row:
                opt_map_bytes = int(row[1] or 0)
                opt_map_resolution = float(row[4] or 0.0)
                saved_ready = opt_map_bytes > 0 and opt_map_resolution > 0.0
                graph_ready = (
                    int(results.get("nodes") or 0) > 0
                    and int(results.get("local_grids") or 0) > 0
                )
                print(f"{'saved_opt_map_ready':20s}: {saved_ready}")
                print(f"{'graph_rebuild_ready':20s}: {graph_ready}")
                print(f"{'opt_map_bytes':20s}: {opt_map_bytes}")
                print(f"{'opt_map_resolution':20s}: {opt_map_resolution}")
        except sqlite3.Error as exc:
            print(f"{'admin':20s}: ERROR {exc}")

        try:
            params = scalar(
                con,
                "SELECT parameters FROM Info ORDER BY time_enter DESC LIMIT 1",
            )
            if params:
                wanted = (
                    "Mem/GenerateIds",
                    "Mem/BinDataKept",
                    "Mem/RawDescriptorsKept",
                    "Rtabmap/DetectionRate",
                    "RGBD/OptimizeFromGraphEnd",
                    "Kp/DetectorStrategy",
                    "Vis/FeatureType",
                )
                print("saved_parameters:")
                for line in str(params).replace(";", "\n").splitlines():
                    if any(key in line for key in wanted):
                        print("  " + line.strip())
        except sqlite3.Error as exc:
            print(f"saved_parameters     : ERROR {exc}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("databases", nargs="+")
    args = parser.parse_args()
    for item in args.databases:
        path = Path(item).expanduser().resolve()
        if not path.is_file():
            print(f"========== {path}\nERROR: not found")
            continue
        audit(path)


if __name__ == "__main__":
    main()
