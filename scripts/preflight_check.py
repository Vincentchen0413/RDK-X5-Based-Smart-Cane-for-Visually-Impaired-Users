#!/usr/bin/env python3
"""Check common submission/runtime problems before starting the system."""

from __future__ import annotations

import os
import shutil
import sys
from pathlib import Path

import yaml


def ok(message: str):
    print(f"[ OK ] {message}")


def warn(message: str):
    print(f"[WARN] {message}")


def fail(message: str):
    print(f"[FAIL] {message}")


def main() -> int:
    root = Path(
        os.environ.get("SMART_CANE_ROOT", Path(__file__).resolve().parents[1])
    )
    problems = 0

    required = [
        "README.md",
        "README_cn.md",
        "config/modules.yaml",
        "config/system.yaml",
        "bringup/smart_cane_supervisor.py",
        "fall/rdk_x5/fall.py",
        "voice/smart_cane_voice_node.py",
    ]
    for rel in required:
        path = root / rel
        if path.exists():
            ok(rel)
        else:
            fail(f"missing {rel}")
            problems += 1

    with open(root / "config/modules.yaml", encoding="utf-8") as f:
        module_cfg = yaml.safe_load(f)

    for name, cfg in module_cfg.get("modules", {}).items():
        command = [str(x) for x in cfg.get("command", [])]
        if not command:
            fail(f"{name}: empty command")
            problems += 1
            continue
        executable = command[0]
        if "/" not in executable and shutil.which(executable) is None:
            fail(f"{name}: executable not found: {executable}")
            problems += 1

        for token in command[1:]:
            if token.endswith((".py", ".sh")) and not (root / token).exists():
                warn(f"{name}: referenced file does not exist yet: {token}")

    for device in ["/dev/video0", "/dev/video1"]:
        if Path(device).exists():
            ok(f"camera device {device}")
        else:
            warn(f"camera device not found: {device}")

    model_candidates = list(root.glob("detection/**/*.bin"))
    if model_candidates:
        ok(f"found {len(model_candidates)} RDK model file(s)")
    else:
        warn("no .bin model found; document the model download/build process")

    if problems:
        print(f"\nPreflight completed with {problems} blocking problem(s).")
        return 1
    print("\nPreflight completed. Warnings still need manual review.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
