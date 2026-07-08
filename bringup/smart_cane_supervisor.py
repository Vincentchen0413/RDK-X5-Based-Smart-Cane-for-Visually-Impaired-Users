#!/usr/bin/env python3
"""Process supervisor for the smart cane.

This file intentionally launches existing scripts without forcing every module
to be converted into a ROS package at once. It provides dependency-free
process management, log redirection, restart policies and graceful shutdown.
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import yaml


@dataclass
class ManagedProcess:
    name: str
    command: List[str]
    required: bool
    restart: str
    startup_delay: float
    process: Optional[subprocess.Popen] = None
    restarts: int = 0
    log_handle: Optional[object] = None


class Supervisor:
    def __init__(self, root: Path, config_path: Path, profile: str):
        self.root = root
        self.config_path = config_path
        self.profile = profile
        self.running = True
        self.items: Dict[str, ManagedProcess] = {}

        data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        names = data["profiles"][profile]
        modules = data["modules"]

        for name in names:
            cfg = modules[name]
            self.items[name] = ManagedProcess(
                name=name,
                command=[str(x) for x in cfg["command"]],
                required=bool(cfg.get("required", False)),
                restart=str(cfg.get("restart", "never")),
                startup_delay=float(cfg.get("startup_delay", 0.0)),
            )

        self.log_dir = root / "logs"
        self.log_dir.mkdir(parents=True, exist_ok=True)

    def start_one(self, item: ManagedProcess) -> None:
        if item.startup_delay:
            time.sleep(item.startup_delay)

        log_path = self.log_dir / f"{item.name}.log"
        item.log_handle = open(log_path, "a", encoding="utf-8", buffering=1)
        env = os.environ.copy()
        env["SMART_CANE_ROOT"] = str(self.root)
        env["PYTHONUNBUFFERED"] = "1"

        print(f"[supervisor] starting {item.name}: {' '.join(item.command)}")
        try:
            item.process = subprocess.Popen(
                item.command,
                cwd=self.root,
                env=env,
                stdout=item.log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                text=True,
            )
        except FileNotFoundError as exc:
            item.log_handle.write(f"start failed: {exc}\n")
            item.process = None
            if item.required:
                raise

    def stop_one(self, item: ManagedProcess, timeout: float = 8.0) -> None:
        proc = item.process
        if proc is None or proc.poll() is not None:
            return
        print(f"[supervisor] stopping {item.name}")
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        finally:
            if item.log_handle:
                item.log_handle.close()

    def run(self) -> int:
        for item in self.items.values():
            self.start_one(item)

        while self.running:
            time.sleep(1.0)
            for item in self.items.values():
                proc = item.process
                if proc is None:
                    continue
                code = proc.poll()
                if code is None:
                    continue

                should_restart = (
                    item.restart == "always"
                    or (item.restart == "on-failure" and code != 0)
                )
                if should_restart and self.running:
                    item.restarts += 1
                    backoff = min(30.0, 2.0 ** min(item.restarts, 5))
                    print(
                        f"[supervisor] {item.name} exited with {code}; "
                        f"restart #{item.restarts} in {backoff:.1f}s"
                    )
                    time.sleep(backoff)
                    self.start_one(item)
                elif item.required and self.running:
                    print(f"[supervisor] required module {item.name} stopped")
                    return 2
        return 0

    def shutdown(self) -> None:
        self.running = False
        for item in reversed(list(self.items.values())):
            self.stop_one(item)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", default="navigation")
    parser.add_argument("--config", default="config/modules.yaml")
    args = parser.parse_args()

    root = Path(os.environ.get("SMART_CANE_ROOT", Path(__file__).resolve().parents[1]))
    supervisor = Supervisor(root, root / args.config, args.profile)

    def handle_signal(_signum, _frame):
        supervisor.shutdown()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        return supervisor.run()
    finally:
        supervisor.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
