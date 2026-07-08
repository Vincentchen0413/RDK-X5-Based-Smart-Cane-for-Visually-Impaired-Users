from __future__ import annotations

from pathlib import Path

CLASS_NAMES = {
    0: "red_signal",
    1: "yellow_signal",
    2: "green_signal",
    3: "puddle",
    4: "closed_manhole",
    5: "ban",
    6: "slippery",
    7: "walk",
}

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
DATASET_ROOT = Path("/kaggle/working/dataset")
RUN_DIR = Path("/kaggle/working/runs/smart_cane_yolov8n_v1")
BEST_PT = RUN_DIR / "weights" / "best.pt"
