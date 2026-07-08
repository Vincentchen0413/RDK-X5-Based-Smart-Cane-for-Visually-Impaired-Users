from __future__ import annotations

import shutil
from pathlib import Path

from common import DATASET_ROOT

INPUT_ROOT = Path("/kaggle/input")

DATA_YAML = """path: /kaggle/working/dataset

train: images/train
val: images/val
test: images/test

names:
  0: red_signal
  1: yellow_signal
  2: green_signal
  3: puddle
  4: closed_manhole
  5: ban
  6: slippery
  7: walk
"""


def is_complete_dataset(root: Path) -> bool:
    required = [
        root / "images/train",
        root / "images/val",
        root / "images/test",
        root / "labels/train",
        root / "labels/val",
        root / "labels/test",
    ]
    return all(path.is_dir() for path in required)


candidates = [p.parent for p in INPUT_ROOT.rglob("data.yaml") if is_complete_dataset(p.parent)]
print("找到的数据集：")
for candidate in candidates:
    print(candidate)

if not candidates:
    raise FileNotFoundError("没有找到包含 train/val/test 图片和标签的完整 YOLO 数据集")
if len(candidates) > 1:
    raise RuntimeError(f"找到多个候选数据集，请只挂载一个完整数据集：{candidates}")

source_root = candidates[0]
if DATASET_ROOT.exists():
    shutil.rmtree(DATASET_ROOT)
shutil.copytree(source_root, DATASET_ROOT)
(DATASET_ROOT / "data.yaml").write_text(DATA_YAML, encoding="utf-8")

print("源目录：", source_root)
print("目标目录：", DATASET_ROOT)
print((DATASET_ROOT / "data.yaml").read_text(encoding="utf-8"))
