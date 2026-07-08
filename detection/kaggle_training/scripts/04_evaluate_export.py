from __future__ import annotations

import shutil
from pathlib import Path

from ultralytics import YOLO

from common import BEST_PT

if not BEST_PT.exists():
    raise FileNotFoundError(f"未找到训练权重：{BEST_PT}")

best = YOLO(str(BEST_PT))

best.val(
    data="/kaggle/working/dataset/data.yaml",
    split="val",
    imgsz=640,
    batch=16,
    device=0,
    workers=2,
    plots=True,
)

best.val(
    data="/kaggle/working/dataset/data.yaml",
    split="test",
    imgsz=640,
    batch=16,
    device=0,
    workers=2,
    plots=True,
    project="/kaggle/working/evaluation",
    name="smart_cane_test_v1",
)

best.predict(
    source="/kaggle/working/dataset/images/test",
    imgsz=640,
    conf=0.25,
    iou=0.7,
    device=0,
    save=True,
    save_txt=False,
    project="/kaggle/working/predictions",
    name="smart_cane_test_predictions",
)

best_output = Path("/kaggle/working/smart_cane_yolov8n_v1_best.pt")
shutil.copy2(BEST_PT, best_output)

# 这是普通 Ultralytics ONNX，仅用于 PC 端验证；RDK 六输出 ONNX 请使用 rdk_conversion 包。
exported = Path(
    best.export(format="onnx", imgsz=640, opset=11, simplify=True, dynamic=False, nms=False)
)
onnx_output = Path("/kaggle/working/smart_cane_yolov8n_v1.onnx")
if exported.resolve() != onnx_output.resolve():
    shutil.copy2(exported, onnx_output)

print("PT：", best_output)
print("普通 ONNX：", onnx_output)
