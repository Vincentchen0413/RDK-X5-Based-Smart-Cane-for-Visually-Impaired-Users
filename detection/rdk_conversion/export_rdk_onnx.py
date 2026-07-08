#!/usr/bin/env python3
"""Export an Ultralytics YOLOv8 Detect model as six NHWC outputs for RDK X5.

Output order: [cls_s8, box_s8, cls_s16, box_s16, cls_s32, box_s32].
The patch follows the output protocol used by D-Robotics RDK Model Zoo.
"""
from __future__ import annotations

import argparse
import shutil
import types
from pathlib import Path

import onnx
from ultralytics import YOLO
from ultralytics.nn.modules.head import Detect


def rdk_detect_forward(self: Detect, x):
    outputs = []
    for i in range(self.nl):
        # Classification logits and DFL box distributions are exported separately.
        # NHWC avoids extra transpose work in the RDK runtime post-process.
        outputs.append(self.cv3[i](x[i]).permute(0, 2, 3, 1).contiguous())
        outputs.append(self.cv2[i](x[i]).permute(0, 2, 3, 1).contiguous())
    return outputs


def patch_detect_heads(model) -> int:
    count = 0
    for module in model.modules():
        if isinstance(module, Detect):
            module.forward = types.MethodType(rdk_detect_forward, module)
            count += 1
    return count


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pt", required=True, help="Path to trained YOLOv8 .pt")
    parser.add_argument("--output", required=True, help="Final RDK six-output ONNX path")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--opset", type=int, default=11)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    pt_path = Path(args.pt).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    if not pt_path.is_file():
        raise FileNotFoundError(pt_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    yolo = YOLO(str(pt_path))
    patched = patch_detect_heads(yolo.model)
    if patched != 1:
        raise RuntimeError(f"期望找到 1 个 Detect 头，实际找到 {patched} 个")

    exported = Path(
        yolo.export(
            format="onnx",
            imgsz=args.imgsz,
            opset=args.opset,
            simplify=False,
            dynamic=False,
            nms=False,
        )
    ).resolve()

    if exported != output_path:
        shutil.copy2(exported, output_path)

    model = onnx.load(str(output_path))
    onnx.checker.check_model(model)
    if len(model.graph.output) != 6:
        raise RuntimeError(
            f"导出结果不是 6 输出，而是 {len(model.graph.output)} 输出；"
            "请确认模型为 YOLOv8 Detect，并使用 ultralytics==8.4.66。"
        )

    print("六输出 RDK ONNX 已生成：", output_path)
    for index, output in enumerate(model.graph.output):
        print(f"output[{index}] name={output.name}")


if __name__ == "__main__":
    main()
