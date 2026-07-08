#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx", required=True)
    parser.add_argument("--num-classes", type=int, default=8)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    path = Path(args.onnx).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(path)

    model = onnx.load(str(path))
    onnx.checker.check_model(model)
    session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    inputs = session.get_inputs()
    outputs = session.get_outputs()
    if len(inputs) != 1:
        raise RuntimeError(f"期望单输入，实际 {len(inputs)} 个")
    if len(outputs) != 6:
        raise RuntimeError(f"期望六输出，实际 {len(outputs)} 个")

    inp = inputs[0]
    if len(inp.shape) != 4 or not all(isinstance(v, int) for v in inp.shape):
        raise RuntimeError(f"输入必须为固定 NCHW，实际：{inp.shape}")
    n, c, h, w = inp.shape
    if n != 1 or c != 3:
        raise RuntimeError(f"输入应为 [1,3,H,W]，实际：{inp.shape}")

    print("ONNX：", path)
    print("input:", inp.name, inp.shape, inp.type)
    dummy = np.zeros((1, 3, h, w), dtype=np.float32)
    values = session.run(None, {inp.name: dummy})

    for i, (meta, value) in enumerate(zip(outputs, values)):
        print(f"output[{i}] {meta.name}: shape={value.shape}, dtype={value.dtype}")
        if value.ndim != 4 or value.shape[0] != 1:
            raise RuntimeError(f"output[{i}] 不是 4D NHWC")
        expected_stride = (8, 8, 16, 16, 32, 32)[i]
        expected_hw = (h // expected_stride, w // expected_stride)
        if value.shape[1:3] != expected_hw:
            raise RuntimeError(
                f"output[{i}] 空间尺寸 {value.shape[1:3]} 与 stride {expected_stride} 不匹配，"
                f"期望 {expected_hw}"
            )
        if i % 2 == 0 and value.shape[-1] != args.num_classes:
            raise RuntimeError(
                f"output[{i}] 分类通道应为 {args.num_classes}，实际 {value.shape[-1]}"
            )
        if i % 2 == 1 and value.shape[-1] % 4 != 0:
            raise RuntimeError(f"output[{i}] 框通道 {value.shape[-1]} 不能被 4 整除")

    print("检查通过：六输出顺序、尺寸和类别数符合 RDK YOLOv8 Detect 协议。")


if __name__ == "__main__":
    main()
