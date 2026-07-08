#!/usr/bin/env python3
from __future__ import annotations

import argparse
import random
import shutil
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx", required=True)
    parser.add_argument("--cal-images", required=True)
    parser.add_argument("--workspace", default="workspace")
    parser.add_argument("--output-prefix", default="smart_cane_yolov8n_v1_rdk_bayese_640x640_nv12")
    parser.add_argument("--sample-num", type=int, default=50)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--optimize-level", choices=["O0", "O1", "O2", "O3"], default="O3")
    parser.add_argument("--keep-workspace", action="store_true", help="不清理已有 workspace")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    onnx_path = Path(args.onnx).expanduser().resolve()
    image_root = Path(args.cal_images).expanduser().resolve()
    workspace = Path(args.workspace).expanduser().resolve()
    if not onnx_path.is_file():
        raise FileNotFoundError(onnx_path)
    if not image_root.is_dir():
        raise NotADirectoryError(image_root)

    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    inputs = session.get_inputs()
    if len(inputs) != 1:
        raise RuntimeError(f"期望单输入 ONNX，实际 {len(inputs)} 个")
    input_meta = inputs[0]
    if len(input_meta.shape) != 4 or not all(isinstance(v, int) for v in input_meta.shape):
        raise RuntimeError(f"ONNX 输入必须是固定 NCHW，实际：{input_meta.shape}")
    n, c, height, width = input_meta.shape
    if (n, c) != (1, 3):
        raise RuntimeError(f"ONNX 输入应为 [1,3,H,W]，实际：{input_meta.shape}")

    images = sorted(
        p for p in image_root.rglob("*")
        if p.is_file() and p.suffix.lower() in IMAGE_EXTENSIONS
    )
    if not images:
        raise FileNotFoundError(f"校准目录中没有图片：{image_root}")
    if args.sample_num <= 0:
        raise ValueError("--sample-num 必须大于 0")
    if len(images) > args.sample_num:
        random.Random(args.seed).shuffle(images)
        images = images[: args.sample_num]
    if len(images) < 20:
        print(f"警告：只有 {len(images)} 张校准图片，建议准备 20~50 张。")

    if workspace.exists() and not args.keep_workspace:
        shutil.rmtree(workspace)
    cal_dir = workspace / "calibration_data_rgb_f32"
    bpu_dir = workspace / "bpu_model_output"
    cal_dir.mkdir(parents=True, exist_ok=True)
    bpu_dir.mkdir(parents=True, exist_ok=True)

    written = 0
    for index, image_path in enumerate(images):
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            print("跳过无法读取的图片：", image_path)
            continue
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (width, height), interpolation=cv2.INTER_LINEAR)
        tensor = np.transpose(rgb, (2, 0, 1))[None].astype(np.float32)
        out_path = cal_dir / f"{index:04d}_{image_path.stem}.rgbchw"
        tensor.tofile(out_path)
        written += 1
    if written == 0:
        raise RuntimeError("没有成功生成任何校准数据")

    config = f"""model_parameters:
  onnx_model: '{onnx_path}'
  march: 'bayes-e'
  layer_out_dump: False
  working_dir: '{bpu_dir}'
  output_model_file_prefix: '{args.output_prefix}'

input_parameters:
  input_name: ''
  input_type_rt: 'nv12'
  input_type_train: 'rgb'
  input_layout_train: 'NCHW'
  norm_type: 'data_scale'
  scale_value: 0.003921568627451

calibration_parameters:
  cal_data_dir: '{cal_dir}'
  cal_data_type: 'float32'
  calibration_type: 'default'
  optimization: 'set_Softmax_input_int8,set_Softmax_output_int8'

compiler_parameters:
  jobs: {args.jobs}
  compile_mode: 'latency'
  debug: true
  optimize_level: '{args.optimize_level}'
"""
    config_path = workspace / "config.yaml"
    config_path.write_text(config, encoding="utf-8")

    print("ONNX 输入：", input_meta.name, input_meta.shape)
    print("校准图片：", written)
    print("校准数据目录：", cal_dir)
    print("配置文件：", config_path)
    print("下一步：bash compile_bin.sh", config_path, "output")


if __name__ == "__main__":
    main()
