#!/usr/bin/env bash
set -euo pipefail

PT_PATH="${1:-models/smart_cane_yolov8n_v1_best.pt}"
ONNX_PATH="${2:-models/smart_cane_yolov8n_v1_rdk.onnx}"
CAL_DIR="${3:-calibration_images}"

python3 export_rdk_onnx.py --pt "$PT_PATH" --output "$ONNX_PATH" --imgsz 640 --opset 11
python3 inspect_onnx.py --onnx "$ONNX_PATH" --num-classes 8
python3 prepare_conversion.py \
  --onnx "$ONNX_PATH" \
  --cal-images "$CAL_DIR" \
  --workspace workspace \
  --output-prefix smart_cane_yolov8n_v1_rdk_bayese_640x640_nv12
bash compile_bin.sh workspace/config.yaml output
