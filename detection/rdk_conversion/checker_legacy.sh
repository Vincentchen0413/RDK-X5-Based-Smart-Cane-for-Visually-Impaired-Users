#!/usr/bin/env bash
set -euo pipefail

ONNX_PATH="${1:-models/smart_cane_yolov8n_v1_rdk.onnx}"
INPUT_NAME="${2:-images}"
INPUT_SHAPE="${3:-1x3x640x640}"

if [[ ! -f "$ONNX_PATH" ]]; then
  echo "ONNX 不存在: $ONNX_PATH" >&2
  exit 1
fi

hb_mapper checker \
  --model-type onnx \
  --model "$(realpath "$ONNX_PATH")" \
  --march bayes-e \
  --input-shape "$INPUT_NAME $INPUT_SHAPE" \
  2>&1 | tee output/hb_mapper_checker_legacy.log
