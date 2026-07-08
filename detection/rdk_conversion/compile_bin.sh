#!/usr/bin/env bash
set -euo pipefail

CONFIG_PATH="${1:-workspace/config.yaml}"
OUTPUT_DIR="${2:-output}"

if [[ ! -f "$CONFIG_PATH" ]]; then
  echo "配置文件不存在: $CONFIG_PATH" >&2
  exit 1
fi
if ! command -v hb_mapper >/dev/null 2>&1; then
  echo "未找到 hb_mapper。请在 RDK X5 OpenExplore 工具链环境中运行。" >&2
  exit 1
fi

CONFIG_PATH="$(realpath "$CONFIG_PATH")"
mkdir -p "$OUTPUT_DIR"
OUTPUT_DIR="$(realpath "$OUTPUT_DIR")"
WORKSPACE_DIR="$(dirname "$CONFIG_PATH")"

(
  cd "$WORKSPACE_DIR"
  hb_mapper checker --model-type onnx --config "$CONFIG_PATH" 2>&1 | tee "$OUTPUT_DIR/hb_mapper_checker.log"
  hb_mapper makertbin --model-type onnx --config "$CONFIG_PATH" 2>&1 | tee "$OUTPUT_DIR/hb_mapper_makertbin.log"
)

find "$WORKSPACE_DIR" -type f -name '*.bin' -print -exec cp -f {} "$OUTPUT_DIR/" \;
echo "BIN 和日志已收集到: $OUTPUT_DIR"
