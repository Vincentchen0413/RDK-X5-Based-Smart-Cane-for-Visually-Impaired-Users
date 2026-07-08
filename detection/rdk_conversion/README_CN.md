# 智能导盲杖 YOLOv8n：RDK X5 六输出 ONNX 与 BIN 转换核心代码

本目录用于把训练得到的 YOLOv8n `best.pt` 转换为适合 RDK X5 的六输出 ONNX，再通过 `hb_mapper` 编译为 BPU `.bin`。

## 固定输出协议

六个输出必须按以下顺序排列：

```text
output[0] = stride 8  分类 logits，NHWC
output[1] = stride 8  DFL 框分布，NHWC
output[2] = stride 16 分类 logits，NHWC
output[3] = stride 16 DFL 框分布，NHWC
output[4] = stride 32 分类 logits，NHWC
output[5] = stride 32 DFL 框分布，NHWC
```

对 640×640、8 类 YOLOv8n，典型形状为：

```text
[1, 80, 80, 8]
[1, 80, 80, 64]
[1, 40, 40, 8]
[1, 40, 40, 64]
[1, 20, 20, 8]
[1, 20, 20, 64]
```

其中 64 = 4 × reg_max，YOLOv8 默认 `reg_max=16`。

## 目录放置

```text
models/
  smart_cane_yolov8n_v1_best.pt       # 自行放入，压缩包未包含
  smart_cane_yolov8n_v1_rdk.onnx      # 导出后生成
calibration_images/
  *.jpg / *.png                       # 自行放入 20~50 张代表性图片
workspace/                             # 自动生成配置与校准二进制
output/                                # 最终 BIN 和日志
```

## 第一步：在 Ultralytics 训练环境导出六输出 ONNX

建议与训练时保持相同版本：

```bash
pip install -r requirements_export.txt
python export_rdk_onnx.py \
  --pt models/smart_cane_yolov8n_v1_best.pt \
  --output models/smart_cane_yolov8n_v1_rdk.onnx \
  --imgsz 640 \
  --opset 11
```

检查输出：

```bash
python inspect_onnx.py \
  --onnx models/smart_cane_yolov8n_v1_rdk.onnx \
  --num-classes 8
```

也可以用一张图片验证六输出后处理：

```bash
python test_six_output_onnx.py \
  --onnx models/smart_cane_yolov8n_v1_rdk.onnx \
  --image test_images/red_signal.jpg \
  --classes classes.list \
  --output output/onnx_result.jpg
```

## 第二步：在 x86 OpenExplore / RDK X5 工具链环境准备量化数据

```bash
pip install -r requirements_mapper.txt
python prepare_conversion.py \
  --onnx models/smart_cane_yolov8n_v1_rdk.onnx \
  --cal-images calibration_images \
  --workspace workspace \
  --output-prefix smart_cane_yolov8n_v1_rdk_bayese_640x640_nv12
```

该脚本会：

- 检查 ONNX 输入尺寸；
- 随机抽取最多 50 张校准图片；
- 按 RGB、NCHW、float32 生成 `.rgbchw`；
- 生成 `workspace/config.yaml`。

## 第三步：编译 BIN

```bash
bash compile_bin.sh workspace/config.yaml output
```

若当前工具链的 `checker` 不支持 `--config`，可使用：

```bash
bash checker_legacy.sh \
  models/smart_cane_yolov8n_v1_rdk.onnx \
  images
```

`images` 是 ONNX 输入节点名；请以 `inspect_onnx.py` 的输出为准。

## 第四步：复制到 RDK X5

把最终 `.bin` 放到：

```text
/home/sunrise/smart_cane_ros/model/
```

并确保板端：

```text
/home/sunrise/smart_cane_ros/config/classes.list
/home/sunrise/smart_cane_ros/config/smart_cane.json
```

与模型类别、六输出顺序、stride `[8,16,32]` 完全一致。

## 重要说明

- 本包不包含 `.pt`、`.onnx`、`.bin`、校准图片等大文件。
- `export_rdk_onnx.py` 专门面向 YOLOv8 Detect；不要用普通 `model.export()` 的单输出 ONNX 代替。
- 校准数据生成方式与官方 RDK Model Zoo 的当前 YOLO 转换示例一致：RGB、直接 resize、NCHW、float32，归一化由 YAML 中 `scale_value=1/255` 完成。
- RDK X5 使用 `march: bayes-e`。
- 官方当前 `rdk_x5` 分支建议 RDK OS ≥ 3.5.0。
