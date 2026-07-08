# 智能导盲杖 YOLOv8n：Kaggle 训练核心代码

本目录由项目实际使用的 `traffic-yolo.ipynb` 整理而成，保留以下流程：

1. 安装并检查训练环境；
2. 从 `/kaggle/input` 自动寻找完整 YOLO 数据集；
3. 复制到 `/kaggle/working/dataset` 并写入最终 8 类 `data.yaml`；
4. 检查图片、标签、类别编号、归一化坐标和越界框；
5. 使用 YOLOv8n 训练、验证、测试和预测；
6. 导出普通 Ultralytics ONNX；
7. 打包训练产物。

## 最终类别顺序

```text
0 red_signal
1 yellow_signal
2 green_signal
3 puddle
4 closed_manhole
5 ban
6 slippery
7 walk
```

类别顺序必须和板端的 `classes.list`、后处理代码保持一致。

## 推荐用法

### 用 Notebook

1. 在 Kaggle Notebook 中添加数据集；
2. 上传本目录，或直接打开 `traffic-yolo-clean.ipynb`；
3. 开启 GPU；
4. 按顺序运行所有单元格。

### 用拆分脚本

```bash
pip install -r requirements.txt
python scripts/01_prepare_dataset.py
python scripts/02_validate_dataset.py
python scripts/03_train.py
python scripts/04_evaluate_export.py
python scripts/05_package_artifacts.py
```

## 重要说明

- `04_evaluate_export.py` 导出的 ONNX 是普通 Ultralytics ONNX，适合 PC 端验证。
- RDK X5 使用的六输出 ONNX 必须使用另一个压缩包 `rdk_conversion` 中的 `export_rdk_onnx.py` 重新导出。
- 大型文件未包含：数据集、`best.pt`、`last.pt`、ONNX、预测图片和训练输出目录。
- 将训练好的权重放入 `artifacts/`，或保持 Kaggle 默认输出路径即可。

## 默认训练参数

- 基础模型：`yolov8n.pt`
- 输入尺寸：640
- epochs：120
- batch：16
- patience：30
- optimizer：由 Ultralytics 自动选择
- seed：42
- 项目目录：`/kaggle/working/runs/smart_cane_yolov8n_v1`
