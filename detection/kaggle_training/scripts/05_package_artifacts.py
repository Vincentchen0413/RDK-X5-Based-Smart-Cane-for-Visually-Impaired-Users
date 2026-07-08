from __future__ import annotations

import shutil
from pathlib import Path

package_dir = Path("/kaggle/working/smart_cane_yolov8n_v1_artifacts")
if package_dir.exists():
    shutil.rmtree(package_dir)
package_dir.mkdir(parents=True)

files = [
    Path("/kaggle/working/smart_cane_yolov8n_v1_best.pt"),
    Path("/kaggle/working/smart_cane_yolov8n_v1.onnx"),
    Path("/kaggle/working/runs/smart_cane_yolov8n_v1/results.png"),
    Path("/kaggle/working/runs/smart_cane_yolov8n_v1/results.csv"),
    Path("/kaggle/working/runs/smart_cane_yolov8n_v1/confusion_matrix.png"),
    Path("/kaggle/working/runs/smart_cane_yolov8n_v1/confusion_matrix_normalized.png"),
    Path("/kaggle/working/dataset/data.yaml"),
]

for source in files:
    if source.exists():
        shutil.copy2(source, package_dir / source.name)
        print("已复制：", source.name)
    else:
        print("未找到，跳过：", source)

zip_path = shutil.make_archive(str(package_dir), "zip", package_dir)
print("结果压缩包：", zip_path)
