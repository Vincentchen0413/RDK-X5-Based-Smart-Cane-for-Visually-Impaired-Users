from __future__ import annotations

from collections import Counter

from common import CLASS_NAMES, DATASET_ROOT, IMAGE_EXTENSIONS

errors: list[str] = []

for split in ("train", "val", "test"):
    image_dir = DATASET_ROOT / "images" / split
    label_dir = DATASET_ROOT / "labels" / split
    if not image_dir.is_dir() or not label_dir.is_dir():
        errors.append(f"{split}: 缺少图片或标签目录")
        continue

    images = sorted(p for p in image_dir.iterdir() if p.is_file() and p.suffix.lower() in IMAGE_EXTENSIONS)
    labels = sorted(
        p for p in label_dir.iterdir()
        if p.is_file() and p.suffix.lower() == ".txt" and p.name.lower() != "classes.txt"
    )
    image_stems = {p.stem for p in images}
    label_stems = {p.stem for p in labels}
    missing_labels = image_stems - label_stems
    orphan_labels = label_stems - image_stems
    class_counter: Counter[int] = Counter()
    empty_labels = 0

    for label_path in labels:
        text = label_path.read_text(encoding="utf-8", errors="ignore").strip()
        if not text:
            empty_labels += 1
            continue
        for line_no, line in enumerate(text.splitlines(), 1):
            parts = line.split()
            if len(parts) != 5:
                errors.append(f"{split}/{label_path.name}:{line_no} 不是 5 列")
                continue
            try:
                class_id = int(parts[0])
                xc, yc, width, height = map(float, parts[1:])
            except ValueError:
                errors.append(f"{split}/{label_path.name}:{line_no} 无法解析")
                continue
            if class_id not in CLASS_NAMES:
                errors.append(f"{split}/{label_path.name}:{line_no} 非法 class_id={class_id}")
            if not all(0.0 <= v <= 1.0 for v in (xc, yc, width, height)):
                errors.append(f"{split}/{label_path.name}:{line_no} 坐标不在 0~1")
            if width <= 0 or height <= 0:
                errors.append(f"{split}/{label_path.name}:{line_no} 宽高小于等于 0")
            x1, y1 = xc - width / 2, yc - height / 2
            x2, y2 = xc + width / 2, yc + height / 2
            if x1 < -1e-6 or y1 < -1e-6 or x2 > 1 + 1e-6 or y2 > 1 + 1e-6:
                errors.append(f"{split}/{label_path.name}:{line_no} 检测框越界")
            class_counter[class_id] += 1

    print(f"\n========== {split} ==========")
    print("图片数量：", len(images))
    print("标签数量：", len(labels))
    print("空标签数量：", empty_labels)
    print("缺少标签的图片：", len(missing_labels))
    print("缺少图片的标签：", len(orphan_labels))
    for class_id, class_name in CLASS_NAMES.items():
        print(class_id, class_name, class_counter[class_id])
    if missing_labels:
        errors.append(f"{split} 有 {len(missing_labels)} 张图片缺少标签")
    if orphan_labels:
        errors.append(f"{split} 有 {len(orphan_labels)} 个标签缺少图片")

print("\n========== 检查结果 ==========")
if errors:
    for error in errors[:200]:
        print("ERROR:", error)
    raise SystemExit(f"数据集检查失败，共发现 {len(errors)} 个问题")
print("数据集检查通过")
