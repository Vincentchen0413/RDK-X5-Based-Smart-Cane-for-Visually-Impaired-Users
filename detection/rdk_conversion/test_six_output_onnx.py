#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

STRIDES = (8, 16, 32)


def sigmoid(x: np.ndarray) -> np.ndarray:
    x = np.clip(x, -60.0, 60.0)
    return 1.0 / (1.0 + np.exp(-x))


def softmax(x: np.ndarray, axis: int = -1) -> np.ndarray:
    x = x - np.max(x, axis=axis, keepdims=True)
    exp_x = np.exp(x)
    return exp_x / np.sum(exp_x, axis=axis, keepdims=True)


def letterbox(image: np.ndarray, size: tuple[int, int]) -> tuple[np.ndarray, float, tuple[int, int]]:
    dst_w, dst_h = size
    src_h, src_w = image.shape[:2]
    scale = min(dst_w / src_w, dst_h / src_h)
    new_w, new_h = int(round(src_w * scale)), int(round(src_h * scale))
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    pad_w, pad_h = dst_w - new_w, dst_h - new_h
    left, top = pad_w // 2, pad_h // 2
    right, bottom = pad_w - left, pad_h - top
    padded = cv2.copyMakeBorder(
        resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114)
    )
    return padded, scale, (left, top)


def decode_pair(cls_logits: np.ndarray, box_logits: np.ndarray, stride: int, conf: float):
    cls_logits = cls_logits[0]
    box_logits = box_logits[0]
    h, w, num_classes = cls_logits.shape
    reg_max = box_logits.shape[-1] // 4
    scores_all = sigmoid(cls_logits)
    class_ids = np.argmax(scores_all, axis=-1)
    scores = np.max(scores_all, axis=-1)
    keep = scores >= conf
    if not np.any(keep):
        return np.empty((0, 4), np.float32), np.empty(0, np.float32), np.empty(0, np.int32)

    ys, xs = np.nonzero(keep)
    selected_box = box_logits[ys, xs].reshape(-1, 4, reg_max)
    prob = softmax(selected_box, axis=-1)
    bins = np.arange(reg_max, dtype=np.float32)
    dist = np.sum(prob * bins, axis=-1) * float(stride)

    center_x = (xs.astype(np.float32) + 0.5) * stride
    center_y = (ys.astype(np.float32) + 0.5) * stride
    boxes = np.stack(
        [
            center_x - dist[:, 0],
            center_y - dist[:, 1],
            center_x + dist[:, 2],
            center_y + dist[:, 3],
        ],
        axis=1,
    )
    return boxes, scores[ys, xs].astype(np.float32), class_ids[ys, xs].astype(np.int32)


def nms(boxes: np.ndarray, scores: np.ndarray, class_ids: np.ndarray, iou: float):
    kept: list[int] = []
    for class_id in np.unique(class_ids):
        idx = np.where(class_ids == class_id)[0]
        b = boxes[idx]
        xywh = np.column_stack([b[:, 0], b[:, 1], b[:, 2] - b[:, 0], b[:, 3] - b[:, 1]])
        selected = cv2.dnn.NMSBoxes(xywh.tolist(), scores[idx].tolist(), 0.0, iou)
        if len(selected):
            kept.extend(idx[np.asarray(selected).reshape(-1)].tolist())
    return np.asarray(sorted(kept, key=lambda i: scores[i], reverse=True), dtype=np.int32)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx", required=True)
    parser.add_argument("--image", required=True)
    parser.add_argument("--classes", default="classes.list")
    parser.add_argument("--output", default="output/onnx_result.jpg")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    names = [line.strip() for line in Path(args.classes).read_text(encoding="utf-8").splitlines() if line.strip()]
    image = cv2.imread(args.image)
    if image is None:
        raise FileNotFoundError(args.image)

    session = ort.InferenceSession(args.onnx, providers=["CPUExecutionProvider"])
    inp = session.get_inputs()[0]
    _, _, h, w = inp.shape
    prepared, scale, (pad_x, pad_y) = letterbox(image, (w, h))
    rgb = cv2.cvtColor(prepared, cv2.COLOR_BGR2RGB)
    tensor = np.transpose(rgb, (2, 0, 1))[None].astype(np.float32) / 255.0
    outputs = session.run(None, {inp.name: tensor})
    if len(outputs) != 6:
        raise RuntimeError(f"期望 6 输出，实际 {len(outputs)}")

    box_parts, score_parts, class_parts = [], [], []
    for level, stride in enumerate(STRIDES):
        boxes, scores, class_ids = decode_pair(outputs[level * 2], outputs[level * 2 + 1], stride, args.conf)
        box_parts.append(boxes)
        score_parts.append(scores)
        class_parts.append(class_ids)

    boxes = np.concatenate(box_parts, axis=0)
    scores = np.concatenate(score_parts, axis=0)
    class_ids = np.concatenate(class_parts, axis=0)
    if boxes.size:
        keep = nms(boxes, scores, class_ids, args.iou)
        boxes, scores, class_ids = boxes[keep], scores[keep], class_ids[keep]

    out = image.copy()
    src_h, src_w = image.shape[:2]
    for box, score, class_id in zip(boxes, scores, class_ids):
        x1 = int(np.clip((box[0] - pad_x) / scale, 0, src_w - 1))
        y1 = int(np.clip((box[1] - pad_y) / scale, 0, src_h - 1))
        x2 = int(np.clip((box[2] - pad_x) / scale, 0, src_w - 1))
        y2 = int(np.clip((box[3] - pad_y) / scale, 0, src_h - 1))
        label = names[class_id] if 0 <= class_id < len(names) else str(class_id)
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(out, f"{label} {score:.2f}", (x1, max(20, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_path), out):
        raise RuntimeError(f"保存失败：{output_path}")
    print("检测数量：", len(boxes))
    print("结果：", output_path.resolve())


if __name__ == "__main__":
    main()
