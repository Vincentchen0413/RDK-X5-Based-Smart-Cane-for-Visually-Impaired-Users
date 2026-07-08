from ultralytics import YOLO

model = YOLO("yolov8n.pt")
model.train(
    data="/kaggle/working/dataset/data.yaml",
    epochs=120,
    imgsz=640,
    batch=16,
    device=0,
    workers=2,
    patience=30,
    cos_lr=True,
    close_mosaic=10,
    amp=True,
    seed=42,
    deterministic=True,
    project="/kaggle/working/runs",
    name="smart_cane_yolov8n_v1",
    save=True,
    plots=True,
)
