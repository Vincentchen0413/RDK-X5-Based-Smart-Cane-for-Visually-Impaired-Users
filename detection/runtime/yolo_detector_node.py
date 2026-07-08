#!/usr/bin/env python3
"""RDK X5 object detection runtime node skeleton.

Replace CameraBackend and BpuYoloBackend with the actual GS130WI camera driver,
Horizon hbDNN/BPU runtime, preprocessing, postprocessing and stereo distance
estimation used by the project.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Iterable, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class Detection:
    class_name: str
    confidence: float
    bbox_xyxy: List[float]
    distance_m: float = -1.0


class CameraBackend:
    def open(self) -> None:
        """Open camera or ROS image subscription."""
        pass

    def read(self):
        """Return one frame or None."""
        return None


class BpuYoloBackend:
    CLASSES = [
        "traffic_light",
        "puddle",
        "manhole_cover",
        "slippery_area",
        "pedestrian_prohibited",
        "crosswalk",
    ]

    def __init__(self, model_path: str):
        self.model_path = model_path
        # TODO: load .bin model with Horizon runtime.

    def infer(self, frame) -> Iterable[Detection]:
        # TODO:
        # 1. resize/letterbox and normalize;
        # 2. execute BPU inference;
        # 3. decode six output tensors;
        # 4. confidence filtering and NMS;
        # 5. use stereo depth/point cloud to estimate distance.
        return []


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_detector_node")
        self.declare_parameter("model_path", "detection/rdk_conversion/models/model.bin")
        self.declare_parameter("confidence_threshold", 0.45)
        self.model = BpuYoloBackend(
            str(self.get_parameter("model_path").value)
        )
        self.camera = CameraBackend()
        self.camera.open()
        self.pub = self.create_publisher(
            String, "/smart_cane/perception/event", 20
        )
        self.timer = self.create_timer(0.05, self.tick)

    def tick(self):
        frame = self.camera.read()
        if frame is None:
            return

        threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        for det in self.model.infer(frame):
            if det.confidence < threshold:
                continue
            payload = {
                "timestamp_unix": time.time(),
                "class_name": det.class_name,
                "confidence": det.confidence,
                "bbox_xyxy": det.bbox_xyxy,
                "distance_m": det.distance_m,
                "source": "rdk_bpu_yolo",
            }
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self.pub.publish(msg)


def main():
    rclpy.init()
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
