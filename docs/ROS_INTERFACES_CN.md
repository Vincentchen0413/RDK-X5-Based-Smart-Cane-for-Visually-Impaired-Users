# ROS 接口约定

原型阶段使用 `std_msgs/String` 和 JSON，便于快速连接已有脚本。正式版本建议使用 `interfaces/msg`。

## `/smart_cane/voice/intent`

```json
{
  "intent": "navigate",
  "slots": {"place": "电梯"},
  "confidence": 0.93
}
```

## `/smart_cane/system/request`

```json
{
  "source": "voice",
  "action": "mark_landmark",
  "place": "洗手间"
}
```

可用 action：

- `start_mapping`
- `stop_mapping`
- `mark_landmark`
- `navigate`
- `cancel_navigation`
- `query_time`
- `query_date`
- `query_weather`
- `system_status`

## `/smart_cane/perception/event`

```json
{
  "timestamp_unix": 1783526400.0,
  "class_name": "puddle",
  "confidence": 0.88,
  "bbox_xyxy": [120, 200, 480, 620],
  "distance_m": 1.7,
  "source": "rdk_bpu_yolo"
}
```

## `/smart_cane/navigation/instruction`

```json
{
  "kind": "turn_left",
  "text": "前方两米左转",
  "distance_to_action_m": 2.0
}
```

## `/smart_cane/fall/event`

```json
{
  "event_type": "fall",
  "timestamp_unix": 1783526400.0,
  "confidence": 0.91,
  "acc_peak_g": 2.8,
  "posture": "lying",
  "source": "imu_fall_detector"
}
```

## `/smart_cane/location/current`

```json
{
  "timestamp_unix": 1783526400.0,
  "frame_id": "map",
  "x": 4.2,
  "y": 1.7,
  "z": 0.0,
  "yaw": 1.57,
  "source": "fused_odom"
}
```

## `/smart_cane/audio/request`

```json
{
  "source": "safety_router",
  "text": "前方可能有水洼，请绕行",
  "priority": 80,
  "dedupe_key": "puddle",
  "cooldown_sec": 4,
  "interruptible": true
}
```

## `/smart_cane/voice/speak`

音频管理器输出给实际 TTS 节点：

```json
{
  "text": "前方两米左转",
  "priority": 60,
  "interruptible": true
}
```

## TF 建议

```text
map -> odom -> base_link -> camera_link
                         -> imu_link
                         -> left_camera_optical_frame
                         -> right_camera_optical_frame
```

OpenVINS 主要提供局部连续里程计，RTAB-Map 通过回环对 `map -> odom` 进行全局修正。不要让多个节点同时发布同一 TF。
