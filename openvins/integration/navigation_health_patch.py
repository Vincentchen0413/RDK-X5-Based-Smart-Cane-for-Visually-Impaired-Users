#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
把本类中的状态判断思路合并进navigate_to_landmark_realtime.py。

重点：
1. 不因单次TF失败立即播报定位失效；
2. 区分IMU、相机、OpenVINS、RTAB-Map和最终全局TF；
3. 失效和恢复都需要持续确认。
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class LocalizationInputs:
    last_imu_time: float = 0.0
    last_camera_time: float = 0.0
    last_openvins_odom_time: float = 0.0
    last_rtabmap_info_time: float = 0.0
    openvins_healthy: bool = False
    map_to_base_available: bool = False


class LocalizationHealthGate:
    def __init__(
        self,
        lost_confirm_seconds: float = 2.0,
        recover_confirm_seconds: float = 2.0,
    ) -> None:
        self.lost_confirm_seconds = lost_confirm_seconds
        self.recover_confirm_seconds = recover_confirm_seconds

        self.bad_since: Optional[float] = None
        self.good_since: Optional[float] = None
        self.state = "unknown"

    @staticmethod
    def check_raw(
        data: LocalizationInputs,
        now: Optional[float] = None,
    ) -> Tuple[bool, str]:
        now = time.monotonic() if now is None else now

        if data.last_imu_time <= 0.0 or now - data.last_imu_time > 0.5:
            return False, "IMU数据中断"

        if data.last_camera_time <= 0.0 or now - data.last_camera_time > 1.0:
            return False, "相机数据中断"

        if not data.openvins_healthy:
            return False, "视觉惯性里程计失效"

        if (
            data.last_openvins_odom_time <= 0.0
            or now - data.last_openvins_odom_time > 1.0
        ):
            return False, "OpenVINS里程计过期"

        if (
            data.last_rtabmap_info_time <= 0.0
            or now - data.last_rtabmap_info_time > 5.0
        ):
            return False, "RTAB-Map状态过期"

        if not data.map_to_base_available:
            return False, "全局位姿不可用"

        return True, "定位正常"

    def update(
        self,
        raw_healthy: bool,
        now: Optional[float] = None,
    ) -> Optional[str]:
        """
        返回状态事件：
            "lost"
            "recovered"
            None
        """
        now = time.monotonic() if now is None else now

        if raw_healthy:
            self.bad_since = None

            if self.good_since is None:
                self.good_since = now

            if (
                self.state != "healthy"
                and now - self.good_since >= self.recover_confirm_seconds
            ):
                previous = self.state
                self.state = "healthy"
                return "recovered" if previous == "lost" else None

            return None

        self.good_since = None

        if self.bad_since is None:
            self.bad_since = now

        if (
            self.state != "lost"
            and now - self.bad_since >= self.lost_confirm_seconds
        ):
            self.state = "lost"
            return "lost"

        return None


# 在实时导航循环中可按以下方式使用：
#
# raw_ok, reason = gate.check_raw(inputs)
# event = gate.update(raw_ok)
#
# if event == "lost":
#     prompt("视觉定位暂时失效，请停止前进并缓慢转动盲杖。")
#
# elif event == "recovered":
#     prompt("视觉定位已恢复，继续导航。")
#
# if gate.state == "lost":
#     continue
#
# 这样不会因一帧TF查询失败反复播报失效和恢复。
