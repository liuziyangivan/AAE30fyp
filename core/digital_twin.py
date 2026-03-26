"""
core/digital_twin.py
数字孪生核心：维护飞行器实时状态，驱动气动计算，发布事件
"""

from __future__ import annotations
import time
from dataclasses import dataclass, field
from core.vehicle import VehicleConfig
from core.aero_engine import AeroEngine, VehiclePerformance
from core.event_bus import EventBus


# ── 飞行器实时状态快照 ────────────────────────────────────
@dataclass
class TwinState:
    timestamp:   float   # 系统时间戳 (s)
    rpm:         float   # 当前转速指令
    performance: VehiclePerformance   # 气动计算结果

    # 位置与速度（仿真/传感器输入，默认悬停）
    position_m:  list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    velocity_ms: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

    @property
    def altitude_m(self) -> float:
        return self.position_m[2]

    @property
    def is_airborne(self) -> bool:
        return self.altitude_m > 0.05   # > 5 cm 视为离地


# ── 数字孪生主类 ──────────────────────────────────────────
class DigitalTwin:
    """
    用法：
        twin = DigitalTwin(vehicle, bus)
        twin.set_rpm(4500)          # 更新转速 → 触发计算 → 发布事件
        state = twin.state          # 获取最新状态
    """

    # 发布的事件名称常量
    EVT_STATE_UPDATED  = "state_updated"    # 每次 set_rpm 后触发
    EVT_HOVER_ACHIEVED = "hover_achieved"   # 推力首次 ≥ 重力时触发
    EVT_BELOW_HOVER    = "below_hover"      # 推力跌破重力时触发

    def __init__(self, vehicle: VehicleConfig, bus: EventBus):
        self._vehicle = vehicle
        self._engine  = AeroEngine(vehicle)
        self._bus     = bus
        self._state:  TwinState | None = None
        self._was_hovering = False

        # 初始化：以 0 RPM 建立第一个状态
        self.set_rpm(0.0)

    # ── 主更新接口 ────────────────────────────────────────
    def set_rpm(self, rpm: float) -> TwinState:
        """
        设置转速，重新计算气动性能，更新状态，发布事件。
        返回最新的 TwinState。
        """
        rpm = max(0.0, min(rpm, self._vehicle.rotors.rpm_max))
        perf = self._engine.compute(rpm)

        self._state = TwinState(
            timestamp=time.time(),
            rpm=rpm,
            performance=perf,
        )

        # 发布通用状态更新事件
        self._bus.publish(
            self.EVT_STATE_UPDATED,
            state=self._state,
        )

        # 悬停状态切换事件
        hovering_now = perf.can_hover
        if hovering_now and not self._was_hovering:
            self._bus.publish(self.EVT_HOVER_ACHIEVED, state=self._state)
        elif not hovering_now and self._was_hovering:
            self._bus.publish(self.EVT_BELOW_HOVER, state=self._state)
        self._was_hovering = hovering_now

        return self._state

    # ── 传感器注入接口（供仿真模块调用）─────────────────
    def inject_position(self, x: float, y: float, z: float) -> None:
        """直接写入位置（来自仿真器或真实传感器）"""
        if self._state:
            self._state.position_m = [x, y, z]
            self._bus.publish(self.EVT_STATE_UPDATED, state=self._state)

    # ── 属性 ─────────────────────────────────────────────
    @property
    def state(self) -> TwinState:
        return self._state

    @property
    def vehicle(self) -> VehicleConfig:
        return self._vehicle

    @property
    def hover_rpm(self) -> float:
        """悬停所需转速（从最新性能结果读取）"""
        return self._state.performance.hover_rpm