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
    timestamp:   float
    rpm:         float
    performance: VehiclePerformance

    position_m:  list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    velocity_ms: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpms:        list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])  

    @property
    def altitude_m(self) -> float:
        return self.position_m[2]

    @property
    def is_airborne(self) -> bool:
        return self.altitude_m > 0.05


# ── 数字孪生主类 ──────────────────────────────────────────
class DigitalTwin:

    EVT_STATE_UPDATED  = "state_updated"
    EVT_HOVER_ACHIEVED = "hover_achieved"
    EVT_BELOW_HOVER    = "below_hover"

    def __init__(self, vehicle: VehicleConfig, bus: EventBus):
        self._vehicle = vehicle
        self._engine  = AeroEngine(vehicle)
        self._bus     = bus
        self._state:  TwinState | None = None
        self._was_hovering = False
        self.set_rpm(0.0)

    # ── 主更新接口 ────────────────────────────────────────
    def set_rpm(self, rpm: float) -> TwinState:
        rpm = max(0.0, min(rpm, self._vehicle.rotors.rpm_max))
        perf = self._engine.compute(rpm)

        self._state = TwinState(
            timestamp=time.time(),
            rpm=rpm,
            performance=perf,
        )

        self._bus.publish(self.EVT_STATE_UPDATED, state=self._state)

        hovering_now = perf.can_hover
        if hovering_now and not self._was_hovering:
            self._bus.publish(self.EVT_HOVER_ACHIEVED, state=self._state)
        elif not hovering_now and self._was_hovering:
            self._bus.publish(self.EVT_BELOW_HOVER, state=self._state)
        self._was_hovering = hovering_now

        return self._state

    # ── 传感器注入接口 ────────────────────────────────────
    def inject_position(self, x: float, y: float, z: float) -> None:
        if self._state:
            self._state.position_m = [x, y, z]
            self._bus.publish(self.EVT_STATE_UPDATED, state=self._state)

    # ── 属性 ──────────────────────────────────────────────
    @property
    def state(self) -> TwinState:
        return self._state

    @property
    def vehicle(self) -> VehicleConfig:
        return self._vehicle

    @vehicle.setter
    def vehicle(self, new_vehicle: VehicleConfig) -> None:
        self._vehicle = new_vehicle
        self._engine  = AeroEngine(new_vehicle)

    @property
    def hover_rpm(self) -> float:
        return self._state.performance.hover_rpm