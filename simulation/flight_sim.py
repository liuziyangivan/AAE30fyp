"""
simulation/flight_sim.py
垂直起降轨迹仿真器（高精度版）
"""

from __future__ import annotations
import math
import time
from dataclasses import dataclass
from typing import Callable

from core.digital_twin import DigitalTwin
from core.data_recorder import DataRecorder, FlightRecord


@dataclass
class SimFrame:
    t:           float
    altitude_m:  float
    velocity_ms: float
    rpm:         float
    thrust_N:    float
    power_W:     float
    accel_ms2:   float


@dataclass
class SimConfig:
    dt:           float = 0.02        # 精度提升：50Hz → 50ms 改为 20ms
    duration_s:   float = 60.0
    target_alt_m: float = 10.0
    Cd_body:      float = 0.8
    frontal_area: float = 0.5


class RpmScheduler:
    def __init__(self, twin: DigitalTwin, cfg: SimConfig):
        self._twin   = twin
        self._cfg    = cfg
        self._hover  = twin.hover_rpm
        self._landed = False

    def get_rpm(self, t: float, alt: float, vel: float) -> float:
        target = self._cfg.target_alt_m
        if self._landed:
            return 0.0
        if t < 2.0 and alt < target * 0.5:
            return self._hover * 1.20
        remaining = target - alt
        if remaining < 3.0 or (alt > target * 0.6 and vel > 2.0):
            err = target - alt
            rpm = self._hover + err * 300 - vel * 150
            return max(self._hover * 0.70, min(self._hover * 1.15, rpm))
        if alt < target - 0.5:
            return self._hover * 1.10
        if abs(alt - target) <= 0.5:
            return self._hover + (target - alt) * 200
        if alt > target + 0.5:
            return self._hover * 0.90
        return self._hover


class FlightSimulator:

    EVT_SIM_FRAME = "sim_frame"
    EVT_SIM_DONE  = "sim_done"

    def __init__(self, twin: DigitalTwin, cfg: SimConfig | None = None):
        self._twin      = twin
        self._cfg       = cfg or SimConfig()
        self._scheduler = RpmScheduler(twin, self._cfg)
        self._rho       = twin.vehicle.environment.air_density_kg_m3
        self._mass      = twin.vehicle.total_mass_kg
        self._g         = twin.vehicle.environment.gravity_m_s2
        self._recorder  = DataRecorder()

    def _step(self, t: float, alt: float, vel: float) -> SimFrame:
        rpm    = self._scheduler.get_rpm(t, alt, vel)
        self._twin.set_rpm(rpm)
        perf   = self._twin.state.performance
        thrust = perf.total_thrust_N
        weight = self._mass * self._g
        drag   = (0.5 * self._rho * self._cfg.Cd_body
                  * self._cfg.frontal_area * vel * abs(vel))
        accel  = (thrust - weight - drag) / self._mass
        new_vel = vel + accel * self._cfg.dt
        new_alt = max(0.0, alt + new_vel * self._cfg.dt)
        if new_alt <= 0.0:
            new_vel = max(0.0, new_vel)
            if vel <= 0.0 and alt <= 0.0 and t > 5.0:
                self._scheduler._landed = True
        self._twin.inject_position(0.0, 0.0, new_alt)
        return SimFrame(t=t, altitude_m=new_alt, velocity_ms=new_vel,
                        rpm=rpm, thrust_N=thrust,
                        power_W=perf.total_power_W, accel_ms2=accel)

    def run(self, callback: Callable[[SimFrame], None] | None = None,
            realtime: bool = False) -> list[SimFrame]:
        self._recorder.start()
        frames: list[SimFrame] = []
        alt, vel, t = 0.0, 0.0, 0.0
        steps = int(self._cfg.duration_s / self._cfg.dt)
        for _ in range(steps):
            frame = self._step(t, alt, vel)
            frames.append(frame)
            if callback:
                callback(frame)
            self._recorder.record(FlightRecord(
                t=frame.t, altitude_m=frame.altitude_m,
                velocity_ms=frame.velocity_ms, rpm=frame.rpm,
                thrust_N=frame.thrust_N, power_W=frame.power_W,
            ))
            if realtime:
                time.sleep(self._cfg.dt)
            alt = frame.altitude_m
            vel = frame.velocity_ms
            t  += self._cfg.dt
        saved = self._recorder.stop()
        if saved:
            print(f"[DataRecorder] Saved → {saved}")
        self._twin._bus.publish(self.EVT_SIM_DONE, frames=frames)
        return frames