"""
simulation/flight_sim.py
垂直起降轨迹仿真器
- 牛顿第二定律 + 欧拉积分
- 驱动 DigitalTwin，通过 EventBus 广播每帧状态
"""

from __future__ import annotations
import math
import time
from dataclasses import dataclass, field
from typing import Callable
from core.digital_twin import DigitalTwin
from core.data_recorder import DataRecorder, FlightRecord


# ── 仿真帧数据 ────────────────────────────────────────────
@dataclass
class SimFrame:
    t:          float   # 仿真时间 (s)
    altitude_m: float   # 高度 (m)
    velocity_ms: float  # 垂直速度 (m/s，向上为正)
    rpm:        float
    thrust_N:   float
    power_W:    float
    accel_ms2:  float   # 加速度 (m/s²)


# ── 仿真配置 ──────────────────────────────────────────────
@dataclass
class SimConfig:
    dt:            float = 0.05    # 时间步长 (s)
    duration_s:    float = 30.0    # 仿真总时长 (s)
    target_alt_m:  float = 10.0    # 目标悬停高度 (m)
    Cd_body:       float = 0.8     # 机身阻力系数
    frontal_area:  float = 0.5     # 机身迎风面积 (m²)


# ── 简单 RPM 调度器（分段线性）────────────────────────────
class RpmScheduler:
    """
    根据高度和速度输出 RPM 指令。
    阶段：起飞爬升 → 高度保持 → 下降着陆
    """

    def __init__(self, twin: DigitalTwin, cfg: SimConfig):
        self._twin   = twin
        self._cfg    = cfg
        self._hover  = twin.hover_rpm
        self._landed = False

    # simulation/flight_sim.py 中的 get_rpm 方法替换为：
    def get_rpm(self, t: float, alt: float, vel: float) -> float:
        target = self._cfg.target_alt_m

        if self._landed:
            return 0.0

        # 起飞阶段（t < 2s 且还没接近目标）
        if t < 2.0 and alt < target * 0.5:
            return self._hover * 1.20

        # 接近目标时提前减速：剩余距离 < 3m 或速度过大时主动减推力
        remaining = target - alt
        if remaining < 3.0 or (alt > target * 0.6 and vel > 2.0):
            # 用比例控制：误差为正（未到）给悬停+补偿，误差为负（超高）给减推
            err = target - alt
            rpm = self._hover + err * 300 - vel * 150  # PD 控制
            return max(self._hover * 0.70, min(self._hover * 1.15, rpm))

        # 爬升阶段：速度不能太快
        if alt < target - 0.5:
            return self._hover * 1.10

        # 高度保持窗口（±0.5m）
        if abs(alt - target) <= 0.5:
            err = target - alt
            return self._hover + err * 200

        # 超高
        if alt > target + 0.5:
            return self._hover * 0.90

        return self._hover


# ── 主仿真器 ──────────────────────────────────────────────
class FlightSimulator:
    """
    用法：
        sim = FlightSimulator(twin, cfg)
        frames = sim.run()           # 离线：返回所有帧
        sim.run(callback=my_fn)      # 在线：每帧调用 callback(frame)
    """

    EVT_SIM_FRAME   = "sim_frame"     # 每帧发布
    EVT_SIM_DONE    = "sim_done"      # 仿真结束

    def __init__(self, twin: DigitalTwin, cfg: SimConfig | None = None):
        self._twin      = twin
        self._cfg       = cfg or SimConfig()
        self._scheduler = RpmScheduler(twin, self._cfg)
        self._rho       = twin.vehicle.environment.air_density_kg_m3
        self._mass      = twin.vehicle.total_mass_kg
        self._g         = twin.vehicle.environment.gravity_m_s2
        self._recorder  = DataRecorder()

    # ── 欧拉积分单步 ──────────────────────────────────────
    def _step(self, t: float, alt: float, vel: float) -> SimFrame:
        rpm = self._scheduler.get_rpm(t, alt, vel)
        self._twin.set_rpm(rpm)
        perf = self._twin.state.performance

        thrust = perf.total_thrust_N
        weight = self._mass * self._g

        # 空气阻力（方向与速度相反）
        drag = (0.5 * self._rho * self._cfg.Cd_body
                * self._cfg.frontal_area * vel * abs(vel))

        accel = (thrust - weight - drag) / self._mass

        # 欧拉积分
        new_vel = vel + accel * self._cfg.dt
        new_alt = max(0.0, alt + new_vel * self._cfg.dt)

        # 地面约束
        if new_alt <= 0.0:
            new_vel = max(0.0, new_vel)   
            if vel <= 0.0 and alt <= 0.0 and t > 5.0:
                self._scheduler._landed = True

        # 注入位置到孪生体
        self._twin.inject_position(0.0, 0.0, new_alt)

        return SimFrame(
            t=t, altitude_m=new_alt,
            velocity_ms=new_vel, rpm=rpm,
            thrust_N=thrust, power_W=perf.total_power_W,
            accel_ms2=accel,
        )

    # ── 运行仿真 ──────────────────────────────────────────
    def run(
        self,
        callback: Callable[[SimFrame], None] | None = None,
        realtime: bool = False,
    ) -> list[SimFrame]:
        """
        离线模式（默认）：瞬间完成，返回所有 SimFrame。
        实时模式（realtime=True）：按 dt 步长 sleep，用于 GUI 驱动。
        """
        cfg    = self._cfg
        frames: list[SimFrame] = []
        alt, vel = 0.0, 0.0
        t = 0.0

        steps = int(cfg.duration_s / cfg.dt)
        self._recorder.start()
        
        for _ in range(steps):
            frame = self._step(t, alt, vel)
            frames.append(frame)

            if callback:
                callback(frame)
                self._recorder.record(FlightRecord(
                    t           = frame.t,
                    altitude_m  = frame.altitude_m,
                    velocity_ms = frame.velocity_ms,
                    rpm         = frame.rpm,
                    thrust_N    = frame.thrust_N,
                    power_W     = frame.power_W,
                ))

            if realtime:
                time.sleep(cfg.dt)

            alt = frame.altitude_m
            vel = frame.velocity_ms
            t  += cfg.dt

        # 发布仿真结束事件
        self._twin._bus.publish(self.EVT_SIM_DONE, frames=frames)
    
        saved = self._recorder.stop()
        if saved:
            print(f"[DataRecorder] Saved → {saved}")

        return frame