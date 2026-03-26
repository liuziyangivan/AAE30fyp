"""
core/performance_analysis.py
飞行包线分析 —— 推力裕度 / 功率预算 / 航时估算
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import List
import numpy as np
from core.vehicle import VehicleConfig
from core.aero_engine import AeroEngine


@dataclass
class EnvelopePoint:
    rpm:               float
    thrust_N:          float   # 全机总推力
    power_kW:          float   # 全机总功率
    thrust_margin_N:   float   # 推力裕度 = 总推力 − 重力
    figure_of_merit:   float   # 悬停效率 FM


@dataclass
class EnduranceResult:
    battery_kWh:          float
    hover_power_kW:       float
    hover_endurance_h:    float   # 悬停航时 (h)
    hover_endurance_min:  float   # 悬停航时 (min)


class PerformanceAnalyzer:
    """飞行包线分析器"""

    def __init__(self, vehicle: VehicleConfig):
        self.vehicle = vehicle
        self.engine  = AeroEngine(vehicle)

    # ── 转速扫描 ─────────────────────────────────────────
    def sweep_rpm(self, n_points: int = 120) -> List[EnvelopePoint]:
        """从 500 RPM 扫到 rpm_max，返回包线点列表"""
        rpms = np.linspace(500.0, float(self.vehicle.rotors.rpm_max), n_points)
        points: List[EnvelopePoint] = []

        for rpm in rpms:
            perf = self.engine.compute(float(rpm))
            points.append(EnvelopePoint(
                rpm             = float(rpm),
                thrust_N        = perf.total_thrust_N,
                power_kW        = perf.total_power_W / 1000.0,
                thrust_margin_N = perf.thrust_margin_N,
                figure_of_merit = perf.rotor.figure_of_merit,
            ))
        return points

    # ── 悬停航时估算 ─────────────────────────────────────
    def endurance(
        self,
        battery_kWh:           float,
        discharge_efficiency:  float = 0.90,
    ) -> EnduranceResult:
        """给定电池容量，估算悬停续航时间"""
        # 用任意转速先拿到 hover_rpm（compute 内部总会解算）
        seed_perf  = self.engine.compute(self.vehicle.rotors.rpm_max * 0.5)
        hover_rpm  = seed_perf.hover_rpm
        hover_perf = self.engine.compute(hover_rpm)
        hover_kW   = hover_perf.total_power_W / 1000.0

        usable_kWh = battery_kWh * discharge_efficiency
        end_h      = (usable_kWh / hover_kW) if hover_kW > 0 else 0.0

        return EnduranceResult(
            battery_kWh         = battery_kWh,
            hover_power_kW      = hover_kW,
            hover_endurance_h   = end_h,
            hover_endurance_min = end_h * 60.0,
        )