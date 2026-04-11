"""
core/design_editor.py
参数化设计编辑器 —— 修改飞行器参数并重新计算气动性能
"""

from __future__ import annotations
from dataclasses import dataclass, replace
from typing import Callable, List

from core.vehicle import VehicleConfig, RotorConfig, FuselageConfig
from core.aero_engine import AeroEngine
from core.digital_twin import DigitalTwin


# ── 可编辑参数快照 ────────────────────────────────────────
@dataclass
class DesignParams:
    # 旋翼
    rotor_radius_m:  float
    rotor_chord_m:   float
    num_blades:      int
    rpm_max:         float
    Cl_alpha:        float
    Cd0:             float
    # 机身
    fuselage_mass_kg: float
    payload_kg:       float

    @staticmethod
    def from_vehicle(v: VehicleConfig) -> "DesignParams":
        return DesignParams(
            rotor_radius_m   = v.rotors.radius_m,
            rotor_chord_m    = v.rotors.chord_m,
            num_blades       = v.rotors.num_blades,
            rpm_max          = v.rotors.rpm_max,
            Cl_alpha         = v.rotors.Cl_alpha,
            Cd0              = v.rotors.Cd0,
            fuselage_mass_kg = v.fuselage.mass_kg,
            payload_kg       = v.payload_kg,
        )


# ── 单次分析结果 ──────────────────────────────────────────
@dataclass
class DesignResult:
    total_mass_kg:    float
    total_weight_N:   float
    hover_rpm:        float
    hover_thrust_N:   float
    hover_power_W:    float
    figure_of_merit:  float
    disk_loading_Pa:  float   # W/A，桨盘载荷
    power_loading:    float   # T/P，功率载荷 (N/W)


# ── 设计编辑器 ────────────────────────────────────────────
class DesignEditor:
    """
    用法：
        editor = DesignEditor(vehicle, twin)
        result = editor.apply(new_params)
    """

    def __init__(self, vehicle: VehicleConfig, twin: DigitalTwin):
        self._base_vehicle = vehicle
        self._twin         = twin
        self._callbacks: List[Callable[[VehicleConfig, DesignResult], None]] = []

    # ── 订阅 ──────────────────────────────────────────────
    def on_update(self, cb: Callable[[VehicleConfig, DesignResult], None]):
        self._callbacks.append(cb)

    # ── 应用新参数 ────────────────────────────────────────
    def apply(self, params: DesignParams) -> DesignResult:
        """用新参数重建 VehicleConfig，重新计算悬停点"""

        new_rotors = replace(
            self._base_vehicle.rotors,
            radius_m   = params.rotor_radius_m,
            chord_m    = params.rotor_chord_m,
            num_blades = params.num_blades,
            rpm_max    = params.rpm_max,
            Cl_alpha   = params.Cl_alpha,
            Cd0        = params.Cd0,
        )
        new_fuselage = replace(
            self._base_vehicle.fuselage,
            mass_kg = params.fuselage_mass_kg,
        )
        new_vehicle = replace(
            self._base_vehicle,
            rotors      = new_rotors,
            fuselage    = new_fuselage,
            payload_kg  = params.payload_kg,
        )

        # 重新计算悬停转速
        engine      = AeroEngine(new_vehicle)
        hover_rpm   = self._find_hover_rpm(engine, new_vehicle)
        perf        = engine.compute(hover_rpm)

        import math
        rho        = new_vehicle.environment.air_density_kg_m3
        total_disk = new_vehicle.rotors.count * new_vehicle.rotors.disk_area_m2
        disk_load  = new_vehicle.total_weight_N / total_disk
        pwr_load   = (perf.total_thrust_N / perf.total_power_W
                      if perf.total_power_W > 0 else 0.0)

        # 悬停品质因数 FM = 理想功率 / 实际功率
        T = perf.total_thrust_N
        if perf.total_power_W > 0 and T > 0:
            ideal_power = T * math.sqrt(T / (2 * rho * total_disk))
            fm = ideal_power / perf.total_power_W
            fm = min(fm, 1.0)
        else:
            fm = 0.0

        result = DesignResult(
            total_mass_kg   = new_vehicle.total_mass_kg,
            total_weight_N  = new_vehicle.total_weight_N,
            hover_rpm       = hover_rpm,
            hover_thrust_N  = perf.total_thrust_N,
            hover_power_W   = perf.total_power_W,
            figure_of_merit = fm,
            disk_loading_Pa = disk_load,
            power_loading   = pwr_load,
        )

        # 通知 twin 更新（让 Live Control 也感知）
        try:
            self._twin.vehicle = new_vehicle
            self._twin.set_rpm(hover_rpm)
        except Exception:
            pass

        for cb in self._callbacks:
            cb(new_vehicle, result)

        return result

    # ── 二分法求悬停转速 ──────────────────────────────────
    @staticmethod
    def _find_hover_rpm(engine: AeroEngine, v: VehicleConfig) -> float:
        target = v.total_weight_N
        lo, hi = 100.0, v.rotors.rpm_max
        for _ in range(60):
            mid  = (lo + hi) / 2
            perf = engine.compute(mid)
            if perf.total_thrust_N < target:
                lo = mid
            else:
                hi = mid
        return (lo + hi) / 2