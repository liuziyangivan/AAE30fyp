"""
core/aero_engine.py
叶素理论（BET）气动引擎
- 输入：转速 (RPM) + 飞行器配置
- 输出：推力 (N)、扭矩 (N·m)、功率 (W)、悬停效率 FM
"""

from __future__ import annotations
import math
import numpy as np
from dataclasses import dataclass
from core.vehicle import VehicleConfig


# ── 单旋翼计算结果 ────────────────────────────────────────
@dataclass
class RotorPerformance:
    rpm:          float   # 输入转速
    omega_rad_s:  float   # 角速度 (rad/s)
    thrust_N:     float   # 推力 (N)
    torque_Nm:    float   # 扭矩 (N·m)
    power_W:      float   # 轴功率 (W)
    figure_of_merit: float  # 悬停效率 FM (0~1)

    def __str__(self) -> str:
        return (
            f"RPM={self.rpm:.0f}  T={self.thrust_N:.2f} N  "
            f"P={self.power_W:.1f} W  FM={self.figure_of_merit:.3f}"
        )


# ── 整机计算结果 ──────────────────────────────────────────
@dataclass
class VehiclePerformance:
    rotor:           RotorPerformance   # 单旋翼（所有旋翼相同）
    total_thrust_N:  float              # 全部旋翼总推力
    total_power_W:   float              # 全部旋翼总功率
    thrust_margin_N: float              # 推力余量 = 总推力 - 重力
    hover_rpm:       float              # 悬停所需转速

    @property
    def can_hover(self) -> bool:
        return self.thrust_margin_N >= 0

    def __str__(self) -> str:
        status = "✓ CAN HOVER" if self.can_hover else "✗ CANNOT HOVER"
        return (
            f"Total Thrust : {self.total_thrust_N:.1f} N\n"
            f"Total Power  : {self.total_power_W / 1000:.2f} kW\n"
            f"Thrust Margin: {self.thrust_margin_N:+.1f} N\n"
            f"Hover RPM    : {self.hover_rpm:.0f} RPM\n"
            f"Status       : {status}"
        )


# ── 气动引擎 ──────────────────────────────────────────────
class AeroEngine:
    """
    基于叶素理论的旋翼气动计算器。

    使用方式：
        engine = AeroEngine(vehicle_config)
        perf   = engine.compute(rpm=3000)
    """

    N_ELEMENTS = 50   # 径向积分节点数

    def __init__(self, vehicle: VehicleConfig):
        self.v   = vehicle
        self.rho = vehicle.environment.air_density_kg_m3
        self.R   = vehicle.rotors.radius_m
        self.c   = vehicle.rotors.chord_m
        self.B   = vehicle.rotors.num_blades
        self.Cl  = vehicle.rotors.Cl_alpha   # 升力线斜率 (1/rad)
        self.Cd0 = vehicle.rotors.Cd0

        # 无量纲来流角（悬停时取固定桨距角 θ = 10°）
        self.theta_rad = math.radians(10.0)

        # 径向积分节点（跳过桨根 5%）
        self.r_arr = np.linspace(0.05 * self.R, self.R, self.N_ELEMENTS)
        self.dr    = self.r_arr[1] - self.r_arr[0]

    # ── 单旋翼 BET 积分 ───────────────────────────────────
    def _compute_rotor(self, rpm: float) -> RotorPerformance:
        if rpm <= 0:
            return RotorPerformance(rpm=0, omega_rad_s=0, thrust_N=0,
                                    torque_Nm=0, power_W=0, figure_of_merit=0)

        omega = rpm * 2 * math.pi / 60
        r     = self.r_arr
        R     = self.R
        A     = math.pi * R ** 2

        # ── 动量理论迭代求诱导速度 ──────────────────────────
        # 初始猜测
        v_tip = omega * R
        lambda_i = math.sqrt(max(
            self.v.total_weight_N / self.v.rotors.count
            / (2 * self.rho * A), 0.0)) / v_tip

        for _ in range(50):   # 迭代收敛
            V_i    = lambda_i * v_tip
            V_t    = omega * r
            phi    = np.arctan2(V_i, V_t)
            alpha  = self.theta_rad - phi
            Cl_loc = self.Cl * np.clip(alpha, -0.3, 0.3)
            dT     = 0.5 * self.rho * (V_t**2 + V_i**2) * self.c * self.B * Cl_loc * self.dr
            T_new  = float(np.sum(dT))
            lambda_new = math.sqrt(max(T_new / (2 * self.rho * A), 0.0)) / v_tip
            if abs(lambda_new - lambda_i) < 1e-6:
                break
            lambda_i = 0.5 * lambda_i + 0.5 * lambda_new   # 松弛

        # ── 最终 BET 积分 ────────────────────────────────────
        V_i    = lambda_i * v_tip
        V_t    = omega * r
        V_eff  = np.sqrt(V_t**2 + V_i**2)
        phi    = np.arctan2(V_i, V_t)
        alpha  = self.theta_rad - phi
        alpha  = np.clip(alpha, -0.3, 0.3)

        Cl_loc = self.Cl * alpha
        Cd_loc = self.Cd0 + 0.02 * alpha**2   # 抛物线极曲线

        q   = 0.5 * self.rho * V_eff**2
        dT  = q * self.c * self.B * (Cl_loc * np.cos(phi) - Cd_loc * np.sin(phi)) * self.dr
        dQ  = q * self.c * self.B * (Cd_loc * np.cos(phi) + Cl_loc * np.sin(phi)) * r * self.dr

        thrust = float(np.sum(dT))
        torque = float(np.sum(dQ))
        power  = torque * omega

        # 悬停效率 FM
        if thrust > 0 and power > 0:
            P_ideal = thrust * math.sqrt(thrust / (2 * self.rho * A))
            fm = min(P_ideal / power, 1.0)
        else:
            fm = 0.0

        return RotorPerformance(
            rpm=rpm, omega_rad_s=omega,
            thrust_N=max(thrust, 0.0), torque_Nm=torque,
            power_W=power, figure_of_merit=fm,
        )

    # ── 求解悬停转速（二分法）────────────────────────────
    def _solve_hover_rpm(self) -> float:
        """求单旋翼需提供 W/n 推力时的 RPM"""
        W_per_rotor = self.v.total_weight_N / self.v.rotors.count
        lo, hi = 100.0, float(self.v.rotors.rpm_max)

        for _ in range(60):   # 二分 60 次，精度 < 0.001 RPM
            mid = (lo + hi) / 2
            if self._compute_rotor(mid).thrust_N < W_per_rotor:
                lo = mid
            else:
                hi = mid

        return (lo + hi) / 2

    # ── 公开接口 ─────────────────────────────────────────
    def compute(self, rpm: float) -> VehiclePerformance:
        """
        给定所有旋翼转速，返回整机性能。
        （四旋翼悬停时所有旋翼转速相同）
        """
        rotor_perf = self._compute_rotor(rpm)
        n          = self.v.rotors.count
        total_T    = rotor_perf.thrust_N * n
        total_P    = rotor_perf.power_W  * n
        margin     = total_T - self.v.total_weight_N
        hover_rpm  = self._solve_hover_rpm()

        return VehiclePerformance(
            rotor=rotor_perf,
            total_thrust_N=total_T,
            total_power_W=total_P,
            thrust_margin_N=margin,
            hover_rpm=hover_rpm,
        )