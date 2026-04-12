"""
core/fault_injector.py
故障注入与容错分析模块

功能：
  - 管理每个旋翼的健康系数（0.0 = 完全失效，1.0 = 正常）
  - 计算非对称推力导致的力矩失衡
  - 判断系统是否仍能维持安全飞行
  - 提供预设故障场景
"""

from __future__ import annotations
import math
from dataclasses import dataclass, field
from typing import List


# ── 单旋翼故障状态 ────────────────────────────────────────
@dataclass
class RotorFault:
    rotor_id:       int    # 0~3，对应 FL/FR/RL/RR
    health:         float  # 0.0（完全失效）~ 1.0（正常）
    label:          str    = ""

    @property
    def is_failed(self) -> bool:
        return self.health < 0.05

    @property
    def is_degraded(self) -> bool:
        return 0.05 <= self.health < 0.9


# ── 力矩失衡分析结果 ──────────────────────────────────────
@dataclass
class FaultAnalysis:
    rotor_thrusts_N:    List[float]   # 各旋翼实际推力
    total_thrust_N:     float         # 总推力
    roll_moment_Nm:     float         # 滚转力矩（左右不对称）
    pitch_moment_Nm:    float         # 俯仰力矩（前后不对称）
    yaw_moment_Nm:      float         # 偏航力矩（对角推力差）
    thrust_loss_pct:    float         # 推力损失百分比
    can_maintain_hover: bool          # 是否能维持悬停
    can_safe_land:      bool          # 是否能安全着陆
    risk_level:         str           # "SAFE" / "DEGRADED" / "CRITICAL"
    recommendation:     str           # 操作建议


# ── 预设故障场景 ──────────────────────────────────────────
FAULT_PRESETS = {
    "normal": {
        "label": "✅ Normal 正常",
        "healths": [1.0, 1.0, 1.0, 1.0],
        "description": "All the rotors are functioning properly. 所有旋翼正常工作",
    },
    "single_fail_fl": {
        "label": "❌ Left front failure 前左失效",
        "healths": [0.0, 1.0, 1.0, 1.0],
        "description": "The rotor FL has completely failed. 旋翼 FL（前左）完全失效",
    },
    "single_fail_fr": {
        "label": "❌ Front right failure 前右失效",
        "healths": [1.0, 0.0, 1.0, 1.0],
        "description": "The rotor FR has completely failed. 旋翼 FR（前右）完全失效",
    },
    "single_degrade": {
        "label": "⚠️ Single-rotor 50% 单旋翼50%",
        "healths": [0.5, 1.0, 1.0, 1.0],
        "description": "The rotor FL thrust has decreased to 50%. 旋翼 FL 推力下降至 50%",
    },
    "diagonal_fail": {
        "label": "❌ Diagonal failure 对角失效",
        "healths": [0.0, 1.0, 1.0, 0.0],
        "description": "Diagonal rotor (FL + RR) both fail simultaneously. 对角旋翼（FL+RR）同时失效",
    },
    "two_degrade": {
        "label": "⚠️ Dual-rotor 70% 双旋翼70%",
        "healths": [0.7, 0.7, 1.0, 1.0],
        "description": "The two rotors ahead have both descended to 70%. 前方两旋翼均下降至 70%",
    },
    "three_degrade": {
        "label": "⚠️ Three-rotor degradation 三旋翼退化",
        "healths": [0.6, 0.8, 1.0, 0.7],
        "description": "Three rotors have experienced varying degrees of deterioration. 三个旋翼不同程度退化",
    },
}


# ── 主类 ─────────────────────────────────────────────────
class FaultInjector:
    """
    管理旋翼故障状态并提供容错分析。

    旋翼编号约定（俯视图）：
        0=FL（前左）  1=FR（前右）
        2=RL（后左）  3=RR（后右）
    """

    ROTOR_LABELS = ["FL 前左", "FR 前右", "RL 后左", "RR 后右"]

    def __init__(self, vehicle_config):
        self._vehicle = vehicle_config
        n = vehicle_config.rotors.count
        self._faults: List[RotorFault] = [
            RotorFault(rotor_id=i, health=1.0,
                       label=self.ROTOR_LABELS[i] if i < 4 else f"R{i}")
            for i in range(n)
        ]
        # 旋翼位置（相对机身中心）
        self._positions = vehicle_config.rotors.positions  # list of (x,y,z)

    # ── 设置故障 ──────────────────────────────────────────
    def set_health(self, rotor_id: int, health: float) -> None:
        """设置指定旋翼健康系数（0.0~1.0）"""
        if 0 <= rotor_id < len(self._faults):
            self._faults[rotor_id].health = max(0.0, min(1.0, health))

    def apply_preset(self, preset_name: str) -> None:
        """应用预设故障场景"""
        preset = FAULT_PRESETS.get(preset_name)
        if not preset:
            return
        for i, h in enumerate(preset["healths"]):
            if i < len(self._faults):
                self._faults[i].health = h

    def reset_all(self) -> None:
        """恢复所有旋翼至正常"""
        for f in self._faults:
            f.health = 1.0

    # ── 获取推力系数 ──────────────────────────────────────
    def get_thrust_multipliers(self) -> List[float]:
        """返回各旋翼推力乘数列表"""
        return [f.health for f in self._faults]

    def get_faults(self) -> List[RotorFault]:
        return self._faults

    # ── 容错分析 ──────────────────────────────────────────
    def analyze(self, base_thrust_per_rotor_N: float) -> FaultAnalysis:
        """
        给定每旋翼正常推力，计算故障后的力矩失衡和飞行能力。

        base_thrust_per_rotor_N：悬停时每个旋翼的推力（N）
        """
        positions = self._positions
        faults    = self._faults
        n         = len(faults)
        g         = self._vehicle.environment.gravity_m_s2
        total_mass= self._vehicle.total_mass_kg

        # 各旋翼实际推力
        thrusts = [f.health * base_thrust_per_rotor_N for f in faults]
        total_T = sum(thrusts)
        weight  = total_mass * g

        # 力矩计算（相对机身中心）
        # 滚转（绕 X 轴）：由 Y 方向偏置产生
        # 俯仰（绕 Y 轴）：由 X 方向偏置产生
        roll_M  = 0.0
        pitch_M = 0.0
        yaw_M   = 0.0

        # 偏航：对角旋翼推力差（四旋翼反扭矩对）
        # 0(FL)+3(RR) 顺时针；1(FR)+2(RL) 逆时针
        yaw_cw  = thrusts[0] + thrusts[3] if n >= 4 else 0.0
        yaw_ccw = thrusts[1] + thrusts[2] if n >= 4 else 0.0
        yaw_M   = (yaw_cw - yaw_ccw) * 0.05   # 力臂约 0.05m（扭矩系数）

        for i, (T, pos) in enumerate(zip(thrusts, positions)):
            px, py = pos[0], pos[1]
            roll_M  += T * py   # Y 偏置 → 滚转
            pitch_M += T * px   # X 偏置 → 俯仰

        thrust_loss_pct = max(0.0, (weight - total_T) / weight * 100)

        # 飞行能力判断
        hover_margin = total_T - weight
        can_hover    = hover_margin >= 0.0
        # 安全着陆：至少 60% 推力（可慢速下降）
        can_land     = total_T >= weight * 0.60

        # 风险等级
        failed_count   = sum(1 for f in faults if f.is_failed)
        degraded_count = sum(1 for f in faults if f.is_degraded)
        max_moment     = max(abs(roll_M), abs(pitch_M))

        if not can_hover and not can_land:
            risk = "CRITICAL"
        elif not can_hover or failed_count >= 2 or max_moment > 150:
            risk = "CRITICAL"
        elif failed_count == 1 or degraded_count >= 2 or max_moment > 60:
            risk = "DEGRADED"
        else:
            risk = "SAFE"

        # 操作建议
        if risk == "CRITICAL":
            rec = "🚨 Immediate emergency landing! There is an extremely high risk of insufficient thrust or loss of control of the aircraft."
        elif risk == "DEGRADED":
            if not can_hover:
                rec = "⚠️ Insufficient thrust. Execute a controlled descent to avoid sudden movements."
            elif max_moment > 60:
                rec = "⚠️ The torque imbalance is obvious. Reduce the speed and search for the nearest landing point."
            else:
                rec = "⚠️ The system is operating in a degraded mode. It is recommended that you return to base as soon as possible and reduce the number of maneuvers."
        else:
            if failed_count == 0 and degraded_count == 0:
                rec = "✅ The system is operating normally and all rotors are in good condition."
            else:
                rec = "✅ The system is operating normally and is under continuous monitoring."

        return FaultAnalysis(
            rotor_thrusts_N=thrusts,
            total_thrust_N=total_T,
            roll_moment_Nm=roll_M,
            pitch_moment_Nm=pitch_M,
            yaw_moment_Nm=yaw_M,
            thrust_loss_pct=thrust_loss_pct,
            can_maintain_hover=can_hover,
            can_safe_land=can_land,
            risk_level=risk,
            recommendation=rec,
        )