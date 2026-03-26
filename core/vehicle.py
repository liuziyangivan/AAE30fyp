"""
core/vehicle.py
飞行器配置数据模型 —— 从 YAML 加载，供所有模块使用
"""

from __future__ import annotations
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Tuple
import yaml


# ── 旋翼参数 ────────────────────────────────────────────
@dataclass
class RotorConfig:
    count:      int
    radius_m:   float
    chord_m:    float
    num_blades: int
    mass_kg:    float
    Cl_alpha:   float
    Cd0:        float
    rpm_max:    float
    positions:  List[Tuple[float, float, float]]

    @property
    def omega_max(self) -> float:
        """最大角速度 (rad/s)"""
        return self.rpm_max * 2 * 3.141592653589793 / 60

    @property
    def disk_area_m2(self) -> float:
        """单个旋翼桨盘面积 (m²)"""
        return 3.141592653589793 * self.radius_m ** 2


# ── 机身参数 ─────────────────────────────────────────────
@dataclass
class FuselageConfig:
    length_m: float
    width_m:  float
    height_m: float
    mass_kg:  float


# ── 环境参数 ─────────────────────────────────────────────
@dataclass
class EnvironmentConfig:
    air_density_kg_m3: float
    gravity_m_s2:      float


# ── 完整飞行器 ────────────────────────────────────────────
@dataclass
class VehicleConfig:
    name:        str
    description: str
    fuselage:    FuselageConfig
    rotors:      RotorConfig
    payload_kg:  float
    environment: EnvironmentConfig

    @property
    def total_mass_kg(self) -> float:
        """总质量 = 机身 + 所有旋翼 + 载荷"""
        return (
            self.fuselage.mass_kg
            + self.rotors.count * self.rotors.mass_kg
            + self.payload_kg
        )

    @property
    def total_weight_N(self) -> float:
        """总重力 (N)"""
        return self.total_mass_kg * self.environment.gravity_m_s2


# ── 加载函数 ──────────────────────────────────────────────
def load_vehicle(yaml_path: str | Path) -> VehicleConfig:
    """从 YAML 文件加载飞行器配置"""
    with open(yaml_path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)

    v = raw["vehicle"]
    r = v["rotors"]
    env = v["environment"]

    rotors = RotorConfig(
        count      = r["count"],
        radius_m   = r["radius_m"],
        chord_m    = r["chord_m"],
        num_blades = r["num_blades"],
        mass_kg    = r["mass_kg"],
        Cl_alpha   = r["Cl_alpha"],
        Cd0        = r["Cd0"],
        rpm_max    = r["rpm_max"],
        positions  = [tuple(p) for p in r["positions"]],
    )

    fuselage = FuselageConfig(**v["fuselage"])
    environment = EnvironmentConfig(**env)

    return VehicleConfig(
        name        = v["name"],
        description = v["description"],
        fuselage    = fuselage,
        rotors      = rotors,
        payload_kg  = v["payload_kg"],
        environment = environment,
    )