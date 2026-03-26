"""
tests/test_aero.py
"""

import math
from pathlib import Path
from core.vehicle import load_vehicle
from core.aero_engine import AeroEngine

YAML = Path("configs/quad_evtol.yaml")


def _engine():
    return AeroEngine(load_vehicle(YAML))


def test_zero_rpm():
    """转速为 0 时推力应为 0"""
    e = _engine()
    p = e.compute(0)
    assert p.total_thrust_N == 0.0


def test_thrust_increases_with_rpm():
    """推力应随转速单调递增"""
    e = _engine()
    t1 = e.compute(2000).total_thrust_N
    t2 = e.compute(4000).total_thrust_N
    assert t2 > t1


def test_power_positive():
    """功率在非零转速下应为正"""
    e = _engine()
    assert e.compute(3000).total_power_W > 0


def test_hover_rpm_achieves_hover():
    """悬停转速下总推力应 ≥ 总重力"""
    e = _engine()
    hover_rpm = e.compute(3000).hover_rpm
    result    = e.compute(hover_rpm)
    assert result.total_thrust_N >= e.v.total_weight_N * 0.99   # 允许 1% 误差


def test_figure_of_merit_range():
    """FM 应在 0~1 之间"""
    e = _engine()
    fm = e.compute(3000).rotor.figure_of_merit
    assert 0.0 <= fm <= 1.0


def test_rpm_cap():
    """转速不超过 rpm_max"""
    e = _engine()
    v = e.v
    assert e.compute(v.rotors.rpm_max).rotor.rpm == v.rotors.rpm_max