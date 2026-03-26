"""
tests/test_analysis.py
"""

from pathlib import Path
from core.vehicle import load_vehicle
from core.performance_analysis import PerformanceAnalyzer

YAML = Path("configs/quad_evtol.yaml")


def _analyzer():
    return PerformanceAnalyzer(load_vehicle(YAML))


def test_sweep_returns_correct_count():
    pts = _analyzer().sweep_rpm(n_points=50)
    assert len(pts) == 50


def test_thrust_increases_with_rpm():
    pts = _analyzer().sweep_rpm(n_points=30)
    thrusts = [p.thrust_N for p in pts]
    assert thrusts[-1] > thrusts[0]


def test_hover_point_in_sweep():
    """悬停转速对应的推力应约等于重力"""
    a   = _analyzer()
    pts = a.sweep_rpm(n_points=200)
    v   = a.vehicle
    # 找裕度符号变化的点 —— 说明 hover_rpm 在扫描范围内
    margins = [p.thrust_margin_N for p in pts]
    has_neg = any(m < 0 for m in margins)
    has_pos = any(m > 0 for m in margins)
    assert has_neg and has_pos


def test_endurance_positive():
    end = _analyzer().endurance(battery_kWh=10.0)
    assert end.hover_endurance_min > 0


def test_endurance_scales_with_battery():
    a    = _analyzer()
    e5   = a.endurance(5.0)
    e10  = a.endurance(10.0)
    assert abs(e10.hover_endurance_min - 2 * e5.hover_endurance_min) < 0.01


def test_fm_between_zero_and_one():
    pts = _analyzer().sweep_rpm(n_points=30)
    for p in pts:
        if p.figure_of_merit > 0:
            assert 0 < p.figure_of_merit <= 1.0