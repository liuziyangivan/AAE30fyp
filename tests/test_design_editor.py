"""
tests/test_design_editor.py
"""

from pathlib import Path
from core.vehicle import load_vehicle
from core.event_bus import EventBus
from core.digital_twin import DigitalTwin
from core.design_editor import DesignEditor, DesignParams

YAML = Path("configs/quad_evtol.yaml")


def _setup():
    vehicle = load_vehicle(YAML)
    bus     = EventBus()
    twin    = DigitalTwin(vehicle, bus)
    editor  = DesignEditor(vehicle, twin)
    return editor, vehicle


def test_apply_returns_result():
    editor, vehicle = _setup()
    params = DesignParams.from_vehicle(vehicle)
    result = editor.apply(params)
    assert result.total_mass_kg > 0
    assert result.hover_rpm > 0


def test_larger_radius_reduces_power():
    """更大桨盘面积 → 悬停功率更低（动量理论）"""
    editor, vehicle = _setup()
    p_small = DesignParams.from_vehicle(vehicle)
    from dataclasses import replace
    p_large = replace(p_small, rotor_radius_m=p_small.rotor_radius_m * 1.5)
    r_small = editor.apply(p_small)
    r_large = editor.apply(p_large)
    assert r_large.hover_power_W < r_small.hover_power_W


def test_heavier_payload_increases_hover_power():
    editor, vehicle = _setup()
    p_light = DesignParams.from_vehicle(vehicle)
    from dataclasses import replace
    p_heavy = replace(p_light, payload_kg=p_light.payload_kg + 20)
    r_light = editor.apply(p_light)
    r_heavy = editor.apply(p_heavy)
    assert r_heavy.hover_power_W > r_light.hover_power_W


def test_fm_between_0_and_1():
    editor, vehicle = _setup()
    params = DesignParams.from_vehicle(vehicle)
    result = editor.apply(params)
    assert 0 < result.figure_of_merit <= 1.0


def test_callback_fires():
    editor, vehicle = _setup()
    fired = []
    editor.on_update(lambda v, r: fired.append(r))
    params = DesignParams.from_vehicle(vehicle)
    editor.apply(params)
    assert len(fired) == 1