"""
tests/test_vehicle.py
"""

from pathlib import Path
from core.vehicle import load_vehicle

YAML_PATH = Path("configs/quad_evtol.yaml")


def test_load_vehicle():
    v = load_vehicle(YAML_PATH)
    assert v.name == "Quad-eVTOL-01"
    assert v.rotors.count == 4


def test_total_mass():
    v = load_vehicle(YAML_PATH)
    # 机身 40 + 4×0.5 + 载荷 20 = 62 kg
    assert abs(v.total_mass_kg - 62.0) < 1e-6


def test_total_weight():
    v = load_vehicle(YAML_PATH)
    expected = 62.0 * 9.81
    assert abs(v.total_weight_N - expected) < 1e-4


def test_rotor_properties():
    v = load_vehicle(YAML_PATH)
    import math
    expected_area = math.pi * 0.4 ** 2
    assert abs(v.rotors.disk_area_m2 - expected_area) < 1e-6


def test_rotor_positions():
    v = load_vehicle(YAML_PATH)
    assert len(v.rotors.positions) == 4