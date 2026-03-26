"""
tests/test_twin.py
"""

from pathlib import Path
from core.vehicle import load_vehicle
from core.event_bus import EventBus
from core.digital_twin import DigitalTwin

YAML = Path("configs/quad_evtol.yaml")


def _twin():
    bus = EventBus()
    return DigitalTwin(load_vehicle(YAML), bus), bus


def test_initial_state():
    twin, _ = _twin()
    assert twin.state is not None
    assert twin.state.rpm == 0.0


def test_set_rpm_updates_state():
    twin, _ = _twin()
    twin.set_rpm(3000)
    assert twin.state.rpm == 3000.0


def test_rpm_clamped_to_max():
    twin, _ = _twin()
    twin.set_rpm(99999)
    assert twin.state.rpm == twin.vehicle.rotors.rpm_max


def test_rpm_clamped_to_zero():
    twin, _ = _twin()
    twin.set_rpm(-100)
    assert twin.state.rpm == 0.0


def test_event_published():
    twin, bus = _twin()
    received = []
    bus.subscribe("state_updated", lambda state: received.append(state.rpm))
    twin.set_rpm(2000)
    twin.set_rpm(4000)
    assert received == [2000.0, 4000.0]


def test_hover_achieved_event():
    twin, bus = _twin()
    events = []
    bus.subscribe("hover_achieved", lambda state: events.append(state.rpm))

    twin.set_rpm(1000)   # below hover
    twin.set_rpm(5000)   # above hover → should fire hover_achieved
    assert len(events) == 1
    assert events[0] == 5000.0


def test_below_hover_event():
    twin, bus = _twin()
    events = []
    bus.subscribe("below_hover", lambda state: events.append(state.rpm))

    twin.set_rpm(5000)   # above hover
    twin.set_rpm(1000)   # drop below → should fire below_hover
    assert len(events) == 1


def test_inject_position():
    twin, _ = _twin()
    twin.inject_position(1.0, 2.0, 10.0)
    assert twin.state.altitude_m == 10.0
    assert twin.state.is_airborne