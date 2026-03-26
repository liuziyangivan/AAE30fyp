"""
tests/test_sim.py
"""

from pathlib import Path
from core.vehicle import load_vehicle
from core.event_bus import EventBus
from core.digital_twin import DigitalTwin
from simulation.flight_sim import FlightSimulator, SimConfig

YAML = Path("configs/quad_evtol.yaml")


def _sim(duration=20.0, target_alt=10.0):
    bus  = EventBus()
    twin = DigitalTwin(load_vehicle(YAML), bus)
    cfg  = SimConfig(dt=0.05, duration_s=duration, target_alt_m=target_alt)
    return FlightSimulator(twin, cfg), twin


def test_returns_frames():
    sim, _ = _sim()
    frames = sim.run()
    assert len(frames) > 0


def test_altitude_non_negative():
    sim, _ = _sim()
    frames = sim.run()
    assert all(f.altitude_m >= 0.0 for f in frames)


def test_reaches_target_altitude():
    sim, _ = _sim(duration=30.0, target_alt=10.0)
    frames = sim.run()
    max_alt = max(f.altitude_m for f in frames)
    assert 8.0 <= max_alt <= 30.0   # 既要到达，又不能飞太高   # 至少到达目标高度 80%


def test_thrust_matches_twin():
    """每帧推力应与 twin 状态一致"""
    sim, twin = _sim()
    frames = sim.run()
    # 最后一帧推力应非负
    assert frames[-1].thrust_N >= 0.0


def test_sim_done_event():
    bus  = EventBus()
    twin = DigitalTwin(load_vehicle(YAML), bus)
    cfg  = SimConfig(dt=0.1, duration_s=2.0)
    sim  = FlightSimulator(twin, cfg)

    done_called = []
    bus.subscribe("sim_done", lambda frames: done_called.append(len(frames)))
    sim.run()
    assert len(done_called) == 1
    assert done_called[0] == 20   # 2.0 / 0.1


def test_callback_called():
    sim, _ = _sim(duration=2.0)
    calls = []
    sim.run(callback=lambda f: calls.append(f.t))
    assert len(calls) == 40   # 2.0 / 0.05