"""
tests/test_controllers.py
"""

import math
import numpy as np
from pathlib import Path
from core.vehicle import load_vehicle
from core.event_bus import EventBus
from core.controllers import (
    CascadedPID, LQRController,
    ControlState, ControlRef,
)

YAML = Path("configs/quad_evtol.yaml")


def _vehicle():
    return load_vehicle(YAML)


def test_pid_hover_output():
    pid = CascadedPID(_vehicle())
    state = ControlState(z=5.0)          # 已在目标高度
    ref   = ControlRef(z=5.0)
    out   = pid.step(state, ref, dt=0.02)
    # 悬停时 RPM 应接近 hover_rpm，误差 < 10%
    assert abs(out.mean_rpm - pid._hover_rpm) / pid._hover_rpm < 0.1


def test_pid_climb():
    pid = CascadedPID(_vehicle())
    state = ControlState(z=0.0)          # 起点在地面
    ref   = ControlRef(z=5.0)
    out   = pid.step(state, ref, dt=0.02)
    assert out.mean_rpm > pid._hover_rpm  # 爬升时 RPM 更高


def test_lqr_output_shape():
    lqr = LQRController(_vehicle())
    state = ControlState(z=5.0)
    ref   = ControlRef(z=5.0)
    out   = lqr.step(state, ref, dt=0.02)
    assert out.rpm.shape == (4,)
    assert np.all(out.rpm > 0)


def test_lqr_position_error():
    lqr = LQRController(_vehicle())
    state = ControlState(x=3.0, z=5.0)   # x 偏移 3m
    ref   = ControlRef(x=0.0, z=5.0)
    out   = lqr.step(state, ref, dt=0.02)
    # 应产生不对称 RPM（俯仰修正）
    assert not np.allclose(out.rpm[0], out.rpm[2])


def test_angle_diff():
    from core.controllers import _angle_diff
    assert abs(_angle_diff(math.pi + 0.1, -math.pi + 0.1)) < 1e-6
    assert abs(_angle_diff(0.0, 2 * math.pi)) < 1e-6