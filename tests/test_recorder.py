"""
tests/test_recorder.py
"""

import tempfile
from pathlib import Path
from core.data_recorder import DataRecorder, FlightRecord


def _make_records(n: int = 10):
    return [
        FlightRecord(
            t           = i * 0.05,
            altitude_m  = float(i) * 0.5,
            velocity_ms = 1.0,
            rpm         = 3000.0,
            thrust_N    = 620.0,
            power_W     = 5000.0,
        )
        for i in range(n)
    ]


def test_record_and_load():
    with tempfile.TemporaryDirectory() as tmp:
        recorder = DataRecorder(save_dir=tmp)
        recorder.start()
        for r in _make_records(20):
            recorder.record(r)
        path = recorder.stop()

        assert path is not None and path.exists()
        loaded = DataRecorder.load(path)
        assert len(loaded) == 20
        assert abs(loaded[0].t - 0.0) < 1e-6
        assert abs(loaded[-1].t - 19 * 0.05) < 1e-4


def test_altitude_preserved():
    with tempfile.TemporaryDirectory() as tmp:
        recorder = DataRecorder(save_dir=tmp)
        recorder.start()
        records = _make_records(5)
        for r in records:
            recorder.record(r)
        path = recorder.stop()
        loaded = DataRecorder.load(path)
        for orig, back in zip(records, loaded):
            assert abs(orig.altitude_m - back.altitude_m) < 1e-3


def test_list_files():
    with tempfile.TemporaryDirectory() as tmp:
        for _ in range(3):
            r = DataRecorder(save_dir=tmp)
            r.start()
            r.record(_make_records(1)[0])
            r.stop()
        files = DataRecorder.list_files(tmp)
        assert len(files) == 3


def test_stop_without_start_returns_none():
    recorder = DataRecorder()
    result = recorder.stop()
    assert result is None