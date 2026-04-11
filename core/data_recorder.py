"""
core/data_recorder.py
仿真数据 CSV 记录器
"""

from __future__ import annotations
import csv
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass, fields
from typing import List


# ── 单帧数据结构 ──────────────────────────────────────────
@dataclass
class FlightRecord:
    t:           float   # 时间 (s)
    altitude_m:  float   # 高度 (m)
    velocity_ms: float   # 垂直速度 (m/s)
    rpm:         float   # 转速 (RPM)
    thrust_N:    float   # 总推力 (N)
    power_W:     float   # 总功率 (W)


# ── 记录器 ────────────────────────────────────────────────
class DataRecorder:
    """
    使用方法：
        recorder = DataRecorder()
        recorder.start()
        recorder.record(FlightRecord(...))
        path = recorder.stop()   # 返回保存的文件路径
    """

    HEADER = [f.name for f in fields(FlightRecord)]

    def __init__(self, save_dir: str | Path = "data"):
        self._dir = Path(save_dir)
        self._dir.mkdir(parents=True, exist_ok=True)
        self._rows: List[FlightRecord] = []
        self._active = False
        self._filepath: Path | None = None

    # ── 控制 ──────────────────────────────────────────────
    def start(self) -> None:
        """开始一次新记录，清空缓冲区"""
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")  # 加入微秒，确保唯一
        self._filepath = self._dir / f"flight_{ts}.csv"
        self._rows.clear()
        self._active = True

    def record(self, rec: FlightRecord) -> None:
        """追加一帧数据（仿真线程调用）"""
        if self._active:
            self._rows.append(rec)

    def stop(self) -> Path | None:
        """停止记录，写入 CSV，返回文件路径"""
        if not self._active or not self._filepath:
            return None
        self._active = False
        self._write_csv()
        return self._filepath

    # ── 读取（回放用）─────────────────────────────────────
    @staticmethod
    def load(csv_path: str | Path) -> List[FlightRecord]:
        """从 CSV 加载历史记录"""
        records = []
        with open(csv_path, newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                records.append(FlightRecord(
                    t           = float(row["t"]),
                    altitude_m  = float(row["altitude_m"]),
                    velocity_ms = float(row["velocity_ms"]),
                    rpm         = float(row["rpm"]),
                    thrust_N    = float(row["thrust_N"]),
                    power_W     = float(row["power_W"]),
                ))
        return records

    @staticmethod
    def list_files(save_dir: str | Path = "data") -> List[Path]:
        """返回 data/ 目录下所有 flight_*.csv，按时间倒序"""
        d = Path(save_dir)
        files = sorted(d.glob("flight_*.csv"), reverse=True)
        return files

    # ── 内部 ──────────────────────────────────────────────
    def _write_csv(self) -> None:
        with open(self._filepath, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=self.HEADER)
            writer.writeheader()
            for r in self._rows:
                writer.writerow({
                    "t":           f"{r.t:.3f}",
                    "altitude_m":  f"{r.altitude_m:.4f}",
                    "velocity_ms": f"{r.velocity_ms:.4f}",
                    "rpm":         f"{r.rpm:.1f}",
                    "thrust_N":    f"{r.thrust_N:.3f}",
                    "power_W":     f"{r.power_W:.2f}",
                })