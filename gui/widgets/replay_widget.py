"""
gui/widgets/replay_widget.py
历史飞行数据回放面板
"""

from __future__ import annotations
import time
import threading
from pathlib import Path

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QComboBox, QLabel, QSlider, QGroupBox, QSizePolicy,
)
from PyQt6.QtCore import Qt, pyqtSignal, QObject
import pyqtgraph as pg

from core.data_recorder import DataRecorder, FlightRecord
from typing import List

# ── 颜色常量（与 main_window 保持一致）────────────────────
CLR_BG      = "#0d1117"
CLR_PANEL   = "#161b22"
CLR_ACCENT  = "#58a6ff"
CLR_GREEN   = "#3fb950"
CLR_YELLOW  = "#d29922"
CLR_RED     = "#f85149"
CLR_TEXT    = "#e6edf3"
CLR_SUBTEXT = "#8b949e"


def _label(text: str, size: int = 10, color: str = CLR_TEXT) -> QLabel:
    lbl = QLabel(text)
    lbl.setStyleSheet(
        f"color:{color}; font-family:Consolas; font-size:{size}pt;"
    )
    return lbl


# ── 信号桥（线程 → GUI）──────────────────────────────────
class _ReplayBridge(QObject):
    frame_ready = pyqtSignal(object)   # FlightRecord
    replay_done = pyqtSignal()


# ── 回放面板主体 ──────────────────────────────────────────
class ReplayWidget(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"background:{CLR_BG};")

        self._records: List[FlightRecord] = []
        self._playing = False
        self._speed   = 1.0          # 回放倍速
        self._bridge  = _ReplayBridge()
        self._bridge.frame_ready.connect(self._on_frame)
        self._bridge.replay_done.connect(self._on_done)

        self._build_ui()
        self._refresh_file_list()

    # ── 构建 UI ───────────────────────────────────────────
    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setSpacing(10)
        root.setContentsMargins(12, 12, 12, 12)

        root.addWidget(self._build_file_bar())
        root.addWidget(self._build_controls())
        root.addWidget(self._build_charts())

    def _build_file_bar(self) -> QGroupBox:
        box = QGroupBox("Select Flight Record")
        box.setStyleSheet(self._group_style())
        row = QHBoxLayout(box)
        row.setSpacing(8)

        self._combo = QComboBox()
        self._combo.setStyleSheet(
            f"QComboBox {{ background:{CLR_PANEL}; color:{CLR_TEXT};"
            f" border:1px solid #30363d; border-radius:6px;"
            f" padding:4px 8px; font-family:Consolas; font-size:10pt; }}"
            f"QComboBox::drop-down {{ border:none; }}"
            f"QComboBox QAbstractItemView {{ background:{CLR_PANEL};"
            f" color:{CLR_TEXT}; selection-background-color:{CLR_ACCENT}; }}"
        )
        self._combo.currentIndexChanged.connect(self._on_file_selected)

        btn_refresh = QPushButton("🔄 Refresh")
        btn_refresh.setStyleSheet(self._btn_style(CLR_SUBTEXT))
        btn_refresh.clicked.connect(self._refresh_file_list)

        self._info_lbl = _label("No file loaded", size=9, color=CLR_SUBTEXT)

        row.addWidget(_label("File:", size=10))
        row.addWidget(self._combo, 4)
        row.addWidget(btn_refresh)
        row.addWidget(self._info_lbl, 2)
        return box

    def _build_controls(self) -> QGroupBox:
        box = QGroupBox("Playback Controls")
        box.setStyleSheet(self._group_style())
        row = QHBoxLayout(box)
        row.setSpacing(10)

        # 进度条
        self._progress = QSlider(Qt.Orientation.Horizontal)
        self._progress.setRange(0, 100)
        self._progress.setValue(0)
        self._progress.setEnabled(False)
        self._progress.setStyleSheet(
            f"QSlider::groove:horizontal {{ height:6px; background:{CLR_PANEL};"
            f" border-radius:3px; }}"
            f"QSlider::handle:horizontal {{ width:14px; height:14px;"
            f" background:{CLR_ACCENT}; border-radius:7px; margin:-4px 0; }}"
            f"QSlider::sub-page:horizontal {{ background:{CLR_ACCENT};"
            f" border-radius:3px; }}"
        )
        self._progress_lbl = _label("0.0 s", size=9, color=CLR_SUBTEXT)

        # 倍速选择
        self._speed_combo = QComboBox()
        self._speed_combo.addItems(["0.5×", "1×", "2×", "4×"])
        self._speed_combo.setCurrentIndex(1)
        self._speed_combo.setFixedWidth(70)
        self._speed_combo.setStyleSheet(
            f"QComboBox {{ background:{CLR_PANEL}; color:{CLR_TEXT};"
            f" border:1px solid #30363d; border-radius:6px;"
            f" padding:2px 6px; font-family:Consolas; font-size:10pt; }}"
            f"QComboBox::drop-down {{ border:none; }}"
            f"QComboBox QAbstractItemView {{ background:{CLR_PANEL};"
            f" color:{CLR_TEXT}; selection-background-color:{CLR_ACCENT}; }}"
        )
        self._speed_combo.currentIndexChanged.connect(self._on_speed_change)

        # 播放 / 停止按钮
        self._btn_play = QPushButton("▶  PLAY")
        self._btn_play.setStyleSheet(self._btn_style(CLR_GREEN))
        self._btn_play.setEnabled(False)
        self._btn_play.clicked.connect(self._toggle_play)

        self._btn_reset = QPushButton("⏮  RESET")
        self._btn_reset.setStyleSheet(self._btn_style(CLR_YELLOW))
        self._btn_reset.setEnabled(False)
        self._btn_reset.clicked.connect(self._reset)

        row.addWidget(_label("Progress:", size=10))
        row.addWidget(self._progress, 5)
        row.addWidget(self._progress_lbl)
        row.addWidget(_label("Speed:", size=10))
        row.addWidget(self._speed_combo)
        row.addWidget(self._btn_play)
        row.addWidget(self._btn_reset)
        return box

    def _build_charts(self) -> QWidget:
        pg.setConfigOption("background", CLR_BG)
        pg.setConfigOption("foreground", CLR_TEXT)

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setSpacing(8)
        layout.setContentsMargins(0, 0, 0, 0)

        def _make_plot(title: str, ylabel: str, color: str):
            pw = pg.PlotWidget(title=title)
            pw.setLabel("left", ylabel)
            pw.setLabel("bottom", "Time (s)")
            pw.showGrid(x=True, y=True, alpha=0.2)
            pw.setSizePolicy(
                QSizePolicy.Policy.Expanding,
                QSizePolicy.Policy.Expanding,
            )
            return pw

        # 高度图
        self._plt_alt  = _make_plot("Altitude",    "m",   CLR_ACCENT)
        self._crv_alt  = self._plt_alt.plot(pen=pg.mkPen(CLR_ACCENT, width=2))
        self._cur_alt  = self._plt_alt.plot(
            pen=None, symbol="o",
            symbolBrush=CLR_GREEN, symbolSize=10,
        )

        # 推力图
        self._plt_thr  = _make_plot("Total Thrust", "N",  CLR_GREEN)
        self._crv_thr  = self._plt_thr.plot(pen=pg.mkPen(CLR_GREEN, width=2))
        self._cur_thr  = self._plt_thr.plot(
            pen=None, symbol="o",
            symbolBrush=CLR_YELLOW, symbolSize=10,
        )

        # 功率图
        self._plt_pwr  = _make_plot("Total Power",  "kW", CLR_YELLOW)
        self._crv_pwr  = self._plt_pwr.plot(pen=pg.mkPen(CLR_YELLOW, width=2))
        self._cur_pwr  = self._plt_pwr.plot(
            pen=None, symbol="o",
            symbolBrush=CLR_RED, symbolSize=10,
        )

        layout.addWidget(self._plt_alt, 1)
        layout.addWidget(self._plt_thr, 1)
        layout.addWidget(self._plt_pwr, 1)
        return container

    # ── 文件列表 ──────────────────────────────────────────
    def _refresh_file_list(self):
        self._combo.blockSignals(True)
        self._combo.clear()
        files = DataRecorder.list_files("data")
        if not files:
            self._combo.addItem("(no records found)")
            self._combo.blockSignals(False)
            return
        for f in files:
            self._combo.addItem(f.name, userData=f)
        self._combo.blockSignals(False)
        self._combo.setCurrentIndex(0)
        self._on_file_selected(0)

    def _on_file_selected(self, idx: int):
        path: Path | None = self._combo.itemData(idx)
        if path is None or not path.exists():
            return
        self._records = DataRecorder.load(path)
        self._reset()

        n   = len(self._records)
        dur = self._records[-1].t if self._records else 0.0
        self._info_lbl.setText(
            f"{n} frames · {dur:.1f} s"
        )
        self._info_lbl.setStyleSheet(f"color:{CLR_GREEN}; font-family:Consolas; font-size:9pt;")

        # 绘制全量曲线
        ts  = [r.t          for r in self._records]
        alt = [r.altitude_m for r in self._records]
        thr = [r.thrust_N   for r in self._records]
        pwr = [r.power_W / 1000 for r in self._records]
        self._crv_alt.setData(ts, alt)
        self._crv_thr.setData(ts, thr)
        self._crv_pwr.setData(ts, pwr)

        # 重置游标到起点
        self._cur_alt.setData([ts[0]], [alt[0]])
        self._cur_thr.setData([ts[0]], [thr[0]])
        self._cur_pwr.setData([ts[0]], [pwr[0]])

        self._progress.setRange(0, n - 1)
        self._progress.setValue(0)
        self._progress.setEnabled(True)
        self._btn_play.setEnabled(True)
        self._btn_reset.setEnabled(True)

    # ── 播放控制 ──────────────────────────────────────────
    def _toggle_play(self):
        if self._playing:
            self._playing = False
            self._btn_play.setText("▶  PLAY")
            self._btn_play.setStyleSheet(self._btn_style(CLR_GREEN))
        else:
            self._start_play()

    def _start_play(self):
        if not self._records:
            return
        self._playing = True
        self._btn_play.setText("⬛  STOP")
        self._btn_play.setStyleSheet(self._btn_style(CLR_RED))

        start_idx = self._progress.value()

        def _run():
            records = self._records
            for i in range(start_idx, len(records)):
                if not self._playing:
                    break
                self._bridge.frame_ready.emit(records[i])
                # 计算帧间延迟
                if i + 1 < len(records):
                    dt = records[i + 1].t - records[i].t
                    time.sleep(dt / self._speed)
            if self._playing:
                self._bridge.replay_done.emit()

        threading.Thread(target=_run, daemon=True).start()

    def _reset(self):
        self._playing = False
        self._btn_play.setText("▶  PLAY")
        self._btn_play.setStyleSheet(self._btn_style(CLR_GREEN))
        self._progress.setValue(0)
        self._progress_lbl.setText("0.0 s")
        # 游标回起点
        if self._records:
            r = self._records[0]
            self._cur_alt.setData([r.t], [r.altitude_m])
            self._cur_thr.setData([r.t], [r.thrust_N])
            self._cur_pwr.setData([r.t], [r.power_W / 1000])

    def _on_speed_change(self, idx: int):
        speeds = [0.5, 1.0, 2.0, 4.0]
        self._speed = speeds[idx]

    # ── 帧回调（GUI 线程）─────────────────────────────────
    def _on_frame(self, rec: FlightRecord):
        idx = self._progress.value()
        # 找到当前帧索引
        try:
            idx = next(
                i for i, r in enumerate(self._records)
                if abs(r.t - rec.t) < 1e-6
            )
        except StopIteration:
            pass

        self._progress.blockSignals(True)
        self._progress.setValue(idx)
        self._progress.blockSignals(False)
        self._progress_lbl.setText(f"{rec.t:.1f} s")

        self._cur_alt.setData([rec.t], [rec.altitude_m])
        self._cur_thr.setData([rec.t], [rec.thrust_N])
        self._cur_pwr.setData([rec.t], [rec.power_W / 1000])

    def _on_done(self):
        self._playing = False
        self._btn_play.setText("▶  PLAY")
        self._btn_play.setStyleSheet(self._btn_style(CLR_GREEN))

    # ── 样式辅助 ──────────────────────────────────────────
    @staticmethod
    def _group_style() -> str:
        return (
            f"QGroupBox {{ color:{CLR_SUBTEXT}; border:1px solid {CLR_PANEL};"
            f" border-radius:8px; margin-top:6px; padding:8px; }}"
            f"QGroupBox::title {{ subcontrol-origin:margin; left:10px; }}"
        )

    @staticmethod
    def _btn_style(color: str) -> str:
        return (
            f"QPushButton {{ background:{CLR_PANEL}; color:{color};"
            f" border:1px solid {color}; border-radius:6px;"
            f" padding:6px 14px; font-family:Consolas; font-size:10pt; }}"
            f"QPushButton:hover {{ background:{color}; color:{CLR_BG}; }}"
        )