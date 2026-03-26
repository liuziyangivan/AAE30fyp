"""
gui/altitude_plot.py
实时飞行数据折线图（pyqtgraph）
显示：高度 / 速度 / 推力，滚动时间窗口
"""

from __future__ import annotations
from collections import deque
import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout
from PyQt6.QtGui import QFont

# 颜色
CLR_BG     = "#1e1e2e"
CLR_PANEL  = "#2a2a3e"
CLR_GREEN  = "#9ece6a"
CLR_ACCENT = "#7aa2f7"
CLR_YELLOW = "#e0af68"
CLR_RED    = "#f7768e"
CLR_TEXT   = "#cdd6f4"
CLR_GRID   = "#2a2a3e"

WINDOW_S   = 30.0      # 滚动窗口宽度（秒）
MAX_PTS    = 2000      # 最多保留的数据点数


class AltitudePlot(QWidget):
    """
    可嵌入主窗口的实时折线图组件。

    用法：
        plot = AltitudePlot()
        plot.append(t, altitude_m, velocity_ms, thrust_N)
        plot.clear()
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._t      = deque(maxlen=MAX_PTS)
        self._alt    = deque(maxlen=MAX_PTS)
        self._vel    = deque(maxlen=MAX_PTS)
        self._thrust = deque(maxlen=MAX_PTS)

        self._build_ui()

    # ── 构建 UI ───────────────────────────────────────────
    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        pg.setConfigOptions(antialias=True, background=CLR_BG)

        # ── 上图：高度 ────────────────────────────────────
        self._pw_alt = pg.PlotWidget()
        self._style_plot(self._pw_alt, "Altitude (m)", CLR_GREEN)
        self._curve_alt = self._pw_alt.plot(
            pen=pg.mkPen(CLR_GREEN, width=2), name="Alt"
        )
        # 目标高度虚线（后续由外部设置）
        self._target_line = pg.InfiniteLine(
            angle=0, pos=0,
            pen=pg.mkPen(CLR_YELLOW, width=1, style=pg.QtCore.Qt.PenStyle.DashLine),
            label="target", labelOpts={"color": CLR_YELLOW, "position": 0.95},
        )
        self._pw_alt.addItem(self._target_line)

        # ── 中图：速度 ────────────────────────────────────
        self._pw_vel = pg.PlotWidget()
        self._style_plot(self._pw_vel, "Velocity (m/s)", CLR_ACCENT)
        self._curve_vel = self._pw_vel.plot(
            pen=pg.mkPen(CLR_ACCENT, width=2), name="Vel"
        )
        # 零线
        self._pw_vel.addItem(
            pg.InfiniteLine(angle=0, pos=0,
                            pen=pg.mkPen(CLR_GRID, width=1))
        )

        # ── 下图：推力 ────────────────────────────────────
        self._pw_thr = pg.PlotWidget()
        self._style_plot(self._pw_thr, "Thrust (N)", CLR_YELLOW)
        self._curve_thr = self._pw_thr.plot(
            pen=pg.mkPen(CLR_YELLOW, width=2), name="Thrust"
        )
        # 重力线（608 N，默认值）
        self._weight_line = pg.InfiniteLine(
            angle=0, pos=608.2,
            pen=pg.mkPen(CLR_RED, width=1,
                         style=pg.QtCore.Qt.PenStyle.DashLine),
            label="weight", labelOpts={"color": CLR_RED, "position": 0.05},
        )
        self._pw_thr.addItem(self._weight_line)

        layout.addWidget(self._pw_alt,  3)
        layout.addWidget(self._pw_vel,  2)
        layout.addWidget(self._pw_thr,  2)

    # ── 样式统一 ──────────────────────────────────────────
    @staticmethod
    def _style_plot(pw: pg.PlotWidget, ylabel: str, color: str):
        pw.setBackground(CLR_BG)
        pw.showGrid(x=True, y=True, alpha=0.15)
        pw.getAxis("left").setTextPen(pg.mkPen(color))
        pw.getAxis("bottom").setTextPen(pg.mkPen(CLR_TEXT))
        pw.getAxis("left").setLabel(ylabel,
                                    **{"color": color, "font-size": "9pt"})
        pw.getAxis("bottom").setLabel(
            "Time (s)", **{"color": CLR_TEXT, "font-size": "9pt"}
        )
        for axis in ("left", "bottom", "right", "top"):
            pw.getAxis(axis).setPen(pg.mkPen(CLR_PANEL))

    # ── 公开接口 ──────────────────────────────────────────
    def set_target_alt(self, alt_m: float):
        self._target_line.setPos(alt_m)

    def set_weight(self, weight_N: float):
        self._weight_line.setPos(weight_N)

    def append(self, t: float, alt: float, vel: float, thrust: float):
        self._t.append(t)
        self._alt.append(alt)
        self._vel.append(vel)
        self._thrust.append(thrust)
        self._redraw()

    def clear(self):
        self._t.clear()
        self._alt.clear()
        self._vel.clear()
        self._thrust.clear()
        self._curve_alt.setData([], [])
        self._curve_vel.setData([], [])
        self._curve_thr.setData([], [])

    # ── 重绘（滚动窗口）─────────────────────────────────
    def _redraw(self):
        t_list = list(self._t)
        if not t_list:
            return

        t_now = t_list[-1]
        t_min = max(0.0, t_now - WINDOW_S)

        # 找滚动窗口起点
        idx = 0
        for i, v in enumerate(t_list):
            if v >= t_min:
                idx = i
                break

        ts  = t_list[idx:]
        als = list(self._alt)[idx:]
        vls = list(self._vel)[idx:]
        trs = list(self._thrust)[idx:]

        self._curve_alt.setData(ts, als)
        self._curve_vel.setData(ts, vls)
        self._curve_thr.setData(ts, trs)

        # X 轴跟随
        for pw in (self._pw_alt, self._pw_vel, self._pw_thr):
            pw.setXRange(t_min, t_now + 1, padding=0)