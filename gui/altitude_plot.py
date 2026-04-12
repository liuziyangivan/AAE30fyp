"""
gui/altitude_plot.py
实时飞行数据折线图 —— 性能优化版

优化内容：
  1. 折线图面板固定总高度（不随数据变化撑大布局），彻底消除右侧抖动
  2. _redraw 节流：仅当距上次重绘 > 80ms 时才真正刷新（约 12fps），
     避免仿真每帧 append 都触发全面重绘
  3. FillBetweenItem 改用 PlotDataItem 对象复用，减少对象重建
  4. 已移除电池（BATTERY/SOC）子图
"""

from __future__ import annotations
import time
from collections import deque
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

# ══════════════════════════════════════════════════════════
#  配色
# ══════════════════════════════════════════════════════════
CLR_BG      = "#0d1117"
CLR_PANEL   = "#161b22"
CLR_BORDER  = "#21262d"

CLR_ALT     = "#00d4ff"
CLR_VEL     = "#3fb950"
CLR_THR     = "#f0a030"
CLR_TARGET  = "#e0af68"
CLR_WEIGHT  = "#f85149"
CLR_TEXT    = "#c9d1d9"
CLR_GRID    = "#30363d"

FILL_ALT    = (0,   212, 255, 35)
FILL_VEL    = (63,  185,  80, 35)
FILL_THR    = (240, 160,  48, 35)

WINDOW_S    = 30.0
MAX_PTS     = 2000
LINE_W      = 2.5
FONT_AXIS   = "10pt"

# 折线图面板固定总高度（px）—— 防止撑大布局
PLOT_FIXED_HEIGHT = 480
# 节流间隔（秒）
REDRAW_THROTTLE_S = 0.08


class _LiveLabel(pg.TextItem):
    def __init__(self, color: str, fmt: str = "{:.1f}"):
        super().__init__(text="", color=color, anchor=(0.0, 0.5))
        self._fmt = fmt
        font = QtGui.QFont("Consolas", 9)
        font.setBold(True)
        self.setFont(font)

    def update_value(self, x: float, y: float, value: float):
        self.setText(self._fmt.format(value))
        self.setPos(x, y)


class AltitudePlot(QtWidgets.QWidget):
    """
    实时三图面板：高度 / 速度 / 推力

    固定高度，节流重绘，不影响外层布局。
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._t      = deque(maxlen=MAX_PTS)
        self._alt    = deque(maxlen=MAX_PTS)
        self._vel    = deque(maxlen=MAX_PTS)
        self._thrust = deque(maxlen=MAX_PTS)

        self._last_redraw = 0.0   # 节流时间戳

        # 固定高度，防止撑大右侧面板
        self.setFixedHeight(PLOT_FIXED_HEIGHT)
        self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding,
                           QtWidgets.QSizePolicy.Policy.Fixed)

        pg.setConfigOptions(antialias=True)
        self._build_ui()

    # ──────────────────────────────────────────────────────
    #  构建 UI
    # ──────────────────────────────────────────────────────
    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        # ── 高度图 ────────────────────────────────────────
        self._pw_alt = self._make_plot_widget()
        self._fill_alt_upper = pg.PlotDataItem([0], [0])
        self._fill_alt_lower = pg.PlotDataItem([0], [0])
        self._fill_alt = pg.FillBetweenItem(
            self._fill_alt_upper, self._fill_alt_lower,
            brush=pg.mkBrush(*FILL_ALT))
        self._pw_alt.addItem(self._fill_alt)
        self._curve_alt = self._pw_alt.plot(
            pen=pg.mkPen(CLR_ALT, width=LINE_W))
        self._target_line = pg.InfiniteLine(
            angle=0, pos=0,
            pen=pg.mkPen(CLR_TARGET, width=1.2,
                         style=QtCore.Qt.PenStyle.DashLine),
            label="target {value:.1f} m",
            labelOpts={"color": CLR_TARGET, "position": 0.92,
                       "fill": pg.mkBrush(13, 17, 23, 180)},
        )
        self._pw_alt.addItem(self._target_line)
        self._label_alt = _LiveLabel(CLR_ALT, "{:.1f} m")
        self._pw_alt.addItem(self._label_alt)
        self._add_title(self._pw_alt, "ALTITUDE", CLR_ALT)
        self._configure_axes(self._pw_alt, "m", CLR_ALT, show_x=False)
        self._pw_alt.setYRange(0, 20)

        # ── 速度图 ────────────────────────────────────────
        self._pw_vel = self._make_plot_widget()
        self._fill_vel_upper = pg.PlotDataItem([0], [0])
        self._fill_vel_lower = pg.PlotDataItem([0], [0])
        self._fill_vel = pg.FillBetweenItem(
            self._fill_vel_upper, self._fill_vel_lower,
            brush=pg.mkBrush(*FILL_VEL))
        self._pw_vel.addItem(self._fill_vel)
        self._curve_vel = self._pw_vel.plot(
            pen=pg.mkPen(CLR_VEL, width=LINE_W))
        self._pw_vel.addItem(
            pg.InfiniteLine(angle=0, pos=0,
                            pen=pg.mkPen(CLR_GRID, width=1)))
        self._label_vel = _LiveLabel(CLR_VEL, "{:.2f} m/s")
        self._pw_vel.addItem(self._label_vel)
        self._add_title(self._pw_vel, "VELOCITY", CLR_VEL)
        self._configure_axes(self._pw_vel, "m/s", CLR_VEL, show_x=False)
        self._pw_vel.setYRange(-2, 5)

        # ── 推力图 ────────────────────────────────────────
        self._pw_thr = self._make_plot_widget()
        self._fill_thr_upper = pg.PlotDataItem([0], [0])
        self._fill_thr_lower = pg.PlotDataItem([0], [0])
        self._fill_thr = pg.FillBetweenItem(
            self._fill_thr_upper, self._fill_thr_lower,
            brush=pg.mkBrush(*FILL_THR))
        self._pw_thr.addItem(self._fill_thr)
        self._curve_thr = self._pw_thr.plot(
            pen=pg.mkPen(CLR_THR, width=LINE_W))
        self._weight_line = pg.InfiniteLine(
            angle=0, pos=608.2,
            pen=pg.mkPen(CLR_WEIGHT, width=1.2,
                         style=QtCore.Qt.PenStyle.DashLine),
            label="weight {value:.0f} N",
            labelOpts={"color": CLR_WEIGHT, "position": 0.08,
                       "fill": pg.mkBrush(13, 17, 23, 180)},
        )
        self._pw_thr.addItem(self._weight_line)
        self._label_thr = _LiveLabel(CLR_THR, "{:.0f} N")
        self._pw_thr.addItem(self._label_thr)
        self._add_title(self._pw_thr, "THRUST", CLR_THR)
        # 推力图显示 X 轴时间刻度（原来由电池图承担，现在改到推力图）
        self._configure_axes(self._pw_thr, "N", CLR_THR, show_x=True)
        self._pw_thr.setYRange(0, 800)
        
        # 各图高度权重（总和 = PLOT_FIXED_HEIGHT）
        layout.addWidget(self._pw_alt, 3)
        layout.addWidget(self._pw_vel, 2)
        layout.addWidget(self._pw_thr, 2)

    # ──────────────────────────────────────────────────────
    #  工厂 & 样式辅助
    # ──────────────────────────────────────────────────────
    @staticmethod
    def _make_plot_widget() -> pg.PlotWidget:
        pw = pg.PlotWidget()
        pw.setBackground(CLR_PANEL)
        pw.getPlotItem().setContentsMargins(2, 2, 12, 2)
        return pw

    @staticmethod
    def _configure_axes(pw, ylabel, color, show_x):
        pi = pw.getPlotItem()
        pi.showGrid(x=True, y=True, alpha=0.18)
        tick_font = QtGui.QFont("Consolas", 8)

        left = pw.getAxis("left")
        left.setTextPen(pg.mkPen(CLR_TEXT))
        left.setPen(pg.mkPen(CLR_BORDER))
        left.setLabel(ylabel, **{"color": color, "font-size": FONT_AXIS})
        left.setTickFont(tick_font)

        bottom = pw.getAxis("bottom")
        bottom.setPen(pg.mkPen(CLR_BORDER))
        if show_x:
            bottom.setTextPen(pg.mkPen(CLR_TEXT))
            bottom.setLabel("Time (s)",
                            **{"color": CLR_TEXT, "font-size": FONT_AXIS})
            bottom.setTickFont(tick_font)
        else:
            bottom.setStyle(showValues=False)
            bottom.setLabel("")

        for side in ("top",):
            pw.getAxis(side).setPen(pg.mkPen(CLR_BORDER))
            pw.getAxis(side).setStyle(showValues=False)

    @staticmethod
    def _add_title(pw, text, color):
        ti = pg.TextItem(text=text, color=color, anchor=(0, 0))
        font = QtGui.QFont("Consolas", 9)
        font.setBold(True)
        font.setLetterSpacing(
            QtGui.QFont.SpacingType.AbsoluteSpacing, 1.5)
        ti.setFont(font)
        pw.getViewBox().sigRangeChanged.connect(
            lambda vb, r, _ti=ti, _pw=pw: _reanchor_title(_ti, _pw))
        pw.addItem(ti, ignoreBounds=True)
        pw._title_item = ti

    # ──────────────────────────────────────────────────────
    #  公开接口
    # ──────────────────────────────────────────────────────
    def set_target_alt(self, alt_m: float):
        self._target_line.setPos(alt_m)

    def set_weight(self, weight_N: float):
        self._weight_line.setPos(weight_N)

    def append(self, t: float, alt: float, vel: float, thrust: float,
               soc: float = 1.0, battery_temp_C: float = 25.0,
               range_est_m: float = 0.0):
        """soc / battery_temp_C / range_est_m 参数保留签名兼容，不再绘制。"""
        self._t.append(t)
        self._alt.append(alt)
        self._vel.append(vel)
        self._thrust.append(thrust)

        # 节流：距上次重绘不足 80ms 则跳过
        now = time.monotonic()
        if now - self._last_redraw >= REDRAW_THROTTLE_S:
            self._last_redraw = now
            self._redraw()

    def clear(self):
        for q in (self._t, self._alt, self._vel, self._thrust):
            q.clear()
        _e = np.array([])
        for c in (self._curve_alt, self._curve_vel, self._curve_thr):
            c.setData(_e, _e)
        for upper, lower in (
            (self._fill_alt_upper, self._fill_alt_lower),
            (self._fill_vel_upper, self._fill_vel_lower),
            (self._fill_thr_upper, self._fill_thr_lower),
        ):
            upper.setData(_e, _e)
            lower.setData(_e, _e)
        self._last_redraw = 0.0

    # ──────────────────────────────────────────────────────
    #  重绘（节流后调用）
    # ──────────────────────────────────────────────────────
    def _redraw(self):
        t_list = list(self._t)
        if not t_list:
            return

        t_now = t_list[-1]
        t_min = max(0.0, t_now - WINDOW_S)
        idx   = next((i for i, v in enumerate(t_list) if v >= t_min), 0)

        ts  = np.array(t_list[idx:])
        als = np.array(list(self._alt)[idx:])
        vls = np.array(list(self._vel)[idx:])
        trs = np.array(list(self._thrust)[idx:])

        if len(ts) < 2:
            return

        zeros = np.zeros_like(ts)

        # 主曲线
        self._curve_alt.setData(ts, als)
        self._curve_vel.setData(ts, vls)
        self._curve_thr.setData(ts, trs)

        # Fill：复用已有 PlotDataItem，只更新数据
        self._fill_alt_upper.setData(ts, als)
        self._fill_alt_lower.setData(ts, zeros)
        self._fill_vel_upper.setData(ts, vls)
        self._fill_vel_lower.setData(ts, zeros)
        self._fill_thr_upper.setData(ts, trs)
        self._fill_thr_lower.setData(ts, zeros)

        # 末端标注
        x_end = float(ts[-1]) + 0.4
        self._label_alt.update_value(x_end, float(als[-1]), float(als[-1]))
        self._label_vel.update_value(x_end, float(vls[-1]), float(vls[-1]))
        self._label_thr.update_value(x_end, float(trs[-1]), float(trs[-1]))

        # X 轴跟随
        for pw in (self._pw_alt, self._pw_vel, self._pw_thr):
            pw.setXRange(t_min, t_now + 2, padding=0)


# ══════════════════════════════════════════════════════════
#  标题定位辅助
# ══════════════════════════════════════════════════════════
def _reanchor_title(ti: pg.TextItem, pw: pg.PlotWidget):
    try:
        [[x0, x1], [y0, y1]] = pw.getViewBox().viewRange()
        ti.setPos(x0 + (x1-x0)*0.015, y1 - (y1-y0)*0.06)
    except Exception:
        pass