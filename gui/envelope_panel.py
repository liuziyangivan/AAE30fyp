"""
gui/envelope_panel.py
飞行包线分析面板 ——
  上图：推力 & 推力裕度 vs RPM
  中图：功率 vs RPM
  下图：悬停效率 FM vs RPM
  顶部：5 张统计卡片
"""

from __future__ import annotations
from typing import List

import pyqtgraph as pg
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QDoubleSpinBox, QFrame,
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont

from core.vehicle import VehicleConfig
from core.performance_analysis import PerformanceAnalyzer, EnvelopePoint

# ── 配色（与 altitude_plot 保持一致）──────────────────────
CLR_BG     = "#1E1E2E"
CLR_PANEL  = "#2A2A3E"
CLR_TEXT   = "#CDD6F4"
CLR_GRID   = "#313244"
CLR_GREEN  = "#A6E3A1"
CLR_ACCENT = "#89B4FA"
CLR_YELLOW = "#F9E2AF"
CLR_RED    = "#F38BA8"
CLR_ORANGE = "#FAB387"


# ── 统计卡片 ──────────────────────────────────────────────
class _StatCard(QFrame):
    def __init__(self, title: str):
        super().__init__()
        self.setStyleSheet(
            f"QFrame {{ background:{CLR_PANEL}; border-radius:8px; }}"
        )
        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 8, 10, 8)
        lay.setSpacing(2)

        lbl_title = QLabel(title)
        lbl_title.setStyleSheet(f"color:{CLR_TEXT}; font-size:10px;")
        lbl_title.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self._val = QLabel("—")
        fnt = QFont()
        fnt.setPointSize(15)
        fnt.setBold(True)
        self._val.setFont(fnt)
        self._val.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self._unit = QLabel("")
        self._unit.setStyleSheet(f"color:{CLR_TEXT}; font-size:9px;")
        self._unit.setAlignment(Qt.AlignmentFlag.AlignCenter)

        for w in (lbl_title, self._val, self._unit):
            lay.addWidget(w)

    def set(self, value: str, unit: str = "", color: str = CLR_GREEN):
        self._val.setText(value)
        self._unit.setText(unit)
        self._val.setStyleSheet(f"color:{color};")


# ── 包线面板主体 ──────────────────────────────────────────
class EnvelopePanel(QWidget):

    def __init__(self, vehicle: VehicleConfig, parent=None):
        super().__init__(parent)
        self._vehicle  = vehicle
        self._analyzer = PerformanceAnalyzer(vehicle)
        self._build_ui()

    # ── 布局 ─────────────────────────────────────────────
    def _build_ui(self):
        self.setStyleSheet(f"background:{CLR_BG}; color:{CLR_TEXT};")
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        # 顶部控制栏
        ctrl = QHBoxLayout()
        ctrl.setSpacing(10)
        lbl = QLabel("Battery Capacity:")
        lbl.setStyleSheet(f"color:{CLR_TEXT};")
        ctrl.addWidget(lbl)

        self._bat_spin = QDoubleSpinBox()
        self._bat_spin.setRange(1.0, 500.0)
        self._bat_spin.setValue(10.0)
        self._bat_spin.setSuffix("  kWh")
        self._bat_spin.setDecimals(1)
        self._bat_spin.setFixedWidth(120)
        self._bat_spin.setStyleSheet(
            f"background:{CLR_PANEL}; color:{CLR_TEXT};"
            "border:1px solid #45475a; border-radius:4px; padding:2px 6px;"
        )
        ctrl.addWidget(self._bat_spin)

        self._btn = QPushButton("🔄  Run Analysis")
        self._btn.setFixedHeight(32)
        self._btn.setStyleSheet(
            f"QPushButton {{ background:{CLR_ACCENT}; color:#1E1E2E;"
            "border-radius:6px; font-weight:bold; font-size:12px; padding:0 14px; }}"
            f"QPushButton:hover {{ background:#B4D0FF; }}"
        )
        self._btn.clicked.connect(self.run)
        ctrl.addWidget(self._btn)
        ctrl.addStretch()
        root.addLayout(ctrl)

        # 统计卡片行
        self._c_rpm  = _StatCard("Hover RPM")
        self._c_pwr  = _StatCard("Hover Power")
        self._c_fm   = _StatCard("Figure of Merit")
        self._c_end  = _StatCard("Est. Endurance")
        self._c_tmax = _StatCard("Max Thrust")

        cards = QHBoxLayout()
        cards.setSpacing(8)
        for c in (self._c_rpm, self._c_pwr, self._c_fm,
                  self._c_end, self._c_tmax):
            cards.addWidget(c)
        root.addLayout(cards)

        # 图表
        self._pw_thr = pg.PlotWidget()
        self._pw_pwr = pg.PlotWidget()
        self._pw_fm  = pg.PlotWidget()

        _style(self._pw_thr, "Thrust & Margin (N)", CLR_GREEN)
        _style(self._pw_pwr, "Power (kW)",           CLR_ACCENT)
        _style(self._pw_fm,  "Figure of Merit",      CLR_YELLOW)

        root.addWidget(self._pw_thr, 3)
        root.addWidget(self._pw_pwr, 2)
        root.addWidget(self._pw_fm,  2)

        # 曲线
        self._cu_thrust = self._pw_thr.plot(
            pen=pg.mkPen(CLR_GREEN,  width=2), name="Total Thrust")
        self._cu_margin = self._pw_thr.plot(
            pen=pg.mkPen(CLR_ORANGE, width=1.5,
                         style=pg.QtCore.Qt.PenStyle.DashLine),
            name="Thrust Margin",
        )
        self._cu_power  = self._pw_pwr.plot(pen=pg.mkPen(CLR_ACCENT, width=2))
        self._cu_fm     = self._pw_fm.plot( pen=pg.mkPen(CLR_YELLOW, width=2))

        # 参考线：重力（推力图）
        W = self._vehicle.total_weight_N
        self._pw_thr.addItem(pg.InfiniteLine(
            angle=0, pos=W,
            pen=pg.mkPen(CLR_RED, width=1,
                         style=pg.QtCore.Qt.PenStyle.DotLine),
            label=f"Weight {W:.0f} N",
            labelOpts={"color": CLR_RED, "position": 0.92},
        ))
        # 零裕度线
        self._pw_thr.addItem(pg.InfiniteLine(
            angle=0, pos=0, pen=pg.mkPen(CLR_GRID, width=1)))

        # 悬停转速竖线（三图各一条）
        _dash = pg.QtCore.Qt.PenStyle.DashLine
        self._vt = pg.InfiniteLine(angle=90, pos=0,
            pen=pg.mkPen(CLR_YELLOW, width=1, style=_dash),
            label="hover", labelOpts={"color": CLR_YELLOW, "position": 0.92})
        self._vp = pg.InfiniteLine(angle=90, pos=0,
            pen=pg.mkPen(CLR_YELLOW, width=1, style=_dash))
        self._vf = pg.InfiniteLine(angle=90, pos=0,
            pen=pg.mkPen(CLR_YELLOW, width=1, style=_dash))
        self._pw_thr.addItem(self._vt)
        self._pw_pwr.addItem(self._vp)
        self._pw_fm.addItem(self._vf)

        self._pw_thr.addLegend(offset=(10, 10))

    # ── 运行分析 ─────────────────────────────────────────
    def run(self):
        pts: List[EnvelopePoint] = self._analyzer.sweep_rpm(n_points=150)

        rpms    = [p.rpm             for p in pts]
        thrusts = [p.thrust_N        for p in pts]
        margins = [p.thrust_margin_N for p in pts]
        powers  = [p.power_kW        for p in pts]
        fms     = [p.figure_of_merit for p in pts]

        self._cu_thrust.setData(rpms, thrusts)
        self._cu_margin.setData(rpms, margins)
        self._cu_power.setData( rpms, powers)
        self._cu_fm.setData(    rpms, fms)

        # 悬停转速
        seed  = self._analyzer.engine.compute(
            self._vehicle.rotors.rpm_max * 0.5)
        hover = self._analyzer.engine.compute(seed.hover_rpm)
        hrpm  = hover.hover_rpm

        for vl in (self._vt, self._vp, self._vf):
            vl.setPos(hrpm)

        # 航时
        bat   = self._bat_spin.value()
        end   = self._analyzer.endurance(bat)

        # 更新卡片
        self._c_rpm.set( f"{hrpm:.0f}",            "RPM",     CLR_GREEN)
        self._c_pwr.set( f"{end.hover_power_kW:.2f}", "kW",   CLR_ACCENT)
        self._c_fm.set(  f"{hover.rotor.figure_of_merit:.3f}", "",CLR_YELLOW)
        self._c_end.set( f"{end.hover_endurance_min:.1f}", "min", CLR_ORANGE)
        self._c_tmax.set(f"{max(thrusts):.0f}",    "N",       CLR_RED)


# ── 图表样式工具函数 ──────────────────────────────────────
def _style(pw: pg.PlotWidget, ylabel: str, color: str):
    pw.setBackground(CLR_BG)
    pw.showGrid(x=True, y=True, alpha=0.15)
    pw.getAxis("left").setTextPen(pg.mkPen(color))
    pw.getAxis("bottom").setTextPen(pg.mkPen(CLR_TEXT))
    pw.getAxis("left").setLabel(
        ylabel, **{"color": color, "font-size": "9pt"})
    pw.getAxis("bottom").setLabel(
        "RPM", **{"color": CLR_TEXT, "font-size": "9pt"})
    for axis in ("left", "bottom", "right", "top"):
        pw.getAxis(axis).setPen(pg.mkPen(CLR_PANEL))