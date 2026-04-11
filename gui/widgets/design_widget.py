"""
gui/widgets/design_widget.py
参数化设计工具面板
"""

from __future__ import annotations
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QDoubleSpinBox, QSpinBox,
    QGroupBox, QFrame, QSizePolicy,
)
from PyQt6.QtCore import Qt
import pyqtgraph as pg

from core.vehicle import VehicleConfig
from core.design_editor import DesignEditor, DesignParams, DesignResult

# ── 颜色 ──────────────────────────────────────────────────
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


def _sep() -> QFrame:
    f = QFrame()
    f.setFrameShape(QFrame.Shape.HLine)
    f.setStyleSheet(f"color:#30363d;")
    return f


# ── 结果卡片 ──────────────────────────────────────────────
class _ResultCard(QWidget):
    def __init__(self, title: str, unit: str, color: str = CLR_ACCENT):
        super().__init__()
        self.setFixedHeight(70)
        self.setStyleSheet(
            f"background:{CLR_PANEL}; border-radius:8px;"
        )
        v = QVBoxLayout(self)
        v.setContentsMargins(10, 6, 10, 6)
        v.setSpacing(2)
        self._title_lbl = _label(title, size=9, color=CLR_SUBTEXT)
        self._val_lbl   = _label("—", size=14, color=color)
        self._unit_lbl  = _label(unit, size=8, color=CLR_SUBTEXT)
        v.addWidget(self._title_lbl)
        v.addWidget(self._val_lbl)
        v.addWidget(self._unit_lbl)

    def set_value(self, v: float, fmt: str = ".2f"):
        self._val_lbl.setText(format(v, fmt))


# ── 主面板 ────────────────────────────────────────────────
class DesignWidget(QWidget):

    def __init__(self, editor: DesignEditor, vehicle: VehicleConfig, parent=None):
        super().__init__(parent)
        self._editor  = editor
        self._vehicle = vehicle
        self._params  = DesignParams.from_vehicle(vehicle)
        self.setStyleSheet(f"background:{CLR_BG};")
        self._build_ui()
        # 首次渲染
        self._run_analysis()

    # ── 构建 UI ───────────────────────────────────────────
    def _build_ui(self):
        root = QHBoxLayout(self)
        root.setSpacing(12)
        root.setContentsMargins(12, 12, 12, 12)

        # 左列：参数输入
        left = QVBoxLayout()
        left.setSpacing(10)
        left.addWidget(self._build_rotor_group())
        left.addWidget(self._build_mass_group())
        left.addStretch()
        left.addWidget(self._build_apply_btn())

        # 右列：结果卡片 + 雷达图
        right = QVBoxLayout()
        right.setSpacing(10)
        right.addWidget(self._build_results_group())
        right.addWidget(self._build_chart())

        root.addLayout(left, 2)
        root.addLayout(right, 3)

    # ── 旋翼参数输入 ──────────────────────────────────────
    def _build_rotor_group(self) -> QGroupBox:
        box = QGroupBox("Rotor Parameters")
        box.setStyleSheet(self._group_style())
        grid = QGridLayout(box)
        grid.setSpacing(8)

        def _spin(lo, hi, val, dec=3, step=0.01) -> QDoubleSpinBox:
            s = QDoubleSpinBox()
            s.setRange(lo, hi)
            s.setValue(val)
            s.setDecimals(dec)
            s.setSingleStep(step)
            s.setStyleSheet(self._spin_style())
            return s

        p = self._params
        self._sp_radius   = _spin(0.1,  2.0,  p.rotor_radius_m,  3, 0.01)
        self._sp_chord    = _spin(0.01, 0.5,  p.rotor_chord_m,   3, 0.005)
        self._sp_Cl_alpha = _spin(1.0,  10.0, p.Cl_alpha,        3, 0.1)
        self._sp_Cd0      = _spin(0.001,0.1,  p.Cd0,             4, 0.001)
        self._sp_rpm_max  = _spin(1000, 12000,p.rpm_max,          0, 100)

        self._sp_blades   = QSpinBox()
        self._sp_blades.setRange(2, 8)
        self._sp_blades.setValue(p.num_blades)
        self._sp_blades.setStyleSheet(self._spin_style())

        rows = [
            ("Radius (m)",     self._sp_radius),
            ("Chord (m)",      self._sp_chord),
            ("Blades",         self._sp_blades),
            ("Cl_alpha",       self._sp_Cl_alpha),
            ("Cd0",            self._sp_Cd0),
            ("RPM max",        self._sp_rpm_max),
        ]
        for i, (lbl, widget) in enumerate(rows):
            grid.addWidget(_label(lbl, size=9, color=CLR_SUBTEXT), i, 0)
            grid.addWidget(widget, i, 1)

        return box

    # ── 质量参数输入 ──────────────────────────────────────
    def _build_mass_group(self) -> QGroupBox:
        box = QGroupBox("Mass Parameters")
        box.setStyleSheet(self._group_style())
        grid = QGridLayout(box)
        grid.setSpacing(8)

        p = self._params

        def _spin(lo, hi, val) -> QDoubleSpinBox:
            s = QDoubleSpinBox()
            s.setRange(lo, hi)
            s.setValue(val)
            s.setDecimals(1)
            s.setSingleStep(1.0)
            s.setStyleSheet(self._spin_style())
            return s

        self._sp_fuse_mass = _spin(5.0, 200.0, p.fuselage_mass_kg)
        self._sp_payload   = _spin(0.0, 200.0, p.payload_kg)

        grid.addWidget(_label("Fuselage mass (kg)", size=9, color=CLR_SUBTEXT), 0, 0)
        grid.addWidget(self._sp_fuse_mass, 0, 1)
        grid.addWidget(_label("Payload (kg)",        size=9, color=CLR_SUBTEXT), 1, 0)
        grid.addWidget(self._sp_payload,   1, 1)

        return box

    # ── Apply 按钮 ────────────────────────────────────────
    def _build_apply_btn(self) -> QWidget:
        w = QWidget()
        h = QHBoxLayout(w)
        h.setContentsMargins(0, 0, 0, 0)

        self._btn_apply = QPushButton("⚡  Apply Design")
        self._btn_apply.setStyleSheet(
            f"QPushButton {{ background:{CLR_ACCENT}; color:{CLR_BG};"
            f" border:none; border-radius:8px;"
            f" padding:10px 24px; font-family:Consolas;"
            f" font-size:11pt; font-weight:bold; }}"
            f"QPushButton:hover {{ background:#79c0ff; }}"
        )
        self._btn_apply.clicked.connect(self._on_apply)

        self._btn_reset = QPushButton("↺  Reset")
        self._btn_reset.setStyleSheet(
            f"QPushButton {{ background:{CLR_PANEL}; color:{CLR_SUBTEXT};"
            f" border:1px solid #30363d; border-radius:8px;"
            f" padding:10px 16px; font-family:Consolas; font-size:11pt; }}"
            f"QPushButton:hover {{ color:{CLR_TEXT}; border-color:{CLR_TEXT}; }}"
        )
        self._btn_reset.clicked.connect(self._on_reset)

        h.addWidget(self._btn_apply, 3)
        h.addWidget(self._btn_reset, 1)
        return w

    # ── 结果卡片 ──────────────────────────────────────────
    def _build_results_group(self) -> QGroupBox:
        box = QGroupBox("Design Results")
        box.setStyleSheet(self._group_style())
        grid = QGridLayout(box)
        grid.setSpacing(8)

        self._card_mass  = _ResultCard("Total Mass",      "kg",   CLR_TEXT)
        self._card_hover = _ResultCard("Hover RPM",       "RPM",  CLR_ACCENT)
        self._card_pwr   = _ResultCard("Hover Power",     "kW",   CLR_YELLOW)
        self._card_fm    = _ResultCard("Figure of Merit", "",     CLR_GREEN)
        self._card_dl    = _ResultCard("Disk Loading",    "N/m²", CLR_SUBTEXT)
        self._card_pl    = _ResultCard("Power Loading",   "N/W",  CLR_RED)

        cards = [
            self._card_mass,  self._card_hover,
            self._card_pwr,   self._card_fm,
            self._card_dl,    self._card_pl,
        ]
        for i, c in enumerate(cards):
            grid.addWidget(c, i // 2, i % 2)

        return box

    # ── 参数扫描图 ────────────────────────────────────────
    def _build_chart(self) -> QWidget:
        pg.setConfigOption("background", CLR_BG)
        pg.setConfigOption("foreground", CLR_TEXT)

        box = QGroupBox("Radius Sweep — Hover Power vs Rotor Radius")
        box.setStyleSheet(self._group_style())
        v = QVBoxLayout(box)

        self._plt = pg.PlotWidget()
        self._plt.setLabel("left",   "Hover Power (kW)")
        self._plt.setLabel("bottom", "Rotor Radius (m)")
        self._plt.showGrid(x=True, y=True, alpha=0.2)
        self._plt.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Expanding,
        )
        self._crv_sweep  = self._plt.plot(
            pen=pg.mkPen(CLR_ACCENT, width=2)
        )
        self._cur_point  = self._plt.plot(
            pen=None, symbol="o",
            symbolBrush=CLR_GREEN, symbolSize=12,
        )
        v.addWidget(self._plt)
        return box

    # ── 逻辑 ──────────────────────────────────────────────
    def _collect_params(self) -> DesignParams:
        return DesignParams(
            rotor_radius_m   = self._sp_radius.value(),
            rotor_chord_m    = self._sp_chord.value(),
            num_blades       = self._sp_blades.value(),
            rpm_max          = self._sp_rpm_max.value(),
            Cl_alpha         = self._sp_Cl_alpha.value(),
            Cd0              = self._sp_Cd0.value(),
            fuselage_mass_kg = self._sp_fuse_mass.value(),
            payload_kg       = self._sp_payload.value(),
        )

    def _on_apply(self):
        params = self._collect_params()
        result = self._editor.apply(params)
        self._update_cards(result)
        self._update_sweep(params)

    def _on_reset(self):
        p = DesignParams.from_vehicle(self._vehicle)
        self._sp_radius.setValue(p.rotor_radius_m)
        self._sp_chord.setValue(p.rotor_chord_m)
        self._sp_blades.setValue(p.num_blades)
        self._sp_rpm_max.setValue(p.rpm_max)
        self._sp_Cl_alpha.setValue(p.Cl_alpha)
        self._sp_Cd0.setValue(p.Cd0)
        self._sp_fuse_mass.setValue(p.fuselage_mass_kg)
        self._sp_payload.setValue(p.payload_kg)
        self._run_analysis()

    def _run_analysis(self):
        result = self._editor.apply(self._collect_params())
        self._update_cards(result)
        self._update_sweep(self._collect_params())

    def _update_cards(self, r: DesignResult):
        self._card_mass.set_value(r.total_mass_kg,   ".1f")
        self._card_hover.set_value(r.hover_rpm,       ".0f")
        self._card_pwr.set_value(r.hover_power_W / 1000, ".2f")
        self._card_fm.set_value(r.figure_of_merit,   ".3f")
        self._card_dl.set_value(r.disk_loading_Pa,   ".1f")
        self._card_pl.set_value(r.power_loading,      ".4f")

    def _update_sweep(self, current: DesignParams):
        """扫描旋翼半径，绘制悬停功率曲线"""
        from core.design_editor import DesignEditor
        from core.aero_engine import AeroEngine
        from dataclasses import replace as dc_replace
        import numpy as np

        radii, powers = [], []
        r_min = max(0.15, current.rotor_radius_m * 0.4)
        r_max = min(1.8,  current.rotor_radius_m * 2.0)

        for r in np.linspace(r_min, r_max, 40):
            p = dc_replace(current, rotor_radius_m=r)
            result = self._editor.apply(p)
            radii.append(r)
            powers.append(result.hover_power_W / 1000)

        self._crv_sweep.setData(radii, powers)

        # 当前设计点
        cur_result = self._editor.apply(current)
        self._cur_point.setData(
            [current.rotor_radius_m],
            [cur_result.hover_power_W / 1000],
        )

        # 还原当前参数（sweep 会改变 twin）
        self._editor.apply(current)

    # ── 样式 ──────────────────────────────────────────────
    @staticmethod
    def _group_style() -> str:
        return (
            f"QGroupBox {{ color:{CLR_SUBTEXT}; border:1px solid {CLR_PANEL};"
            f" border-radius:8px; margin-top:6px; padding:8px; }}"
            f"QGroupBox::title {{ subcontrol-origin:margin; left:10px; }}"
        )

    @staticmethod
    def _spin_style() -> str:
        return (
            f"QDoubleSpinBox, QSpinBox {{"
            f" background:{CLR_PANEL}; color:{CLR_TEXT};"
            f" border:1px solid #30363d; border-radius:6px;"
            f" padding:4px 8px; font-family:Consolas; font-size:10pt; }}"
        )