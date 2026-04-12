"""
gui/main_window.py  (v2 — Design-First 主控台)
====================================================
Tab 1  "🎛  主控台"
  左侧  : 3D 实时可视化（View3DWidget）
  右侧上: 👁  可观测性
            仪表卡片 × 6（ALT / V-SPEED / OPT RPM / THRUST / POWER / FM）
            高度进度条 + 实时三图折线
  右侧下: 🕹  可干预性
            ① Design Editor — 旋翼 & 机身参数输入，Apply 后
               由 AeroEngine + 二分法求解最优悬停 RPM，
               自动推送至 DigitalTwin，无需手动调节
            ② Auto Flight Simulation — 目标高度 + Start/Stop
            ③ Fault Injection — 旋翼失效模拟

Tab 2  "⏪  数据回放"
  保留原 ReplayWidget

手动 RPM 控制面板已移除；RPM 完全由算法决定。
====================================================
"""

from __future__ import annotations
import sys
import time
import threading
from pathlib import Path
from dataclasses import replace as dc_replace

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QSlider, QPushButton, QGroupBox,
    QProgressBar, QSizePolicy, QTabWidget,
    QSplitter, QScrollArea, QFrame,
    QDoubleSpinBox, QSpinBox,
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt6.QtGui import QFont, QPalette, QColor

from core.vehicle import load_vehicle
from core.event_bus import EventBus
from core.digital_twin import DigitalTwin, TwinState
from core.aero_engine import AeroEngine
from core.design_editor import DesignEditor, DesignParams, DesignResult
from core.fault_injector import FaultInjector, FAULT_PRESETS
from simulation.flight_sim import FlightSimulator, SimConfig, SimFrame
from gui.altitude_plot import AltitudePlot
from gui.widgets.replay_widget import ReplayWidget
from gui.widgets.view3d_widget import View3DWidget


# ═══════════════════════════════════════════════════════
#  颜色常量
# ═══════════════════════════════════════════════════════
CLR_BG      = "#0d1117"
CLR_PANEL   = "#161b22"
CLR_BORDER  = "#30363d"
CLR_ACCENT  = "#58a6ff"
CLR_GREEN   = "#3fb950"
CLR_YELLOW  = "#d29922"
CLR_RED     = "#f85149"
CLR_ORANGE  = "#e3b341"
CLR_PURPLE  = "#bc8cff"
CLR_TEXT    = "#e6edf3"
CLR_SUBTEXT = "#8b949e"


# ═══════════════════════════════════════════════════════
#  工具函数
# ═══════════════════════════════════════════════════════
def _label(text: str, size: int = 10, bold: bool = False,
           color: str = CLR_TEXT) -> QLabel:
    lbl = QLabel(text)
    font = QFont("Consolas", size)
    font.setBold(bold)
    lbl.setFont(font)
    lbl.setStyleSheet(f"color:{color}; background:transparent;")
    return lbl


def _sep() -> QFrame:
    f = QFrame()
    f.setFrameShape(QFrame.Shape.HLine)
    f.setStyleSheet(f"color:{CLR_BORDER};")
    return f


# ═══════════════════════════════════════════════════════
#  信号桥（仿真线程 → GUI 主线程）
# ═══════════════════════════════════════════════════════
class _SimBridge(QObject):
    frame_ready = pyqtSignal(object)   # SimFrame
    sim_done    = pyqtSignal()


# ═══════════════════════════════════════════════════════
#  仪表卡片
# ═══════════════════════════════════════════════════════
class MetricCard(QWidget):
    """大字数值 + 标题 + 单位 + 可选警告红框。"""

    def __init__(self, title: str, unit: str,
                 fmt: str = "{:.1f}", accent: str = CLR_ACCENT):
        super().__init__()
        self._fmt = fmt
        self.setFixedHeight(82)
        self.setSizePolicy(QSizePolicy.Policy.Expanding,
                           QSizePolicy.Policy.Fixed)
        self._set_border(CLR_BORDER)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(2)
        self._title_lbl = _label(title, size=8,  color=CLR_SUBTEXT)
        self._value_lbl = _label("—",   size=20, bold=True, color=accent)
        self._unit_lbl  = _label(unit,  size=8,  color=CLR_SUBTEXT)
        layout.addWidget(self._title_lbl)
        layout.addWidget(self._value_lbl)
        layout.addWidget(self._unit_lbl)

    def set_value(self, v: float) -> None:
        self._value_lbl.setText(self._fmt.format(v))

    def flash_warning(self, on: bool) -> None:
        self._set_border(CLR_RED if on else CLR_BORDER)

    def _set_border(self, color: str):
        self.setStyleSheet(
            f"background:{CLR_PANEL}; border-radius:8px;"
            f" border:1px solid {color};"
        )


# ═══════════════════════════════════════════════════════
#  设计参数行（Label + SpinBox）
# ═══════════════════════════════════════════════════════
class _ParamRow(QWidget):
    def __init__(self, label: str, lo: float, hi: float, val: float,
                 decimals: int = 3, step: float = 0.01,
                 unit: str = "", is_int: bool = False):
        super().__init__()
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(8)

        lbl = _label(label, size=9, color=CLR_SUBTEXT)
        lbl.setFixedWidth(128)
        row.addWidget(lbl)

        _ss = (
            f"background:{CLR_PANEL}; color:{CLR_TEXT};"
            f" border:1px solid {CLR_BORDER}; border-radius:5px;"
            f" padding:3px 6px; font-family:Consolas; font-size:9pt;"
        )
        if is_int:
            self._spin: QSpinBox | QDoubleSpinBox = QSpinBox()
            self._spin.setRange(int(lo), int(hi))
            self._spin.setValue(int(val))
            self._spin.setStyleSheet(f"QSpinBox {{ {_ss} }}")
        else:
            self._spin = QDoubleSpinBox()
            self._spin.setRange(lo, hi)
            self._spin.setValue(val)
            self._spin.setDecimals(decimals)
            self._spin.setSingleStep(step)
            self._spin.setStyleSheet(f"QDoubleSpinBox {{ {_ss} }}")

        row.addWidget(self._spin, 1)
        if unit:
            row.addWidget(_label(unit, size=8, color=CLR_SUBTEXT))

    @property
    def value(self) -> float:
        return float(self._spin.value())

    def set_value(self, v: float):
        self._spin.blockSignals(True)
        self._spin.setValue(v)
        self._spin.blockSignals(False)


# ═══════════════════════════════════════════════════════
#  旋翼故障按钮（双态）
# ═══════════════════════════════════════════════════════
class _RotorFaultBtn(QPushButton):
    def __init__(self, label: str):
        super().__init__(label)
        self._faulted = False
        self._apply_style()
        self.clicked.connect(self._toggle)

    def _toggle(self):
        self._faulted = not self._faulted
        self._apply_style()

    def _apply_style(self):
        if self._faulted:
            self.setStyleSheet(
                f"QPushButton {{ background:{CLR_RED}; color:#0d1117;"
                f" border:none; border-radius:6px; padding:6px 10px;"
                f" font-family:Consolas; font-size:9pt; font-weight:bold; }}"
                f"QPushButton:hover {{ background:#ff7b72; }}"
            )
        else:
            self.setStyleSheet(
                f"QPushButton {{ background:{CLR_PANEL}; color:{CLR_SUBTEXT};"
                f" border:1px solid {CLR_BORDER}; border-radius:6px;"
                f" padding:6px 10px; font-family:Consolas; font-size:9pt; }}"
                f"QPushButton:hover {{ color:{CLR_TEXT}; border-color:{CLR_TEXT}; }}"
            )

    @property
    def faulted(self) -> bool:
        return self._faulted

    def reset(self):
        self._faulted = False
        self._apply_style()


# ═══════════════════════════════════════════════════════
#  主窗口
# ═══════════════════════════════════════════════════════
class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("eVTOL Digital Twin")
        self.setMinimumSize(1160, 760)
        self._setup_palette()

        # ── 核心模块 ──────────────────────────────────────
        vehicle      = load_vehicle(Path("configs/quad_evtol.yaml"))
        self._bus    = EventBus()
        self._twin   = DigitalTwin(vehicle, self._bus)
        self._editor = DesignEditor(vehicle, self._twin)

        # ── 先初始化所有状态变量，再订阅事件 ──────────────
        # 必须在 subscribe 和 set_rpm 之前完成，否则回调触发时
        # 属性尚不存在会抛出 AttributeError。
        self._sim_thread: threading.Thread | None = None
        self._sim_running  = False
        self._target_alt   = 50.0   # 保留供折线图使用
        self._hover_hold_s = 10.0   # 悬停时间（秒），由滑块控制
        self._fault_mask   = [False, False, False, False]
        self._fault_injector: FaultInjector | None = None  # 仿真启动后创建
        self._current_sim: FlightSimulator | None = None
        self._opt_rpm: float = self._twin.hover_rpm

        # ── 订阅事件（此后 publish 可安全回调）──────────
        self._bus.subscribe(DigitalTwin.EVT_STATE_UPDATED, self._on_state)

        # ── 信号桥 ────────────────────────────────────────
        self._bridge = _SimBridge()
        self._bridge.frame_ready.connect(self._on_frame)
        self._bridge.sim_done.connect(self._on_sim_done)

        # ── 标签页 ────────────────────────────────────────
        self._tabs = QTabWidget()
        self._tabs.setStyleSheet(f"""
            QTabWidget::pane   {{ border:none; background:{CLR_BG}; }}
            QTabBar::tab       {{ background:{CLR_PANEL}; color:{CLR_SUBTEXT};
                                  padding:6px 22px;
                                  border-radius:4px 4px 0 0; }}
            QTabBar::tab:selected {{
                background:#21262d; color:{CLR_TEXT}; font-weight:bold; }}
        """)
        self.setCentralWidget(self._tabs)

        self._tabs.addTab(self._build_main_tab(vehicle), "🎛  Main control console 主控台")
        self._replay = ReplayWidget()
        self._tabs.addTab(self._replay, "⏪  Data playback 数据回放")

        # UI 完全构建完毕后再触发 set_rpm，确保所有回调依赖的控件都已存在
        self._twin.set_rpm(self._opt_rpm)

        # ── 状态栏 ────────────────────────────────────────
        self.statusBar().setStyleSheet(
            f"background:{CLR_PANEL}; color:{CLR_SUBTEXT};"
            f" font-family:Consolas; font-size:8pt;"
        )
        _tmr = QTimer(self)
        _tmr.timeout.connect(self._tick_clock)
        _tmr.start(1000)

    # ────────────────────────────────────────────────────
    def _setup_palette(self):
        pal = QPalette()
        pal.setColor(QPalette.ColorRole.Window, QColor(CLR_BG))
        self.setPalette(pal)
        self.setStyleSheet(f"background:{CLR_BG};")

    # ════════════════════════════════════════════════════
    #  Tab 1 — 主控台
    # ════════════════════════════════════════════════════
    def _build_main_tab(self, vehicle) -> QWidget:
        container = QWidget()
        container.setStyleSheet(f"background:{CLR_BG};")
        outer = QVBoxLayout(container)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # 顶部标题栏
        header = QWidget()
        header.setFixedHeight(38)
        header.setStyleSheet(
            f"background:{CLR_PANEL}; border-bottom:1px solid {CLR_BORDER};")
        hbox = QHBoxLayout(header)
        hbox.setContentsMargins(16, 0, 16, 0)
        hbox.addWidget(
            _label("⬡  eVTOL Digital Twin",
                   size=11, bold=True, color=CLR_ACCENT))
        hbox.addStretch()
        self._status_lbl = _label("Status: GROUNDED", size=9, color=CLR_YELLOW)
        self._status_lbl.setAlignment(
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        hbox.addWidget(self._status_lbl)
        outer.addWidget(header)

        # 主体水平分割
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setHandleWidth(2)
        splitter.setStyleSheet(
            f"QSplitter::handle {{ background:{CLR_BORDER}; }}")

        # 左：3D 视图
        self._view3d = View3DWidget(self._twin.vehicle, self._bus)
        self._view3d.setMinimumWidth(460)
        splitter.addWidget(self._view3d)

        # 右：滚动面板
        right_scroll = QScrollArea()
        right_scroll.setWidgetResizable(True)
        right_scroll.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        right_scroll.setStyleSheet(
            f"QScrollArea {{ background:{CLR_BG}; border:none; }}"
            f"QScrollBar:vertical {{ background:{CLR_PANEL}; width:5px;"
            f"  border-radius:2px; }}"
            f"QScrollBar::handle:vertical {{ background:{CLR_BORDER};"
            f"  border-radius:2px; min-height:24px; }}"
        )
        right_panel = QWidget()
        right_panel.setStyleSheet(f"background:{CLR_BG};")
        rvbox = QVBoxLayout(right_panel)
        rvbox.setContentsMargins(12, 12, 12, 12)
        rvbox.setSpacing(10)

        # ── 👁 可观测性 ───────────────────────────────────
        obs_box = self._section_box("👁   Observability 可观测性")
        obs_inner = QVBoxLayout(obs_box)
        obs_inner.setSpacing(8)
        obs_inner.setContentsMargins(8, 8, 8, 8)

        # 6 张仪表卡片（OPT RPM 为算法输出，只读展示）
        self._card_alt    = MetricCard("ALTITUDE",   "m",    "{:.2f}", CLR_GREEN)
        self._card_vel    = MetricCard("V-SPEED",    "m/s",  "{:+.2f}", CLR_ACCENT)
        self._card_rpm    = MetricCard("OPT RPM",    "rpm",  "{:.0f}", CLR_PURPLE)
        self._card_thrust = MetricCard("THRUST",     "N",    "{:.1f}", CLR_YELLOW)
        self._card_power  = MetricCard("POWER",      "kW",   "{:.2f}", CLR_ORANGE)
        self._card_soc    = MetricCard("BATTERY",    "%",    "{:.1f}", CLR_GREEN)

        cards_row = QHBoxLayout()
        cards_row.setSpacing(5)
        for c in (self._card_alt, self._card_vel, self._card_rpm,
                  self._card_thrust, self._card_power, self._card_soc):
            cards_row.addWidget(c)
        obs_inner.addLayout(cards_row)

        obs_inner.addWidget(self._build_altitude_bar())

        self._plot = AltitudePlot()
        self._plot.setMinimumHeight(210)
        self._plot.set_weight(self._twin.vehicle.total_weight_N)
        self._plot.set_target_alt(self._target_alt)
        obs_inner.addWidget(self._plot)

        rvbox.addWidget(obs_box)

        # ── 🕹 可干预性 ───────────────────────────────────
        ctrl_box = self._section_box("🕹   Controllability 可干预性")
        ctrl_inner = QVBoxLayout(ctrl_box)
        ctrl_inner.setSpacing(8)
        ctrl_inner.setContentsMargins(8, 8, 8, 8)

        ctrl_inner.addWidget(self._build_design_editor(vehicle))
        ctrl_inner.addWidget(_sep())
        ctrl_inner.addWidget(self._build_sim_controls())
        ctrl_inner.addWidget(_sep())
        ctrl_inner.addWidget(self._build_fault_injection())

        rvbox.addWidget(ctrl_box)
        rvbox.addStretch()

        right_scroll.setWidget(right_panel)
        splitter.addWidget(right_scroll)
        splitter.setSizes([620, 480])
        outer.addWidget(splitter)
        return container

    # ════════════════════════════════════════════════════
    #  可观测性：高度进度条
    # ════════════════════════════════════════════════════
    def _build_altitude_bar(self) -> QWidget:
        w = QWidget()
        w.setFixedHeight(28)
        w.setStyleSheet(f"background:{CLR_PANEL}; border-radius:6px;")
        h = QHBoxLayout(w)
        h.setContentsMargins(10, 4, 10, 4)
        h.setSpacing(8)
        h.addWidget(_label("0 m", size=8, color=CLR_SUBTEXT))
        self._alt_bar = QProgressBar()
        self._alt_bar.setRange(0, 200)
        self._alt_bar.setValue(0)
        self._alt_bar.setTextVisible(False)
        self._alt_bar.setFixedHeight(10)
        self._alt_bar.setStyleSheet(
            f"QProgressBar {{ background:{CLR_BG}; border-radius:5px; }}"
            f"QProgressBar::chunk {{ background:{CLR_GREEN}; border-radius:5px; }}"
        )
        h.addWidget(self._alt_bar, 1)
        h.addWidget(_label("200 m", size=8, color=CLR_SUBTEXT))
        return w

    # ════════════════════════════════════════════════════
    #  可干预性 ① Design Editor
    # ════════════════════════════════════════════════════
    def _build_design_editor(self, vehicle) -> QGroupBox:
        box = QGroupBox("① Design Editor — 飞行器参数设计")
        box.setStyleSheet(self._inner_group_style())
        root = QVBoxLayout(box)
        root.setSpacing(6)
        root.setContentsMargins(8, 10, 8, 8)

        # 参数输入：两列网格
        grid = QGridLayout()
        grid.setSpacing(6)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)

        p = DesignParams.from_vehicle(vehicle)

        # 左列：旋翼参数
        grid.addWidget(
            _label("— Rotor —", size=8, bold=True, color=CLR_ACCENT), 0, 0)
        self._pr_radius   = _ParamRow("Radius",        0.10, 2.0,  p.rotor_radius_m,  3, 0.01,  "m")
        self._pr_chord    = _ParamRow("Chord",         0.01, 0.5,  p.rotor_chord_m,   3, 0.005, "m")
        self._pr_blades   = _ParamRow("# Blades",      2,    8,    p.num_blades,      is_int=True)
        self._pr_Cl_alpha = _ParamRow("Cl_alpha",      1.0,  10.0, p.Cl_alpha,        3, 0.1,   "1/rad")
        self._pr_Cd0      = _ParamRow("Cd0",           0.001,0.1,  p.Cd0,             4, 0.001)
        for i, w in enumerate(
                [self._pr_radius, self._pr_chord, self._pr_blades,
                 self._pr_Cl_alpha, self._pr_Cd0], start=1):
            grid.addWidget(w, i, 0)

        # 右列：质量参数
        grid.addWidget(
            _label("— Mass —", size=8, bold=True, color=CLR_YELLOW), 0, 1)
        self._pr_fuse_mass = _ParamRow("Fuselage", 5.0,  200.0, p.fuselage_mass_kg, 1, 1.0, "kg")
        self._pr_payload   = _ParamRow("Payload",  0.0,  200.0, p.payload_kg,       1, 1.0, "kg")
        for i, w in enumerate(
                [self._pr_fuse_mass, self._pr_payload], start=1):
            grid.addWidget(w, i, 1)

        root.addLayout(grid)

        # 操作按钮
        btn_row = QHBoxLayout()
        btn_row.setSpacing(8)

        self._btn_apply = QPushButton("⚡  Apply & Solve RPM")
        self._btn_apply.setStyleSheet(
            f"QPushButton {{ background:{CLR_ACCENT}; color:{CLR_BG};"
            f" border:none; border-radius:6px; padding:7px 18px;"
            f" font-family:Consolas; font-size:10pt; font-weight:bold; }}"
            f"QPushButton:hover {{ background:#79c0ff; }}"
        )
        self._btn_apply.clicked.connect(self._on_apply_design)

        self._btn_reset_design = QPushButton("↺  Reset")
        self._btn_reset_design.setStyleSheet(self._btn_style(CLR_SUBTEXT))
        self._btn_reset_design.clicked.connect(
            lambda: self._reset_design_params(vehicle))

        btn_row.addWidget(self._btn_apply, 3)
        btn_row.addWidget(self._btn_reset_design, 1)
        root.addLayout(btn_row)

        # 求解结果展示行（5 个小卡片）
        root.addWidget(_sep())
        result_row = QHBoxLayout()
        result_row.setSpacing(5)

        def _res_card(title: str, color: str) -> QLabel:
            card = QWidget()
            card.setStyleSheet(
                f"background:{CLR_BG}; border-radius:6px;"
                f" border:1px solid {CLR_BORDER};")
            cv = QVBoxLayout(card)
            cv.setContentsMargins(8, 5, 8, 5)
            cv.setSpacing(1)
            cv.addWidget(_label(title, size=7, color=CLR_SUBTEXT))
            val_lbl = _label("—", size=13, bold=True, color=color)
            cv.addWidget(val_lbl)
            result_row.addWidget(card)
            return val_lbl

        self._res_rpm   = _res_card("Optimal RPM",     CLR_PURPLE)
        self._res_power = _res_card("Hover Power",     CLR_ORANGE)
        self._res_mass  = _res_card("Total Mass",      CLR_TEXT)
        self._res_fm    = _res_card("Figure of Merit", CLR_GREEN)
        self._res_dl    = _res_card("Disk Loading",    CLR_SUBTEXT)

        root.addLayout(result_row)

        # 首次渲染
        self._run_design_solve(DesignParams.from_vehicle(vehicle))
        return box

    # ════════════════════════════════════════════════════
    #  可干预性 ② Auto Flight Simulation
    # ════════════════════════════════════════════════════
    def _build_sim_controls(self) -> QGroupBox:
        box = QGroupBox("② Auto Flight Simulation")
        box.setStyleSheet(self._inner_group_style())
        vbox = QVBoxLayout(box)
        vbox.setSpacing(6)

        # 说明文字
        vbox.addWidget(_label(
            "Route：Take off 15m → East 40m → North 30m → Hover → Return → Land",
            size=8, color=CLR_SUBTEXT))

        row = QHBoxLayout()
        row.setSpacing(8)
        row.addWidget(_label("Hover Time:", size=9))
        self._alt_slider = QSlider(Qt.Orientation.Horizontal)
        self._alt_slider.setRange(5, 120)
        self._alt_slider.setValue(int(self._hover_hold_s))
        self._alt_slider.setStyleSheet(self._slider_style(CLR_GREEN))
        self._alt_label = _label(f"{int(self._hover_hold_s)} s",
                                  size=9, color=CLR_GREEN)
        self._alt_slider.valueChanged.connect(self._on_alt_slider)
        row.addWidget(self._alt_slider, 3)
        row.addWidget(self._alt_label)

        self._btn_fly = QPushButton("▶  START SIM")
        self._btn_fly.setStyleSheet(self._btn_style(CLR_GREEN))
        self._btn_fly.clicked.connect(self._toggle_sim)
        row.addWidget(self._btn_fly)
        vbox.addLayout(row)
        return box

    # ════════════════════════════════════════════════════
    #  可干预性 ③ Fault Injection
    # ════════════════════════════════════════════════════
    def _build_fault_injection(self) -> QGroupBox:
        from PyQt6.QtWidgets import QComboBox
        box = QGroupBox("③ Fault Injection — Rotor Failure Simulation")
        box.setStyleSheet(self._inner_group_style())
        layout = QVBoxLayout(box)
        layout.setSpacing(6)
        layout.addWidget(
            _label("Select from the preset scenarios to simulate failure.",
                   size=8, color=CLR_SUBTEXT))

        # 预设场景下拉
        preset_row = QHBoxLayout()
        preset_row.addWidget(_label("Preset scenario:", size=9))
        self._preset_combo = QComboBox()
        self._preset_combo.setStyleSheet(
            f"QComboBox {{ background:{CLR_PANEL}; color:{CLR_TEXT};"
            f" border:1px solid {CLR_BORDER}; border-radius:5px;"
            f" padding:3px 8px; font-family:Consolas; font-size:9pt; }}"
            f"QComboBox::drop-down {{ border:none; }}"
            f"QComboBox QAbstractItemView {{ background:{CLR_PANEL};"
            f" color:{CLR_TEXT}; selection-background-color:{CLR_ACCENT}; }}"
        )
        for key, val in FAULT_PRESETS.items():
            self._preset_combo.addItem(val["label"], key)
        self._preset_combo.currentIndexChanged.connect(self._on_preset_selected)
        preset_row.addWidget(self._preset_combo, 1)
        layout.addLayout(preset_row)

        # 旋翼按钮行
        btn_row = QHBoxLayout()
        btn_row.setSpacing(8)
        self._fault_btns: list[_RotorFaultBtn] = []
        for i, name in enumerate(["FL  前左", "FR  前右", "RL  后左", "RR  后右"]):
            btn = _RotorFaultBtn(name)
            btn.clicked.connect(
                lambda _checked, idx=i: self._on_fault_toggle(idx))
            self._fault_btns.append(btn)
            btn_row.addWidget(btn)

        clear_btn = QPushButton("🔄  Clear All")
        clear_btn.setStyleSheet(self._btn_style(CLR_SUBTEXT))
        clear_btn.clicked.connect(self._clear_faults)
        btn_row.addWidget(clear_btn)
        layout.addLayout(btn_row)

        self._fault_status_lbl = _label(
            "All rotors nominal  ✓", size=8, color=CLR_GREEN)
        layout.addWidget(self._fault_status_lbl)

        # 容错分析结果标签
        self._fault_analysis_lbl = _label("", size=8, color=CLR_SUBTEXT)
        self._fault_analysis_lbl.setWordWrap(True)
        layout.addWidget(self._fault_analysis_lbl)
        return box

    # ════════════════════════════════════════════════════
    #  样式辅助
    # ════════════════════════════════════════════════════
    def _section_box(self, title: str) -> QGroupBox:
        box = QGroupBox(title)
        box.setStyleSheet(
            f"QGroupBox {{ color:{CLR_ACCENT}; border:1px solid {CLR_BORDER};"
            f" border-radius:8px; margin-top:8px; padding:10px;"
            f" font-family:Consolas; font-size:10pt; font-weight:bold; }}"
            f"QGroupBox::title {{"
            f" subcontrol-origin:margin; left:10px; padding:0 4px; }}"
        )
        return box

    @staticmethod
    def _inner_group_style() -> str:
        return (
            f"QGroupBox {{ color:{CLR_SUBTEXT}; border:1px solid {CLR_BORDER};"
            f" border-radius:6px; margin-top:6px; padding:8px;"
            f" font-family:Consolas; }}"
            f"QGroupBox::title {{"
            f" subcontrol-origin:margin; left:8px; }}"
        )

    @staticmethod
    def _slider_style(color: str) -> str:
        return (
            f"QSlider::groove:horizontal {{ height:5px;"
            f" background:{CLR_PANEL}; border-radius:2px; }}"
            f"QSlider::handle:horizontal {{ width:14px; height:14px;"
            f" background:{color}; border-radius:7px; margin:-4px 0; }}"
            f"QSlider::sub-page:horizontal {{ background:{color};"
            f" border-radius:2px; }}"
        )

    @staticmethod
    def _btn_style(color: str) -> str:
        return (
            f"QPushButton {{ background:{CLR_PANEL}; color:{color};"
            f" border:1px solid {color}; border-radius:6px;"
            f" padding:5px 10px; font-family:Consolas; font-size:9pt; }}"
            f"QPushButton:hover {{ background:{color}; color:{CLR_BG}; }}"
        )

    # ════════════════════════════════════════════════════
    #  Design Editor 核心逻辑
    # ════════════════════════════════════════════════════
    def _collect_design_params(self) -> DesignParams:
        return DesignParams(
            rotor_radius_m   = self._pr_radius.value,
            rotor_chord_m    = self._pr_chord.value,
            num_blades       = int(self._pr_blades.value),
            rpm_max          = self._twin.vehicle.rotors.rpm_max,
            Cl_alpha         = self._pr_Cl_alpha.value,
            Cd0              = self._pr_Cd0.value,
            fuselage_mass_kg = self._pr_fuse_mass.value,
            payload_kg       = self._pr_payload.value,
        )

    def _on_apply_design(self):
        """
        Apply 按钮回调：
          1. 读取 UI 参数
          2. DesignEditor.apply() → 内部调用 AeroEngine 二分法求最优悬停 RPM
          3. 更新 DigitalTwin（让 3D 旋翼动画速度同步）
          4. 更新折线图重力参考线
        """
        self._btn_apply.setEnabled(False)
        self._btn_apply.setText("⏳  Solving…")

        params = self._collect_design_params()
        self._run_design_solve(params)

        # 将算法求出的 RPM 写回 twin
        self._twin.set_rpm(self._opt_rpm)
        # 折线图重力参考线（总质量可能已变）
        self._plot.set_weight(self._twin.vehicle.total_weight_N)

        self._btn_apply.setEnabled(True)
        self._btn_apply.setText("⚡  Apply & Solve RPM")

    def _run_design_solve(self, params: DesignParams) -> DesignResult:
        """
        核心求解流程（DesignEditor 内部）：
          DesignEditor.apply(params)
            → 重建 VehicleConfig
            → AeroEngine(new_vehicle)._solve_hover_rpm()  # 60 次二分迭代
            → compute(hover_rpm) 取完整性能
        将结果写入 UI 小卡片和上方仪表。
        """
        result: DesignResult = self._editor.apply(params)
        self._opt_rpm = result.hover_rpm

        # 下方求解结果小卡片
        self._res_rpm.setText(f"{result.hover_rpm:.0f}")
        self._res_power.setText(f"{result.hover_power_W / 1000:.2f} kW")
        self._res_mass.setText(f"{result.total_mass_kg:.1f} kg")
        self._res_fm.setText(f"{result.figure_of_merit:.3f}")
        self._res_dl.setText(f"{result.disk_loading_Pa:.1f} N/m²")

        # 上方可观测性仪表（即时同步）
        self._card_rpm.set_value(self._opt_rpm)
        self._card_thrust.set_value(result.hover_thrust_N)
        self._card_power.set_value(result.hover_power_W / 1000)
        self._card_soc.set_value(100.0)  # 仿真未启动时显示满电

        return result

    def _reset_design_params(self, vehicle):
        p = DesignParams.from_vehicle(vehicle)
        self._pr_radius.set_value(p.rotor_radius_m)
        self._pr_chord.set_value(p.rotor_chord_m)
        self._pr_blades.set_value(p.num_blades)
        self._pr_Cl_alpha.set_value(p.Cl_alpha)
        self._pr_Cd0.set_value(p.Cd0)
        self._pr_fuse_mass.set_value(p.fuselage_mass_kg)
        self._pr_payload.set_value(p.payload_kg)
        self._run_design_solve(p)
        self._twin.set_rpm(self._opt_rpm)

    # ════════════════════════════════════════════════════
    #  故障注入
    # ════════════════════════════════════════════════════
    def _on_fault_toggle(self, idx: int):
        self._fault_mask[idx] = self._fault_btns[idx].faulted
        self._update_fault_status()
        self._apply_fault_to_injector()
        if self._current_sim is not None:
            try:
                self._current_sim.fault_mask = list(self._fault_mask)
            except AttributeError:
                pass

    def _clear_faults(self):
        for btn in self._fault_btns:
            btn.reset()
        self._fault_mask = [False, False, False, False]
        self._update_fault_status()
        self._apply_fault_to_injector()
        if self._current_sim is not None:
            try:
                self._current_sim.fault_mask = [False, False, False, False]
            except AttributeError:
                pass
        # 重置下拉到"正常"
        self._preset_combo.blockSignals(True)
        self._preset_combo.setCurrentIndex(0)
        self._preset_combo.blockSignals(False)

    def _update_fault_status(self):
        faulted = [i for i, f in enumerate(self._fault_mask) if f]
        names   = ["FL", "FR", "RL", "RR"]
        if faulted:
            ns = ", ".join(names[i] for i in faulted)
            self._fault_status_lbl.setText(
                f"⚠  {len(faulted)} rotor(s) FAILED: {ns}")
            self._fault_status_lbl.setStyleSheet(
                f"color:{CLR_RED}; font-family:Consolas; font-size:8pt;")
        else:
            self._fault_status_lbl.setText("All rotors nominal  ✓")
            self._fault_status_lbl.setStyleSheet(
                f"color:{CLR_GREEN}; font-family:Consolas; font-size:8pt;")

    def _on_preset_selected(self, _index: int):
        """预设场景下拉回调：同步按钮状态和 fault_mask"""
        key = self._preset_combo.currentData()
        preset = FAULT_PRESETS.get(key, {})
        healths = preset.get("healths", [1.0, 1.0, 1.0, 1.0])
        for i, (btn, h) in enumerate(zip(self._fault_btns, healths)):
            # 健康值 < 0.95 视为故障，点亮按钮
            faulted = h < 0.95
            if btn.faulted != faulted:
                btn._faulted = faulted
                btn._apply_style()
            self._fault_mask[i] = faulted
        self._update_fault_status()
        self._apply_fault_to_injector()
        if self._current_sim is not None:
            try:
                self._current_sim.fault_mask = list(self._fault_mask)
            except AttributeError:
                pass

    def _apply_fault_to_injector(self):
        """将 fault_mask 同步到 FaultInjector，并刷新分析标签"""
        if self._fault_injector is None:
            return
        for i, faulted in enumerate(self._fault_mask):
            self._fault_injector.set_health(i, 0.0 if faulted else 1.0)
        # 容错分析（用当前悬停推力估算）
        try:
            base_T = self._twin.vehicle.total_weight_N / self._twin.vehicle.rotors.count
            analysis = self._fault_injector.analyze(base_T)
            risk_color = {
                "SAFE": CLR_GREEN, "DEGRADED": CLR_YELLOW, "CRITICAL": CLR_RED
            }.get(analysis.risk_level, CLR_SUBTEXT)
            self._fault_analysis_lbl.setText(analysis.recommendation)
            self._fault_analysis_lbl.setStyleSheet(
                f"color:{risk_color}; font-family:Consolas; font-size:8pt;")
        except Exception:
            pass

    # ════════════════════════════════════════════════════
    #  仿真线程
    # ════════════════════════════════════════════════════
    def _toggle_sim(self):
        if self._sim_running:
            self._sim_running = False
            self._btn_fly.setText("▶  START SIM")
            self._btn_fly.setStyleSheet(self._btn_style(CLR_GREEN))
        else:
            self._start_sim()

    def _start_sim(self):
        self._sim_running = True
        self._btn_fly.setText("⬛  STOP SIM")
        self._btn_fly.setStyleSheet(self._btn_style(CLR_RED))
        self._plot.clear()

        # 使用 default_mission()，并将悬停航路点的 hold_s 替换为滑块值
        from simulation.flight_sim import Waypoint, default_mission
        mission = default_mission()
        # default_mission 第 4 个航路点（index=4）是悬停点
        hold_wp = mission[4]
        mission[4] = Waypoint(
            x=hold_wp.x, y=hold_wp.y, z=hold_wp.z,
            speed_ms=hold_wp.speed_ms,
            hold_s=float(self._hover_hold_s),
            label=hold_wp.label,
        )
        cfg = SimConfig(
            dt=0.05,
            duration_s=180.0 + float(self._hover_hold_s),
            waypoints=mission,
        )
        sim = FlightSimulator(self._twin, cfg)
        sim._fault_injector = self._fault_injector
        self._current_sim = sim

        # 创建 FaultInjector（绑定当前 vehicle）
        self._fault_injector = FaultInjector(self._twin.vehicle)
        self._apply_fault_to_injector()

        def _run():
            def _cb(frame: SimFrame):
                if not self._sim_running:
                    return
                self._bridge.frame_ready.emit(frame)
                time.sleep(cfg.dt * 0.8)
            sim.run(callback=_cb)
            self._bridge.sim_done.emit()

        self._sim_thread = threading.Thread(target=_run, daemon=True)
        self._sim_thread.start()

    # ════════════════════════════════════════════════════
    #  槽函数
    # ════════════════════════════════════════════════════
    def _on_frame(self, frame: SimFrame):
        self._card_alt.set_value(frame.altitude_m)
        self._card_vel.set_value(frame.velocity_ms)
        self._card_rpm.set_value(frame.rpm)
        self._card_thrust.set_value(frame.thrust_N)
        self._card_power.set_value(frame.power_W / 1000)
        self._card_soc.set_value(frame.soc * 100)
        # SOC 低于 20% 时电量卡片变红
        self._card_soc.flash_warning(frame.soc < 0.20)
        self._alt_bar.setValue(int(min(frame.altitude_m, 200)))

        fault_active = any(self._fault_mask)
        low_thrust   = frame.thrust_N < self._twin.vehicle.total_weight_N * 0.6
        self._card_thrust.flash_warning(fault_active and low_thrust)

        if frame.altitude_m > 0.05:
            self._status_lbl.setText(
                f"AIRBORNE ✈   alt={frame.altitude_m:.1f} m  "
                f"vel={frame.velocity_ms:+.2f} m/s")
            self._status_lbl.setStyleSheet(
                f"color:{CLR_GREEN}; font-family:Consolas; font-size:9pt;")
        else:
            self._status_lbl.setText("Status: GROUNDED")
            self._status_lbl.setStyleSheet(
                f"color:{CLR_YELLOW}; font-family:Consolas; font-size:9pt;")

        self._plot.append(frame.t, frame.altitude_m,
                          frame.velocity_ms, frame.thrust_N)

    def _on_sim_done(self):
        self._sim_running = False
        self._current_sim = None
        self._btn_fly.setText("▶  START SIM")
        self._btn_fly.setStyleSheet(self._btn_style(CLR_GREEN))
        self._status_lbl.setText("SIM COMPLETE  ✓")
        self._status_lbl.setStyleSheet(
            f"color:{CLR_ACCENT}; font-family:Consolas; font-size:9pt;")

    def _on_state(self, state: TwinState):
        """仿真未运行时 EventBus 回调（Design Apply 后触发）。"""
        if not self._sim_running:
            self._card_rpm.set_value(state.rpm)
            self._card_thrust.set_value(state.performance.total_thrust_N)
            self._card_power.set_value(state.performance.total_power_W / 1000)

    def _on_alt_slider(self, value: int):
        self._hover_hold_s = float(value)
        self._alt_label.setText(f"{value} s")

    def _tick_clock(self):
        faults = sum(self._fault_mask)
        fault_str = (f"  |  ⚠  {faults} ROTOR FAULT(S)" if faults
                     else "  |  All Systems Nominal")
        self.statusBar().showMessage(
            f"  eVTOL Digital Twin"
            f"  |  OPT RPM: {self._opt_rpm:.0f}"
            f"  |  {time.strftime('%Y-%m-%d  %H:%M:%S')}"
            f"{fault_str}"
        )


# ═══════════════════════════════════════════════════════
#  入口
# ═══════════════════════════════════════════════════════
def launch_gui() -> None:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    launch_gui()