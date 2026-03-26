"""
gui/main_window.py
eVTOL Digital Twin — PyQt6 主窗口（Step 7：实时仿真接入）
"""

from __future__ import annotations
import sys
import time
import threading
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QPushButton, QGroupBox,
    QProgressBar, QSizePolicy,
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt6.QtGui import QFont, QPalette, QColor
from pathlib import Path

from core.vehicle import load_vehicle
from core.event_bus import EventBus
from core.digital_twin import DigitalTwin, TwinState
from simulation.flight_sim import FlightSimulator, SimConfig
from simulation.flight_sim import SimFrame                   # ← 帧数据


# ── 颜色常量 ─────────────────────────────────────────────
CLR_BG      = "#1e1e2e"
CLR_PANEL   = "#2a2a3e"
CLR_ACCENT  = "#7aa2f7"
CLR_GREEN   = "#9ece6a"
CLR_YELLOW  = "#e0af68"
CLR_RED     = "#f7768e"
CLR_TEXT    = "#cdd6f4"
CLR_SUBTEXT = "#6c7086"


def _label(text: str, size: int = 11, bold: bool = False,
           color: str = CLR_TEXT) -> QLabel:
    lbl = QLabel(text)
    font = QFont("Consolas", size)
    font.setBold(bold)
    lbl.setFont(font)
    lbl.setStyleSheet(f"color: {color}; background: transparent;")
    return lbl


# ── 线程 → 主线程信号桥 ───────────────────────────────────
class _SimBridge(QObject):
    frame_ready = pyqtSignal(object)   # 发送 SimFrame
    sim_done    = pyqtSignal()


# ── 仪表卡片 ──────────────────────────────────────────────
class MetricCard(QWidget):
    def __init__(self, title: str, unit: str, fmt: str = "{:.1f}",
                 accent: str = CLR_ACCENT):
        super().__init__()
        self._fmt = fmt
        self.setStyleSheet(
            f"background-color: {CLR_PANEL}; border-radius: 10px; padding: 8px;"
        )
        self.setSizePolicy(QSizePolicy.Policy.Expanding,
                           QSizePolicy.Policy.Preferred)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(2)
        self._title_lbl = _label(title, size=9,  color=CLR_SUBTEXT)
        self._value_lbl = _label("—",   size=26, bold=True, color=accent)
        self._unit_lbl  = _label(unit,  size=10, color=CLR_SUBTEXT)
        layout.addWidget(self._title_lbl)
        layout.addWidget(self._value_lbl)
        layout.addWidget(self._unit_lbl)

    def set_value(self, v: float) -> None:
        self._value_lbl.setText(self._fmt.format(v))


# ── 主窗口 ────────────────────────────────────────────────
class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("eVTOL Digital Twin")
        self.setMinimumSize(760, 560)
        self._setup_palette()

        # 数字孪生
        vehicle    = load_vehicle(Path("configs/quad_evtol.yaml"))
        self._bus  = EventBus()
        self._twin = DigitalTwin(vehicle, self._bus)
        self._bus.subscribe(DigitalTwin.EVT_STATE_UPDATED, self._on_state)

        self._sim_thread: threading.Thread | None = None
        self._sim_running = False
        self._target_alt  = 50.0          # 默认目标高度

        # 信号桥
        self._bridge = _SimBridge()
        self._bridge.frame_ready.connect(self._on_frame)
        self._bridge.sim_done.connect(self._on_sim_done)

        # ── 布局 ────────────────────────────────────────
        root = QWidget()
        self.setCentralWidget(root)
        root.setStyleSheet(f"background-color: {CLR_BG};")
        vbox = QVBoxLayout(root)
        vbox.setContentsMargins(16, 16, 16, 16)
        vbox.setSpacing(12)

        title = _label("⬡  eVTOL Digital Twin", size=14,
                        bold=True, color=CLR_ACCENT)
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        vbox.addWidget(title)

        vbox.addLayout(self._build_metrics())
        vbox.addWidget(self._build_altitude_bar())
        self._status_lbl = _label("Status: GROUNDED", size=10,
                                   color=CLR_YELLOW)
        self._status_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        vbox.addWidget(self._status_lbl)
        vbox.addWidget(self._build_sim_controls())
        vbox.addWidget(self._build_manual_controls())

        self._time_lbl = _label("", size=8, color=CLR_SUBTEXT)
        self._time_lbl.setAlignment(Qt.AlignmentFlag.AlignRight)
        vbox.addWidget(self._time_lbl)

        # 时钟定时器
        tmr = QTimer(self)
        tmr.timeout.connect(
            lambda: self._time_lbl.setText(time.strftime("  %Y-%m-%d  %H:%M:%S"))
        )
        tmr.start(1000)

    # ── 调色板 ────────────────────────────────────────────
    def _setup_palette(self):
        pal = QPalette()
        pal.setColor(QPalette.ColorRole.Window, QColor(CLR_BG))
        self.setPalette(pal)

    # ── 仪表行 ────────────────────────────────────────────
    def _build_metrics(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(10)
        self._card_alt    = MetricCard("ALTITUDE",  "m",   "{:.2f}", CLR_GREEN)
        self._card_rpm    = MetricCard("RPM",       "rpm", "{:.0f}", CLR_ACCENT)
        self._card_thrust = MetricCard("THRUST",    "N",   "{:.1f}", CLR_YELLOW)
        self._card_power  = MetricCard("POWER",     "kW",  "{:.2f}", CLR_RED)
        for c in (self._card_alt, self._card_rpm,
                  self._card_thrust, self._card_power):
            row.addWidget(c)
        return row

    # ── 高度进度条 ────────────────────────────────────────
    def _build_altitude_bar(self) -> QWidget:
        w = QWidget()
        w.setStyleSheet(f"background: {CLR_PANEL}; border-radius: 8px;")
        h = QHBoxLayout(w)
        h.setContentsMargins(12, 8, 12, 8)
        h.addWidget(_label("0 m", size=9, color=CLR_SUBTEXT))
        self._alt_bar = QProgressBar()
        self._alt_bar.setRange(0, 200)
        self._alt_bar.setValue(0)
        self._alt_bar.setTextVisible(False)
        self._alt_bar.setFixedHeight(14)
        self._alt_bar.setStyleSheet(
            f"QProgressBar {{ background: {CLR_BG}; border-radius: 7px; }}"
            f"QProgressBar::chunk {{ background: {CLR_GREEN};"
            f" border-radius: 7px; }}"
        )
        h.addWidget(self._alt_bar, 1)
        h.addWidget(_label("200 m", size=9, color=CLR_SUBTEXT))
        return w

    # ── 仿真控制面板 ──────────────────────────────────────
    def _build_sim_controls(self) -> QGroupBox:
        box = QGroupBox("Auto Flight Simulation")
        box.setStyleSheet(
            f"QGroupBox {{ color:{CLR_SUBTEXT}; border:1px solid {CLR_PANEL};"
            f" border-radius:8px; margin-top:6px; padding:8px; }}"
            f"QGroupBox::title {{ subcontrol-origin:margin; left:10px; }}"
        )
        row = QHBoxLayout(box)
        row.setSpacing(8)

        # 目标高度滑块
        row.addWidget(_label("Target Alt:", size=10))
        self._alt_slider = QSlider(Qt.Orientation.Horizontal)
        self._alt_slider.setRange(5, 150)
        self._alt_slider.setValue(int(self._target_alt))
        self._alt_slider.setStyleSheet(self._slider_style(CLR_GREEN))
        self._alt_label = _label(f"{int(self._target_alt)} m", size=10,
                                  color=CLR_GREEN)
        self._alt_slider.valueChanged.connect(self._on_alt_slider)
        row.addWidget(self._alt_slider, 3)
        row.addWidget(self._alt_label)

        # 起飞 / 停止按钮
        self._btn_fly = QPushButton("▶  START SIM")
        self._btn_fly.setStyleSheet(self._btn_style(CLR_GREEN))
        self._btn_fly.clicked.connect(self._toggle_sim)
        row.addWidget(self._btn_fly)

        return box

    # ── 手动控制面板 ──────────────────────────────────────
    def _build_manual_controls(self) -> QGroupBox:
        box = QGroupBox("Manual RPM Control")
        box.setStyleSheet(
            f"QGroupBox {{ color:{CLR_SUBTEXT}; border:1px solid {CLR_PANEL};"
            f" border-radius:8px; margin-top:6px; padding:8px; }}"
            f"QGroupBox::title {{ subcontrol-origin:margin; left:10px; }}"
        )
        layout = QVBoxLayout(box)
        layout.setSpacing(8)

        sr = QHBoxLayout()
        self._rpm_label = _label("RPM: 0", size=10)
        self._slider = QSlider(Qt.Orientation.Horizontal)
        self._slider.setRange(0, int(self._twin.vehicle.rotors.rpm_max))
        self._slider.setValue(0)
        self._slider.setStyleSheet(self._slider_style(CLR_ACCENT))
        self._slider.valueChanged.connect(self._on_rpm_slider)
        sr.addWidget(self._rpm_label, 1)
        sr.addWidget(self._slider, 5)
        layout.addLayout(sr)

        br = QHBoxLayout()
        br.setSpacing(8)
        hover = int(self._twin.hover_rpm)
        for label, rpm, color in [
            ("⬛  STOP",    0,                CLR_RED),
            ("▲  TAKEOFF", int(hover * 1.30), CLR_GREEN),
            ("◆  HOVER",   hover,             CLR_ACCENT),
            ("▼  LAND",    int(hover * 0.80), CLR_YELLOW),
        ]:
            btn = QPushButton(label)
            btn.setStyleSheet(self._btn_style(color))
            btn.clicked.connect(lambda _, r=rpm: self._set_rpm(r))
            br.addWidget(btn)
        layout.addLayout(br)
        return box

    # ── 样式辅助 ──────────────────────────────────────────
    @staticmethod
    def _slider_style(color: str) -> str:
        return (
            f"QSlider::groove:horizontal {{ height:6px; background:{CLR_PANEL};"
            f" border-radius:3px; }}"
            f"QSlider::handle:horizontal {{ width:16px; height:16px;"
            f" background:{color}; border-radius:8px; margin:-5px 0; }}"
            f"QSlider::sub-page:horizontal {{ background:{color};"
            f" border-radius:3px; }}"
        )

    @staticmethod
    def _btn_style(color: str) -> str:
        return (
            f"QPushButton {{ background:{CLR_PANEL}; color:{color};"
            f" border:1px solid {color}; border-radius:6px;"
            f" padding:6px 12px; font-family:Consolas; font-size:10pt; }}"
            f"QPushButton:hover {{ background:{color}; color:{CLR_BG}; }}"
        )

    # ── 仿真线程 ──────────────────────────────────────────
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

        cfg = SimConfig(
            dt=0.05,
            duration_s=60.0,
            target_alt_m=float(self._target_alt),
        )
        sim = FlightSimulator(self._twin, cfg)

        def _run():
            def _cb(frame: SimFrame):
                if not self._sim_running:
                    raise StopIteration        # 强制提前退出
                self._bridge.frame_ready.emit(frame)
                time.sleep(cfg.dt * 0.8)       # 接近实时播放

            try:
                sim.run(callback=_cb)
            except StopIteration:
                pass
            self._bridge.sim_done.emit()

        self._sim_thread = threading.Thread(target=_run, daemon=True)
        self._sim_thread.start()

    # ── 槽函数 ────────────────────────────────────────────
    def _on_frame(self, frame: SimFrame):
        self._card_alt.set_value(frame.altitude_m)
        self._card_rpm.set_value(frame.rpm)
        self._card_thrust.set_value(frame.thrust_N)
        self._card_power.set_value(frame.power_W / 1000)
        self._alt_bar.setValue(int(min(frame.altitude_m, 200)))

        if frame.altitude_m > 0.05:
            self._status_lbl.setText(
                f"Status: AIRBORNE ✈   alt={frame.altitude_m:.1f} m  "
                f"vel={frame.velocity_ms:+.2f} m/s"
            )
            self._status_lbl.setStyleSheet(f"color: {CLR_GREEN};")
        else:
            self._status_lbl.setText("Status: GROUNDED")
            self._status_lbl.setStyleSheet(f"color: {CLR_YELLOW};")

    def _on_sim_done(self):
        self._sim_running = False
        self._btn_fly.setText("▶  START SIM")
        self._btn_fly.setStyleSheet(self._btn_style(CLR_GREEN))
        self._status_lbl.setText("Status: SIM COMPLETE ✓")
        self._status_lbl.setStyleSheet(f"color: {CLR_ACCENT};")

    def _on_state(self, state: TwinState):
        """手动 RPM 控制时更新仪表（仿真未运行时）"""
        if not self._sim_running:
            self._card_rpm.set_value(state.rpm)
            self._card_thrust.set_value(state.performance.total_thrust_N)
            self._card_power.set_value(state.performance.total_power_W / 1000)

    def _on_rpm_slider(self, value: int):
        self._rpm_label.setText(f"RPM: {value}")
        self._set_rpm(float(value))

    def _on_alt_slider(self, value: int):
        self._target_alt = float(value)
        self._alt_label.setText(f"{value} m")

    def _set_rpm(self, rpm: float):
        self._slider.blockSignals(True)
        self._slider.setValue(int(rpm))
        self._slider.blockSignals(False)
        self._rpm_label.setText(f"RPM: {int(rpm)}")
        self._twin.set_rpm(rpm)


# ── 入口 ─────────────────────────────────────────────────
def launch_gui() -> None:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    launch_gui()