"""
gui/main_window.py
eVTOL Digital Twin — PyQt6 主窗口
实时显示：高度 / 转速 / 推力 / 功率
手动控制：RPM 滑块 + 起飞/降落按钮
"""

from __future__ import annotations
import sys
import time
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QSlider, QPushButton, QGroupBox, QSizePolicy,
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont, QPalette, QColor

from core.vehicle import load_vehicle
from core.event_bus import EventBus
from core.digital_twin import DigitalTwin, TwinState
from pathlib import Path


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


# ── 单个仪表卡片 ──────────────────────────────────────────
class MetricCard(QWidget):
    def __init__(self, title: str, unit: str, fmt: str = "{:.1f}",
                 accent: str = CLR_ACCENT):
        super().__init__()
        self._fmt = fmt
        self.setStyleSheet(
            f"background-color: {CLR_PANEL};"
            "border-radius: 10px;"
            "padding: 8px;"
        )
        self.setSizePolicy(QSizePolicy.Policy.Expanding,
                           QSizePolicy.Policy.Preferred)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(2)

        self._title_lbl = _label(title, size=9, color=CLR_SUBTEXT)
        self._value_lbl = _label("—", size=26, bold=True, color=accent)
        self._unit_lbl  = _label(unit, size=10, color=CLR_SUBTEXT)

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
        self.setMinimumSize(720, 480)
        self._setup_palette()

        # ── 数字孪生核心 ──────────────────────────────────
        vehicle   = load_vehicle(Path("configs/quad_evtol.yaml"))
        self._bus = EventBus()
        self._twin = DigitalTwin(vehicle, self._bus)
        self._bus.subscribe(DigitalTwin.EVT_STATE_UPDATED, self._on_state)

        self._target_rpm: float = 0.0   # 当前手动指令转速

        # ── 布局 ──────────────────────────────────────────
        root = QWidget()
        self.setCentralWidget(root)
        root.setStyleSheet(f"background-color: {CLR_BG};")
        vbox = QVBoxLayout(root)
        vbox.setContentsMargins(16, 16, 16, 16)
        vbox.setSpacing(12)

        # 标题
        title = _label("⬡  eVTOL Digital Twin", size=14, bold=True,
                        color=CLR_ACCENT)
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        vbox.addWidget(title)

        # 仪表行
        vbox.addLayout(self._build_metrics())

        # 状态栏
        self._status_lbl = _label("Status: GROUNDED", size=10,
                                   color=CLR_YELLOW)
        self._status_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        vbox.addWidget(self._status_lbl)

        # 控制面板
        vbox.addWidget(self._build_controls())

        # 底部时间戳
        self._time_lbl = _label("", size=8, color=CLR_SUBTEXT)
        self._time_lbl.setAlignment(Qt.AlignmentFlag.AlignRight)
        vbox.addWidget(self._time_lbl)

        # ── 刷新定时器（20 Hz）────────────────────────────
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(50)   # 50 ms = 20 Hz

    # ── 调色板 ────────────────────────────────────────────
    def _setup_palette(self):
        pal = QPalette()
        pal.setColor(QPalette.ColorRole.Window,
                     QColor(CLR_BG))
        self.setPalette(pal)

    # ── 仪表行 ────────────────────────────────────────────
    def _build_metrics(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(10)

        self._card_alt   = MetricCard("ALTITUDE",  "m",   "{:.2f}", CLR_GREEN)
        self._card_rpm   = MetricCard("RPM",       "rpm", "{:.0f}", CLR_ACCENT)
        self._card_thrust= MetricCard("THRUST",    "N",   "{:.1f}", CLR_YELLOW)
        self._card_power = MetricCard("POWER",     "kW",  "{:.2f}", CLR_RED)

        for card in (self._card_alt, self._card_rpm,
                     self._card_thrust, self._card_power):
            row.addWidget(card)

        return row

    # ── 控制面板 ──────────────────────────────────────────
    def _build_controls(self) -> QGroupBox:
        box = QGroupBox("Manual RPM Control")
        box.setStyleSheet(
            f"QGroupBox {{ color: {CLR_SUBTEXT}; border: 1px solid {CLR_PANEL};"
            f" border-radius: 8px; margin-top: 6px; padding: 8px; }}"
            f"QGroupBox::title {{ subcontrol-origin: margin; left: 10px; }}"
        )
        layout = QVBoxLayout(box)
        layout.setSpacing(8)

        # 滑块行
        slider_row = QHBoxLayout()
        self._rpm_label = _label("RPM: 0", size=10, color=CLR_TEXT)
        self._slider = QSlider(Qt.Orientation.Horizontal)
        max_rpm = self._twin.vehicle.rotors.rpm_max
        self._slider.setRange(0, int(max_rpm))
        self._slider.setValue(0)
        self._slider.setTickInterval(500)
        self._slider.setStyleSheet(
            f"QSlider::groove:horizontal {{ height: 6px;"
            f" background: {CLR_PANEL}; border-radius: 3px; }}"
            f"QSlider::handle:horizontal {{ width: 16px; height: 16px;"
            f" background: {CLR_ACCENT}; border-radius: 8px; margin: -5px 0; }}"
            f"QSlider::sub-page:horizontal {{ background: {CLR_ACCENT};"
            f" border-radius: 3px; }}"
        )
        self._slider.valueChanged.connect(self._on_slider)

        slider_row.addWidget(self._rpm_label, 1)
        slider_row.addWidget(self._slider, 5)
        layout.addLayout(slider_row)

        # 按钮行
        btn_row = QHBoxLayout()
        btn_row.setSpacing(8)

        hover_rpm = int(self._twin.hover_rpm)
        for label, rpm, color in [
            ("⬛  STOP",      0,            CLR_RED),
            ("▲  TAKEOFF",   int(hover_rpm * 1.30), CLR_GREEN),
            ("◆  HOVER",     hover_rpm,    CLR_ACCENT),
            ("▼  LAND",      int(hover_rpm * 0.80), CLR_YELLOW),
        ]:
            btn = QPushButton(label)
            btn.setStyleSheet(
                f"QPushButton {{ background: {CLR_PANEL}; color: {color};"
                f" border: 1px solid {color}; border-radius: 6px;"
                f" padding: 6px 12px; font-family: Consolas; font-size: 10pt; }}"
                f"QPushButton:hover {{ background: {color}; color: {CLR_BG}; }}"
            )
            btn.clicked.connect(lambda _, r=rpm: self._set_rpm(r))
            btn_row.addWidget(btn)

        layout.addLayout(btn_row)
        return box

    # ── 信号处理 ──────────────────────────────────────────
    def _on_slider(self, value: int) -> None:
        self._rpm_label.setText(f"RPM: {value}")
        self._set_rpm(float(value))

    def _set_rpm(self, rpm: float) -> None:
        self._target_rpm = rpm
        self._slider.blockSignals(True)
        self._slider.setValue(int(rpm))
        self._slider.blockSignals(False)
        self._rpm_label.setText(f"RPM: {int(rpm)}")
        self._twin.set_rpm(rpm)

    def _on_state(self, state: TwinState) -> None:
        """EventBus 回调——更新仪表卡片"""
        self._card_rpm.set_value(state.rpm)
        self._card_thrust.set_value(state.performance.total_thrust_N)
        self._card_power.set_value(state.performance.total_power_W / 1000)

    def _tick(self) -> None:
        """定时器——更新时间戳"""
        self._time_lbl.setText(
            time.strftime("  %Y-%m-%d  %H:%M:%S")
        )

    # ── 高度仪表需从外部注入（GUI 模式下无仿真器）─────────
    def set_altitude(self, alt_m: float) -> None:
        self._card_alt.set_value(alt_m)
        if alt_m > 0.05:
            self._status_lbl.setText("Status: AIRBORNE ✈")
            self._status_lbl.setStyleSheet(f"color: {CLR_GREEN};")
        else:
            self._status_lbl.setText("Status: GROUNDED")
            self._status_lbl.setStyleSheet(f"color: {CLR_YELLOW};")


# ── 入口 ─────────────────────────────────────────────────
def launch_gui() -> None:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    launch_gui()