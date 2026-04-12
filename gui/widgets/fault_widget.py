"""
gui/widgets/fault_widget.py
故障注入与容错分析面板

布局：
  左列：4个旋翼健康度滑块 + 预设场景按钮
  右列：推力分布可视化（俯视图）+ 分析结果卡片
"""

from __future__ import annotations
import math
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QSlider, QPushButton, QGroupBox, QGridLayout,
    QSizePolicy, QFrame,
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QFont, QPainter, QColor, QPen, QBrush, QPainterPath

from core.fault_injector import FaultInjector, FAULT_PRESETS, FaultAnalysis
from core.vehicle import VehicleConfig

# ── 颜色 ─────────────────────────────────────────────────
CLR_BG      = "#1e1e2e"
CLR_PANEL   = "#2a2a3e"
CLR_ACCENT  = "#7aa2f7"
CLR_GREEN   = "#9ece6a"
CLR_YELLOW  = "#e0af68"
CLR_RED     = "#f7768e"
CLR_TEXT    = "#cdd6f4"
CLR_SUBTEXT = "#6c7086"
CLR_ORANGE  = "#ff9e64"


def _lbl(text, size=10, bold=False, color=CLR_TEXT):
    l = QLabel(text)
    f = QFont("Consolas", size)
    f.setBold(bold)
    l.setFont(f)
    l.setStyleSheet(f"color:{color}; background:transparent;")
    return l


# ── 旋翼俯视图可视化 ──────────────────────────────────────
class RotorMapWidget(QWidget):
    """绘制四旋翼俯视图，旋翼颜色随健康度变化"""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(220, 220)
        self.setSizePolicy(QSizePolicy.Policy.Expanding,
                           QSizePolicy.Policy.Expanding)
        self._healths = [1.0, 1.0, 1.0, 1.0]
        self._thrusts = [0.0, 0.0, 0.0, 0.0]
        self._labels  = ["FL", "FR", "RL", "RR"]

    def update_state(self, healths, thrusts):
        self._healths = healths
        self._thrusts = thrusts
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2
        arm = min(w, h) * 0.32   # 机臂长度
        r_rotor = min(w, h) * 0.12  # 旋翼半径

        # 背景
        p.fillRect(0, 0, w, h, QColor(CLR_BG))

        # 机身（中心矩形）
        body_w, body_h = arm * 0.5, arm * 0.35
        p.setBrush(QBrush(QColor("#313244")))
        p.setPen(QPen(QColor(CLR_ACCENT), 1.5))
        p.drawRoundedRect(int(cx - body_w/2), int(cy - body_h/2),
                          int(body_w), int(body_h), 6, 6)

        # 机头方向箭头
        p.setPen(QPen(QColor(CLR_ACCENT), 2))
        p.drawLine(int(cx), int(cy - body_h/2 - 2),
                   int(cx), int(cy - body_h/2 - 14))
        p.drawLine(int(cx), int(cy - body_h/2 - 14),
                   int(cx - 6), int(cy - body_h/2 - 8))
        p.drawLine(int(cx), int(cy - body_h/2 - 14),
                   int(cx + 6), int(cy - body_h/2 - 8))

        # 旋翼位置（俯视图：前=上，右=右）
        positions = [
            (cx - arm, cy - arm),   # FL 前左（左上）
            (cx + arm, cy - arm),   # FR 前右（右上）
            (cx - arm, cy + arm),   # RL 后左（左下）
            (cx + arm, cy + arm),   # RR 后右（右下）
        ]

        # 机臂
        p.setPen(QPen(QColor("#45475a"), 3))
        for px, py in positions:
            p.drawLine(int(cx), int(cy), int(px), int(py))

        # 各旋翼
        for i, ((px, py), health, thrust) in enumerate(
                zip(positions, self._healths, self._thrusts)):

            # 颜色：绿→黄→红 随健康度
            if health >= 0.8:
                col = QColor(CLR_GREEN)
            elif health >= 0.4:
                col = QColor(CLR_YELLOW)
            elif health >= 0.05:
                col = QColor(CLR_ORANGE)
            else:
                col = QColor(CLR_RED)

            # 外圈
            col.setAlpha(60)
            p.setBrush(QBrush(col))
            col.setAlpha(200)
            p.setPen(QPen(col, 2))
            p.drawEllipse(int(px - r_rotor), int(py - r_rotor),
                          int(r_rotor * 2), int(r_rotor * 2))

            # 推力圆点（大小随推力比例）
            dot_r = r_rotor * 0.28 * (health + 0.15)
            col.setAlpha(220)
            p.setBrush(QBrush(col))
            p.drawEllipse(int(px - dot_r), int(py - dot_r),
                          int(dot_r * 2), int(dot_r * 2))

            # 标签
            p.setPen(QPen(QColor(CLR_TEXT), 1))
            p.setFont(QFont("Consolas", 8, QFont.Weight.Bold))
            lbl = self._labels[i]
            pct = f"{int(health*100)}%"
            p.drawText(int(px - 14), int(py + r_rotor + 14), lbl)
            pct_col = col
            pct_col.setAlpha(255)
            p.setPen(QPen(pct_col, 1))
            p.drawText(int(px - 14), int(py + r_rotor + 26), pct)

        p.end()


# ── 单旋翼健康度控件 ──────────────────────────────────────
class RotorHealthControl(QWidget):
    health_changed = pyqtSignal(int, float)   # (rotor_id, health)

    def __init__(self, rotor_id: int, label: str):
        super().__init__()
        self._id = rotor_id
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        layout.setSpacing(8)

        self._name_lbl = _lbl(label, size=9, bold=True, color=CLR_ACCENT)
        self._name_lbl.setFixedWidth(60)
        layout.addWidget(self._name_lbl)

        self._slider = QSlider(Qt.Orientation.Horizontal)
        self._slider.setRange(0, 100)
        self._slider.setValue(100)
        self._slider.setStyleSheet(self._slider_style(CLR_GREEN))
        self._slider.valueChanged.connect(self._on_change)
        layout.addWidget(self._slider, 1)

        self._pct_lbl = _lbl("100%", size=9, color=CLR_GREEN)
        self._pct_lbl.setFixedWidth(38)
        self._pct_lbl.setAlignment(Qt.AlignmentFlag.AlignRight)
        layout.addWidget(self._pct_lbl)

        btn_fail = QPushButton("✕")
        btn_fail.setFixedSize(24, 24)
        btn_fail.setStyleSheet(
            f"QPushButton{{background:#3b1f2b;color:{CLR_RED};border:1px solid {CLR_RED};"
            f"border-radius:4px;font-size:9pt;}}"
            f"QPushButton:hover{{background:{CLR_RED};color:{CLR_BG};}}"
        )
        btn_fail.clicked.connect(lambda: self._slider.setValue(0))
        layout.addWidget(btn_fail)

        btn_ok = QPushButton("✓")
        btn_ok.setFixedSize(24, 24)
        btn_ok.setStyleSheet(
            f"QPushButton{{background:#1f3b2b;color:{CLR_GREEN};border:1px solid {CLR_GREEN};"
            f"border-radius:4px;font-size:9pt;}}"
            f"QPushButton:hover{{background:{CLR_GREEN};color:{CLR_BG};}}"
        )
        btn_ok.clicked.connect(lambda: self._slider.setValue(100))
        layout.addWidget(btn_ok)

    def _on_change(self, value: int):
        health = value / 100.0
        color = CLR_GREEN if health >= 0.8 else (CLR_YELLOW if health >= 0.4 else CLR_RED)
        self._pct_lbl.setText(f"{value}%")
        self._pct_lbl.setStyleSheet(f"color:{color}; background:transparent;")
        self._slider.setStyleSheet(self._slider_style(color))
        self.health_changed.emit(self._id, health)

    def set_value(self, pct: int):
        self._slider.blockSignals(True)
        self._slider.setValue(pct)
        self._slider.blockSignals(False)
        self._on_change(pct)

    @staticmethod
    def _slider_style(color):
        return (
            f"QSlider::groove:horizontal{{height:5px;background:{CLR_PANEL};"
            f"border-radius:3px;}}"
            f"QSlider::handle:horizontal{{width:14px;height:14px;"
            f"background:{color};border-radius:7px;margin:-5px 0;}}"
            f"QSlider::sub-page:horizontal{{background:{color};border-radius:3px;}}"
        )


# ── 分析结果显示 ──────────────────────────────────────────
class AnalysisPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.setStyleSheet(
            f"background:{CLR_PANEL}; border-radius:10px;"
        )
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(6)

        self._risk_lbl = _lbl("● SAFE", size=13, bold=True, color=CLR_GREEN)
        self._risk_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self._risk_lbl)

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet(f"color:{CLR_SUBTEXT};")
        layout.addWidget(sep)

        grid = QGridLayout()
        grid.setSpacing(4)

        def row(r, title, init_val, color=CLR_TEXT):
            tl = _lbl(title, size=9, color=CLR_SUBTEXT)
            vl = _lbl(init_val, size=10, bold=True, color=color)
            grid.addWidget(tl, r, 0)
            grid.addWidget(vl, r, 1)
            return vl

        self._total_T_lbl  = row(0, "总推力",     "— N",   CLR_ACCENT)
        self._loss_lbl     = row(1, "推力损失",    "—%",    CLR_YELLOW)
        self._roll_lbl     = row(2, "滚转力矩",    "— N·m", CLR_YELLOW)
        self._pitch_lbl    = row(3, "俯仰力矩",    "— N·m", CLR_YELLOW)
        self._yaw_lbl      = row(4, "偏航力矩",    "— N·m", CLR_YELLOW)
        self._hover_lbl    = row(5, "能否悬停",    "—",     CLR_GREEN)
        self._land_lbl     = row(6, "能否安全着陆","—",     CLR_GREEN)
        layout.addLayout(grid)

        sep2 = QFrame()
        sep2.setFrameShape(QFrame.Shape.HLine)
        sep2.setStyleSheet(f"color:{CLR_SUBTEXT};")
        layout.addWidget(sep2)

        self._rec_lbl = _lbl("", size=9, color=CLR_TEXT)
        self._rec_lbl.setWordWrap(True)
        layout.addWidget(self._rec_lbl)

    def update_analysis(self, a: FaultAnalysis):
        color_map = {
            "SAFE":     CLR_GREEN,
            "DEGRADED": CLR_YELLOW,
            "CRITICAL": CLR_RED,
        }
        risk_color = color_map.get(a.risk_level, CLR_TEXT)
        icon = {"SAFE": "●", "DEGRADED": "▲", "CRITICAL": "✖"}[a.risk_level]
        self._risk_lbl.setText(f"{icon}  {a.risk_level}")
        self._risk_lbl.setStyleSheet(
            f"color:{risk_color}; background:transparent; font-size:13pt; font-weight:bold;"
        )

        def moment_color(v):
            return CLR_GREEN if abs(v) < 30 else (CLR_YELLOW if abs(v) < 80 else CLR_RED)

        self._total_T_lbl.setText(f"{a.total_thrust_N:.1f} N")
        self._loss_lbl.setText(f"{a.thrust_loss_pct:.1f}%")
        self._loss_lbl.setStyleSheet(
            f"color:{'CLR_GREEN' if a.thrust_loss_pct < 5 else CLR_YELLOW if a.thrust_loss_pct < 20 else CLR_RED};"
            f" background:transparent; font-weight:bold;"
        )
        self._roll_lbl.setText(f"{a.roll_moment_Nm:+.2f} N·m")
        self._roll_lbl.setStyleSheet(
            f"color:{moment_color(a.roll_moment_Nm)}; background:transparent; font-weight:bold;"
        )
        self._pitch_lbl.setText(f"{a.pitch_moment_Nm:+.2f} N·m")
        self._pitch_lbl.setStyleSheet(
            f"color:{moment_color(a.pitch_moment_Nm)}; background:transparent; font-weight:bold;"
        )
        self._yaw_lbl.setText(f"{a.yaw_moment_Nm:+.2f} N·m")
        self._yaw_lbl.setStyleSheet(
            f"color:{moment_color(a.yaw_moment_Nm)}; background:transparent; font-weight:bold;"
        )
        self._hover_lbl.setText("✅ 是" if a.can_maintain_hover else "❌ 否")
        self._hover_lbl.setStyleSheet(
            f"color:{CLR_GREEN if a.can_maintain_hover else CLR_RED};"
            f" background:transparent; font-weight:bold;"
        )
        self._land_lbl.setText("✅ 是" if a.can_safe_land else "❌ 否")
        self._land_lbl.setStyleSheet(
            f"color:{CLR_GREEN if a.can_safe_land else CLR_RED};"
            f" background:transparent; font-weight:bold;"
        )
        self._rec_lbl.setText(a.recommendation)
        self._rec_lbl.setStyleSheet(
            f"color:{risk_color}; background:transparent;"
        )


# ── 主面板 ────────────────────────────────────────────────
class FaultWidget(QWidget):
    """故障注入与容错分析主面板"""

    def __init__(self, vehicle: VehicleConfig):
        super().__init__()
        self.setStyleSheet(f"background:{CLR_BG};")
        self._vehicle  = vehicle
        self._injector = FaultInjector(vehicle)

        # 悬停推力（每旋翼）
        weight = vehicle.total_mass_kg * vehicle.environment.gravity_m_s2
        self._hover_thrust_per_rotor = weight / vehicle.rotors.count

        main = QHBoxLayout(self)
        main.setContentsMargins(16, 16, 16, 16)
        main.setSpacing(16)

        # ── 左列：控制 ────────────────────────────────────
        left = QVBoxLayout()
        left.setSpacing(12)

        title = _lbl("🔧  Fault Injection", size=13, bold=True, color=CLR_ACCENT)
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        left.addWidget(title)

        # 旋翼滑块组
        rotor_box = QGroupBox("旋翼健康度控制")
        rotor_box.setStyleSheet(self._group_style())
        rotor_layout = QVBoxLayout(rotor_box)
        rotor_layout.setSpacing(6)

        self._controls: list[RotorHealthControl] = []
        labels = FaultInjector.ROTOR_LABELS
        for i in range(vehicle.rotors.count):
            ctrl = RotorHealthControl(
                i, labels[i] if i < len(labels) else f"R{i}")
            ctrl.health_changed.connect(self._on_health_changed)
            rotor_layout.addWidget(ctrl)
            self._controls.append(ctrl)

        btn_reset = QPushButton("↺  全部恢复正常")
        btn_reset.setStyleSheet(self._btn_style(CLR_GREEN))
        btn_reset.clicked.connect(self._reset_all)
        rotor_layout.addWidget(btn_reset)
        left.addWidget(rotor_box)

        # 预设场景
        preset_box = QGroupBox("预设故障场景")
        preset_box.setStyleSheet(self._group_style())
        preset_layout = QGridLayout(preset_box)
        preset_layout.setSpacing(6)

        preset_keys = list(FAULT_PRESETS.keys())
        for idx, key in enumerate(preset_keys):
            info = FAULT_PRESETS[key]
            btn = QPushButton(info["label"])
            healths = info["healths"]
            # 按钮颜色：全正常绿，含失效红，含退化黄
            if all(h == 1.0 for h in healths):
                color = CLR_GREEN
            elif any(h == 0.0 for h in healths):
                color = CLR_RED
            else:
                color = CLR_YELLOW
            btn.setStyleSheet(self._btn_style(color))
            btn.setToolTip(info["description"])
            btn.clicked.connect(lambda _, k=key: self._apply_preset(k))
            preset_layout.addWidget(btn, idx // 2, idx % 2)

        left.addWidget(preset_box)
        left.addStretch()
        main.addLayout(left, 4)

        # ── 右列：可视化 + 分析 ───────────────────────────
        right = QVBoxLayout()
        right.setSpacing(12)

        # 俯视图
        map_box = QGroupBox("推力分布（俯视图）")
        map_box.setStyleSheet(self._group_style())
        map_layout = QVBoxLayout(map_box)
        self._rotor_map = RotorMapWidget()
        map_layout.addWidget(self._rotor_map)
        right.addWidget(map_box, 3)

        # 分析结果
        analysis_box = QGroupBox("容错分析结果")
        analysis_box.setStyleSheet(self._group_style())
        analysis_layout = QVBoxLayout(analysis_box)
        self._analysis_panel = AnalysisPanel()
        analysis_layout.addWidget(self._analysis_panel)
        right.addWidget(analysis_box, 4)

        main.addLayout(right, 5)

        # 初始刷新
        self._refresh()

    # ── 槽函数 ────────────────────────────────────────────
    def _on_health_changed(self, rotor_id: int, health: float):
        self._injector.set_health(rotor_id, health)
        self._refresh()

    def _reset_all(self):
        self._injector.reset_all()
        for ctrl in self._controls:
            ctrl.set_value(100)
        self._refresh()

    def _apply_preset(self, preset_name: str):
        self._injector.apply_preset(preset_name)
        preset = FAULT_PRESETS[preset_name]
        for i, h in enumerate(preset["healths"]):
            if i < len(self._controls):
                self._controls[i].set_value(int(h * 100))
        self._refresh()

    def _refresh(self):
        healths = self._injector.get_thrust_multipliers()
        analysis = self._injector.analyze(self._hover_thrust_per_rotor)

        thrusts = analysis.rotor_thrusts_N
        self._rotor_map.update_state(healths, thrusts)
        self._analysis_panel.update_analysis(analysis)

    # ── 样式辅助 ──────────────────────────────────────────
    @staticmethod
    def _group_style():
        return (
            f"QGroupBox{{color:{CLR_SUBTEXT}; border:1px solid {CLR_PANEL};"
            f" border-radius:8px; margin-top:6px; padding:8px;}}"
            f"QGroupBox::title{{subcontrol-origin:margin; left:10px;}}"
        )

    @staticmethod
    def _btn_style(color):
        return (
            f"QPushButton{{background:{CLR_PANEL}; color:{color};"
            f" border:1px solid {color}; border-radius:6px;"
            f" padding:5px 8px; font-family:'Consolas'; font-size:9pt;}}"
            f"QPushButton:hover{{background:{color}; color:{CLR_BG};}}"
        )