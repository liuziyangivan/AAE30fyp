"""
gui/widgets/view3d_widget.py
eVTOL 三维实时可视化（美化版）
- 旋翼旋转动画
- 渐变轨迹（新→绿 旧→暗）
- 机身高光边框
- 地面阴影投影
- 实时 HUD 数据叠加
"""

from __future__ import annotations
import math
import numpy as np

import pyqtgraph.opengl as gl
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore

from core.vehicle import VehicleConfig
from core.event_bus import EventBus

# ── 颜色 ──────────────────────────────────────────────────
_BG          = (13,  17,  23)
_FUSE_FACE   = (48,  54,  61,  160)
_FUSE_EDGE   = (100, 120, 140, 220)
_ROTOR_COL   = (88,  166, 255, 130)
_ROTOR_SPIN  = (120, 200, 255, 180)
_TRAIL_NEW   = (63,  185,  80, 255)
_TRAIL_OLD   = (20,   60,  25, 255)
_SHADOW_COL  = (30,   80,  30,  80)
_GRID_COL    = (25,   35,  45, 255)
_GROUND_COL  = (16,   24,  32, 255)

def _r(t): return tuple(c/255 for c in t)


class View3DWidget(QtWidgets.QWidget):

    MAX_TRAIL   = 600
    ROTOR_SEGS  = 48      # 旋翼圆盘精细度
    ROTOR_RPM   = 300     # 动画转速（视觉效果）

    def __init__(self, vehicle: VehicleConfig, bus: EventBus, parent=None):
        super().__init__(parent)
        self._vehicle = vehicle
        self._bus     = bus
        self._trail:  list[np.ndarray] = []
        self._rotor_angles = [0.0] * vehicle.rotors.count
        self._last_pos     = np.zeros(3)
        self._current_rpm  = 0.0

        self._build_ui()
        self._build_scene()
        self._reset_camera()

        # 订阅孪生状态（字符串事件名）
        bus.subscribe("state_updated", self._on_state)

        # 旋翼动画定时器（30 fps）
        self._anim_timer = QtCore.QTimer(self)
        self._anim_timer.timeout.connect(self._animate_rotors)
        self._anim_timer.start(33)

    # ── UI ────────────────────────────────────────────────
    def _build_ui(self):
        vbox = QtWidgets.QVBoxLayout(self)
        vbox.setContentsMargins(0, 0, 0, 0)
        vbox.setSpacing(4)

        # 工具栏
        toolbar = QtWidgets.QHBoxLayout()
        toolbar.setContentsMargins(8, 4, 8, 0)

        def _btn(text, color, fn):
            b = QtWidgets.QPushButton(text)
            b.setStyleSheet(
                f"QPushButton{{background:#161b22;color:{color};"
                f"border:1px solid {color};border-radius:5px;"
                f"padding:3px 10px;font-family:Consolas;font-size:9pt;}}"
                f"QPushButton:hover{{background:{color};color:#0d1117;}}"
            )
            b.clicked.connect(fn)
            return b

        toolbar.addWidget(_btn("🎥 Reset Camera", "#58a6ff", self._reset_camera))
        toolbar.addWidget(_btn("🗑 Clear Trail",  "#3fb950", self._clear_trail))
        toolbar.addStretch()

        self._hud = QtWidgets.QLabel("—")
        self._hud.setStyleSheet(
            "color:#8b949e;font-family:Consolas;font-size:9pt;"
            "background:transparent;padding:2px 8px;"
        )
        toolbar.addWidget(self._hud)
        vbox.addLayout(toolbar)

        # OpenGL 视图
        self._view = gl.GLViewWidget()
        self._view.setBackgroundColor(_BG)
        self._view.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Expanding,
        )
        vbox.addWidget(self._view)

    # ── 场景 ──────────────────────────────────────────────
    def _build_scene(self):
        v = self._vehicle

        # 地面网格（两层：粗 + 细）
        for size, spacing, alpha in [(40, 5, 0.4), (40, 1, 0.15)]:
            g = gl.GLGridItem()
            g.setSize(size, size)
            g.setSpacing(spacing, spacing)
            g.setColor(_r(_GRID_COL) + (alpha,))
            self._view.addItem(g)

        # 地面实色平面
        gv, gf = _box_mesh(40, 40, 0.02)
        gc = np.array([_r(_GROUND_COL)] * len(gf))
        gplane = gl.GLMeshItem(vertexes=gv, faces=gf, faceColors=gc,
                               smooth=False, drawEdges=False)
        gplane.translate(0, 0, -0.02)
        self._view.addItem(gplane)

        # 坐标轴
        ax = gl.GLAxisItem()
        ax.setSize(3, 3, 3)
        self._view.addItem(ax)

        # ── 机身 ──────────────────────────────────────────
        fx = v.fuselage.length_m
        fy = v.fuselage.width_m
        fz = v.fuselage.height_m
        fv, ff = _box_mesh(fx, fy, fz)
        fc = np.array([_r(_FUSE_FACE)] * len(ff))
        self._fuse = gl.GLMeshItem(
            vertexes=fv, faces=ff, faceColors=fc,
            smooth=False, drawEdges=True, edgeColor=_r(_FUSE_EDGE),
        )
        self._view.addItem(self._fuse)

        # 机身高光顶盖（薄一点，亮色）
        tv, tf = _box_mesh(fx * 0.9, fy * 0.9, fz * 0.08)
        tc = np.array([_r((120, 160, 200, 200))] * len(tf))
        self._fuse_top = gl.GLMeshItem(vertexes=tv, faces=tf,
                                        faceColors=tc, smooth=False)
        self._view.addItem(self._fuse_top)

        # ── 旋翼 ──────────────────────────────────────────
        self._rotor_meshes: list[gl.GLMeshItem] = []
        self._rotor_ring_meshes: list[gl.GLMeshItem] = []
        rv, rf = _disk_mesh(v.rotors.radius_m, self.ROTOR_SEGS)
        rc = np.array([_r(_ROTOR_COL)] * len(rf))

        # 旋翼外环
        ringv, ringf = _ring_mesh(v.rotors.radius_m,
                                   v.rotors.radius_m * 1.08, 32)
        ring_c = np.array([_r((88, 166, 255, 220))] * len(ringf))

        self._rotor_offsets = [np.array(p, dtype=float)
                               for p in v.rotors.positions]

        for pos in v.rotors.positions:
            # 圆盘
            m = gl.GLMeshItem(vertexes=rv.copy(), faces=rf,
                              faceColors=rc, smooth=True, drawEdges=False)
            m.translate(*pos)
            self._view.addItem(m)
            self._rotor_meshes.append(m)
            # 外环
            rm = gl.GLMeshItem(vertexes=ringv.copy(), faces=ringf,
                               faceColors=ring_c, smooth=False, drawEdges=False)
            rm.translate(*pos)
            self._view.addItem(rm)
            self._rotor_ring_meshes.append(rm)

        # ── 地面阴影（投影圆盘）────────────────────────────
        sv, sf = _disk_mesh(max(fx, fy) * 0.6, 32)
        sc = np.array([_r(_SHADOW_COL)] * len(sf))
        self._shadow = gl.GLMeshItem(vertexes=sv, faces=sf,
                                     faceColors=sc, smooth=True)
        self._shadow.translate(0, 0, 0.01)
        self._view.addItem(self._shadow)

        # ── 轨迹 ──────────────────────────────────────────
        self._trail_line = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0.0]]),
            color=_r(_TRAIL_NEW),
            width=3, antialias=True,
        )
        self._view.addItem(self._trail_line)

    # ── 相机 & 控制 ───────────────────────────────────────
    def _reset_camera(self):
        self._view.setCameraPosition(distance=14, elevation=28, azimuth=40)

    def _clear_trail(self):
        self._trail.clear()
        self._trail_line.setData(pos=np.array([[0, 0, 0.0]]))

    # ── 旋翼旋转动画 ──────────────────────────────────────
    def _animate_rotors(self):
        speed_deg = self._current_rpm / 60 * 33 * 6   # deg per frame
        dirs = [1, -1, -1, 1]   # 交替方向（反扭矩对）
        for i, (mesh, ring, offset) in enumerate(
                zip(self._rotor_meshes, self._rotor_ring_meshes,
                    self._rotor_offsets)):
            self._rotor_angles[i] += speed_deg * dirs[i]
            world_pos = self._last_pos + offset
            # 重置并重新设置变换
            T = _make_transform(
                _euler_to_rotation(0, 0,
                    math.radians(self._rotor_angles[i])),
                world_pos,
            )
            mesh.setTransform(T)
            ring.setTransform(_make_transform(np.eye(3), world_pos))

    # ── 状态回调 ──────────────────────────────────────────
    def _on_state(self, state):
        pos = np.array(state.position_m)   # [x, y, z]
        self._last_pos     = pos
        self._current_rpm  = state.rpm

        # 机身
        T = _make_transform(np.eye(3), pos)
        self._fuse.setTransform(T)
        # 顶盖（偏移到顶部）
        top_pos = pos + np.array([0, 0, self._vehicle.fuselage.height_m * 0.46])
        self._fuse_top.setTransform(_make_transform(np.eye(3), top_pos))

        # 阴影（始终在地面，xy 跟随）
        self._shadow.setTransform(
            _make_transform(np.eye(3), np.array([pos[0], pos[1], 0.01]))
        )

        # 轨迹
        self._trail.append(pos.copy())
        if len(self._trail) > self.MAX_TRAIL:
            self._trail.pop(0)
        if len(self._trail) >= 2:
            arr  = np.array(self._trail)
            n    = len(arr)
            # 渐变颜色：新的绿，旧的暗
            cols = np.zeros((n, 4))
            for k in range(n):
                a = k / max(n - 1, 1)
                cols[k] = (
                    _r(_TRAIL_OLD)[0] * (1-a) + _r(_TRAIL_NEW)[0] * a,
                    _r(_TRAIL_OLD)[1] * (1-a) + _r(_TRAIL_NEW)[1] * a,
                    _r(_TRAIL_OLD)[2] * (1-a) + _r(_TRAIL_NEW)[2] * a,
                    1.0,
                )
            self._trail_line.setData(pos=arr, color=cols)

        # HUD
        self._hud.setText(
            f"z = {pos[2]:.2f} m  |  RPM = {state.rpm:.0f}  |  "
            f"T = {state.performance.total_thrust_N:.1f} N  |  "
            f"P = {state.performance.total_power_W/1000:.2f} kW"
        )


# ══════════════════════════════════════════════════════════
#  几何辅助
# ══════════════════════════════════════════════════════════

def _box_mesh(lx, ly, lz):
    hx, hy, hz = lx/2, ly/2, lz/2
    v = np.array([
        [-hx,-hy,-hz],[ hx,-hy,-hz],[ hx, hy,-hz],[-hx, hy,-hz],
        [-hx,-hy, hz],[ hx,-hy, hz],[ hx, hy, hz],[-hx, hy, hz],
    ], dtype=float)
    f = np.array([
        [0,1,2],[0,2,3],[4,5,6],[4,6,7],
        [0,1,5],[0,5,4],[2,3,7],[2,7,6],
        [1,2,6],[1,6,5],[0,3,7],[0,7,4],
    ], dtype=np.uint32)
    return v, f

def _disk_mesh(radius, segments=32):
    a = np.linspace(0, 2*math.pi, segments, endpoint=False)
    rim = np.stack([radius*np.cos(a), radius*np.sin(a), np.zeros(segments)], 1)
    v = np.vstack([[[0,0,0]], rim]).astype(float)
    f = np.array([[0, i+1, (i+1)%segments+1] for i in range(segments)],
                 dtype=np.uint32)
    return v, f

def _ring_mesh(r_inner, r_outer, segments=32):
    a = np.linspace(0, 2*math.pi, segments, endpoint=False)
    inner = np.stack([r_inner*np.cos(a), r_inner*np.sin(a), np.zeros(segments)], 1)
    outer = np.stack([r_outer*np.cos(a), r_outer*np.sin(a), np.zeros(segments)], 1)
    v = np.vstack([inner, outer]).astype(float)
    n = segments
    faces = []
    for i in range(n):
        j = (i+1) % n
        faces += [[i, j, n+j], [i, n+j, n+i]]
    return v, np.array(faces, dtype=np.uint32)

def _euler_to_rotation(roll, pitch, yaw):
    cr,sr = math.cos(roll), math.sin(roll)
    cp,sp = math.cos(pitch),math.sin(pitch)
    cy,sy = math.cos(yaw),  math.sin(yaw)
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    return Rz @ Ry @ Rx

def _make_transform(R, t):
    M = np.eye(4)
    M[:3,:3] = R
    M[:3, 3] = t
    return M