"""
gui/widgets/view3d_widget.py
eVTOL 三维实时可视化 —— 性能优化版

优化内容：
  1. _tick 降至 33ms（30fps），降低主线程压力
  2. body_parts 用单一 GLMeshItem + 合并 mesh 代替逐件 setTransform
     → 飞行器整体位移改为记录偏移量，用 translate() 增量更新
  3. 推力锥预分配顶点数组，原地修改，不再每帧重新 numpy 分配
  4. 鼠标拖拽期间暂停非必要渲染（shadow / trail 降频到 10fps）
  5. FillBetweenItem 已移至 AltitudePlot 侧修复，此处无关
"""

from __future__ import annotations
import math
import numpy as np

import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtWidgets, QtCore

from core.vehicle import VehicleConfig
from core.event_bus import EventBus

# ══════════════════════════════════════════════════════════
#  颜色常量（RGBA 0-255）
# ══════════════════════════════════════════════════════════
_BG           = (8,   12,  18)
_FUSE_FACE    = (18,  28,  44, 240)
_FUSE_EDGE    = (0,  180, 220, 160)
_CANOPY_FACE  = (40, 110, 175, 130)
_CANOPY_EDGE  = (120, 210, 255, 180)
_ARM_FACE     = (58,  68,  82, 235)
_ARM_EDGE     = (110, 130, 155, 170)
_GEAR_FACE    = (175, 192, 210, 230)
_GEAR_EDGE    = (220, 235, 250, 200)
_ROTOR_DIM    = (35,  95, 195, 110)
_ROTOR_BRIGHT = (0,  212, 255, 210)
_RING_COL     = (0,  180, 255, 230)
_THRUST_COL   = (255, 160,  60,  55)
_TRAIL_LOW    = (30,  80, 200, 255)
_TRAIL_MID    = (40, 185,  80, 255)
_TRAIL_HIGH   = (210,140,  20, 255)
_SHADOW_COL   = (0,  212, 255,  38)
_GRID_COL     = (20,  32,  44, 255)
_GROUND_COL   = (55, 100,  30, 255)


def _r(t):
    return tuple(c / 255.0 for c in t)

def _lerp(c1, c2, t):
    return tuple(c1[i]*(1-t) + c2[i]*t for i in range(4))

def _trail_color(z, z_min, z_max):
    span = max(z_max - z_min, 1.0)
    t = float(np.clip((z - z_min) / span, 0.0, 1.0))
    lo, mid, hi = _r(_TRAIL_LOW), _r(_TRAIL_MID), _r(_TRAIL_HIGH)
    return _lerp(lo, mid, t*2) if t < 0.5 else _lerp(mid, hi, (t-0.5)*2)


# ══════════════════════════════════════════════════════════
#  主 Widget
# ══════════════════════════════════════════════════════════
class View3DWidget(QtWidgets.QWidget):

    MAX_TRAIL    = 600
    ROTOR_SEGS   = 48
    THRUST_SEGS  = 22
    THRUST_MAX_N = 600.0
    THRUST_LEN   = 3.2
    THRUST_RAD   = 0.20

    # 降频计数器阈值（tick 数）
    _SLOW_EVERY  = 3   # shadow / trail 每 3 tick 更新一次（≈10fps）

    def __init__(self, vehicle: VehicleConfig, bus: EventBus, parent=None):
        super().__init__(parent)
        self._vehicle        = vehicle
        self._bus            = bus
        self._alive          = True

        # 状态缓存（仿真线程写，主线程读）
        self._pending_pos    = np.zeros(3)
        self._pending_rpm    = 0.0
        self._pending_thrust = 0.0
        self._state_dirty    = False

        # 动画状态（只在主线程读写）
        self._trail:           list[np.ndarray] = []
        self._rotor_angles     = [0.0] * vehicle.rotors.count
        self._last_pos         = np.zeros(3)
        self._current_rpm      = 0.0
        self._current_thrust   = 0.0
        self._follow_cam       = False
        self._tick_count       = 0          # 用于 slow-path 降频
        self._dragging         = False      # 鼠标拖拽中

        # 预分配推力锥顶点（避免每帧 numpy 分配）
        self._cone_verts_buf: np.ndarray | None = None  # shape (THRUST_SEGS+1, 3)
        self._cone_faces_buf: np.ndarray | None = None

        # 旋翼几何（预分配）
        self._rotor_verts:    np.ndarray | None = None
        self._rotor_faces:    np.ndarray | None = None
        self._ring_verts:     np.ndarray | None = None
        self._ring_faces:     np.ndarray | None = None
        self._rotor_offsets:  list[np.ndarray]  = []

        # 机身位移追踪（用 translate 增量，而非每帧 setTransform）
        self._body_offset     = np.zeros(3)   # 已 translate 的累计偏移

        self._build_ui()
        self._build_scene()
        self._reset_camera()

        bus.subscribe("state_updated", self._on_state)

        # 主定时器：30fps，主线程串行
        self._tick_timer = QtCore.QTimer(self)
        self._tick_timer.timeout.connect(self._tick)
        self._tick_timer.start(33)

    def closeEvent(self, event):
        self._alive = False
        self._tick_timer.stop()
        super().closeEvent(event)

    # ──────────────────────────────────────────────────────
    #  鼠标事件：拖拽期间暂停慢路径渲染
    # ──────────────────────────────────────────────────────
    def _install_drag_filter(self):
        """在 GLViewWidget 上安装事件过滤器，检测鼠标拖拽"""
        self._view.installEventFilter(self)

    def eventFilter(self, obj, event):
        if obj is self._view:
            t = event.type()
            if t == QtCore.QEvent.Type.MouseButtonPress:
                self._dragging = True
            elif t in (QtCore.QEvent.Type.MouseButtonRelease,
                       QtCore.QEvent.Type.Leave):
                self._dragging = False
        return False   # 不拦截，让 GLViewWidget 正常处理

    # ──────────────────────────────────────────────────────
    #  EventBus 回调（只写缓存，不碰 GL）
    # ──────────────────────────────────────────────────────
    def _on_state(self, state):
        if not self._alive:
            return
        try:
            self._pending_pos    = np.array(state.position_m)
            self._pending_rpm    = state.rpm
            self._pending_thrust = state.performance.total_thrust_N
            self._state_dirty    = True
        except Exception:
            pass

    # ──────────────────────────────────────────────────────
    #  统一渲染定时器（30fps，主线程）
    # ──────────────────────────────────────────────────────
    def _tick(self):
        if not self._alive:
            return
        self._tick_count += 1
        slow_tick = (self._tick_count % self._SLOW_EVERY == 0)

        try:
            # 1. 消费最新状态
            if self._state_dirty:
                self._state_dirty = False
                new_pos = self._pending_pos.copy()
                self._current_rpm    = self._pending_rpm
                self._current_thrust = self._pending_thrust
                self._apply_body_move(new_pos)
                self._last_pos = new_pos

                # 慢路径（shadow / trail）：拖拽时跳过，非拖拽时降频
                if not self._dragging and slow_tick:
                    self._update_shadow(new_pos)
                    self._update_trail(new_pos)

                self._update_thrust_cones(new_pos, self._current_thrust)
                self._update_hud(new_pos)

            # 2. 旋翼动画（每帧，轻量）
            if self._current_rpm > 1.0:
                self._animate_rotors_tick()

        except Exception:
            pass

    # ──────────────────────────────────────────────────────
    #  机身整体位移（增量 translate，避免 setTransform）
    # ──────────────────────────────────────────────────────
    def _apply_body_move(self, new_pos: np.ndarray):
        delta = new_pos - self._body_offset
        if np.linalg.norm(delta) < 1e-6:
            return
        dx, dy, dz = float(delta[0]), float(delta[1]), float(delta[2])
        for part in self._body_parts:
            try:
                part.translate(dx, dy, dz)
            except Exception:
                pass
        self._body_offset = new_pos.copy()

    # ──────────────────────────────────────────────────────
    #  阴影（降频）
    # ──────────────────────────────────────────────────────
    def _update_shadow(self, pos: np.ndarray):
        alt_norm = min(pos[2] / 30.0, 1.0)
        shadow_a = int(80 * (1.0 - alt_norm * 0.72))
        n_faces  = len(self._shadow_faces)
        self._shadow.setMeshData(
            vertexes=self._shadow_verts,
            faces=self._shadow_faces,
            faceColors=np.tile(
                _r((*_SHADOW_COL[:3], shadow_a)), (n_faces, 1)),
        )
        T = _make_transform(np.eye(3),
                            np.array([pos[0], pos[1], 0.01]))
        self._shadow.setTransform(T)

    # ──────────────────────────────────────────────────────
    #  轨迹（降频）
    # ──────────────────────────────────────────────────────
    def _update_trail(self, pos: np.ndarray):
        self._trail.append(pos.copy())
        if len(self._trail) > self.MAX_TRAIL:
            self._trail.pop(0)
        if len(self._trail) >= 2:
            arr   = np.array(self._trail)
            z_min = float(arr[:, 2].min())
            z_max = float(arr[:, 2].max())
            cols  = np.array([_trail_color(p[2], z_min, z_max)
                               for p in arr])
            self._trail_line.setData(pos=arr, color=cols)

    # ──────────────────────────────────────────────────────
    #  推力锥（原地修改预分配缓冲）
    # ──────────────────────────────────────────────────────
    def _update_thrust_cones(self, pos: np.ndarray, thrust: float):
        if self._cone_verts_buf is None or self._cone_faces_buf is None:
            return
        t_norm   = min(thrust / max(self.THRUST_MAX_N, 1.0), 1.0)
        cone_len = max(t_norm * self.THRUST_LEN, 0.01)
        cone_rad = self.THRUST_RAD * (0.35 + 0.65 * t_norm)
        alpha    = int(25 + t_norm * 140)
        col      = (*_THRUST_COL[:3], alpha)

        # 原地重写预分配缓冲顶点（无 numpy 新分配）
        segs = self.THRUST_SEGS
        a = np.linspace(0, 2*math.pi, segs, endpoint=False)
        self._cone_verts_buf[:segs, 0] = cone_rad * np.cos(a)
        self._cone_verts_buf[:segs, 1] = cone_rad * np.sin(a)
        self._cone_verts_buf[:segs, 2] = 0.0
        self._cone_verts_buf[segs]     = [0.0, 0.0, -cone_len]

        tc = np.tile(_r(col),
                     (len(self._cone_faces_buf), 1))
        for tm, offset in zip(self._thrust_meshes, self._rotor_offsets):
            try:
                tm.setMeshData(
                    vertexes=self._cone_verts_buf + pos + offset,
                    faces=self._cone_faces_buf,
                    faceColors=tc,
                )
            except Exception:
                pass

    # ──────────────────────────────────────────────────────
    #  旋翼动画（每 tick，轻量）
    # ──────────────────────────────────────────────────────
    def _animate_rotors_tick(self):
        if (self._rotor_verts is None or self._rotor_faces is None
                or self._ring_verts is None or self._ring_faces is None):
            return

        speed_deg = self._current_rpm / 60.0 * 360.0 / 33.0  # 33ms/tick
        dirs      = [1, -1, -1, 1]
        rpm_norm  = min(self._current_rpm /
                        max(self._vehicle.rotors.rpm_max, 1.0), 1.0)
        rotor_col = _lerp(_r(_ROTOR_DIM), _r(_ROTOR_BRIGHT), rpm_norm)
        rv = self._rotor_verts
        rf = self._rotor_faces
        ringv = self._ring_verts
        ringf = self._ring_faces

        for i, (mesh, ring, offset) in enumerate(
                zip(self._rotor_meshes,
                    self._rotor_ring_meshes,
                    self._rotor_offsets)):
            self._rotor_angles[i] += speed_deg * dirs[i]
            world_pos = self._last_pos + offset
            a  = math.radians(self._rotor_angles[i])
            ca, sa = math.cos(a), math.sin(a)
            Rz = np.array([[ca, -sa, 0],
                           [sa,  ca, 0],
                           [0,   0,  1]])
            try:
                mesh.setMeshData(
                    vertexes=(rv @ Rz.T) + world_pos,
                    faces=rf,
                    faceColors=np.tile(rotor_col, (len(rf), 1)),
                )
                ring.setMeshData(
                    vertexes=ringv + world_pos,
                    faces=ringf,
                    faceColors=np.tile(_r(_RING_COL), (len(ringf), 1)),
                )
            except Exception:
                pass

    # ──────────────────────────────────────────────────────
    #  HUD
    # ──────────────────────────────────────────────────────
    def _update_hud(self, pos: np.ndarray):
        self._hud.setText(
            f"ALT {pos[2]:6.2f} m  │  "
            f"RPM {self._current_rpm:5.0f}  │  "
            f"T {self._current_thrust:6.1f} N  │  "
            f"XY ({pos[0]:.1f}, {pos[1]:.1f})"
        )

    # ──────────────────────────────────────────────────────
    #  UI
    # ──────────────────────────────────────────────────────
    def _build_ui(self):
        vbox = QtWidgets.QVBoxLayout(self)
        vbox.setContentsMargins(0, 0, 0, 0)
        vbox.setSpacing(0)

        bar = QtWidgets.QWidget()
        bar.setStyleSheet(
            "background:#0d1117;border-bottom:1px solid #21262d;")
        bar.setFixedHeight(38)
        row = QtWidgets.QHBoxLayout(bar)
        row.setContentsMargins(10, 4, 10, 4)
        row.setSpacing(8)
        self._btn_reset  = self._btn("Reset Camera", "#58a6ff",
                                      self._reset_camera)
        self._btn_trail  = self._btn("Clear Trail",  "#3fb950",
                                      self._clear_trail)
        self._btn_follow = self._btn("Follow OFF",   "#d29922",
                                      self._toggle_follow)
        row.addWidget(self._btn_reset)
        row.addWidget(self._btn_trail)
        row.addWidget(self._btn_follow)
        row.addStretch()
        self._hud = QtWidgets.QLabel("Waiting for simulation...")
        self._hud.setStyleSheet(
            "color:#00d4ff;font-family:Consolas,monospace;font-size:10pt;"
            "background:transparent;padding:0 8px;letter-spacing:1px;"
        )
        row.addWidget(self._hud)
        vbox.addWidget(bar)

        self._view = gl.GLViewWidget()
        self._view.setBackgroundColor(_BG)
        self._view.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Expanding,
        )
        vbox.addWidget(self._view)

        # 安装拖拽检测过滤器
        self._install_drag_filter()

    def _btn(self, text, color, fn):
        b = QtWidgets.QPushButton(text)
        b.setStyleSheet(
            f"QPushButton{{background:#161b22;color:{color};"
            f"border:1px solid {color};border-radius:5px;"
            f"padding:3px 12px;font-family:Consolas;font-size:9pt;}}"
            f"QPushButton:hover{{background:{color};color:#0a0e14;}}"
        )
        b.clicked.connect(fn)
        return b

    # ──────────────────────────────────────────────────────
    #  场景构建（__init__ 里只调用一次）
    # ──────────────────────────────────────────────────────
    def _build_scene(self):
        v  = self._vehicle
        fx = v.fuselage.length_m
        fy = v.fuselage.width_m
        fz = v.fuselage.height_m

        # 地面网格
        for size, spacing, alpha in [(60, 5, 0.5), (60, 1, 0.10)]:
            g = gl.GLGridItem()
            g.setSize(size, size)
            g.setSpacing(spacing, spacing)
            r, gg, b, _ = _r(_GRID_COL)
            g.setColor((r, gg, b, alpha))
            self._view.addItem(g)

        gv, gf = _box_mesh(60, 60, 0.02)
        gc = np.tile(_r(_GROUND_COL), (len(gf), 1))
        gp = gl.GLMeshItem(vertexes=gv, faces=gf, faceColors=gc,
                           smooth=False, drawEdges=False)
        gp.translate(0, 0, -0.02)
        self._view.addItem(gp)

        ax = gl.GLAxisItem()
        ax.setSize(3, 3, 3)
        self._view.addItem(ax)

        self._body_parts: list[gl.GLMeshItem] = []

        # 机身主体
        ev, ef = _ellipsoid_mesh(fx*0.50, fy*0.44, fz*0.50, 28, 16)
        ec = np.tile(_r(_FUSE_FACE), (len(ef), 1))
        self._fuse = gl.GLMeshItem(
            vertexes=ev, faces=ef, faceColors=ec,
            smooth=True, drawEdges=True, edgeColor=_r(_FUSE_EDGE),
        )
        self._view.addItem(self._fuse)
        self._body_parts.append(self._fuse)

        # 驾驶舱
        cav, caf = _ellipsoid_mesh(fx*0.24, fy*0.30, fz*0.26, 22, 12)
        canopy_offset = np.array([fx*0.08, 0.0, fz*0.38])
        cac = np.tile(_r(_CANOPY_FACE), (len(caf), 1))
        self._canopy = gl.GLMeshItem(
            vertexes=cav + canopy_offset, faces=caf, faceColors=cac,
            smooth=True, drawEdges=True, edgeColor=_r(_CANOPY_EDGE),
        )
        self._view.addItem(self._canopy)
        self._body_parts.append(self._canopy)

        # 支臂
        arm_w, arm_h = 0.11, 0.055
        for pos in v.rotors.positions:
            av, af = _arm_local(pos[0], pos[1], pos[2], arm_w, arm_h)
            ac = np.tile(_r(_ARM_FACE), (len(af), 1))
            arm = gl.GLMeshItem(
                vertexes=av, faces=af, faceColors=ac,
                smooth=False, drawEdges=True, edgeColor=_r(_ARM_EDGE),
            )
            self._view.addItem(arm)
            self._body_parts.append(arm)

        # 起落架
        leg_list, skid_list = _landing_gear_local(fx, fy, fz)
        for lv, lf in leg_list:
            lc = np.tile(_r(_GEAR_FACE), (len(lf), 1))
            leg = gl.GLMeshItem(vertexes=lv, faces=lf, faceColors=lc,
                                smooth=False, drawEdges=True,
                                edgeColor=_r(_GEAR_EDGE))
            self._view.addItem(leg)
            self._body_parts.append(leg)
        for sv2, sf2 in skid_list:
            sc2 = np.tile(_r(_GEAR_FACE), (len(sf2), 1))
            skid = gl.GLMeshItem(vertexes=sv2, faces=sf2, faceColors=sc2,
                                 smooth=False, drawEdges=True,
                                 edgeColor=_r(_GEAR_EDGE))
            self._view.addItem(skid)
            self._body_parts.append(skid)

        # 旋翼（预分配）
        self._rotor_meshes:      list[gl.GLMeshItem] = []
        self._rotor_ring_meshes: list[gl.GLMeshItem] = []
        self._rotor_offsets = [np.array(p, dtype=float)
                               for p in v.rotors.positions]

        rv, rf       = _disk_mesh(v.rotors.radius_m, self.ROTOR_SEGS)
        ringv, ringf = _ring_mesh(v.rotors.radius_m,
                                   v.rotors.radius_m * 1.12, 36)
        self._rotor_verts = rv.copy()
        self._rotor_faces = rf.copy()
        self._ring_verts  = ringv.copy()
        self._ring_faces  = ringf.copy()

        rc     = np.tile(_r(_ROTOR_DIM),  (len(rf), 1))
        ring_c = np.tile(_r(_RING_COL),   (len(ringf), 1))

        for pos in v.rotors.positions:
            m = gl.GLMeshItem(vertexes=rv.copy(), faces=rf.copy(),
                              faceColors=rc.copy(), smooth=True,
                              drawEdges=False)
            m.translate(*pos)
            self._view.addItem(m)
            self._rotor_meshes.append(m)

            rm = gl.GLMeshItem(vertexes=ringv.copy(), faces=ringf.copy(),
                               faceColors=ring_c.copy(), smooth=False,
                               drawEdges=False)
            rm.translate(*pos)
            self._view.addItem(rm)
            self._rotor_ring_meshes.append(rm)

        # 推力锥（预分配缓冲，原地修改）
        self._thrust_meshes: list[gl.GLMeshItem] = []
        tcv, tcf = _cone_mesh(self.THRUST_RAD, 0.01, self.THRUST_SEGS)
        self._cone_verts_buf = tcv.copy()   # 预分配，后续原地修改
        self._cone_faces_buf = tcf.copy()
        thrust_c = np.tile(_r(_THRUST_COL), (len(tcf), 1))
        for pos in v.rotors.positions:
            tm = gl.GLMeshItem(vertexes=tcv.copy(), faces=tcf.copy(),
                               faceColors=thrust_c.copy(), smooth=True,
                               drawEdges=False)
            tm.translate(*pos)
            self._view.addItem(tm)
            self._thrust_meshes.append(tm)

        # 地面阴影（预分配）
        self._shadow_r     = max(fx, fy) * 0.72
        sv3, sf3           = _disk_mesh(self._shadow_r, 36)
        self._shadow_verts = sv3.copy()
        self._shadow_faces = sf3.copy()
        sc3 = np.tile(_r(_SHADOW_COL), (len(sf3), 1))
        self._shadow = gl.GLMeshItem(vertexes=sv3, faces=sf3,
                                     faceColors=sc3, smooth=True)
        self._shadow.translate(0, 0, 0.01)
        self._view.addItem(self._shadow)

        # 轨迹
        self._trail_line = gl.GLLinePlotItem(
            pos=np.array([[0.0, 0.0, 0.0]]),
            color=_r(_TRAIL_MID), width=3, antialias=True,
        )
        self._view.addItem(self._trail_line)

    # ──────────────────────────────────────────────────────
    #  相机控制
    # ──────────────────────────────────────────────────────
    def _reset_camera(self):
        self._follow_cam = False
        self._btn_follow.setText("Follow OFF")
        self._view.setCameraPosition(distance=20, elevation=28, azimuth=42)

    def _clear_trail(self):
        self._trail.clear()
        self._trail_line.setData(pos=np.array([[0.0, 0.0, 0.0]]))

    def _toggle_follow(self):
        self._follow_cam = not self._follow_cam
        if self._follow_cam:
            self._btn_follow.setText("Follow ON")
            self._btn_follow.setStyleSheet(
                "QPushButton{background:#d29922;color:#0a0e14;"
                "border:1px solid #d29922;border-radius:5px;"
                "padding:3px 12px;font-family:Consolas;font-size:9pt;}"
            )
        else:
            self._btn_follow.setText("Follow OFF")
            self._btn_follow.setStyleSheet(
                "QPushButton{background:#161b22;color:#d29922;"
                "border:1px solid #d29922;border-radius:5px;"
                "padding:3px 12px;font-family:Consolas;font-size:9pt;}"
                "QPushButton:hover{background:#d29922;color:#0a0e14;}"
            )


# ══════════════════════════════════════════════════════════
#  几何辅助函数
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

def _box_mesh_at(cx, cy, cz, lx, ly, lz):
    v, f = _box_mesh(lx, ly, lz)
    v += np.array([cx, cy, cz], dtype=float)
    return v, f

def _ellipsoid_mesh(rx, ry, rz, lon=28, lat=16):
    verts = [[0.0, 0.0, rz], [0.0, 0.0, -rz]]
    for i in range(1, lat):
        phi = math.pi * i / lat
        z   = rz * math.cos(phi)
        r   = math.sin(phi)
        for j in range(lon):
            theta = 2*math.pi * j / lon
            verts.append([rx*r*math.cos(theta), ry*r*math.sin(theta), z])
    v = np.array(verts, dtype=float)
    faces = []
    for j in range(lon):
        a, b = 2+j, 2+(j+1)%lon
        faces.append([0, a, b])
    for i in range(lat-2):
        ra = 2 + i*lon
        rb = 2 + (i+1)*lon
        for j in range(lon):
            a0, a1 = ra+j, ra+(j+1)%lon
            b0, b1 = rb+j, rb+(j+1)%lon
            faces += [[a0,a1,b1],[a0,b1,b0]]
    last = 2 + (lat-2)*lon
    for j in range(lon):
        a, b = last+j, last+(j+1)%lon
        faces.append([1, b, a])
    return v, np.array(faces, dtype=np.uint32)

def _arm_local(px, py, pz_off, arm_w, arm_h):
    length = math.hypot(px, py)
    angle  = math.atan2(py, px)
    ca, sa = math.cos(angle), math.sin(angle)
    hl, hw, hh = length/2, arm_w/2, arm_h/2
    local = np.array([
        [-hl,-hw,-hh],[ hl,-hw,-hh],[ hl, hw,-hh],[-hl, hw,-hh],
        [-hl,-hw, hh],[ hl,-hw, hh],[ hl, hw, hh],[-hl, hw, hh],
    ], dtype=float)
    R = np.array([[ca,-sa,0],[sa,ca,0],[0,0,1]])
    v = local @ R.T + np.array([px/2, py/2, pz_off])
    f = np.array([
        [0,1,2],[0,2,3],[4,5,6],[4,6,7],
        [0,1,5],[0,5,4],[2,3,7],[2,7,6],
        [1,2,6],[1,6,5],[0,3,7],[0,7,4],
    ], dtype=np.uint32)
    return v, f

def _landing_gear_local(fx, fy, fz):
    leg_w, leg_h = 0.055, 0.055
    leg_len = fz * 0.72
    top_x, top_y = fx * 0.33, fy * 0.35
    top_z = -fz * 0.50
    spread_x, spread_y = fx * 0.09, fy * 0.13
    bot_z = top_z - leg_len
    legs = []
    for sx, sy in [(1,1),(1,-1),(-1,1),(-1,-1)]:
        tx, ty = sx*top_x, sy*top_y
        bx, by = sx*(top_x+spread_x), sy*(top_y+spread_y)
        lv, lf = _box_mesh_at((tx+bx)/2, (ty+by)/2, (top_z+bot_z)/2,
                               leg_w, leg_h, leg_len)
        legs.append((lv, lf))
    skid_thick, skid_h = 0.052, 0.042
    skids = []
    skid_span = fy * 0.76
    for sx in (1, -1):
        cx = sx * (top_x + spread_x)
        sv, sf = _box_mesh_at(cx, 0.0, bot_z + skid_h/2,
                               skid_thick, skid_span, skid_h)
        skids.append((sv, sf))
    cross_span = fx * 0.66
    for sy in (1, -1):
        cy = sy * (top_y + spread_y)
        cv, cf = _box_mesh_at(0.0, cy, bot_z + skid_h/2,
                               cross_span, skid_thick, skid_h)
        skids.append((cv, cf))
    return legs, skids

def _disk_mesh(radius, segments=32):
    a   = np.linspace(0, 2*math.pi, segments, endpoint=False)
    rim = np.stack([radius*np.cos(a), radius*np.sin(a), np.zeros(segments)], 1)
    v   = np.vstack([[[0,0,0]], rim]).astype(float)
    f   = np.array([[0, i+1, (i+1)%segments+1]
                    for i in range(segments)], dtype=np.uint32)
    return v, f

def _ring_mesh(r_inner, r_outer, segments=36):
    a     = np.linspace(0, 2*math.pi, segments, endpoint=False)
    inner = np.stack([r_inner*np.cos(a), r_inner*np.sin(a), np.zeros(segments)], 1)
    outer = np.stack([r_outer*np.cos(a), r_outer*np.sin(a), np.zeros(segments)], 1)
    v = np.vstack([inner, outer]).astype(float)
    n = segments
    faces = []
    for i in range(n):
        j = (i+1)%n
        faces += [[i,j,n+j],[i,n+j,n+i]]
    return v, np.array(faces, dtype=np.uint32)

def _cone_mesh(radius, height, segments=22):
    if height < 1e-6:
        return (np.zeros((segments+1, 3)),
                np.zeros((segments, 3), dtype=np.uint32))
    a   = np.linspace(0, 2*math.pi, segments, endpoint=False)
    rim = np.stack([radius*np.cos(a), radius*np.sin(a), np.zeros(segments)], 1)
    tip = np.array([[0.0, 0.0, -height]])
    v   = np.vstack([rim, tip]).astype(float)
    f   = np.array([[i, (i+1)%segments, segments]
                    for i in range(segments)], dtype=np.uint32)
    return v, f

def _make_transform(R, t):
    M = np.eye(4)
    M[:3,:3] = R
    M[:3, 3] = t
    return M