"""
gui/widgets/view3d_widget.py
eVTOL 三维实时可视化 Widget
依赖：pyqtgraph >= 0.13, PyOpenGL
"""

from __future__ import annotations
import math
import numpy as np

import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtWidgets, QtCore

from core.vehicle import VehicleConfig
from core.event_bus import EventBus, TwinState

# ── 颜色常量（与 GUI 主题对齐）────────────────────────────
CLR_BG       = (13,  17,  23,  255)   # #0d1117
CLR_FUSELAGE = (48, 54, 61, 180)      # 半透明深灰
CLR_ROTOR    = (88, 166, 255, 160)    # 蓝色半透明
CLR_TRAIL    = (63, 185, 80,  255)    # 绿色轨迹
CLR_GRID     = (30,  37,  44,  255)


def _rgba(t): return tuple(c / 255.0 for c in t)


class View3DWidget(QtWidgets.QWidget):
    """
    三维视图 Widget。
    调用方式：
        w = View3DWidget(vehicle, bus)
    当 bus 发布 TwinState 时自动刷新。
    """

    MAX_TRAIL = 500   # 轨迹最多保留的点数

    def __init__(self, vehicle: VehicleConfig, bus: EventBus,
                 parent=None):
        super().__init__(parent)
        self._vehicle = vehicle
        self._bus = bus
        self._trail: list[np.ndarray] = []   # 历史位置

        self._build_ui()
        self._build_scene()
        self._reset_camera()

        # 订阅孪生状态更新
        bus.subscribe(TwinState, self._on_state)

    # ── UI 布局 ───────────────────────────────────────────
    def _build_ui(self):
        vbox = QtWidgets.QVBoxLayout(self)
        vbox.setContentsMargins(4, 4, 4, 4)
        vbox.setSpacing(4)

        # 顶部工具栏
        toolbar = QtWidgets.QHBoxLayout()
        self._btn_reset_cam = QtWidgets.QPushButton("🎥 Reset Camera")
        self._btn_reset_cam.clicked.connect(self._reset_camera)
        self._btn_clear_trail = QtWidgets.QPushButton("🗑 Clear Trail")
        self._btn_clear_trail.clicked.connect(self._clear_trail)
        self._lbl_pos = QtWidgets.QLabel("Position: —")
        self._lbl_pos.setStyleSheet("color:#8b949e; font-family:Consolas;")

        for btn in (self._btn_reset_cam, self._btn_clear_trail):
            btn.setStyleSheet(
                "QPushButton { background:#21262d; color:#c9d1d9;"
                " border:1px solid #30363d; border-radius:6px; padding:4px 12px; }"
                "QPushButton:hover { background:#30363d; }"
            )
        toolbar.addWidget(self._btn_reset_cam)
        toolbar.addWidget(self._btn_clear_trail)
        toolbar.addStretch()
        toolbar.addWidget(self._lbl_pos)
        vbox.addLayout(toolbar)

        # OpenGL 视图
        self._view = gl.GLViewWidget()
        self._view.setBackgroundColor(CLR_BG[:3])  # RGB
        self._view.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Expanding,
        )
        vbox.addWidget(self._view)

    # ── 场景构建 ──────────────────────────────────────────
    def _build_scene(self):
        v = self._vehicle

        # 网格地面
        grid = gl.GLGridItem()
        grid.setSize(20, 20)
        grid.setSpacing(1, 1)
        grid.setColor(_rgba(CLR_GRID))
        self._view.addItem(grid)

        # XYZ 坐标轴
        ax = gl.GLAxisItem()
        ax.setSize(2, 2, 2)
        self._view.addItem(ax)

        # ── 机身（长方体）────────────────────────────────
        fx, fy, fz = (v.fuselage.length_m,
                      v.fuselage.width_m,
                      v.fuselage.height_m)
        fuse_verts, fuse_faces = _box_mesh(fx, fy, fz)
        fuse_color = np.array([_rgba(CLR_FUSELAGE)] * len(fuse_faces))
        self._fuse = gl.GLMeshItem(
            vertexes=fuse_verts,
            faces=fuse_faces,
            faceColors=fuse_color,
            smooth=False,
            drawEdges=True,
            edgeColor=_rgba((80, 90, 100, 200)),
        )
        self._view.addItem(self._fuse)

        # ── 旋翼圆盘 ──────────────────────────────────────
        self._rotor_meshes: list[gl.GLMeshItem] = []
        rotor_verts, rotor_faces = _disk_mesh(v.rotors.radius_m, segments=32)
        rotor_color = np.array([_rgba(CLR_ROTOR)] * len(rotor_faces))

        for pos in v.rotors.positions:
            m = gl.GLMeshItem(
                vertexes=rotor_verts.copy(),
                faces=rotor_faces,
                faceColors=rotor_color,
                smooth=True,
                drawEdges=False,
            )
            m.translate(*pos)
            self._view.addItem(m)
            self._rotor_meshes.append(m)

        # ── 轨迹折线 ──────────────────────────────────────
        self._trail_line = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0]]),
            color=_rgba(CLR_TRAIL),
            width=2,
            antialias=True,
        )
        self._view.addItem(self._trail_line)

        # 记录初始旋翼偏移（用于平移）
        self._rotor_offsets = [np.array(p, dtype=float)
                               for p in v.rotors.positions]
        self._last_pos = np.zeros(3)

    # ── 相机 ──────────────────────────────────────────────
    def _reset_camera(self):
        self._view.setCameraPosition(distance=12, elevation=25, azimuth=45)

    def _clear_trail(self):
        self._trail.clear()
        self._trail_line.setData(pos=np.array([[0, 0, 0]]))

    # ── 状态更新回调 ──────────────────────────────────────
    def _on_state(self, state: TwinState):
        pos = np.array([state.x, state.y, state.z])
        roll, pitch, yaw = state.roll, state.pitch, state.yaw

        # 更新机身位置 + 姿态
        R = _euler_to_rotation(roll, pitch, yaw)
        transform = _make_transform(R, pos)
        self._fuse.setTransform(transform)

        # 更新旋翼位置（随机身平移 + 旋转）
        for mesh, offset in zip(self._rotor_meshes, self._rotor_offsets):
            rotor_world = pos + R @ offset
            rt = _make_transform(R, rotor_world)
            mesh.setTransform(rt)

        # 更新轨迹
        self._trail.append(pos.copy())
        if len(self._trail) > self.MAX_TRAIL:
            self._trail.pop(0)
        if len(self._trail) >= 2:
            trail_arr = np.array(self._trail)
            self._trail_line.setData(pos=trail_arr)

        # 更新标签
        self._lbl_pos.setText(
            f"Position: x={pos[0]:.1f}  y={pos[1]:.1f}  z={pos[2]:.1f} m  "
            f"| Roll={math.degrees(roll):.1f}°  Pitch={math.degrees(pitch):.1f}°  "
            f"Yaw={math.degrees(yaw):.1f}°"
        )

        # 记录上次位置
        self._last_pos = pos


# ═══════════════════════════════════════════════════════════
#  几何辅助函数
# ═══════════════════════════════════════════════════════════

def _box_mesh(lx: float, ly: float, lz: float):
    """生成长方体网格（顶点 + 三角面）"""
    hx, hy, hz = lx / 2, ly / 2, lz / 2
    verts = np.array([
        [-hx, -hy, -hz], [ hx, -hy, -hz],
        [ hx,  hy, -hz], [-hx,  hy, -hz],
        [-hx, -hy,  hz], [ hx, -hy,  hz],
        [ hx,  hy,  hz], [-hx,  hy,  hz],
    ], dtype=float)
    faces = np.array([
        [0,1,2],[0,2,3],  # 底面
        [4,5,6],[4,6,7],  # 顶面
        [0,1,5],[0,5,4],  # 前
        [2,3,7],[2,7,6],  # 后
        [1,2,6],[1,6,5],  # 右
        [0,3,7],[0,7,4],  # 左
    ], dtype=np.uint32)
    return verts, faces


def _disk_mesh(radius: float, segments: int = 32):
    """生成圆盘网格（旋翼俯视）"""
    angles = np.linspace(0, 2 * math.pi, segments, endpoint=False)
    rim = np.stack([
        radius * np.cos(angles),
        radius * np.sin(angles),
        np.zeros(segments),
    ], axis=1)
    center = np.array([[0.0, 0.0, 0.0]])
    verts = np.vstack([center, rim]).astype(float)

    # 扇形三角面
    faces = []
    for i in range(segments):
        a = i + 1
        b = (i + 1) % segments + 1
        faces.append([0, a, b])
    faces = np.array(faces, dtype=np.uint32)
    return verts, faces


def _euler_to_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX 欧拉角 → 旋转矩阵 (3×3)"""
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)

    Rz = np.array([[cy,-sy, 0],[sy, cy, 0],[0,  0, 1]])
    Ry = np.array([[cp, 0, sp],[0,  1,  0],[-sp,0, cp]])
    Rx = np.array([[1,  0,  0],[0, cr,-sr],[0, sr, cr]])
    return Rz @ Ry @ Rx


def _make_transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """拼装 4×4 齐次变换矩阵（列主序，OpenGL 约定）"""
    M = np.eye(4)
    M[:3, :3] = R
    M[:3,  3] = t
    # pyqtgraph GLMeshItem.setTransform 接受 4×4 行主序
    return M