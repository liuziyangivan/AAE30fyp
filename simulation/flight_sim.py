from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Callable, List, Tuple

from core.digital_twin import DigitalTwin
from core.data_recorder import DataRecorder, FlightRecord


# ══════════════════════════════════════════════════════════
#  Data Structures
# ══════════════════════════════════════════════════════════

@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    speed_ms: float = 6.0
    hold_s:   float = 0.0
    label:    str   = ""


@dataclass
class BatteryConfig:
    capacity_Wh:     float = 1200.0
    voltage_V:       float = 60.0
    internal_resist: float = 0.03
    temp_init_C:     float = 25.0
    temp_max_C:      float = 65.0
    battery_mass_kg: float = 18.0
    energy_density:  float = 180.0
    soc_voltage_k:   float = 0.10


@dataclass
class RotorConfig:
    count:       int   = 4
    diameter_m:  float = 0.8
    Ct:          float = 0.012
    Cq:          float = 0.0015
    rpm_max:     float = 6000.0
    tau_s:       float = 0.18


@dataclass
class SimConfig:
    dt:              float = 0.02
    duration_s:      float = 180.0
    total_mass_kg:   float = 60.0
    Cd_body:         float = 0.85
    Cd_lateral:      float = 0.30
    frontal_area_m2: float = 0.35
    top_area_m2:     float = 0.80
    pitch_max_deg:   float = 15.0
    wind_speed_ms:   float = 0.0
    wind_dir_deg:    float = 0.0
    rotor:   RotorConfig   = field(default_factory=RotorConfig)
    battery: BatteryConfig = field(default_factory=BatteryConfig)
    waypoints: List[Waypoint] = field(default_factory=list)


@dataclass
class BatteryState:
    soc:             float = 1.0
    voltage_V:       float = 60.0
    temp_C:          float = 25.0
    energy_used_Wh:  float = 0.0
    current_A:       float = 0.0
    range_est_m:     float = 0.0
    mass_kg:         float = 18.0


@dataclass
class SimFrame:
    t:               float
    x_m:             float
    y_m:             float
    altitude_m:      float
    vx_ms:           float
    vy_ms:           float
    velocity_ms:     float
    pitch_deg:       float
    rpm:             float
    rpm_actual:      float
    thrust_N:        float
    power_W:         float
    accel_ms2:       float
    air_density:     float
    ground_effect:   float
    soc:             float
    battery_temp_C:  float
    range_est_m:     float
    energy_used_Wh:  float
    waypoint_idx:    int
    phase:           str


# ══════════════════════════════════════════════════════════
#  Default Mission
# ══════════════════════════════════════════════════════════

def default_mission() -> List[Waypoint]:
    return [
        Waypoint(x=0,  y=0,  z=0,  speed_ms=0,   label="Origin"),
        Waypoint(x=0,  y=0,  z=15, speed_ms=1.5, label="Climb"),
        Waypoint(x=40, y=0,  z=15, speed_ms=3.0, label="Fly East"),
        Waypoint(x=40, y=30, z=15, speed_ms=2.5, label="Fly North"),
        Waypoint(x=40, y=30, z=15, speed_ms=0,   hold_s=8.0, label="Hover"),
        Waypoint(x=0,  y=0,  z=15, speed_ms=3.0, label="Return"),
        Waypoint(x=0,  y=0,  z=0,  speed_ms=1.0, label="Land"),
    ]


# ══════════════════════════════════════════════════════════
#  Physics Helpers
# ══════════════════════════════════════════════════════════

def air_density(altitude_m: float) -> float:
    return 1.225 * math.exp(-altitude_m / 8500.0)


def rotor_thrust(rpm: float, rho: float, cfg: RotorConfig) -> float:
    n = rpm / 60.0
    D = cfg.diameter_m
    return cfg.Ct * rho * (n ** 2) * (D ** 4) * cfg.count


def rotor_power(rpm: float, rho: float, cfg: RotorConfig) -> float:
    n = rpm / 60.0
    D = cfg.diameter_m
    return cfg.Cq * rho * (n ** 3) * (D ** 5) * cfg.count


def ground_effect_factor(z: float, R: float) -> float:
    if z < 0.1:
        return 1.35
    ratio = R / (4 * z)
    if ratio >= 1.0:
        return 1.35
    return min(1.0 / (1.0 - ratio ** 2), 1.35)


def drag_force(v_rel: float, Cd: float, area: float, rho: float) -> float:
    return 0.5 * rho * Cd * area * v_rel * abs(v_rel)


# ══════════════════════════════════════════════════════════
#  Battery Model
# ══════════════════════════════════════════════════════════

class BatteryModel:
    def __init__(self, cfg: BatteryConfig):
        self._cfg  = cfg
        self.state = BatteryState(
            soc=1.0,
            voltage_V=cfg.voltage_V * 1.05,
            temp_C=cfg.temp_init_C,
            mass_kg=cfg.battery_mass_kg,
        )

    def step(self, power_W: float, dt: float,
             speed_ms: float = 0.0) -> BatteryState:
        s, cfg = self.state, self._cfg

        v_factor = 1.05 - cfg.soc_voltage_k * (1.0 - s.soc)
        voltage  = cfg.voltage_V * v_factor
        current  = power_W / max(voltage, 1.0)
        heat_W   = current ** 2 * cfg.internal_resist
        total_W  = power_W + heat_W

        dE_Wh             = total_W * dt / 3600.0
        s.energy_used_Wh += dE_Wh
        s.soc             = max(0.0, 1.0 - s.energy_used_Wh / cfg.capacity_Wh)
        s.voltage_V       = voltage
        s.current_A       = current
        s.mass_kg         = cfg.battery_mass_kg * (1.0 - 0.003 * (1.0 - s.soc))

        thermal_rise = heat_W * 0.003 * dt
        thermal_cool = (s.temp_C - cfg.temp_init_C) * 0.0008 * dt
        s.temp_C = min(cfg.temp_max_C, s.temp_C + thermal_rise - thermal_cool)

        remaining_Wh = s.soc * cfg.capacity_Wh
        if total_W > 1.0 and speed_ms > 0.1:
            s.range_est_m = (remaining_Wh / total_W) * 3600 * speed_ms
        else:
            s.range_est_m = 0.0

        return s


# ══════════════════════════════════════════════════════════
#  Flight Controller
# ══════════════════════════════════════════════════════════

class FlightController:
    def __init__(self, cfg: SimConfig, hover_rpm: float):
        self._cfg    = cfg
        self._hover  = hover_rpm
        self._wp_idx = 0
        self._hold_t = 0.0
        self._phase  = "takeoff"
        self._landed = False
        self._wps    = cfg.waypoints if cfg.waypoints else default_mission()
        # Vertical velocity PI integral term
        self._vz_integral = 0.0

    @property
    def waypoint_idx(self): return self._wp_idx

    @property
    def phase(self):        return self._phase

    def _current_wp(self) -> Waypoint:
        return self._wps[min(self._wp_idx, len(self._wps) - 1)]

    def _next_wp(self):
        if self._wp_idx < len(self._wps) - 1:
            self._wp_idx += 1
            self._hold_t  = 0.0
            self._vz_integral *= 0.8

    def compute(self, t, x, y, z, vx, vy, vz, dt
                ) -> Tuple[float, float, float, str]:

        # ── 调试打印（独立，不影响任何返回值）─────────────
        if int(t * 10) % 20 == 0:
            wp_ = self._current_wp()
            print(
                f"[DBG] t={t:6.1f}s  phase={self._phase:8s}  wp={self._wp_idx}"
                f"  pos=({x:.1f},{y:.1f},{z:.1f})"
                f"  target=({wp_.x},{wp_.y},{wp_.z})"
                f"  dz={wp_.z - z:+.2f}  vz={vz:+.3f}"
                f"  dist_h={math.hypot(wp_.x-x, wp_.y-y):.2f}"
                f"  hold={self._hold_t:.1f}/{wp_.hold_s}"
            )

        if self._landed:
            return 0.0, 0.0, 0.0, "landed"

        wp         = self._current_wp()
        dx         = wp.x - x
        dy         = wp.y - y
        dz         = wp.z - z
        dist_horiz = math.hypot(dx, dy)

        if wp.hold_s > 0:
            h_thresh = 5.0
            dz_thresh = 8.0
        else:
            h_thresh = 3.5
            dz_thresh = 4.0
        reached = dist_horiz < h_thresh and abs(dz) < dz_thresh

        # 垂直控制
        if self._phase in ("land", "landed") or wp.z < 3.0:
            if z > 30.0:   target_vz = -1.2
            elif z > 20.0: target_vz = -0.9
            elif z > 10.0: target_vz = -0.6
            elif z > 5.0:  target_vz = -0.3
            elif z > 2.0:  target_vz = -0.15
            else:          target_vz = -0.05

            vz_err = target_vz - vz
            rpm = self._hover + vz_err * 400
            rpm = max(self._hover * 0.60, min(self._hover * 1.25, rpm))
            self._vz_integral = 0.0

        else:
            KP_Z   = 0.5
            MAX_VZ =  0.8
            MIN_VZ = -0.8
            target_vz = max(MIN_VZ, min(MAX_VZ, dz * KP_Z))
            vz_err    = target_vz - vz

            if abs(dz) > 1.0:
                self._vz_integral += vz_err * dt
                self._vz_integral  = max(-600.0, min(600.0, self._vz_integral))
            else:
                self._vz_integral *= 0.90

            KP_VZ = 180.0
            KI_VZ = 20.0
            self._vz_integral = max(-300.0, min(300.0, self._vz_integral))

            rpm = self._hover + vz_err * KP_VZ + self._vz_integral * KI_VZ
            rpm = max(self._hover * 0.75, min(self._hover * 1.30, rpm))

        # 水平控制
        target_spd = wp.speed_ms
        if dist_horiz > 0.5 and target_spd > 0.1:
            ux         = dx / dist_horiz
            uy         = dy / dist_horiz
            v_horiz    = math.hypot(vx, vy)
            decel_dist = max(v_horiz ** 2 / (2 * 2.0), 1.5)
            if dist_horiz < decel_dist:
                target_spd = max(0.5, target_spd * dist_horiz / decel_dist)
            spd_err = target_spd - v_horiz
            ax = ux * spd_err * 1.5 - vx * 0.5
            ay = uy * spd_err * 1.5 - vy * 0.5
        else:
            ax = -vx * 1.0
            ay = -vy * 1.0

        ax = max(-4.0, min(4.0, ax))
        ay = max(-4.0, min(4.0, ay))



        # 在 compute() 里，航路点切换部分整体替换为：
        if reached:
            if wp.hold_s > 0:
                self._phase = "hover"
            else:
                if self._wp_idx == len(self._wps) - 1 and z < 0.3:
                    self._landed = True
                    self._phase  = "landed"
                else:
                    self._next_wp()
                    next_wp     = self._current_wp()
                    self._phase = "land" if next_wp.z < 0.5 else "cruise"
        else:
            if z < 3.0 and t < 8.0:
                self._phase = "takeoff"
            elif wp.z < 0.5 and z < 0.3:
                self._landed = True
                self._phase  = "landed"
            elif dist_horiz > 3.0:
                self._phase = "cruise"

        # ← 关键：hover 计时独立于 reached，放在分支判断之后
        if self._phase == "hover":
            self._hold_t += dt
            if self._hold_t >= wp.hold_s:
                self._next_wp()

        return rpm, ax, ay, self._phase

# ══════════════════════════════════════════════════════════
#  Main Simulator
# ══════════════════════════════════════════════════════════

class FlightSimulator:

    EVT_SIM_FRAME = "sim_frame"
    EVT_SIM_DONE  = "sim_done"

    def __init__(self, twin: DigitalTwin, cfg: SimConfig | None = None):
        self._twin = twin
        self._cfg  = cfg or SimConfig()

        if not self._cfg.waypoints:
            self._cfg.waypoints = default_mission()

        # ── Step 1: rotor config 必须最先初始化 ──────────────
        v = twin.vehicle.rotors
        self._rotor_cfg = RotorConfig(
            count      = v.count,
            diameter_m = v.radius_m * 2,
            rpm_max    = v.rpm_max,
            Ct         = 0.0324,
            Cq         = 0.0041,
            tau_s      = 0.18,
        )
        self._cfg.rotor = self._rotor_cfg
        self._R         = self._rotor_cfg.diameter_m / 2.0

                # 从模拟器自身物理参数反算悬停转速（不依赖 twin 的标定值）
        rho0    = 1.225
        weight  = self._cfg.total_mass_kg * twin.vehicle.environment.gravity_m_s2
        n_hover = math.sqrt(
            weight / (
                self._rotor_cfg.Ct
                * rho0
                * (self._rotor_cfg.diameter_m ** 4)
                * self._rotor_cfg.count
            )
        )
        self._hover_rpm = min(n_hover * 60.0, self._rotor_cfg.rpm_max * 0.90)
        print(f"[Init] hover_rpm={self._hover_rpm:.1f}  twin.hover_rpm={twin.hover_rpm:.1f}")

        # ── Step 3: 其余组件 ──────────────────────────────────
        self._last_published_t = -1.0
        self._ctrl             = FlightController(self._cfg, self._hover_rpm)
        self._battery          = BatteryModel(self._cfg.battery)
        self._g                = twin.vehicle.environment.gravity_m_s2
        self._rpm_actual       = 0.0

        wd = math.radians(self._cfg.wind_dir_deg)
        ws = self._cfg.wind_speed_ms
        self._wind_x = -ws * math.sin(wd)
        self._wind_y = -ws * math.cos(wd)

        self._recorder = DataRecorder()
        self._fault_injector = None 

    def _current_mass(self) -> float:
        base = self._cfg.total_mass_kg - self._cfg.battery.battery_mass_kg
        return base + self._battery.state.mass_kg

    def _step(self, t, x, y, z, vx, vy, vz) -> SimFrame:
        dt   = self._cfg.dt
        cfg  = self._cfg
        rcfg = self._rotor_cfg

        # 控制器指令
        rpm_cmd, ax_cmd, ay_cmd, phase = self._ctrl.compute(
            t, x, y, z, vx, vy, vz, dt)

        # 电机响应延迟
        tau = rcfg.tau_s
        self._rpm_actual += (rpm_cmd - self._rpm_actual) * (dt / tau)
        self._rpm_actual  = max(0.0, min(rcfg.rpm_max, self._rpm_actual))
        rpm = self._rpm_actual

        # 空气密度
        rho = air_density(z)

        # 计算原始推力与地效
        T_raw     = rotor_thrust(rpm, rho, rcfg)
        ge_factor = ground_effect_factor(z, self._R)

        # ── 故障注入（包含力矩扰动） ──────────────────────────────
        if self._fault_injector is not None:
            mults = self._fault_injector.get_thrust_multipliers()
            health_ratio = sum(mults) / max(len(mults), 1)

            # 估算每个旋翼的当前推力（用于力矩分析）
            base_T_per_rotor = (T_raw * ge_factor) / max(rcfg.count, 1)
            analysis = self._fault_injector.analyze(base_T_per_rotor)

            # 将力矩转换为等效水平加速度扰动
            mass = self._current_mass()
            roll_disturb  = analysis.roll_moment_Nm  / (mass * self._R)
            pitch_disturb = analysis.pitch_moment_Nm / (mass * self._R)

            # 将扰动注入水平控制指令（模拟姿态耦合）
            ax_cmd += pitch_disturb * 0.5   # 俯仰扰动 → 纵向漂移
            ay_cmd += roll_disturb  * 0.5   # 滚转扰动 → 横向漂移
        else:
            health_ratio  = 1.0
            roll_disturb  = 0.0
            pitch_disturb = 0.0

        # 实际推力和功率（乘健康系数）
        thrust = T_raw * ge_factor * health_ratio
        power  = rotor_power(rpm, rho, rcfg) * health_ratio

        # 同步数字孪生（降频）
        if t - self._last_published_t >= 0.2:
            self._twin.set_rpm(rpm_cmd)
            self._twin.inject_position(x, y, z)
            self._last_published_t = t

        # 质量与重力
        mass   = self._current_mass()
        weight = mass * self._g

        # 垂直动力学
        drag_z = drag_force(vz, cfg.Cd_body, cfg.frontal_area_m2, rho)
        az     = (thrust - weight - drag_z) / mass
        new_vz = vz + az * dt

        MAX_DESCENT = 0.8
        if z > 1.0:
            new_vz = max(-MAX_DESCENT, new_vz)

        new_z = max(0.0, z + new_vz * dt)
        if new_z <= 0.0:
            new_vz = max(0.0, new_vz)

        # 俯仰角（用于计算有效迎风面积）
        v_horiz   = math.hypot(vx - self._wind_x, vy - self._wind_y)
        pitch_rad = math.atan2(v_horiz, max(thrust / mass, 0.1)) * 0.4
        pitch_rad = max(-math.radians(cfg.pitch_max_deg),
                        min(math.radians(cfg.pitch_max_deg), pitch_rad))
        pitch_deg = math.degrees(pitch_rad)

        eff_area = (cfg.frontal_area_m2 * math.cos(abs(pitch_rad)) +
                    cfg.top_area_m2     * math.sin(abs(pitch_rad)))

        # 水平动力学（注意 ax_cmd/ay_cmd 已包含故障扰动）
        v_rel_x = vx - self._wind_x
        v_rel_y = vy - self._wind_y
        drag_x  = drag_force(v_rel_x, cfg.Cd_lateral, eff_area, rho)
        drag_y  = drag_force(v_rel_y, cfg.Cd_lateral, eff_area, rho)
        new_vx  = vx + (ax_cmd - drag_x / mass) * dt
        new_vy  = vy + (ay_cmd - drag_y / mass) * dt
        new_x   = x  + new_vx * dt
        new_y   = y  + new_vy * dt

        # 电池模型
        bat = self._battery.step(power, dt, v_horiz)

        return SimFrame(
            t=t,
            x_m=new_x, y_m=new_y, altitude_m=new_z,
            vx_ms=new_vx, vy_ms=new_vy, velocity_ms=new_vz,
            pitch_deg=pitch_deg,
            rpm=rpm_cmd, rpm_actual=rpm,
            thrust_N=thrust, power_W=power, accel_ms2=az,
            air_density=rho,
            ground_effect=ge_factor,
            soc=bat.soc,
            battery_temp_C=bat.temp_C,
            range_est_m=bat.range_est_m,
            energy_used_Wh=bat.energy_used_Wh,
            waypoint_idx=self._ctrl.waypoint_idx,
            phase=phase,
        )
    def run(self,
            callback: Callable[[SimFrame], None] | None = None,
            realtime: bool = False) -> list[SimFrame]:
        self._recorder.start()
        frames: list[SimFrame] = []
        x, y, z    = 0.0, 0.0, 0.0
        vx, vy, vz = 0.0, 0.0, 0.0
        t          = 0.0
        steps      = int(self._cfg.duration_s / self._cfg.dt)

        for _ in range(steps):
            frame = self._step(t, x, y, z, vx, vy, vz)
            frames.append(frame)

            if callback:
                callback(frame)

            self._recorder.record(FlightRecord(
                t=frame.t,
                altitude_m=frame.altitude_m,
                velocity_ms=frame.velocity_ms,
                rpm=frame.rpm,
                thrust_N=frame.thrust_N,
                power_W=frame.power_W,
            ))

            if realtime:
                time.sleep(self._cfg.dt)

            x, y, z    = frame.x_m, frame.y_m, frame.altitude_m
            vx, vy, vz = frame.vx_ms, frame.vy_ms, frame.velocity_ms
            t          += self._cfg.dt

            if frame.soc <= 0.02:
                print(f"[Battery] Empty at t={t:.1f}s, simulation terminated.")
                break

            if frame.phase == "landed" and t > 10.0:
                print(
                    f"[Mission] Complete  t={t:.1f}s  "
                    f"energy={frame.energy_used_Wh:.1f}Wh  "
                    f"remaining={frame.soc*100:.1f}%"
                )
                if callback:
                    callback(frame)
                break

        saved = self._recorder.stop()
        if saved:
            print(f"[DataRecorder] Saved → {saved}")

        self._twin._bus.publish(self.EVT_SIM_DONE, frames=frames)
        return frames
