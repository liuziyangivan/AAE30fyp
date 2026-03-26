"""
eVTOL Digital Twin — entry point
"""

import sys
from pathlib import Path
import numpy as np
import yaml

from core.vehicle import load_vehicle
from core.event_bus import EventBus
from core.digital_twin import DigitalTwin
from simulation.flight_sim import FlightSimulator, SimConfig


def main():
    print("=" * 56)
    print("  eVTOL Digital Twin — Step 5: Flight Simulator")
    print("=" * 56)

    vehicle = load_vehicle(Path("configs/quad_evtol.yaml"))
    bus     = EventBus()
    twin    = DigitalTwin(vehicle, bus)

    cfg = SimConfig(dt=0.05, duration_s=30.0, target_alt_m=10.0)
    sim = FlightSimulator(twin, cfg)

    print(f"  Hover RPM : {twin.hover_rpm:.0f} RPM")
    print(f"  Target Alt: {cfg.target_alt_m} m")
    print(f"  Duration  : {cfg.duration_s} s  (dt={cfg.dt} s)")
    print(f"\n  Running simulation...\n")

    frames = sim.run()

    # 打印关键帧（每 2 秒一行）
    step = int(2.0 / cfg.dt)
    print(f"  {'t(s)':>5}  {'Alt(m)':>7}  {'Vel(m/s)':>8}  "
          f"{'RPM':>6}  {'Thrust(N)':>10}  {'Power(kW)':>9}")
    print("  " + "-" * 56)
    for i, f in enumerate(frames):
        if i % step == 0:
            print(f"  {f.t:5.1f}  {f.altitude_m:7.2f}  {f.velocity_ms:8.3f}  "
                  f"  {f.rpm:6.0f}  {f.thrust_N:10.1f}  {f.power_W/1000:9.2f}")

    max_alt = max(f.altitude_m for f in frames)
    max_pwr = max(f.power_W    for f in frames)
    print("  " + "-" * 56)
    print(f"\n  Peak altitude : {max_alt:.2f} m")
    print(f"  Peak power    : {max_pwr/1000:.2f} kW")
    print(f"  Total frames  : {len(frames)}")
    print("=" * 56)
    print("  Step 5 complete.")
    print("=" * 56)


if __name__ == "__main__":
    main()