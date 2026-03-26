"""
eVTOL Digital Twin — entry point
"""

import sys
from pathlib import Path
import numpy as np
import yaml

from core.vehicle import load_vehicle
from core.aero_engine import AeroEngine


def main():
    print("=" * 52)
    print("  eVTOL Digital Twin — Step 3: Aero Engine")
    print("=" * 52)

    vehicle = load_vehicle(Path("configs/quad_evtol.yaml"))
    engine  = AeroEngine(vehicle)

    print(f"  Vehicle : {vehicle.name}")
    print(f"  Mass    : {vehicle.total_mass_kg:.1f} kg  "
          f"Weight: {vehicle.total_weight_N:.1f} N")
    print("-" * 52)

    # 测试三个转速点
    for rpm in [2000, 4000, 6000]:
        perf = engine.compute(rpm)
        print(f"\n  @ {rpm} RPM")
        print(f"    Rotor    : {perf.rotor}")
        print(f"    Total T  : {perf.total_thrust_N:.1f} N  "
              f"Total P: {perf.total_power_W/1000:.2f} kW")
        print(f"    Margin   : {perf.thrust_margin_N:+.1f} N  "
              f"{'✓ hover OK' if perf.can_hover else '✗ below hover'}")

    print("\n" + "-" * 52)
    hover = engine.compute(3000)
    print(f"  Hover RPM: {hover.hover_rpm:.0f} RPM")
    print("=" * 52)
    print("  Step 3 complete.")
    print("=" * 52)


if __name__ == "__main__":
    main()