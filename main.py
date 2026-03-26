"""
eVTOL Digital Twin — entry point
"""

import sys, time
from pathlib import Path
import numpy as np
import yaml

from core.vehicle import load_vehicle
from core.event_bus import EventBus
from core.digital_twin import DigitalTwin


def main():
    print("=" * 54)
    print("  eVTOL Digital Twin — Step 4: Twin State Machine")
    print("=" * 54)

    vehicle = load_vehicle(Path("configs/quad_evtol.yaml"))
    bus     = EventBus()
    twin    = DigitalTwin(vehicle, bus)

    # ── 注册事件监听器 ──────────────────────────────────
    log = []
    bus.subscribe("state_updated",
                  lambda state: log.append(f"  [UPDATE] RPM={state.rpm:.0f}  "
                                           f"T={state.performance.total_thrust_N:.1f} N  "
                                           f"P={state.performance.total_power_W/1000:.2f} kW"))

    bus.subscribe("hover_achieved",
                  lambda state: print(f"  *** HOVER ACHIEVED @ {state.rpm:.0f} RPM ***"))

    bus.subscribe("below_hover",
                  lambda state: print(f"  *** BELOW HOVER   @ {state.rpm:.0f} RPM ***"))

    # ── 模拟转速爬升序列 ────────────────────────────────
    print(f"\n  Simulating RPM ramp-up...")
    print(f"  Hover RPM target: {twin.hover_rpm:.0f} RPM\n")

    for rpm in [0, 1000, 2000, 3000, 4000, 4508, 5000, 6000, 4000, 2000]:
        twin.set_rpm(rpm)

    print("\n  Event log:")
    for entry in log:
        print(entry)

    print("\n" + "=" * 54)
    print("  Final state:")
    print(f"    RPM      : {twin.state.rpm:.0f}")
    print(f"    Thrust   : {twin.state.performance.total_thrust_N:.1f} N")
    print(f"    Power    : {twin.state.performance.total_power_W/1000:.2f} kW")
    print(f"    Hovering : {twin.state.performance.can_hover}")
    print("=" * 54)
    print("  Step 4 complete.")
    print("=" * 54)


if __name__ == "__main__":
    main()