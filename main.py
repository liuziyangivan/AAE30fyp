"""
eVTOL Digital Twin — entry point
"""

import sys
from pathlib import Path
import numpy as np
import yaml

from core.vehicle import load_vehicle


def main():
    print("=" * 50)
    print("  eVTOL Digital Twin — System Check")
    print("=" * 50)
    print(f"  Python  : {sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}")
    print(f"  NumPy   : {np.__version__}")
    print(f"  PyYAML  : {yaml.__version__}")
    print("=" * 50)

    # 加载飞行器配置
    vehicle = load_vehicle(Path("configs/quad_evtol.yaml"))
    print(f"  Vehicle : {vehicle.name}")
    print(f"  Rotors  : {vehicle.rotors.count} × R={vehicle.rotors.radius_m} m")
    print(f"  Mass    : {vehicle.total_mass_kg:.1f} kg")
    print(f"  Weight  : {vehicle.total_weight_N:.1f} N")
    print("=" * 50)
    print("  Step 2 complete.")
    print("=" * 50)


if __name__ == "__main__":
    main()