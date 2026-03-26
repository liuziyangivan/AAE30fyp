"""
eVTOL Digital Twin — entry point
"""

import sys
import numpy as np
import yaml


def main():
    print("=" * 50)
    print("  eVTOL Digital Twin — System Check")
    print("=" * 50)
    print(f"  Python  : {sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}")
    print(f"  NumPy   : {np.__version__}")
    print(f"  PyYAML  : {yaml.__version__}")
    print("=" * 50)
    print("  All checks passed. Ready to build.")
    print("=" * 50)


if __name__ == "__main__":
    main()