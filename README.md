# **eVTOL Digital Twin**  


## Final Year Project · Real‑Time Simulation & Visualization Platform


[https://img.shields.io/badge/Python-3.11+-blue](URL)  

[https://img.shields.io/badge/GUI-PyQt6-green](URL)  

[https://img.shields.io/badge/License-MIT-yellow](URL)


A comprehensive digital twin framework for a conceptual electric Vertical Take‑Off and Landing (eVTOL) aircraft. The system integrates real‑time aerodynamic simulation, parametric design exploration, 3D visualization, data recording and Fault introduction simulation — all built in Python.


## Table of Contents  
- Features
  
- Project Structure
  
- Installation
  
- Quick Start
  
- Usage Guide
  
- Testing
  
- Contributor



## Features  
| Module | Description |
|-------|-------|
| Aerodynamic Core | Blade Element Theory (BET) engine computing thrust, torque, power, and Figure of Merit (FM). |
| Flight Simulator | 6‑DOF physics with waypoint navigation, battery discharge modeling, ground effect & motor lag. |  
| Live Dashboard | Real‑time altitude, RPM, thrust, power gauges with manual RPM override and auto‑flight controls. |  
| Parametric Design | Interactive modification of rotor radius, chord, blade count, mass, etc. with immediate feedback of RPM. |
| Data Recording | Automatic CSV logging of all flight parameters; replay any previous flight with speed control. |  
| 3D Visualization | OpenGL rendering of the eVTOL model, animated rotors, thrust cones, and altitude‑colored trail. |  
| Fault Injection | Simulate single‑rotor failure to evaluate emergency descent behavior (planned). |


## Project Structure  
```plaintext
evtol_digital_twin/
├── configs/
│   └── quad_evtol.yaml          # Vehicle parameters
├── core/
│   ├── vehicle.py               # YAML loader & dataclasses
│   ├── aero_engine.py           # Blade Element Theory implementation
│   ├── event_bus.py             # Publish/subscribe event system
│   ├── digital_twin.py          # Real‑time state machine
│   ├── performance_analysis.py  # RPM sweep & endurance
│   ├── design_editor.py         # Parametric design logic
│   └── data_recorder.py         # CSV logging  
│   └── fault_injector.py        # Planned fault calculation
├── simulation/
│   └── flight_sim.py            # 6‑DOF simulator with waypoints & battery
├── gui/
│   ├── main_window.py           # Main PyQt6 application
│   ├── altitude_plot.py         # Real‑time scrolling plot
│   ├── envelope_panel.py        # Performance analysis tab
│   └── widgets/
│       ├── replay_widget.py     # Flight replay interface
│       ├── design_widget.py     # Parametric design panel
│       └── view3d_widget.py     # OpenGL 3D visualization  
│       └── fault_widget.py      # Fault introduction panel
├── tests/                       # Unit tests
├── data/                        # Recorded flight CSV files
├── main.py                      # Application entry point
├── requirements.txt
└── README.md
```


## Installation  
1.Clone the repository
```bash
git clone https://github.com/liuziyangivan/AAE-30-Final-Year-Project.git
cd evtol-digital-twin
```

2.Create and activate a virtual environment (recommended)
```bash
python -m venv .venv
.venv\Scripts\activate
```

3.Install dependencies
```bash
pip install -r requirements.txt
```

## Quick start
Launch the main application:  
```bash
cd evtol-digital-twin
python main.py
```
Run all unit tests:
```bash
pytest tests/ -v
```
## Usage Guide  
Tab 1: Main control console  
![]("C:\Users\admin\Desktop\Tab1.1.png")
![]("C:\Users\admin\Desktop\Tab1.2.png")


Tab 2: Data playback  
- Recorded Flights: Every simulation automatically saves a CSV file in the data/ folder.
- Playback: Select a file from the dropdown, then click PLAY to replay the flight. Adjust playback speed (0.5× – 4×) and watch the trajectory cursor move along the curves.



