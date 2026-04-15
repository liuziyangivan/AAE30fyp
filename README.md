# **eVTOL Digital Twin**  


## Final Year Project · Real‑Time Simulation & Visualization Platform

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
git clone https://github.com/liuziyangivan/AAE30fyp.git
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

The tab serves as the primary cockpit for real‑time monitoring and manual intervention.

It is organized into four functional zones:  
| Zone | Purpose | Key Elements |  
|------|---------|--------------|
| **Observability Dashboard** | Real‑time telemetry | Altitude, vertical speed, optimal RPM, thrust, power gauges |
| **3D Visualization Canvas** | Spatial awareness | eVTOL model with animated rotors, altitude‑colored trail, reset/clear/follow camera controls |
| **Controllability Panel** | Manual overrides | RPM slider with STOP / TAKEOFF / HOVER / LAND presets |
| **Design Editor Sidebar** | Quick parameter tuning | Rotor geometry (radius, chord, blades, Cl_alpha, Cd0) and mass (fuselage, payload) with one‑click **Apply / Solve RPM** |

A typical workflow:  
1. **Adjust rotor/mass parameters** in the Design Editor and click **Apply_Solve RPM** – the optimal hover RPM updates instantly.  
2. **Use manual presets** to test takeoff, hover, or landing behaviour.  
3. **Watch the 3D view** – the aircraft model responds in real time, and the trail color shifts with altitude.  
4. **Monitor the dashboard cards** – all critical values (altitude, RPM, thrust, power) are displayed with large, high‑contrast fonts suitable for presentation screens.

> *Screenshot reference (Tab 1.1and1.2)*  


Tab 2: Data playback

The tab provides a complete flight data playback environment, allowing you to review any previously recorded simulation session.

#### Flight Record Selection

| Control | Description |
|---------|-------------|
| **File dropdown** | Lists all CSV flight logs stored in the `data/` directory (format: `flight_YYYYMMDD_HHMMSS_microseconds.csv`). |
| **Refresh button** | Rescans the `data/` folder and updates the dropdown list. |
| **Info label** | Displays the number of recorded frames and total flight duration (e.g., *"2235 frames · 111.7 s"*). |

#### Playback Controls

| Element | Function |
|---------|----------|
| **Progress slider** | Shows current playback position; you can drag to seek anywhere in the timeline. |
| **Time label** | Current playback timestamp (e.g., *"0.0 s"*). |
| **Speed selector** | Choose playback speed: `0.5×`, `1×`, `2×`, or `4×`. |
| **PLAY / STOP** | Start or pause the replay. |
| **RESET** | Jump back to the beginning of the flight. |

#### Telemetry Charts

Three synchronized plots display the full flight history:

| Chart | Y‑Axis | Color | Description |
|-------|--------|-------|-------------|
| **Altitude** | Meters (m) | Blue | Aircraft height above ground. |
| **Total Thrust** | Newtons (N) | Green | Combined thrust from all rotors. |
| **Total Power** | Kilowatts (kW) | Yellow | Electrical power consumed by the propulsion system. |

- **Live cursor**: A circular marker moves along each curve in real time during playback, indicating the exact data point corresponding to the current timestamp.
- **Zoom & pan**: Use the mouse to zoom into specific time intervals or pan across the chart (standard `pyqtgraph` interactions).

#### Typical Usage Scenario

1. Run a simulation in the **Live Control** tab – the flight data is automatically saved to `data/`.
2. Switch to the **Replay** tab and click **Refresh** to load the latest file.
3. Select the desired CSV file from the dropdown.
4. Press **PLAY** to watch the flight unfold on the charts.
5. Adjust the speed or seek to interesting events (e.g., takeoff, hover, fault injection).

> *Screenshot reference (Tab 2.2)*  


## Testing

The project includes comprehensive unit tests covering:

- Vehicle configuration loading (test_vehicle.py)

- Aerodynamic engine (BET) (test_aero.py)

- Event bus and digital twin state machine (test_twin.py)

- Flight simulator with waypoint navigation and battery model (test_sim.py)

- Performance analyzer (test_analysis.py)

- Data recorder and replay (test_recorder.py)

- Design editor (test_design_editor.py)

- Flight with PID and LQR (test_controllers.py)

Run all tests with:
```bash
pytest tests/ -v --tb=short
```

## Contributor

- LIU Ziyang, CAI Jialiang, and WANG Yetian.

- *Supervisor: ZHONG Siyang*

This project was developed as part of the Final Year Project requirement for the Bachelor of Aviation Engineering degree.


