# Space VDE: UI Architecture & Network Telemetry Guide

This document provides a simple, clear explanation of:
1. How the **Dynamics Engine** and **UI Client** communicate over the network.
2. Why the Dynamics Engine broadcasts at **8 ms (125 Hz)**.
3. What each file in the **UI codebase** does.
4. An itemized breakdown of **all telemetry data** sent from the Dynamics Engine, and why it is needed.

---

## 1. High-Level Architecture (Approach A: Decoupled UDP)

The system consists of two independent, decoupled programs communicating over local network sockets:

```
┌────────────────────────────────────────┐                       ┌────────────────────────────────────────┐
│     Dynamics Engine                    │                       │     UI Client (vde_sim.exe)            │
│   (physics_engine.exe)                 │                       │                                        │
│                                        │                       │                                        │
│   • Headless C++ physics solver        │                       │   • Interactive OpenGL 3.3/4.5 + ImGui │
│   • 1000 Hz cycle (every 1 ms)         │                       │   • 120 FPS render loop (~8.33 ms)     │
│   • Runs numerical RK4 integration     │                       │   • Real-time 3D scenes & ImPlot charts│
│                                        │                       │                                        │
│   Listens on: UDP Port 5001 (Config)   │ ◄─── RunConfigPacket ───  Transmits to Port 5001               │
│   Streams to: UDP Port 5002 (State)    │ ──── StatePacket ─────►   Listens on: Port 5002                │
│               (at 125 Hz / every 8 ms) │      (~1 KB payload)   │                                        │
└────────────────────────────────────────┘                       └────────────────────────────────────────┘
```

### Why UDP instead of TCP?
- **Zero Lag & No Head-of-Line Blocking:** Physics data is high-frequency streaming telemetry. If a single packet drops over WiFi or Ethernet, TCP would pause the entire application to re-transmit old data. With UDP, the visualizer simply renders the next fresh packet arriving 8 ms later.
- **Non-Blocking Execution:** Both programs use non-blocking sockets (`O_NONBLOCK` / `ioctlsocket(FIONBIO)`). Neither the 1000 Hz physics math nor the 120 FPS visualizer ever freezes waiting on network I/O.

---

## 2. Why Does the Dynamics Engine Send at 8 ms?

The physics engine calculates motion every **1 ms (1000 Hz)**, but it sends UDP packets every **8 ms (125 Hz)** (`stepCount % 8 == 0`).

Here is the exact mathematical and engineering reason:

1. **Target Display Rate (120 FPS):**
   - The UI visualizer renders a new frame every **8.33 ms** ($\frac{1}{120}\text{ s} \approx 8.333\text{ ms}$).
2. **Matching Frame Rates (125 Hz vs 120 Hz):**
   - By sending every **8 ms (125 Hz)**, the engine produces data slightly faster than the screen consumes it ($125\text{ Hz} > 120\text{ Hz}$).
   - Every single screen frame always has a fresh packet waiting in the buffer.
3. **Why NOT every 1 ms (1000 Hz)?**
   - Sending 1000 packets/sec over UDP would mean 880 packets are thrown away unrendered every second. This wastes CPU interrupts and network resources for zero visual improvement.
4. **Why NOT every 10 ms (100 Hz)?**
   - If the engine sent at 100 Hz, it would be slower than the 120 Hz screen. Some screen frames would receive no new data and duplicate the previous frame, creating visible **micro-stuttering** (judder).
5. **Why 8 ms is the Goldilocks Number:**
   - $1000 / 8 = 125\text{ Hz}$. 8 is a whole-integer divisor of the 1 ms physics clock, ensuring silky-smooth 120 FPS motion with zero stutter.

---

## 3. UI Codebase: What Each File Does

All UI source code is located in [`ui-space-docking-system-refactored/`](file:///c:/Users/Prem%20M%20Thakur/Desktop/space_vde/space_vde/ui-space-docking-system-refactored):

```
ui-space-docking-system-refactored/
├── include/
│   ├── Client.hpp          ← Layer 1: Network socket, math types, telemetry state
│   ├── Renderer.hpp        ← Layer 2: 3D OpenGL shaders, meshes, camera, scenes
│   ├── UiCore.hpp          ← Layer 3: UI styling, theme, fonts, plots (ImPlot)
│   └── UiWindows.hpp       ← Layer 4: Screen workspaces, layouts, panels
├── src/
│   ├── main.cpp            ← Application entry point & 120 FPS loop
│   ├── Client.cpp          ← Socket implementation & telemetry decoding
│   ├── Renderer.cpp        ← 3D rendering (Stewart platform & spacecraft)
│   ├── UiCore.cpp          ← Widget rendering & plot curves
│   └── UiWindows.cpp       ← Panel layout code & toolbar logic
└── third_party/            ← Dependencies (GLFW, GLAD, Dear ImGui, ImPlot, GLM)
```

### Detailed Breakdown

#### 1. `src/main.cpp` — The Main Orchestrator
- **Role:** Application entry point.
- **What it does:**
  - Creates the OS window using **GLFW**.
  - Initializes **OpenGL 3.3/4.5** via **GLAD**.
  - Configures **Dear ImGui** (docking multi-window UI) and **ImPlot** (charting).
  - Connects the UDP client (`VisualizationClient`) to the Dynamics Engine.
  - Drives the **120 FPS render loop** (draining incoming UDP packets and rendering UI frames every 8.33 ms).

#### 2. `include/Client.hpp` & `src/Client.cpp` — The Network & Data Hub
- **Role:** Communication layer and data storage.
- **What it does:**
  - **`VisualizationClient`:** Opens the non-blocking UDP sockets on ports `5001` (TX) and `5002` (RX).
  - **The "Drain Queue" Logic:** Reads all pending UDP packets off the socket and keeps only the newest sequence number, ensuring zero lag.
  - **`SimSnapshot` & `AppState`:** The central memory state holding current telemetry, 60-second rolling history buffers, and recording/replay status.
  - **`ParameterRegistry`:** A catalog of 40+ plottable channels (converts SI units like meters and radians into display units like mm and degrees).

#### 3. `include/Renderer.hpp` & `src/Renderer.cpp` — The 3D Graphics Engine
- **Role:** 3D rendering subsystem.
- **What it does:**
  - Renders 3D scenes into an off-screen multisampled buffer (4x MSAA) and displays them inside ImGui panels.
  - **`VdeScene`:** Visualizes the ground test facility: the 6-legged Stewart platform (hexapod), overhead gantry, base plate, docking rings, and coordinate frame axes (Red=X, Green=Y, Blue=Z).
  - **`SpacecraftScene`:** Visualizes the orbital rendezvous: Chaser and Target satellites, solar arrays, thrusters, docking ports, and the curved Earth horizon.
  - **`OrbitCamera`:** Handles interactive mouse navigation (left-click drag to rotate, right-click drag to pan, scroll wheel to zoom).

#### 4. `include/UiCore.hpp` & `src/UiCore.cpp` — UI Primitives & Plotting
- **Role:** Visual components and real-time graphs.
- **What it does:**
  - Applies a modern light UI theme and loads proportional and monospace fonts.
  - **`SceneView`:** Connects the 3D rendered texture to mouse drag events.
  - **`GraphWidget` & `GraphGrid`:** Uses **ImPlot** to draw scrolling real-time strip-charts (e.g., $X, Y, Z$ positions, closing velocities, contact forces vs. time).

#### 5. `include/UiWindows.hpp` & `src/UiWindows.cpp` — Workspace Layouts
- **Role:** Top-level UI layouts and windows.
- **What it does:**
  - Organizes the dockable interface into 3 dedicated workspaces:
    1. **VDE View:** Focuses on the Stewart platform hardware, stroke limits, and load-cell contact forces.
    2. **Spacecraft View:** Focuses on orbital docking, closing velocity, relative attitude, and orbit parameters.
    3. **Analysis View:** A 12-plot engineering grid displaying complete 6-DOF trajectory curves simultaneously.
  - Manages the top toolbar (Simulation Clock, Play/Pause, Record, Reset, Run Configuration dialog).

---

## 4. What Does the Dynamics Engine Send? (`StatePacket`)

Every 8 ms, the Dynamics Engine transmits a binary datagram (`StatePacket`) to UDP port `5002`.

Here is the exact data layout:

| Section | Field | Data Type | Physical Meaning |
| :--- | :--- | :--- | :--- |
| **Header** | `magic` | `uint32` | Protocol identifier (`0x56444531` = ASCII `"VDE1"`). |
| | `version` | `uint32` | Protocol version number (`1`). |
| | `sequence` | `uint64` | Incremental packet counter (`0, 1, 2, ...`). |
| | `timeS` | `double` | Total elapsed simulation time in seconds. |
| **Relative State** | `relative.positionM` | `Vec3` (3 doubles) | Relative distance vector $(X, Y, Z)$ between docking ports in meters. |
| | `relative.velocityMS`| `Vec3` (3 doubles) | Relative closing/separation velocity $(V_x, V_y, V_z)$ in m/s. |
| | `relative.orientation`| `Quaternion` (4 doubles)| Relative orientation quaternion: $q(w, x, y, z)$. |
| | `relative.orientationRate`| `Quaternion` (4 doubles)| Angular rate quaternion: $\dot{q}(w, x, y, z)$. |
| **VDE Hardware** | `vde.positionM` | `Vec3` (3 doubles) | Stewart platform carriage position $(X, Y, Z)$ in meters. |
| | `vde.linearRateMS` | `Vec3` (3 doubles) | Stewart platform linear velocity $(V_x, V_y, V_z)$ in m/s. |
| | `vde.orientationRad` | `Rpy` (3 doubles) | Stewart platform orientation in Roll, Pitch, Yaw (radians). |
| | `vde.orientationRateRadS`| `Rpy` (3 doubles) | Platform angular velocity in Roll, Pitch, Yaw rates (rad/s). |
| | `vde.moving` | `ForceTorque` (6 doubles)| 6-DOF load-cell forces ($N$) and torques ($N\cdot m$) on moving platform. |
| | `vde.overhead` | `ForceTorque` (6 doubles)| 6-DOF load-cell forces ($N$) and torques ($N\cdot m$) on overhead gantry. |
| **Chaser Spacecraft**| `chaser.massKg` | `double` | Chaser satellite mass ($2150.0\text{ kg}$). |
| | `chaser.cgM` | `Vec3` (3 doubles) | Center of gravity offset $(X, Y, Z)$ in meters. |
| | `chaser.inertiaKgM2` | `Mat3` (9 doubles) | $3\times 3$ mass moment of inertia tensor ($kg\cdot m^2$). |
| | `chaser.eciPositionM`| `Vec3` (3 doubles) | Earth-Centered Inertial orbital position $(X, Y, Z)$ in meters. |
| | `chaser.eciVelocityMS`| `Vec3` (3 doubles)| Earth-Centered Inertial orbital velocity in m/s. |
| | `chaser.orientationRateRadS`| `Rpy` (3 doubles)| Body-axis angular rotation rates ($\omega_x, \omega_y, \omega_z$) in rad/s. |
| **Target Spacecraft**| `target.*` | (Same as Chaser) | Identical orbital & inertial telemetry for the Target satellite ($7800.0\text{ kg}$). |
| **Thruster / NGC** | `ngcChaser` | `ForceTorque` (6 doubles)| Commanded Guidance & Control thruster forces ($N$) and torques ($N\cdot m$). |
| | `ngcTarget` | `ForceTorque` (6 doubles)| Commanded Target thruster forces ($N$) and torques ($N\cdot m$). |
| **Status Flags** | `disturbancesOn`| `uint8` | Flag indicating whether orbital disturbances are active ($1$ or $0$). |
| | `ngcMode` | `char[64]` | Mission phase text (e.g. `"FINAL APPROACH (closing)"`). |
| | `ngcFailureCase`| `char[64]` | Diagnostics string (e.g. `"None (nominal)"`). |

---

## 5. Is All This Info Needed or Not?

### Distinction: Basic 3D Game vs. ISRO Engineering Ground Station
- **If this were just a basic 3D game:** You would only need `relative.positionM` and `relative.orientation` (~56 bytes) to position the 3D models.
- **For the ISRO SOW & PES University Ground Support Facility:** **Every single field in `StatePacket` is required and actively utilized by the UI.**

Here is the exact mapping of where the data goes:

1. **The 3D Scene Viewports:**
   - Uses `relative.positionM`, `relative.orientation`, `vde.positionM`, and `vde.orientationRad` to dynamically move the 3D meshes, coordinate axes, and camera focal points.
2. **The VDE Values Side Panel:**
   - Displays real-time Stewart platform displacement and verifies movement against **stroke travel limits** (e.g. $X, Y \in [-100, 100]\text{ mm}$, $Z \in [0, 550]\text{ mm}$, angles $\pm 5^\circ$).
   - Displays `vde.moving` and `vde.overhead` load-cell readouts to monitor physical contact forces during mating.
3. **The Spacecraft Values Side Panel:**
   - Displays real-time mass, inertia tensors, orbital altitude, and Earth-relative velocity.
4. **The Real-Time Strip Charts & 12-Plot Analysis Window:**
   - The user can select any of the 40+ scalar telemetry parameters from a dropdown to plot them live or review them during trajectory replays.
5. **The NGC Status Bar:**
   - Displays the current autonomous docking phase (e.g. Closing vs. Capture) and flags system faults.

### Bandwidth and Performance Impact
- The complete `StatePacket` is **~1,050 bytes (roughly 1 KB)**.
- Transmitted at 125 Hz:
  $$\text{Bandwidth} = 1.05\text{ KB} \times 125\text{ packets/sec} \approx 131\text{ KB/second} \approx 1.05\text{ Mbps}$$
- On modern localhost or Ethernet (100 Mbps or 1 Gbps), 131 KB/s consumes **less than 0.1%** of network capacity. It imposes zero measurable CPU or network bottleneck.

### Active Fields vs. Placeholder Fields
In the current build:
- **Active fields:** Positions, velocities, orientations, time, sequence, masses, and status strings update dynamically every tick.
- **Placeholder fields:** Certain fields (such as `chaser.cgM = {0,0,0}` or active thruster forces `ngcChaser = {0,0,0}`) are currently transmitted as zeros.
- **Why keep them?** They future-proof the network protocol. When hardware load cells, GSP leg extensions, and active thruster firing logic are connected to the math solver, the visualizer will immediately display them without requiring changes to the network protocol.

---

## 6. What the UI Client Sends Back (`RunConfigPacket`)

When initializing or resetting a docking scenario, the UI Client transmits a `RunConfigPacket` to the Dynamics Engine on UDP port `5001`.

It contains:
- **`chaserMassKg` & `targetMassKg`:** Masses of both spacecraft.
- **`chaserInertia` & `targetInertia`:** $3\times 3$ mass moments of inertia.
- **`orbitRadiusM` & `inclinationRad`:** Orbital altitude and inclination.
- **`alongTrackSepM`:** Initial starting distance (e.g., $-12.0\text{ m}$).
- **`frameMapping`:** `0` for Standard, `1` for Flipped frame conventions.
- **`disturbancesOn`:** Toggles gravity gradient, solar radiation, and drag perturbations.
- **`ngcMode`:** Mission operational profile.

