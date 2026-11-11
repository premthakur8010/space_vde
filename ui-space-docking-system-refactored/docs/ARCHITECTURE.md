# VDE Docking Simulator — UI Architecture & Codebase Guide

## 1. Overview

`vde_sim` is a standalone **120 Hz C++20 / OpenGL 4.5 visualization client** for spacecraft rendezvous, docking, and hardware-in-the-loop Stewart platform (VDE) simulation.

It operates as a lightweight UDP client paired with an external **Dynamics Engine** server process:
- **Outbound Configuration (`UDP port 5001` by default):** Sends initial spacecraft mass/inertia, orbit parameters, NGC mode, disturbance flags, and coordinate-frame mapping (`vde::net::RunConfigPacket`).
- **Inbound Telemetry Stream (`UDP port 5002` by default):** Continuously drains non-blocking binary physics state packets (`vde::net::StatePacket`), keeping only the highest sequence number on every `8.333 ms` (120 Hz) UI frame so rendering and physics remain completely decoupled without blocking either process.

The UI provides two primary docked workspaces (**VDE** hardware testbed view and **Spacecraft** orbital view), real-time scrolling time-series plots, a 12-chart **Analysis** window, multi-monitor pop-out windows, and offline replay controls.

---

## 2. Project Structure & File-by-File Breakdown

The entire application is organized into **9 C++ files** (`4` headers in `include/` and `5` implementation files in `src/`), plus `CMakeLists.txt` and vendored graphics/UI libraries under `third_party/`:

```text
ui-space-docking/
├── CMakeLists.txt          # CMake build script (C++20, OpenGL 4.5, GLFW, GLM, GLAD, ImGui, ImPlot)
├── docs/
│   └── ARCHITECTURE.md     # System architecture, wire protocol, and file-by-file documentation
├── include/
│   ├── Client.hpp          # Simulation types, unit conversions, UDP packets, VisualizationClient, AppState
│   ├── Renderer.hpp        # OpenGL 4.5 Shader, 4x MSAA Framebuffer, Mesh, OrbitCamera, Painter, 3D Scenes
│   ├── UiCore.hpp          # Theme, read-only widgets, DockLayout, Toolbar, SceneView, GraphWidget/Grid
│   └── UiWindows.hpp       # Top-level MainUi Pimpl facade
├── src/
│   ├── main.cpp            # Entry point, CLI flag parser, GLFW/OpenGL/ImGui init, 120 Hz frame loop
│   ├── Client.cpp          # Non-blocking UDP sockets, packet unpacking, ParameterRegistry, AppState logic
│   ├── Renderer.cpp        # GLSL shaders, DSA framebuffer/meshes, camera math, VdeScene & SpacecraftScene
│   ├── UiCore.cpp          # Light theme, custom widgets, DockBuilder layout, 3D viewport & ImPlot charts
│   └── UiWindows.cpp       # Telemetry panels, XML/Replay modal dialogs, VDE/Spacecraft/Analysis windows
└── third_party/
    ├── glad/               # OpenGL 4.5 Core Profile function loader
    ├── imgui/              # Dear ImGui (docking + multi-viewport branches) and Roboto/Cousine fonts
    └── implot/             # ImPlot real-time plotting library
```

### Detailed File Descriptions

#### `CMakeLists.txt`
Configures the C++20 build, fetches `glfw` (3.4) and `glm` (1.0.1) if not already installed on the machine, compiles `glad` and `imgui` + `implot` as static libraries, links platform socket/timer libraries (`ws2_32`, `winmm`, `comdlg32` on Windows), builds the `vde_sim` executable, and copies the `Roboto-Medium.ttf` and `Cousine-Regular.ttf` fonts next to the binary.

#### `include/Client.hpp` & `src/Client.cpp`
Handles all simulation data structures, networking, telemetry channel lookup, and application state:
- **Math & Units (`Vec3`, `Rpy`, `Mat3`, `Quaternion`, `vde::units`):** Stores all internal physics values in strict SI units (`m`, `kg`, `s`, `rad`, `N`, `N·m`) with scalar-last quaternions `(q1, q2, q3, q4)` and converts to human-readable display units (`mm`, `km`, `km/h`, `deg`) at the UI/plot boundary.
- **Snapshots & Configuration (`VdeState`, `SpacecraftState`, `NgcState`, `SimSnapshot`, `RunConfig`):** Structured representations of VDE Stewart platform pose/loads, Chaser and Target mass/orbit telemetry, NGC forces/torques, and initial run settings.
- **UDP Wire Protocol (`vde::net::RunConfigPacket`, `vde::net::StatePacket`):** Fixed-size plain-old-data binary datagrams validated by magic header `0x56444531` (`"VDE1"`) and protocol version `1`.
- **`VisualizationClient`:** Opens two non-blocking IPv4 UDP sockets (`Winsock2` on Windows, POSIX sockets on Linux) with a 256 KB receive buffer. Transmits `RunConfigPacket` datagrams to `<host>:5001` and drains all queued `StatePacket` datagrams on `0.0.0.0:5002`, keeping the newest `sequence` number.
- **`ParameterRegistry`:** Registers every plottable scalar telemetry channel (VDE kinematics/loads, Chaser/Target mass/rates/orbit, NGC forces/torques) and maps channel names to unit labels and snapshot extractor lambdas.
- **`Recorder` & `ReplayPlayer`:** Manages viewport video-recording state and offline trajectory playback (play, pause, seek, `0.5x` / `1x` / `2x` speed).
- **`AppState`:** Central coordinator updated once per 120 Hz frame. Processes queued UI `Command` variants (`StartRunCmd`, replay commands, recording commands), polls `VisualizationClient`, and routes either live UDP telemetry or replay snapshots to each workspace.

#### `include/Renderer.hpp` & `src/Renderer.cpp`
Implements the hardware-accelerated OpenGL 4.5 3D visualization engine:
- **`Shader`:** Compiles and links a GLSL 450 core shader pair implementing ambient + key + fill directional lighting with automatic two-sided surface normal orientation.
- **`Framebuffer`:** Manages an off-screen 4x MSAA color + depth/stencil renderbuffer pair using OpenGL 4.5 Direct State Access (`glCreateFramebuffers`, `glCreateTextures`, `glBlitNamedFramebuffer`) and resolves into a 2D `GL_RGBA8` texture displayed inside Dear ImGui windows.
- **`Mesh`:** Uploads interleaved `(position, normal)` triangle arrays to a DSA VAO/VBO and procedurally generates unit boxes, cylinders, cones, and standard/high-resolution spheres.
- **`OrbitCamera`:** Z-up spherical orbit camera supporting cursor-driven yaw/pitch rotation, screen-plane panning, and exponential scroll-wheel zoom.
- **`Painter` & `drawAxisTriad`:** Immediate-mode 3D primitive renderer that draws transformed boxes, segment-aligned cylinders/cones, spheres, and RGB XYZ coordinate triads (`Red = X`, `Green = Y`, `Blue = Z`), while recording 3D world-space label anchors for 2D overlay projection.
- **`VdeScene`:** Renders the physical laboratory testbed: floor plate, cylindrical base (`B_sp`), 6 telescoping Stewart platform actuators with ball joints, moving platform (`P_sp`) driven by live `VdeState` pose, 4-post overhead gantry (`O_sp`), and Chaser/Target petal docking mechanisms.
- **`SpacecraftScene`:** Renders the orbital scene: high-resolution curved Earth horizon sphere, Chaser and Target spacecraft buses, solar array booms/wings, docking adapters, and body/docking coordinate frames (`B_C`, `D_C`, `D_T`, `B_T`).

#### `include/UiCore.hpp` & `src/UiCore.cpp`
Provides the visual theme, reusable custom widgets, layout management, 3D viewport integration, and plotting widgets:
- **Theme & Fonts (`setupTheme`, `fonts`):** Loads `Roboto-Medium` and `Cousine-Regular`, applies a clean light grey/blue engineering palette, and scales widget sizes and fonts to the monitor DPI.
- **Read-Only Widgets & Buttons (`readOnlyText`, `valueRow`, `toolbarButton`, `primaryButton`, `statusTag`):** Renders bordered right-aligned numeric telemetry rows (`Vec3`, `Rpy`, matrices, scalars) and status pills.
- **Pop-Out Helpers (`poppedOutWindow`, `poppedOutPlaceholder`):** Allows any control panel, 3D viewport, or individual graph to detach into a standalone OS window (with `ImGuiViewportFlags_NoAutoMerge`) and dock back with one click.
- **`DockLayout`:** Uses Dear ImGui's `DockBuilder` API to construct an outer workspace tab bar (`VDE` and `Spacecraft`) and split each workspace into a left control panel, a top-right 3D viewport, and a bottom-right graph grid. Supports instant layout restoration via `View -> Reset layout`.
- **`drawToolbar`:** Draws workspace action buttons (`RECORD VIDEO`, `EXPORT DATA`, `OPEN ANALYSIS`, `REPLAY`, `INITIAL CONFIG (XML)`) alongside the `T+ MM:SS.mmm · 120 Hz` clock and `Status: LIVE` / `Status: WAITING FOR ENGINE` indicator.
- **`SceneView`:** Bridges `IScene`, `Painter`, `Framebuffer`, and `OrbitCamera` into an interactive Dear ImGui widget. Projects queued 3D `Label` positions through `proj * view` to draw crisp 2D frame badges and horizontal callout leader lines on top of the 3D image.
- **`GraphWidget` & `GraphGrid`:** Implements 8,192-sample circular buffers plotted over a sliding 30-second time window using ImPlot, complete with live substring filtering across all registered telemetry channels and individual chart pop-out windows.

#### `include/UiWindows.hpp` & `src/UiWindows.cpp`
Assembles the building blocks from `UiCore` into the application's concrete panels, dialogs, and windows:
- **`drawVdeValuesPanel`:** Displays `B_sp` to `P_sp` position (`mm`), linear rate (`mm/s`), orientation (`°`), angular rate (`°/s`), moving and overhead platform forces/torques (`N`, `N·m`), and workspace travel limits.
- **`drawSpacecraftValuesPanel`:** Switches between `Chaser` and `Target` and organizes telemetry into three tabs:
  - **Properties:** Docking hardware frame (`D_C` / `D_T`), mass (`kg`), centre of gravity (`m`), 3x3 inertia tensor (`kg·m²`), and body rates.
  - **Orbit:** Earth-Centred Inertial (`ECI`) position (`km`), velocity (`km/h`), attitude (`°`), and angular rates (`°/s`).
  - **NGC:** Active guidance mode, failure case, Chaser/Target commanded forces and torques, and environmental disturbance status.
- **`ConfigXmlDialog`:** Modal popup showing the formatted XML summary of the active `RunConfig` and UDP ports, with a **RE-SEND RUNCONFIG (UDP)** button to re-trigger run initialization on the Dynamics Engine.
- **`ReplayDialog` & `drawReplayBar`:** Modal file picker (using native Win32 `GetOpenFileNameA` on Windows or text input on Linux) to start CSV replay on VDE, Spacecraft, or Both views, plus the bottom timeline transport bar.
- **`VdeWindow`, `SpacecraftWindow`, `AnalysisWindow`:** Self-contained controllers for the two 6-graph workspaces and the floating 12-graph Analysis window.
- **`MainUi`:** Top-level Pimpl class invoked by `main.cpp`. Renders the main menu bar (`Simulation -> Initialize Run`, `Simulation -> Flipped Frame Mapping`, `View -> Reset layout`) along with live FPS, frame time, UDP endpoint, sequence number (`seq #`), and received packet count (`pkts`).

#### `src/main.cpp`
Bootstraps the OS window, graphics context, and 120 Hz main loop:
1. Parses optional CLI flags (`--host`, `--config-port`, `--state-port`).
2. Initializes GLFW and creates a maximized OpenGL 4.5 Core Profile window with `glfwSwapInterval(0)` so monitor 60 Hz VSync does not cap the render rate.
3. Initializes GLAD, Dear ImGui (docking + multi-viewport), ImPlot, and the DPI-scaled theme.
4. Instantiates `vde::AppState`, connects `VisualizationClient` over UDP, and constructs `vde::ui::MainUi`.
5. Executes the main loop paced to `8,333,333 ns` (`120 Hz`) using `std::chrono::steady_clock`, Windows 1 ms timer resolution (`timeBeginPeriod(1)`), `sleep_until` for coarse waits, and `std::this_thread::yield()` for final sub-millisecond precision.

---

## 3. Client–Server UDP Wire Protocol

Both processes communicate on localhost (`127.0.0.1`) or across a LAN using little-endian C++ structs defined in `include/Client.hpp` under `namespace vde::net`.

### Connection Lifecycle
1. On startup, `vde_sim` binds non-blocking UDP port `5002` (`0.0.0.0:5002`) and sends an initial `RunConfigPacket` to `<host>:5001`.
2. Until the first valid `StatePacket` arrives, the toolbar shows `Status: WAITING FOR ENGINE` and the menu bar shows `seq #0 · pkts 0`.
3. If the Dynamics Engine is started *after* `vde_sim`, click **`Simulation -> Initialize Run (Send RunConfig)`** (or **`INITIAL CONFIG (XML) -> RE-SEND RUNCONFIG (UDP)`**) to transmit `RunConfigPacket` again.
4. As soon as the Dynamics Engine streams `StatePacket` datagrams to port `5002`, `pkts` and `seq #` increment in the top-right menu bar, the toolbar switches to `Status: LIVE`, and all 3D views, panels, and plots update at 120 Hz.

### Packet Validation Requirements
For an incoming `StatePacket` to be accepted by `VisualizationClient::getLatestState()`:
- Datagram byte length must equal `sizeof(vde::net::StatePacket)`.
- `magic` must equal `0x56444531u` (`"VDE1"`).
- `version` must equal `1u`.
- `sequence` must be monotonically non-decreasing (`pkt.sequence >= lastSequence`).
- All physical quantities inside `StatePacket` (`positionM`, `linearRateMS`, `orientationRad`, `eciPositionM`, `eciVelocityMS`, forces, torques, etc.) must be populated in **SI units** (`m`, `m/s`, `rad`, `rad/s`, `kg`, `kg·m²`, `N`, `N·m`), with `Quaternion` ordered scalar-first `(w, x, y, z)` matching `space_vde`.

---

## 4. Coordinate Frames & Unit Conventions

### Coordinate Frames Visualized
| Frame | Workspace | Meaning |
| :--- | :--- | :--- |
| `B_sp` | VDE View | Fixed base frame of the Stewart platform hexapod |
| `P_sp` | VDE View | Moving upper platform frame carrying the Chaser docking mechanism |
| `O_sp` | VDE View | Fixed overhead gantry platform frame carrying the Target docking mechanism |
| `B_C` / `B_T` | Spacecraft View | Chaser and Target spacecraft body frames |
| `D_C` / `D_T` | Spacecraft View | Chaser and Target docking port frames |
| `ECI` | Spacecraft Panel / Plots | Earth-Centred Inertial orbital reference frame |

### Unit Conventions
| Quantity | Internal & UDP Wire Unit (SI) | UI Panel & Plot Display Unit |
| :--- | :--- | :--- |
| VDE translation / rate | `m`, `m/s` | `mm`, `mm/s` |
| Spacecraft orbital position / velocity | `m`, `m/s` | `km`, `km/h` |
| Spacecraft body linear rate & CG | `m/s`, `m` | `m/s`, `m` |
| Angles & angular rates | `rad`, `rad/s` | `deg` (`°`), `deg/s` (`°/s`) |
| Mass & inertia | `kg`, `kg·m²` | `kg`, `kg·m²` |
| Forces & torques | `N`, `N·m` | `N`, `N·m` |

---

## 5. Building & Running

### Build with CMake
```powershell
cmake -B build -G Ninja
cmake --build build
```

### Run the Visualization Client
```powershell
# Default endpoints (sends RunConfig to 127.0.0.1:5001, listens for StatePacket on :5002)
.\build\vde_sim.exe

# Custom Dynamics Engine IP or UDP ports
.\build\vde_sim.exe --host 192.168.1.50 --config-port 5001 --state-port 5002
```

