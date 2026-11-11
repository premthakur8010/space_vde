# Architecture & System Design

## Overview
The Space Docking System is an ultra-low latency, deterministic real-time physics simulation and 3D visualization platform.
It uses a decoupled client-server architecture:
- **Server (Dynamic Engine)**: Runs rigid-body physics, microsecond-accurate tick scheduling, contact collision dynamics, and UDP state streaming.
- **Client (UI & Visualization)**: OpenGL 4.5 Core profile renderer using Direct State Access (DSA) and Dear ImGui immediate-mode telemetry dashboard.

```mermaid
graph TD
	subgraph Server [Physics Engine Process]
		Config[SimulationConfig] --> Engine[DynamicEngine]
		Engine --> Solver[ContactSolver & Integrator]
		Solver --> HexFixture[Hexagonal Load Cell Distributor]
		Engine --> NetServer[UdpSocket Server]
	end

	subgraph Network [UDP IPC Loopback / Multicast]
		NetServer -->|SnapshotMsg (Port 9870)| NetClient[UdpSocket Client]
		NetClient -.->|InputCommandMsg (Port 9871)| NetServer
	end

	subgraph Client [Visualization & UI Process]
		NetClient --> SnapshotBuffer[Snapshot Queue & Interpolator]
		SnapshotBuffer --> Renderer[GL 4.5 DSA Renderer]
		InputMgr[Centralized InputManager] --> ImGuiPanels[UI Panels]
		ImGuiPanels --> ActionMap[Command Dispatcher]
		ActionMap --> NetClient
		Renderer --> Display[GLFW Window]
		ImGuiPanels --> Display
	end
```

## Threading & Execution Model
1. **Dynamic Engine Thread**:
	- Executes in a high-priority loop with microsecond timing.
	- On Windows, uses multimedia timer resolution (`timeBeginPeriod(1)`) and high-resolution QPC spin-waits.
	- On Linux, configures `SCHED_FIFO` real-time scheduling with memory locking (`mlockall`).
	- Dispatches wire-format `SnapshotMsg` over UDP.
2. **Client Render Thread**:
	- Decoupled from the physics tick rate (runs at display refresh rate, e.g. 60–144 Hz).
	- Consumes the latest `SnapshotMsg` and interpolates transforms between physics steps.
	- Renders spacecraft geometry using Direct State Access (DSA) in OpenGL 4.5.
	- Updates UI panels (Inspector, Stats, SceneTree, Control).
