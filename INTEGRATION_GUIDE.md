# Approach A Integration Blueprint: Decoupled UDP Network Architecture

## 1. Architectural Summary & Concept
In **Approach A**, the two codebases run as completely separate OS processes:
1. **`space_vde` (Physics Engine)**: Runs the high-rate, high-precision rigid-body orbital mechanics and docking physics loop (e.g. at 1000 Hz / $dt = 0.001\text{ s}$).
2. **`ui-space-docking-system-refactored` (Visualizer / UI)**: Runs the real-time Dear ImGui + OpenGL 3.3 rendering and telemetry monitoring dashboard at 120 Hz ($dt \approx 8.333\text{ ms}$).

### Communication Topology
- **State Telemetry (Physics -> UI)**: Streamed over UDP to port `5002` using a packed binary `StatePacket` (1056 bytes).
- **Run Configuration (UI -> Physics)**: Sent over UDP to port `5001` using a packed binary `RunConfigPacket` (264 bytes).
- Sockets are non-blocking on both sides to avoid stalling either execution loop.

---

## 2. Packet Specifications & Layout

### A. Protocol Header Constants
- **Magic**: `0x56444531` (ASCII `"VDE1"`)
- **Version**: `1`
- **Default Ports**:
  - `5001`: Inbound configuration port on Physics Engine (UI -> Physics)
  - `5002`: Inbound state telemetry port on Visualizer Client (Physics -> UI)

### B. Binary Memory Sizes
- `RunConfigPacket`: exactly **264 bytes**
- `StatePacket`: exactly **1056 bytes**

---

## 3. Unified Data Conventions (Adopting `space_vde` Native Formats)

To eliminate translation overhead, runtime conversions, and indexing errors, the UI client (`ui-space-docking-system-refactored`) was refactored to directly adopt `space_vde`'s exact native conventions across the wire protocol and UI math:

### 1. Quaternion Convention: Scalar-First $q(w, x, y, z)$
- **Unified Standard:** Scalar component $w$ is stored first, followed by Cartesian vector components $(x, y, z)$.
- **Definition in both `space_vde` and `ui-space-docking`:**
  ```cpp
  struct Quaternion {
      double w = 1.0; // Scalar part W (cosine of half-rotation angle)
      double x = 0.0; // Vector part X (i)
      double y = 0.0; // Vector part Y (j)
      double z = 0.0; // Vector part Z (k)
  };
  ```
- **Direct Assignment (Zero Translation):**
  ```cpp
  const auto& q = rel.pose.orientation;
  pkt.relative.orientation = { q.w, q.x, q.y, q.z };
  ```

### 2. Matrix 3x3 Layout: Linear 1D Array `m[9]`
- **Unified Standard:** Stored as a contiguous 9-element 1D array (`std::array<double, 9> m`) in row-major layout, exactly matching `space_vde::dynamics::Matrix3`.
- **Definition in both `space_vde` and `ui-space-docking`:**
  ```cpp
  struct Mat3 {
      std::array<double, 9> m{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  };
  ```
- **Direct Assignment (No Nested Loops):**
  ```cpp
  pkt.chaser.inertiaKgM2.m = chaser.getInertia().m; // Direct 1D array copy
  config.chaserInertia.m = cfgPkt.chaserInertia.m;   // Direct 1D array copy
  ```

### 3. ForceTorque Struct Order: Torque First, Force Second
- **Unified Standard:** In both physics and UI, torque is declared before force:
  ```cpp
  struct ForceTorque {
      Vec3 torque{}; // Torque vector first (N*m)
      Vec3 force{};  // Force vector second (N)
  };
  ```
- **Direct Assignment:**
  ```cpp
  pkt.vde.moving = { movingTorque, movingForce };
  pkt.vde.overhead = { overheadTorque, overheadForce };
  ```
- **Benefits:** Identical memory layout across wire packets and UI data structures; no field swapping or manual copying required.

---

## 4. Compilation & Verification Commands

```bash
# 1. Build Physics Engine:
cmake -B space_vde/build -S space_vde
cmake --build space_vde/build

# 2. Build UI Visualizer:
cmake -B ui-space-docking-system-refactored/build -S ui-space-docking-system-refactored
cmake --build ui-space-docking-system-refactored/build

# 3. Run Physics Engine:
./space_vde/build/physics_engine space_vde/config/simulation_config.xml

# 4. Run UI Visualizer:
./ui-space-docking-system-refactored/build/vde_sim
```
