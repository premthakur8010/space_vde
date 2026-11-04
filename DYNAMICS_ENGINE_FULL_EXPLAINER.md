# Space VDE Dynamics Engine: Complete Architecture, Class & Code Explainer

This document is an exhaustive, line-by-line and file-by-file reference for the **Space Docking Dynamics Engine** (`space_vde/space_vde`). It details:
1. Every header (`.hpp`) and source (`.cpp`) file: class structure, member variables, and responsibilities.
2. The exact mathematical equations and their corresponding C++ code locations.
3. System dataflow and call graphs across the 1000 Hz real-time cyclic loop.
4. The **Single Source of Truth** architecture across the Dynamics Engine, Network Protocol, and UI.
5. An honest audit against the **ISRO SOW** (what is completed vs. what remains to be built for contact dynamics and capture).

---

## 1. High-Level Architectural Flow

The Dynamics Engine propagates the 6-DOF orbital states of two spacecraft (Chaser and Target) at **1000 Hz** ($dt = 1\text{ ms}$) and streams telemetry to the 3D Visualizer over UDP at **125 Hz** ($dt = 8\text{ ms}$).

### System Call Graph per 1 ms Cycle (`step(dt)`):
```
dynamics_engine_main.cpp (1 kHz while loop)
  │
  ├── 1. DynamicsEngine::step(0.001)
  │     │
  │     ├── 1.1 Direct Telemetry Channel Routing
  │     │     └─ Maps load-cell forces to Chaser and Target based on FrameMapping (DEFAULT/FLIPPED)
  │     │
  │     ├── 1.2 SpacecraftDynamics::move(FT_chaser, dt) [Chaser Body]
  │     │     └─ RK4 Integrator: Newton's 2nd Law + Euler's Rotational Equations
  │     │
  │     ├── 1.3 SpacecraftDynamics::move(FT_target, dt) [Target Body]
  │     │     └─ RK4 Integrator: Newton's 2nd Law + Euler's Rotational Equations
  │     │
  │     ├── 1.4 SpacecraftPoseComputation::compute(chaser, target, gspFeedback)
  │     │     └─ Blends simulated body poses with physical platform encoder feedback (if active)
  │     │
  │     ├── 1.5 RelativeKinematics::compute(chaser, target)
  │     │     └─ Computes r_rel, v_rel, q_rel = q_target^* ⊗ q_chaser, and qDot_rel
  │     │
  │     └── 1.6 TelemetryAccumulator::addSample(chaser, target, relative)
  │           └─ Batches state vectors every 10 ms for CsvLogger (100 Hz output)
  │
  ├── 2. UDP Config Receiver: recvFrom(RunConfigPacket) [Non-blocking]
  │     └─ If received: DynamicsEngine::updateParameters() (updates mass/inertia/mode mid-run)
  │
  └── 3. UDP State Broadcaster (every 8 steps = 125 Hz):
        └─ Packs StatePacket (1049 bytes, packed) and fires to Port 5002
```

---

## 2. File-by-File Breakdown (`.hpp` & `.cpp`)

```
space_docking/
├── dynamics_engine/
│   ├── Types.hpp                    ← Core data structures, canonical math types, error codes
│   ├── SpacecraftDynamics.hpp/.cpp  ← 6-DOF Newton-Euler dynamics + RK4 integrator
│   ├── RelativeKinematics.hpp/.cpp  ← Relative pose, velocity, and quaternion kinematics
│   ├── SpacecraftPoseComputation.hpp/.cpp ← Platform pose offset calculator & encoder blender
│   ├── DynamicsEngine.hpp/.cpp      ← Central simulation coordinator & channel router
│   ├── CsvLogger.hpp/.cpp           ← Multi-threaded asynchronous 25-column CSV logger
│   └── XmlConfigLoader.hpp/.cpp     ← TinyXML2 parser for simulation_config.xml
├── network/
│   ├── Protocol.hpp                 ← 1-byte packed binary UDP wire datagrams (RunConfigPacket & StatePacket)
│   └── UdpSocket.hpp/.cpp           ← Non-blocking BSD/Winsock UDP socket wrapper
└── utils/
    ├── Logger.hpp/.cpp              ← Diagnostic event logger (simulation.log)
    ├── LogMacros.hpp                ← SD_LOG_INFO, SD_LOG_ERROR macros
    └── MathUtils.hpp                ← Canonical quatToEuler, eulerToQuat, tolerances
```

> **Architectural Refactoring Note:**  
> `FrameTransformation.hpp/.cpp` and `NetworkInterface.hpp` were **retired and removed**:
> - As confirmed by technical review, the 4 intermediate coordinate rotation matrices were unused in software simulation. The channel routing between Ground Stewart Platform (GSP) and Flight Stewart Platform (FSP) is now handled directly in `DynamicsEngine::step()`.
> - `NetworkInterface.hpp` was an unused abstract interface. The network communication is cleanly handled by `UdpSocket.hpp` and `Protocol.hpp`.

---

### Module 1: `Types.hpp` — Single Source of Truth
- **Path:** `include/space_docking/dynamics_engine/Types.hpp`
- **Role:** Pure canonical header defining all fundamental physical quantities, coordinate vectors, and error codes. Serves as the **Single Source of Truth** for the entire project.

#### Key Types & Aliases:
1. **`Vector3` (`Vec3`):**
   - Fields: `double x = 0.0, y = 0.0, z = 0.0`.
   - Used for Cartesian positions ($\text{m}$), velocities ($\text{m/s}$), forces ($\text{N}$), and torques ($\text{N}\cdot\text{m}$).
2. **`Matrix3` (`Mat3`):**
   - Fields: `std::array<double, 9> m` (row-major: $I_{xx}, I_{xy}, I_{xz}, I_{yx}, I_{yy}, I_{yz}, I_{zx}, I_{zy}, I_{zz}$).
   - Represents mass moments of inertia ($\text{kg}\cdot\text{m}^2$) and $3\times 3$ coordinate rotation matrices.
3. **`Quaternion`:**
   - Fields: `double w = 1.0, x = 0.0, y = 0.0, z = 0.0` (**Scalar-first convention** $q = [w, x, y, z]$).
   - Represents singularity-free 3D angular orientations.
4. **`EulerAngles` (`Rpy`):**
   - Fields: `double roll = 0.0, pitch = 0.0, yaw = 0.0` (Roll about X, Pitch about Y, Yaw about Z, in radians).
5. **`Pose`:**
   - Fields: `Vector3 position`, `Quaternion orientation`.
   - Combines position and attitude into a single atomic 6-DOF transform.
6. **`ForceTorque`:**
   - Fields: `Vector3 torque`, `Vector3 force` (torque stored first, matching space_vde network convention).
7. **`VdeState` & `SpacecraftState`:**
   - Canonical definitions representing the physical testbed platform state and full orbital spacecraft state.
8. **`RelativeState`:**
   - Fields: `Pose pose`, `Vector3 velocity`, `Quaternion quaternionRate`.
   - Method: `toEuler()` returns instantaneous Roll-Pitch-Yaw angles without gimbal lock.
9. **`FrameMapping`:**
   - Supports both `DEFAULT` (0) and `FLIPPED` (1).

---

### Module 2: `SpacecraftDynamics.hpp` & `SpacecraftDynamics.cpp` — 6-DOF Physics & RK4
- **Paths:** 
  - `include/space_docking/dynamics_engine/SpacecraftDynamics.hpp`
  - `src/dynamics_engine/SpacecraftDynamics.cpp`
- **Role:** Implements the rigid-body equations of motion for a single spacecraft using **Newton's 2nd Law** (translation) and **Euler's Equations** (rotation), integrated with a **4th-Order Runge-Kutta (RK4)** numerical integrator.

#### The Physics Equations & Code Mapping:

##### 1. Translational Acceleration (Newton's 2nd Law)
$$\vec{a} = \frac{\vec{F}_{\text{total}}}{m}$$
- **Code:** `SpacecraftDynamics.cpp`:
  ```cpp
  Vector3 linearAcceleration;
  linearAcceleration.x = totalFT.force.x / m_mass;
  linearAcceleration.y = totalFT.force.y / m_mass;
  linearAcceleration.z = totalFT.force.z / m_mass;
  ```

##### 2. Rotational Acceleration (Euler's Rotational Equations)
In the spacecraft principal body frame:
$$\vec{\tau} = \mathbf{I}\dot{\vec{\omega}} + \vec{\omega} \times (\mathbf{I}\vec{\omega}) \implies \dot{\vec{\omega}} = \mathbf{I}^{-1} \left( \vec{\tau} - \vec{\omega} \times (\mathbf{I}\vec{\omega}) \right)$$
- **Code:** `SpacecraftDynamics.cpp::computeDerivative()`:
  - First, compute angular momentum: $\vec{H} = \mathbf{I}\vec{\omega}$.
  - Next, compute gyroscopic torque: $\vec{\tau}_{gyro} = \vec{\omega} \times \vec{H}$.
  - Subtract gyroscopic torque from external torque: $\vec{\tau}_{net} = \vec{\tau}_{ext} - \vec{\tau}_{gyro}$.
  - Multiply by inverse inertia: $\dot{\vec{\omega}} = \mathbf{I}^{-1} \vec{\tau}_{net}$.

##### 3. Quaternion Kinematics (Attitude Rate of Change)
$$\dot{q} = \frac{1}{2} q \otimes \vec{\omega} = \frac{1}{2} \begin{bmatrix}
-x\omega_x - y\omega_y - z\omega_z \\
 w\omega_x + y\omega_z - z\omega_y \\
 w\omega_y - x\omega_z + z\omega_x \\
 w\omega_z + x\omega_y - y\omega_x
\end{bmatrix}$$
- **Code:** `SpacecraftDynamics.cpp`:
  ```cpp
  Quaternion qDot;
  qDot.w = 0.5 * (-q.x * w.x - q.y * w.y - q.z * w.z);
  qDot.x = 0.5 * ( q.w * w.x + q.y * w.z - q.z * w.y);
  qDot.y = 0.5 * ( q.w * w.y - q.x * w.z + q.z * w.x);
  qDot.z = 0.5 * ( q.w * w.z + q.x * w.y - q.y * w.x);
  ```

##### 4. Runge-Kutta 4th-Order (RK4) Integration Stepper
Instead of Euler integration ($\Delta x = v \cdot dt$, which drifts rapidly), the engine evaluates 4 intermediate slopes ($k_1, k_2, k_3, k_4$) per millisecond:
$$k_1 = f(t, y)$$
$$k_2 = f(t + \frac{dt}{2}, y + \frac{dt}{2} k_1)$$
$$k_3 = f(t + \frac{dt}{2}, y + \frac{dt}{2} k_2)$$
$$k_4 = f(t + dt, y + dt \cdot k_3)$$
$$y_{n+1} = y_n + \frac{dt}{6} (k_1 + 2k_2 + 2k_3 + k_4)$$
- **Code:** `SpacecraftDynamics.cpp::integrateRK4()` computes slopes for both position/velocity and quaternion/angular rates, normalizing the quaternion at the end of every step:
  $$\|q\| = \sqrt{w^2 + x^2 + y^2 + z^2} \implies q \leftarrow \frac{q}{\|q\|}$$

---

### Module 3: `RelativeKinematics.hpp` & `RelativeKinematics.cpp`
- **Paths:** 
  - `include/space_docking/dynamics_engine/RelativeKinematics.hpp`
  - `src/dynamics_engine/RelativeKinematics.cpp`
- **Role:** Computes the relative state between the Chaser and Target docking ports.

#### The Physics Equations:
1. **Relative Position:**
   $$\vec{r}_{rel} = \vec{r}_{chaser} - \vec{r}_{target}$$
2. **Relative Velocity:**
   $$\vec{v}_{rel} = \vec{v}_{chaser} - \vec{v}_{target}$$
3. **Relative Attitude Quaternion:**
   Given target quaternion $q_t$ and chaser quaternion $q_c$, the relative orientation is:
   $$q_{rel} = q_t^* \otimes q_c$$
   Where $q_t^* = [w_t, -x_t, -y_t, -z_t]$ is the quaternion conjugate (inverse rotation).
4. **Relative Quaternion Rate:**
   $$\dot{q}_{rel} = \frac{1}{2} q_{rel} \otimes (\vec{\omega}_{chaser} - \vec{\omega}_{target})$$

---

### Module 4: `SpacecraftPoseComputation.hpp` & `SpacecraftPoseComputation.cpp`
- **Paths:**
  - `include/space_docking/dynamics_engine/SpacecraftPoseComputation.hpp`
  - `src/dynamics_engine/SpacecraftPoseComputation.cpp`
- **Role:** Blends simulated body poses with physical Stewart platform feedback.
- **Logic:**
  ```cpp
  const double qNormSq = f.pose.orientation.w * f.pose.orientation.w + ...;
  if (qNormSq > 0.1) {
      // Hardware in the loop: blend measured hexapod encoder offset
      return { c.pose, targetWithPlatform };
  }
  // Software only mode: return pure simulated poses
  return { c.pose, t.pose };
  ```

---

### Module 5: `DynamicsEngine.hpp` & `DynamicsEngine.cpp` — The Central Coordinator
- **Paths:**
  - `include/space_docking/dynamics_engine/DynamicsEngine.hpp`
  - `src/dynamics_engine/DynamicsEngine.cpp`
- **Role:** Owns instances of `SpacecraftDynamics` (chaser & target), `RelativeKinematics`, `SpacecraftPoseComputation`, and `CsvLogger`.

#### Core Methods:
- **`initialize(const RunConfig& config)`:** Sets up initial spacecraft masses, inertias, starting separation, and opens the CSV log file.
- **`updateParameters(...)`:** Dynamically updates mass, inertia, and mounting mode without wiping trajectory history or truncating CSV logs.
- **`step(double dt)`:** The primary 1 kHz tick:
  1. Routes telemetry sensor forces to Chaser and Target based on `m_config.frameMapping`.
  2. Adds guidance/control forces (`m_ngcForces`).
  3. Steps chaser and target using RK4.
  4. Computes relative kinematics and platform poses.
  5. Feeds state to the decimation accumulator for asynchronous CSV logging.

---

### Module 6: `MathUtils.hpp` — Canonical Math & Unit Conversions
- **Path:** `include/space_docking/utils/MathUtils.hpp`
- **Role:** Shared, robust, header-only mathematical utilities:
  - `quatToEuler(const Quaternion& q)`: converts scalar-first quaternion to Roll-Pitch-Yaw angles without gimbal lock or NaN pitfalls.
  - `eulerToQuat(const EulerAngles& r)`: converts Roll-Pitch-Yaw angles to a unit quaternion.
  - Portable C++20 `std::numbers::pi` (eliminating compiler-specific `M_PI` macro issues).
  - Floating-point `nearlyEqual()` and `assertFinite()`.

---

### Module 7: `Protocol.hpp` & UI Integration (`Client.hpp`)
- **Protocol Path:** `include/space_docking/network/Protocol.hpp`
- **UI Client Path:** `ui-space-docking-system-refactored/include/Client.hpp`
- **Role:** Fixed binary UDP datagrams shared between the physics engine and the 120 FPS UI.
- **Single Source of Truth:**
  - `Protocol.hpp` directly includes `Types.hpp` and uses `Vector3`, `EulerAngles`, `Matrix3`, `Quaternion`, etc.
  - `Client.hpp` in the UI client directly includes `Protocol.hpp` and `MathUtils.hpp`.
  - **Zero duplicated structs across the UI and engine.**
- **Guaranteed Byte Packing:**
  - `RunConfigPacket`: exactly **261 bytes** (`#pragma pack(push, 1)` + `static_assert`).
  - `StatePacket`: exactly **1049 bytes** (`#pragma pack(push, 1)` + `static_assert`).

---

### Module 8: `CsvLogger.hpp` & `CsvLogger.cpp` — Asynchronous Telemetry Logger
- **Paths:**
  - `include/space_docking/dynamics_engine/CsvLogger.hpp`
  - `src/dynamics_engine/CsvLogger.cpp`
- **Role:** Records numerical telemetry state vectors to disk at **100 Hz** (every 10 ms decimation).
- **Architecture:** **Producer-Consumer multi-threading**.
  - Physics loop pushes state snapshots into a thread-safe `std::queue`.
  - Background worker thread batch-writes rows to disk so disk I/O never causes jitter in the 1 kHz physics loop.
- **Output:** Exact **25 columns** (Timestamp, cycle, Chaser pos/quat, Target pos/quat, Rel pos/vel, Rel Euler angles).

---

## 3. Audit Against ISRO SOW: What Is Done vs. What Is Still Needed

| Requirement from ISRO SOW | Current Implementation Status | What Remains to Be Built |
| :--- | :---: | :--- |
| **6-DOF Spacecraft Dynamics** | **100% COMPLETE** | Newton-Euler equations with RK4 numerical integration fully implemented and verified. |
| **Orbital Relative Kinematics** | **100% COMPLETE** | Relative position, velocity, and quaternion rates computed cleanly every millisecond. |
| **Decoupled Real-Time UDP Streaming** | **100% COMPLETE** | 125 Hz binary streaming (`StatePacket`, 1049 bytes packed) to Port 5002. |
| **120 FPS 3D Visualization** | **100% COMPLETE** | OpenGL 3.3/4.5 visualizer with Stewart Platform and orbital views, plus 12-chart analysis grid. |
| **Asynchronous CSV Telemetry Logging**| **100% COMPLETE** | Multi-threaded producer-consumer logger writing 25 parameters at 100 Hz. |
| **Contact Detection & Collision Dynamics** | ⚠️ **NOT YET IMPLEMENTED** | When docking probe touches cone, compute normal/tangential contact forces (Hunt-Crossley model). |
| **Docking Latch Engagement Logic** | ⚠️ **NOT YET IMPLEMENTED** | Check latch capture corridor (velocity $[0.05, 0.15]\text{ m/s}$, lateral error $\le 20\text{ mm}$, angular error $\le 2^\circ$). |
| **Hexapod Inverse Kinematics (IK)** | ⚠️ **NOT YET IMPLEMENTED** | Solve 6 physical actuator leg lengths ($L_1 \dots L_6$) from Stewart platform pose. |
| **Closed-Loop GNC Thruster Guidance** | ⚠️ **NOT YET IMPLEMENTED** | Compute feedback control commands (`ngcChaser`, `ngcTarget`) for autonomous approach. |

---

## 4. How Contact Dynamics Will Slot Into the 1 kHz Loop

When Contact Dynamics and Collision Detection are implemented, they plug directly between Step 1.1 and Step 1.2:

```
DynamicsEngine::step(0.001)
  │
  ├── [NEW] ContactDetection::checkCollision(chaserPose, targetPose)
  │     └─ Tests probe tip penetration δ into docking cone petals
  │
  ├── [NEW] ContactDynamics::computeForces(δ, v_rel)
  │     └─ Hunt-Crossley spring-damper calculates normal force F_n and Coulomb friction F_t
  │
  ├── SpacecraftDynamics::move(FT_chaser + F_contact, dt)
  └── SpacecraftDynamics::move(FT_target - F_contact, dt)   <-- Newton's 3rd Law (Equal & Opposite)
```
Spacecraft will naturally bounce, slide, recoil, or capture based on first-principles physics!
