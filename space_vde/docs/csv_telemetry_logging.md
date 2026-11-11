# Telemetry CSV Logging & 3D Visualization Replay Architecture

This document details the high-frequency telemetry and trajectory CSV logging system implemented for the Augmented Vertical Docking Experiment (VDE) Digital Twin.

It follows the project engineering documentation standards:
- **What was done**
- **Why it was done**
- **How it was done**
- **Where it was done** (exact files and functions)
- **Replay Visualization Mapping** (how the 3D renderer consumes the CSV)
- **Pushing Guide** (exact files for your sir's repository)

---

## 1. Executive Summary & Purpose

The Dynamics Engine integrates 6-DOF spacecraft equations of motion at **1 kHz** ($1\text{ ms}$ cycle time). 
After a simulation run completes, operators and engineers require:
1. **Deterministic 3D Visualization Replay**: Replaying the full docking maneuver in OpenGL at 60 FPS without re-running the physics engine.
2. **Post-Simulation Engineering Analysis**: Plotting relative distance, closing velocities, attitude quaternions, and thruster forces in MATLAB, Python (pandas), or Excel.

To fulfill these requirements without introducing real-time latency or disk jitter into the 1 kHz loop, an **Asynchronous CSV Telemetry Logger** (`CsvLogger`) and a **10 ms Decimation Accumulator** (`TelemetryAccumulator`) were implemented.

---

## 2. Key Distinction: Code-Level Diagnostics vs. Telemetry Logging

A common question is why two different loggers exist in the architecture:

| Feature / Property | Code-Level Diagnostic Logger (`Logger.hpp` / `LogMacros.hpp`) | Telemetry Trajectory Logger (`CsvLogger.hpp` / `CsvLogger.cpp`) |
| :--- | :--- | :--- |
| **Primary Purpose** | Diagnostic, error, and lifecycle event logging. | High-frequency numerical trajectory logging for 3D replay. |
| **Data Format** | Human-readable text strings: `[ERROR] [ErrorCode: -10] Failed to load XML`. | Pure tabular CSV: comma-separated floating-point numbers. |
| **Trigger Mechanism** | **Event-driven**: Only fires when an event, warning, error, or state change occurs. | **Periodic / Time-driven**: Fires predictably every 10 ms ($100\text{ Hz}$). |
| **Destination** | Console (`stderr` / `stdout`) and `logs/simulation.log`. | `logs/simulation_trajectory.csv`. |
| **Target Audience** | Software engineers diagnosing code crashes, failed assertions, or invalid inputs. | **3D Visualization Software** (for 60 FPS replay) and flight dynamics analysts. |
| **SDD Reference** | Component `Logger` (Software Design Document Figure 2 & Table on Page 8). | Component `CsvLogger` (Software Design Document Figure 2, Figure 3, & Table on Page 8). |

> **Why they must never be combined:**
> If diagnostic text strings like `"[INFO] Initializing engine"` were written into `simulation_trajectory.csv`, it would corrupt the numeric table and crash the visualization CSV parser.
> Conversely, writing 100 Hz floating-point telemetry into `simulation.log` would drown out critical error messages and degrade diagnostic readability.

---

## 3. Real-Time Latency & Threading Model

### The Problem with Synchronous Disk I/O:
* The physics loop must complete every step within **$1.0\text{ ms}$** ($1000\ \mu\text{s}$).
* At the **11th millisecond**, the Stewart Platform (GSP) pose command must be transmitted over the network (hard deadline $\le 12\text{ ms}$, per ISRO SOW Section 3.3.2).
* Direct file writes (`std::ofstream << ...`) usually copy bytes into memory buffers. However, when the OS buffer fills or background flush occurs (NTFS journaling, disk driver write-back, or antivirus scans), synchronous file calls block for **$5\text{ ms}$ to $50+\text{ ms}$**.
* If a synchronous disk flush occurs at the 10th millisecond, **the 1 ms physics thread stalls**, immediately missing its real-time deadline (`REALTIME_DEADLINE_MISSED = -51`) and corrupting the 11th ms Stewart Platform motion command window.

### The Asynchronous Solution:
```
┌─────────────────────────────────────────────────────────────────┐
│ Physics Thread (High Priority, 1 kHz, 1.0 ms budget)            │
│                                                                 │
│   Step 1..9:  Simulate dynamics, accumulate 1 ms states         │
│   Step 10:    Capture net state ──> push to queue (<50 ns)      │
│               (Zero disk I/O, zero formatting overhead)         │
└───────────────────────────────┬─────────────────────────────────┘
                                │ std::mutex / std::condition_variable
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│ Asynchronous Logger Thread (Low Priority Background Worker)     │
│                                                                 │
│   1. Wakes up when batch is ready (or 20 ms timeout)            │
│   2. Drains up to 128 snapshots in one lock acquisition         │
│   3. Formats ASCII CSV numbers and writes to disk               │
│   4. Flushes to disk without affecting physics timing           │
└─────────────────────────────────────────────────────────────────┘
```

* **Caller Latency**: Pushing a snapshot (`logger.log(entry)`) takes **$< 50\text{ nanoseconds}$** (pure memory copy into a queue).
* **Zero Real-Time Jitter**: Disk latency cannot stall or jitter the 1 kHz physics loop.

---

## 4. The 10 ms Decimation and "Net Pose" Calculation

### Why 10 ms Decimation?
* The physics loop integrates at **1 kHz** ($1\text{ ms}$ interval, $1000\text{ Hz}$).
* Writing every single millisecond to disk would generate 3,600,000 rows per hour (unnecessary file bloat).
* 3D visualization displays at **60 Hz** ($16.67\text{ ms}$ frame time).
* Logging at **100 Hz** ($10\text{ ms}$ interval) provides $\approx 1.67$ telemetry samples per display frame—more than enough for silky-smooth 60 FPS playback.

### Mathematical Definition of the "Net Pose":
Because the physics engine integrates differential equations step-by-step:
$$\mathbf{r}_{k} = \mathbf{r}_{k-1} + \int_{(k-1)\Delta t}^{k\Delta t} \mathbf{v}(\tau)\, d\tau$$
The state recorded at the 10th step ($k = 10, 20, 30, \dots$):
$$\mathbf{r}_{10} = \mathbf{r}_0 + \sum_{i=1}^{10} \Delta \mathbf{r}_i, \quad q_{10} = q_0 \otimes \Delta q_1 \otimes \dots \otimes \Delta q_{10}$$
**is already the exact mathematical net accumulation of all 10 preceding physics steps.**

It is **not** an average (which would artificially lag by 5 ms). It represents the true, physically exact location and attitude of the spacecraft at $t = 10\text{ ms}, 20\text{ ms}, 30\text{ ms}, \dots$.

---

## 5. Streamlined CSV Format & 3D Visualization Replay

The output trajectory CSV contains exactly **25 columns** tailored specifically for 3D graphics engines:

```csv
timestamp_s,cycle_count,chaser_pos_x_m,chaser_pos_y_m,chaser_pos_z_m,chaser_quat_w,chaser_quat_x,chaser_quat_y,chaser_quat_z,target_pos_x_m,target_pos_y_m,target_pos_z_m,target_quat_w,target_quat_x,target_quat_y,target_quat_z,rel_pos_x_m,rel_pos_y_m,rel_pos_z_m,rel_vel_x_mps,rel_vel_y_mps,rel_vel_z_mps,rel_yaw_deg,rel_pitch_deg,rel_roll_deg
```

### Column Reference & Visualizer Usage:

| Column Indices | Column Name(s) | Units | How 3D Visualizer Consumes It |
| :--- | :--- | :--- | :--- |
| **1–2** | `timestamp_s`, `cycle_count` | $s$, count | Synchronizes playback timeline with user play/pause/scrub controls. |
| **3–5** | `chaser_pos_x_m, y_m, z_m` | meters | Translates Chaser 3D mesh: `glTranslate(x, y, z)`. |
| **6–9** | `chaser_quat_w, x, y, z` | unitless | Rotates Chaser 3D mesh: `glm::mat4_cast(quat)`. |
| **10–12** | `target_pos_x_m, y_m, z_m` | meters | Translates Target 3D mesh. |
| **13–16** | `target_quat_w, x, y, z` | unitless | Rotates Target 3D mesh. |
| **17–19** | `rel_pos_x_m, y_m, z_m` | meters | Positions relative docking camera and docking corridor guides. |
| **20–22** | `rel_vel_x_mps, y_mps, z_mps` | $m/s$ | Displays closing speed on HUD dashboard gauges. |
| **23–25** | `rel_yaw_deg, pitch_deg, roll_deg` | degrees | Drives attitude readouts on virtual flight instruments. |

### How the Visualizer Replays Smooth 60 FPS from 100 Hz CSV:
When replaying at arbitrary display refresh rates (e.g. 60 Hz or 144 Hz), the renderer interpolates between the two adjacent CSV rows bounding the current replay time $t$:
$$\alpha = \frac{t - t_A}{t_B - t_A}$$
$$\mathbf{r}(t) = (1 - \alpha)\mathbf{r}_A + \alpha \mathbf{r}_B \quad (\text{LERP})$$
$$q(t) = \text{slerp}(q_A, q_B, \alpha) \quad (\text{SLERP})$$

---

## 6. Pushing Guide for Sir's Repository

To maintain clean repository hygiene and adhere to the requirement of not pushing unreviewed engine coordinator files or test files, follow this guide:

### Files to Push to Sir's Repo:
1. **`include/space_docking/dynamics_engine/Types.hpp`**
   * *Status:* Modified.
   * *Reason:* Contains `FILE_OPEN_FAILED = -70` and `LOG_WRITE_FAILED = -71` in `ErrorCode` and `errorCodeToString()`.
2. **`include/space_docking/dynamics_engine/CsvLogger.hpp`**
   * *Status:* New file.
   * *Reason:* Standalone header defining `TelemetryLogEntry`, `TelemetryAccumulator`, and `CsvLogger`. Zero dependencies on `DynamicsEngine`.
3. **`src/dynamics_engine/CsvLogger.cpp`**
   * *Status:* New file.
   * *Reason:* Standalone implementation of the asynchronous disk writing thread and 25-column CSV serializer.
4. **`include/space_docking/dynamics_engine/EngineLogger.hpp`**
   * *Status:* Modified.
   * *Reason:* Updated to `#include "space_docking/dynamics_engine/CsvLogger.hpp"` and maintain the SDD `EngineLogger` interface.
5. **`CMakeLists.txt`**
   * *Status:* Modified.
   * *Reason:* Added `src/dynamics_engine/CsvLogger.cpp` to the `space_docking_dynamics_engine` static library.
6. **`docs/csv_telemetry_logging.md`**
   * *Status:* New file.
   * *Reason:* This documentation specification.

### Files to KEEP Local in R Repo (Do NOT push to Sir's repo):
* `include/space_docking/dynamics_engine/DynamicsEngine.hpp` *(Keep local until coordinator review)*
* `src/dynamics_engine/DynamicsEngine.cpp` *(Keep local until coordinator review)*
* `src/dynamics_engine/dynamics_engine_main.cpp` *(Keep local until coordinator review)*
* `tests/unit/TestCsvLogger.cpp` *(Test suite belongs to the testing team)*

