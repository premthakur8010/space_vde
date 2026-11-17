# Project Progress & Change Log

This file tracks all modifications, architectural updates, and feature additions made to the Space VDE docking simulation and UI visualizer codebase.

---

## [2026-11-17] - Interactive Simulation Controls & Hot XML Reloading

### 1. Problem Statement & Motivation
* Previously, the Dynamics Engine (`physics_engine`) started advancing the 1000 Hz physics simulation immediately upon launching, causing time and motion to elapse before the operator could inspect the initial setup in the 3D UI.
* Modifying configuration parameters in `simulation_config.xml` or via the UI dialog required terminating and restarting all background and foreground processes to reset the trajectory to initial conditions.

### 2. Architectural & Protocol Enhancements (`space_vde/include/space_docking/network/Protocol.hpp`)
* **`SimCommand` Enum Added**:
  * `0: None` — Mid-run dynamic parameter update (mass, inertia, frame mapping).
  * `1: Start` — Start or resume 1000 Hz physics integration.
  * `2: Pause` — Halt physics integration while preserving UDP telemetry broadcasting.
  * `3: Reset` — Reset kinematics, clocks, and CSV logger to initial conditions at $T = 0.0\text{ s}$ (Paused).
  * `4: ReloadXml` — Re-parse `simulation_config.xml` from disk, re-initialize spacecraft states, and reset to $T = 0.0\text{ s}$ (Paused).
* **`SimRunState` Enum Added**:
  * `0: Paused` — Engine halted at $T = 0.0\text{ s}$ or paused mid-run.
  * `1: Running` — Physics loop actively stepping.
* **Byte-Packing & Strict Protocol Guarantees Maintained**:
  * `RunConfigPacket`: Embedded `uint8_t simCommand` while trimming `ngcMode` from 64 to 63 bytes. Total size remains **exactly 261 bytes**.
  * `StatePacket`: Embedded `uint8_t simRunState` while trimming `ngcMode` from 64 to 63 bytes. Total size remains **exactly 1049 bytes**.
  * Zero wire alignment changes; all static assertions and unit tests continue passing.

### 3. Dynamics Engine Updates (`space_vde/src/dynamics_engine/dynamics_engine_main.cpp`)
* **Initial Paused State**: Simulation defaults to `isSimRunning = false`, broadcasting static initial pose telemetry at 125 Hz while waiting for UI start signal. An optional `--auto-start` CLI flag is supported for headless runs.
* **Non-Blocking Command Drain**: Drains all pending `RunConfigPacket` datagrams from UDP port `5001` per cycle to ensure immediate response to operator commands.
* **Command Dispatch**:
  * `Start` / `Pause`: Toggles `isSimRunning` and logs state changes.
  * `Reset`: Resets `stepCount = 0`, calls `engine.initialize(config)`, and reopens CSV logger.
  * `ReloadXml`: Re-parses `simulation_config.xml` via `XmlConfigLoader`, applies new masses/inertias/initial positions/approach velocities, calls `engine.initialize(config)`, and resets to $T = 0.0\text{ s}$.
* **`XmlTags.hpp`**: Added declarations for `<hardwareenabled>`, `<hardwarehost>`, `<hardwarecommandport>`, and `<hardwaretelemetryport>` to align with `simulation_config.xml`.

### 4. UI Visualization Client (`ui-space-docking-system-refactored/include/Client.hpp`, `src/Client.cpp`)
* **State Unpacking**: Added `isSimRunning` to `SimSnapshot` decoded directly from `StatePacket::simRunState`.
* **AppState Command Routing**: Added `SimControlCmd` command variant and convenience helpers:
  * `app.startSimulation()`
  * `app.pauseSimulation()`
  * `app.resetSimulation()`
  * `app.reloadXmlSimulation()`
  * `app.isSimRunning()`
* **Sequence Reset Handling**: Enhanced `VisualizationClient::getLatestState` to automatically detect sequence number resets to 0 (upon simulation restart) and avoid discarding valid reset packets as stale.

### 5. UI Controls & XML Dialog (`ui-space-docking-system-refactored/src/UiCore.cpp`, `src/UiWindows.cpp`)
* **Toolbar Controls**:
  * Added dynamic **`START SIM`** (green) / **`PAUSE SIM`** (orange) button.
  * Added **`RESET (T=0)`** button to instantly restore initial conditions.
  * Added **`RELOAD XML`** button for instant reload when editing `simulation_config.xml` in an external editor.
  * Added real-time status pill tags: `Status: READY (PAUSED)` (orange) and `Status: LIVE (RUNNING)` (green).
* **Config XML Dialog**:
  * Replaced `SAVE XML & SEND (UDP)` with **`SAVE XML & RESTART`**: persists edits to disk and commands the engine to hot-reload and reset at $T = 0.0\text{ s}$.
  * Added **`RELOAD FROM DISK`** button: re-reads changes made externally into both the UI dialog and the running physics engine.

### 6. Build & Verification
* Built both `physics_engine` and `vde_sim` with CMake.
* Executed CTest unit test suite (`MathUtilsTest`, `ProtocolTest`, `DynamicsEngineBaseTest`, `CsvLoggerTest`): 100% passed.
* Executed automated loopback UDP test verifying:
  1. Initial state paused at $T = 0.0\text{ s}$ (`simRunState = 0`).
  2. `START` command advances physics and sets `simRunState = 1`.
  3. `PAUSE` command halts physics advancement and sets `simRunState = 0`.
  4. `RESET` command restores $T = 0.0\text{ s}$ and `simRunState = 0`.
  5. `RELOAD_XML` command hot-reloads file and resets to initial conditions at $T = 0.0\text{ s}$.

---

## [2026-11-17] - Tamper-Proof Factory Defaults (`default.xml` Auto-Regeneration)

### 1. Problem Statement & Motivation
* `config/default.xml` serves as the factory-default configuration reference when reverting edited XML settings.
* Because it exists as a plain file on disk, a user could directly edit or delete `default.xml`, corrupting the known-good baseline.
* To make `default.xml` tamper-proof, the canonical default configuration must live compiled inside C++ code so any modification or deletion on disk is automatically detected and overwritten.

### 2. Compiled-In Factory Defaults & Standalone Generator (`DefaultConfig.hpp` & `generate_default_xml.cpp`)
* Created `space_vde/include/space_docking/dynamics_engine/DefaultConfig.hpp` containing:
  * `kDefaultConfigXml`: The single-source-of-truth factory XML configuration embedded as a `constexpr std::string_view` raw string literal in the compiled binary.
  * `ensureDefaultConfigFile(path)` / `ensureDefaultConfigNextTo(simConfigPath)`:
    * Reads `default.xml` from disk and compares it against `kDefaultConfigXml` (normalizing CRLF/LF line endings).
    * If `default.xml` is missing or its contents differ from `kDefaultConfigXml`, automatically rewrites `default.xml` from the compiled-in C++ template (restoring write permissions first if needed) and prints a `[DEFAULT CONFIG WARNING]` alert to the console/log.
    * Returns `DefaultFileStatus` (`Intact`, `RestoredMissing`, `RestoredTampered`, or `WriteFailed`).
* Created standalone C++ generator `space_vde/src/dynamics_engine/generate_default_xml.cpp` (compiled to `space_vde/build/generate_default_xml`) which can be invoked directly at any time to verify and regenerate `config/default.xml`.

### 3. Automatic Enforcement in Physics Engine & UI
* **Dynamics Engine (`dynamics_engine_main.cpp`)**:
  * Calls `ensureDefaultConfigNextTo(configPath)` automatically on startup and whenever a `ReloadXml` command is received.
  * Logs a warning via `SD_LOG_WARN` whenever tampering is detected and restored.
* **UI Visualizer (`src/main.cpp` & `src/UiWindows.cpp`)**:
  * Calls `ensureDefaultConfigNextTo(resolvedConfig)` on UI startup and whenever the **`EDIT XML`** dialog is opened.
  * **`REVERT TO DEFAULT` Button**: Directly populates all input boxes from the compiled-in `kDefaultConfigXml` constant in C++ memory (so even mid-session disk tampering cannot affect the revert) while simultaneously verifying/restoring `default.xml` on disk and displaying a warning banner in the UI if tampering was repaired.

### 4. Unit & Integration Verification (`tests/unit/TestDynamicsEngineBase.cpp`)
* Added automated unit tests verifying all three states of `ensureDefaultConfigFile`:
  1. Missing file $\rightarrow$ `RestoredMissing` and valid XML generated.
  2. Unmodified file $\rightarrow$ `Intact`.
  3. Tampered file $\rightarrow$ `RestoredTampered` and original factory values (`Chaser = 50 kg`, `Target = 100 kg`) restored.
* Verified all 4 CTest suites pass 100%.

---
