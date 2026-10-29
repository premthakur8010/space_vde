# Code-Level Logging & Error Handling Architecture

This document details the code-level logging and error-handling infrastructure implemented today.

Each section follows the engineering specification format:
- **What was done**
- **Why it was done**
- **How it was done**
- **Where it was done** (exact files and functions)

---

## 1. Unified Error Codes (`ErrorCode`)

### What Was Done
Created a strongly-typed, categorized `ErrorCode` enum and a zero-allocation string conversion function (`errorCodeToString`).

### Why It Was Done
Previously, simulation components lacked standardized error codes. Functions returned generic integers or booleans, leaving no structured way to identify the exact cause of a failure (e.g., whether an error was caused by a missing XML tag, a negative mass, a matrix inversion failure, or a UDP socket error).

### How It Was Done
Defined negative 32-bit integer error codes partitioned into functional categories:
- **General / Lifecycle (`0` to `-9`)**: `OK` (0), `UNKNOWN_ERROR` (-1), `INVALID_ARGUMENT` (-2), `NOT_INITIALIZED` (-3).
- **Configuration & XML (`-10` to `-19`)**: `CONFIG_FILE_NOT_FOUND` (-10), `CONFIG_PARSE_ERROR` (-11), `CONFIG_MISSING_TAG` (-12), `CONFIG_INVALID_VALUE` (-13).
- **Spacecraft Physical Parameters (`-20` to `-29`)**: `INVALID_MASS` (-20), `INVALID_INERTIA` (-21), `INVALID_TIMESTEP` (-22).
- **Mathematical & Integration (`-30` to `-39`)**: `STEP_FAILED` (-30), `MATH_NAN_OR_INF` (-31), `MATH_QUATERNION_ZERO_NORM` (-32), `MATH_SINGULAR_MATRIX` (-33).
- **Flight Safety & Corridors (`-40` to `-49`)**: `OUT_OF_BOUNDS` (-40), `DOCKING_COLLISION` (-41), `DOCKING_CORRIDOR_EXCEEDED` (-42), `PLATFORM_LIMIT_EXCEEDED` (-43).
- **Timing & UDP Network (`-50` to `-69`)**: `TIMEOUT` (-50), `REALTIME_DEADLINE_MISSED` (-51), `NETWORK_ERROR` (-60), `SOCKET_BIND_FAILED` (-61).

Implemented `errorCodeToString(ErrorCode code)` as a `constexpr` function returning string literals (`"CONFIG_FILE_NOT_FOUND"`, `"INVALID_MASS"`, etc.) without heap allocation.

### Where It Was Done
- [`include/space_docking/dynamics_engine/Types.hpp`](file:///c:/Users/Prem%20M%20Thakur/Desktop/space_vde/space_vde/include/space_docking/dynamics_engine/Types.hpp)

---

## 2. Core Logger Enhancements (`Logger.hpp` & `Logger.cpp`)

### What Was Done
Upgraded the singleton `Logger` class with source context tracking (filename, line number, calling function), ISO timestamps with millisecond precision, and crash-resilient dual output.

### Why It Was Done
1. Raw `std::cout` statements lacked timestamps, severity tags, and source locations, making post-mortem crash debugging impossible.
2. In multi-threaded execution (1000 Hz physics loop + network I/O), concurrent calls to `std::cout` interleave characters.
3. Buffered file streams lose log lines if the simulation crashes or terminates abruptly.

### How It Was Done
1. **Source Context Logging (`logWithContext`)**: 
   - Strips directory prefixes from `__FILE__` to log clean basenames.
   - Formats entries as:
     ```text
     YYYY-MM-DD HH:MM:SS.mmm [LEVEL] (Filename:Line in func): message
     ```
2. **Error Code Logging (`logError`)**:
   - Automatically prefixes messages with `[ERROR_CODE_STRING]`.
3. **Thread Safety**: Guarded all console and file operations with `std::lock_guard<std::mutex> lock(m_mutex)`.
4. **Crash Flush**: Added `m_fileStream.flush()` immediately after every line written to disk.
5. **Severity Filtering**: Retained minimum threshold filtering:
   - `LogLevel::Trace` (0): Granular mathematical step data.
   - `LogLevel::Debug` (1): Developer inspection.
   - `LogLevel::Info` (2, Default): Operational milestones.
   - `LogLevel::Warn` (3): Non-fatal timing or boundary warnings.
   - `LogLevel::Error` (4): Failures requiring abort or error propagation.

### Where It Was Done
- Header: [`include/space_docking/utils/Logger.hpp`](file:///c:/Users/Prem%20M%20Thakur/Desktop/space_vde/space_vde/include/space_docking/utils/Logger.hpp)
- Implementation: [`src/utils/Logger.cpp`](file:///c:/Users/Prem%20M%20Thakur/Desktop/space_vde/space_vde/src/utils/Logger.cpp)

---

## 3. Ergonomic Logging Macros (`LogMacros.hpp`)

### What Was Done
Created lightweight streaming macros (`SD_LOG_INFO`, `SD_LOG_WARN`, `SD_LOG_DEBUG`, `SD_LOG_TRACE`, `SD_LOG_ERROR`) that automatically pass `__FILE__`, `__LINE__`, and `__func__`.

### Why It Was Done
Without macros, logging with source context requires 5–7 lines of boilerplate code per call site (creating an `std::ostringstream`, streaming variables, fetching the `Logger` singleton, and passing compiler macros). The macros condense this to a single line.

### How It Was Done
Defined preprocessor macros utilizing standard `do { ... } while(0)` scoping:
- `SD_LOG_INFO(msg)`: Formats `msg` via `std::ostringstream` and calls `Logger::logWithContext(LogLevel::Info, ...)`.
- `SD_LOG_WARN(msg)`: Formats `msg` and calls `Logger::logWithContext(LogLevel::Warn, ...)`.
- `SD_LOG_DEBUG(msg)`: Formats `msg` and calls `Logger::logWithContext(LogLevel::Debug, ...)`.
- `SD_LOG_TRACE(msg)`: Formats `msg` and calls `Logger::logWithContext(LogLevel::Trace, ...)`.
- `SD_LOG_ERROR(code, msg)`: Passes typed `ErrorCode` and formatted `msg` to `Logger::logError()`.

### Where It Was Done
- [`include/space_docking/utils/LogMacros.hpp`](file:///c:/Users/Prem%20M%20Thakur/Desktop/space_vde/space_vde/include/space_docking/utils/LogMacros.hpp)

---

## 4. Cold-Path Validation: XML Configuration Loader

### What Was Done
Replaced silent defaults and unformatted console output with typed error returns and `SD_LOG_ERROR` logging for all configuration loading failures.

### Why It Was Done
If a configuration file was missing, malformed, or contained illegal physical parameters (e.g. mass $\le 0$), the application previously fell back silently or crashed downstream during acceleration calculation ($a = F/m$).

### How It Was Done
Injected validation checks returning explicit `ErrorCode` values:
1. File existence check $\rightarrow$ logs and returns `ErrorCode::CONFIG_FILE_NOT_FOUND`.
2. Empty buffer check $\rightarrow$ logs and returns `ErrorCode::CONFIG_PARSE_ERROR`.
3. Required `<Chaser>` block check $\rightarrow$ logs and returns `ErrorCode::CONFIG_MISSING_TAG`.
4. Required `<Target>` block check $\rightarrow$ logs and returns `ErrorCode::CONFIG_MISSING_TAG`.
5. Non-positive mass check ($m \le 0$) $\rightarrow$ logs and returns `ErrorCode::INVALID_MASS`.
6. Successful load $\rightarrow$ logs `SD_LOG_INFO` with parsed Chaser and Target masses.

### Where It Was Done
- [`src/dynamics_engine/XmlConfigLoader.cpp`](file:///c:/Users/Prem%20M%20Thakur/Desktop/space_vde/space_vde/src/dynamics_engine/XmlConfigLoader.cpp)

---

## 5. Hot-Path & Parameter Validation: Spacecraft Dynamics

### What Was Done
Added defensive sanity checks in `SpacecraftDynamics` to prevent division by zero and invalid state integration.

### Why It Was Done
Newton's second law ($a = F/m$) divides by mass. If a body is integrated without being configured, mass is 0, causing a fatal IEEE-754 floating-point `NaN`/`Inf` exception. Similarly, non-positive integration timesteps ($dt \le 0$) corrupt kinematics.

### How It Was Done
1. **`setup(mass, inertia)`**: Checks `mass <= 0.0`. If true, logs `SD_LOG_ERROR(ErrorCode::INVALID_MASS, ...)` and returns `INVALID_MASS`.
2. **`move(forceTorque, dt)`**:
   - Checks if `m_mass <= 0.0`. If unconfigured, logs `SD_LOG_ERROR(ErrorCode::NOT_INITIALIZED, ...)` and returns current state without advancing.
   - Checks if `dt <= 0.0`. If invalid, logs `SD_LOG_ERROR(ErrorCode::INVALID_TIMESTEP, ...)` and returns current state.

### Where It Was Done
- [`src/dynamics_engine/SpacecraftDynamics.cpp`](file:///c:/Users/Prem%20M%20Thakur/Desktop/space_vde/space_vde/src/dynamics_engine/SpacecraftDynamics.cpp)

---

## 6. Lifecycle & Initialization Guards: Dynamics Engine

### What Was Done
Added error propagation during body initialization and state guards in the simulation stepping loop.

### Why It Was Done
Prevents the simulation coordinator from stepping physics if initialization failed or if `step()` is called before `initialize()`.

### How It Was Done
1. **`initialize(config)`**: Checks return codes of `m_chaser.setup(...)` and `m_target.setup(...)`. If an error occurs, logs `SD_LOG_ERROR` and halts initialization. If successful, logs `SD_LOG_INFO("DynamicsEngine successfully initialized with 2 spacecraft bodies")`.
2. **`step(dt)`**: Validates `m_initialized`. If false, logs `SD_LOG_ERROR(ErrorCode::NOT_INITIALIZED, ...)` and aborts step execution.

### Where It Was Done
- [`src/dynamics_engine/DynamicsEngine.cpp`](file:///c:/Users/Prem%20M%20Thakur/Desktop/space_vde/space_vde/src/dynamics_engine/DynamicsEngine.cpp)

---

## 7. Log Storage Routing & Output Separation

### What Was Done
Separated human-readable event logging from numerical flight trajectory telemetry data.

### Why It Was Done
The XML configuration specifies `simulation_trajectory.csv` for flight telemetry (time, position, velocity, quaternions). Writing text logs into this CSV file corrupts headers and breaks CSV parsers in Python pandas, MATLAB, and Excel.

### How It Was Done
Configured `dynamics_engine_main.cpp` to initialize `Logger` with `logs/simulation.log` if `<Logging><Enabled>true</Enabled>` is set in XML.
- **`logs/simulation.log`**: Dedicated to diagnostic event text, errors, warnings, timestamps, and line numbers.
- **`logs/simulation_trajectory.csv`**: Dedicated to numerical time-series flight telemetry data.

### Where It Was Done
- [`src/dynamics_engine/dynamics_engine_main.cpp`](file:///c:/Users/Prem%20M%20Thakur/Desktop/space_vde/space_vde/src/dynamics_engine/dynamics_engine_main.cpp#L53-L58)

---

## 8. Complete Reference Table of Error Codes

| Error Code | Int Value | Category | Trigger Condition / Meaning | Detection Location |
| :--- | :---: | :--- | :--- | :--- |
| **`OK`** | `0` | General | Successful execution with no faults | All functions |
| **`UNKNOWN_ERROR`** | `-1` | General | Unclassified fallback error | Fallback |
| **`INVALID_ARGUMENT`** | `-2` | General | Illegal argument or null pointer passed | API boundaries |
| **`NOT_INITIALIZED`** | `-3` | General | `step()` called before `initialize()` or `move()` before `setup()` | `DynamicsEngine.cpp`, `SpacecraftDynamics.cpp` |
| **`CONFIG_FILE_NOT_FOUND`** | `-10` | Config | Specified XML config file does not exist on disk | `XmlConfigLoader.cpp` |
| **`CONFIG_PARSE_ERROR`** | `-11` | Config | XML file is 0 bytes or has invalid syntax | `XmlConfigLoader.cpp` |
| **`CONFIG_MISSING_TAG`** | `-12` | Config | Missing required `<Chaser>` or `<Target>` block | `XmlConfigLoader.cpp` |
| **`CONFIG_INVALID_VALUE`** | `-13` | Config | Non-numeric string in numeric XML tag | `XmlConfigLoader.cpp` |
| **`INVALID_MASS`** | `-20` | Physical | Spacecraft mass $\le 0.0$ kg (division by zero guard) | `XmlConfigLoader.cpp`, `SpacecraftDynamics.cpp` |
| **`INVALID_INERTIA`** | `-21` | Physical | Principal inertia tensor $\le 0.0$ kg*m² | `SpacecraftDynamics.cpp` |
| **`INVALID_TIMESTEP`** | `-22` | Physical | Timestep $dt \le 0.0$ s | `SpacecraftDynamics.cpp` |
| **`STEP_FAILED`** | `-30` | Math | State integration failed to advance | `DynamicsEngine.cpp` |
| **`MATH_NAN_OR_INF`** | `-31` | Math | Floating point overflow to NaN or Infinity | Numerical integration |
| **`MATH_QUATERNION_ZERO_NORM`** | `-32` | Math | Quaternion length is 0 (normalization impossible) | Kinematics |
| **`MATH_SINGULAR_MATRIX`** | `-33` | Math | Inertia matrix determinant is 0 (inversion impossible) | Rotational dynamics |
| **`OUT_OF_BOUNDS`** | `-40` | Flight Safety | Vehicle exceeded maximum simulation boundary ($> 100$ m) | Flight envelope |
| **`DOCKING_COLLISION`** | `-41` | Flight Safety | Contact velocity exceeded docking port structural limits | Contact dynamics |
| **`DOCKING_CORRIDOR_EXCEEDED`** | `-42` | Flight Safety | Lateral displacement exceeded approach cone angle | Trajectory monitor |
| **`PLATFORM_LIMIT_EXCEEDED`** | `-43` | Flight Safety | Pose command exceeded physical Stewart platform stroke | Platform kinematics |
| **`TIMEOUT`** | `-50` | Timing/Net | Hardware handshake or socket receive timeout | Network / HIL |
| **`REALTIME_DEADLINE_MISSED`** | `-51` | Timing/Net | 1000 Hz physics cycle execution exceeded 1.0 ms deadline | Timing monitor |
| **`NETWORK_ERROR`** | `-60` | Timing/Net | UDP packet transmission or reception failed | `UdpSocket.cpp` |
| **`SOCKET_BIND_FAILED`** | `-61` | Timing/Net | UDP socket port already bound or permission denied | `UdpSocket.cpp` |
