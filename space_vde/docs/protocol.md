# Wire Protocol Specification

## Message Formats

All messages use `#pragma pack(push, 1)` binary packaging, with little-endian numerical encoding.

### 1. SnapshotMsg (Physics Engine -> Visualization Client)
- **Magic**: `0x53504143` (`'SPAC'`)
- **Port**: `9870`

| Field | Type | Unit / Range | Description |
| :--- | :--- | :--- | :--- |
| `magic` | `uint32_t` | `0x53504143` | Packet validation magic |
| `tick` | `uint64_t` | steps | Monotonically increasing simulation tick |
| `simTimeSec` | `double` | seconds | Total simulation epoch time |
| `targetPosM` | `double[3]` | meters | Target body position $(x, y, z)$ |
| `targetQuat` | `double[4]` | unitless | Target body attitude quaternion $(w, x, y, z)$ |
| `targetVelMps` | `double[3]` | $m/s$ | Target body linear velocity $(v_x, v_y, v_z)$ |
| `chaserPosM` | `double[3]` | meters | Chaser body position $(x, y, z)$ |
| `chaserQuat` | `double[4]` | unitless | Chaser body attitude quaternion $(w, x, y, z)$ |
| `chaserVelMps` | `double[3]` | $m/s$ | Chaser body linear velocity $(v_x, v_y, v_z)$ |
| `contactOccurred` | `uint8_t` | 0 or 1 | Active contact indicator |
| `paused` | `uint8_t` | 0 or 1 | Engine pause state |
| `scenario` | `uint8_t` | 1 or 2 | Active mission scenario ID |
| `secondsSpecified` | `uint8_t` | 0 or 1 | Flag indicating bounded execution duration |
| `physicsPeriodUs` | `uint32_t` | microseconds | Configured physics loop period |
| `avgPeriodNs` | `uint32_t` | nanoseconds | Rolling average cycle time |
| `maxPeriodNs` | `uint32_t` | nanoseconds | Maximum cycle time recorded |
| `maxWorkNs` | `uint32_t` | nanoseconds | Peak computational workload duration |
| `maxLatencyNs` | `uint32_t` | nanoseconds | Peak loop scheduling latency |
| `latencyViolations`| `uint64_t`| count | Ticks exceeding jitter tolerance threshold |
| `lateTicks` | `uint64_t` | count | Ticks missing real-time deadlines |
| `tolUs` | `double` | microseconds | Jitter tolerance threshold |
| `headlessSeconds` | `double` | seconds | Total bounded run duration |

### 2. InputCommandMsg (Visualization Client -> Physics Engine)
- **Magic**: `0x434D4E44` (`'CMND'`)
- **Port**: `9871`

| Field | Type | Description |
| :--- | :--- | :--- |
| `magic` | `uint32_t` | Packet validation magic (`0x434D4E44`) |
| `command` | `uint32_t` | `1`: Scenario1, `2`: Scenario2, `3`: Reset, `4`: TogglePause |
