# Issue 34 Changes (branch `issue34changes`)

Changes made in response to the pull request review comments on `config/simulation_config.xml`
and `src/dynamics_engine/XmlConfigLoader.cpp`.

## Summary

| Review comment | Resolution | Status |
| :--- | :--- | :--- |
| What is the "docking reference frame"? | Renamed to **target reference frame** and defined in the config comments | Done |
| Position should be in mm in the target frame, with a 1:1 mapping to OpenGL | Config uses mm; 1:1 mm-to-OpenGL mapping added | Done |
| Hardcode strings in `XmlConfigLoader.cpp` | Moved tag names and values into `XmlTags.hpp` | Done |
| Should the tags be lowercase? | All XML tags switched to lowercase; tag search made case-insensitive; values trimmed and lowercased before validation | Done |
| `LogDirectory`: relative or absolute path? | Relative by default (`logs`); absolute-path support and file writing not implemented yet | Open |

## 1. Target reference frame (mm)

The **target reference frame** is a right-handed 3D frame whose origin `(0, 0, 0)` is at the
Target spacecraft's docking port:

| Axis | Meaning |
| :--- | :--- |
| +X | Docking / approach axis |
| +Y | Up |
| +Z | Right (lateral) |

The Chaser's `<InitialPosition>` is its offset from the Target in this frame, now in **millimetres**.

```xml
<!-- Before -->  <X>-3.0</X>      <!-- meters -->
<!-- After  -->  <X>-3000.0</X>   <!-- millimetres (3 m) -->
```

- The loader multiplies mm by `kMmToM` (1e-3), so the physics engine still works in SI **meters**
  (see `docs/conventions.md`). The simulation behaves exactly as before.
- Velocity is still in **m/s**.
- The Target's `<InitialPosition>` is informational only. The engine always places the Target at
  the origin.

## 2. 1:1 mapping from mm to the OpenGL frame

Added in `include/space_docking/utils/MathUtils.hpp`:

| Name | Value / purpose |
| :--- | :--- |
| `kMmToM` | `1e-3`, millimetres to meters |
| `kMToMm` | `1e3`, meters to millimetres |
| `kOpenGlUnitsPerMm` | `1.0`, so **1 OpenGL unit = 1 mm** |
| `mmToOpenGl(mm)` | Position in mm to OpenGL units |
| `metersToOpenGl(m)` | Engine position in meters to OpenGL units |

```
XML (mm) --kMmToM--> engine (meters) --metersToOpenGl--> OpenGL units (= mm)
 -3000                 -3.0                                 -3000
```

The axes are identical in both frames, so only scaling is applied. There is no OpenGL renderer in
this repository yet, so nothing uses `mmToOpenGl()` / `metersToOpenGl()` except the startup print.

`dynamics_engine_main.cpp` now prints the position in both units:

```
Chaser Initial Pos: (-3, 0, 0) m (config: -3000, 0, 0 mm = OpenGL units, 1:1)
```

## 3. XML tag constants (`XmlTags.hpp`)

New header `include/space_docking/dynamics_engine/XmlTags.hpp`:

- `space_docking::dynamics::xml_tags`: tag names, e.g. `kChaser`, `kMassKg`, `kInitialPosition`,
  `kLogDecimation`.
- `space_docking::dynamics::xml_values`: compared values (`FLIPPED`, `true`, `1`).

`XmlConfigLoader.cpp` includes the header and uses these constants in place of inline string
literals. Behaviour is unchanged. If a tag is renamed, edit this header and the XML together.

> The loader falls back to default values when a tag is missing or misspelled. Check the printed
> values after renaming anything.

## 4. Open items

- **Tag casing:** the XML uses PascalCase (`InitialPosition`). If switching, `snake_case`
  (`initial_position`) is more readable than plain lowercase. Only `XmlTags.hpp` and the XML
  would need to change.
- **Log directory:** `<LogDirectory>` is read and printed, but no CSV is written yet. Planned:
  relative paths by default, absolute paths accepted, directory created if missing, resolved path
  printed at startup.
- **`LogDecimation`:** counts physics steps (1 step = 1 ms at the fixed 1 kHz timestep). The
  startup message says "ms", which is only true for a 1 ms timestep.

## Files changed

| File | Change |
| :--- | :--- |
| `config/simulation_config.xml` | Chaser position in mm; target-frame comments |
| `include/space_docking/utils/MathUtils.hpp` | Unit constants and OpenGL mapping helpers |
| `include/space_docking/dynamics_engine/XmlTags.hpp` | **New:** tag name and value constants |
| `src/dynamics_engine/XmlConfigLoader.cpp` | mm to m conversion; uses `XmlTags.hpp` |
| `src/dynamics_engine/dynamics_engine_main.cpp` | Prints position in mm / OpenGL units |

## Verification

```bash
cmake --build build -j
ctest --test-dir build        # 3/3 tests pass
./build/physics_engine        # Chaser starts at -3 m (-3000 mm), closing at 0.05 m/s
```
