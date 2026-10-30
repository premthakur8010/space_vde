# XML Tag Lowercase & Value Normalization Architecture

This document details the changes made to transition the simulation configuration system from PascalCase XML tags to lowercase tags, with case-insensitive tag matching, value normalization, and strict validation.

Following standard engineering specification practice, each section covers:
- **What was done**
- **Why it was done**
- **How it was done**
- **Where it was done** (exact files, functions, and lines)

---

## 1. Executive Summary

| Scope | Before | After | Benefit |
| :--- | :--- | :--- | :--- |
| **XML Tag Naming** | PascalCase (`<Chaser>`, `<InitialPosition>`) | Plain lowercase (`<chaser>`, `<initialposition>`) | Cleaner configuration files aligned with modern XML/HTML standards |
| **Tag Matching** | Strict case-sensitive `std::string::find()` | Case-insensitive tag matching (`findCaseInsensitive`) | Tolerates casing variations (`<chaser>`, `<Chaser>`, `<CHASER>`) without parsing failures |
| **Extracted Values** | Raw substring used directly; case-sensitive `==` | Trimmed and lowercased via `toLowerTrimmed` | Handles leading/trailing whitespace and mixed case (e.g. `DEFAULT`, `flipped`, `True`) |
| **Path Casing** | Unmodified | **Preserved intact** (case-sensitive) | Linux filesystem safety: directory paths like `Logs/` and `logs/` remain distinct |
| **Enum & Bool Validation** | Silent fallbacks to default on unrecognized strings | Explicit validation against permitted set; logs `CONFIG_INVALID_VALUE` and returns error | Prevents silent misconfigurations from propagating into the physics loop |
| **Decimation Validation** | Uncaught fallback to `10` | Validates integer $\ge 1$; rejects `0` and non-numeric values | Avoids division-by-zero or corrupted logging cadences |

---

## 2. Tag & Value Constants (`XmlTags.hpp`)

### What Was Done
Updated all compile-time tag name constants and accepted value string constants to plain lowercase. Added constants for the complete set of supported boolean and enum options.

### Why It Was Done
Centralizing XML tag names in `XmlTags.hpp` prevents raw string literals across the codebase. Lowercasing the constants aligns them with the lowercase XML schema while keeping a single point of truth.

### How It Was Done
1. Converted all string literals in namespace `space_docking::dynamics::xml_tags` to lowercase:
   - Spacecraft: `kChaser = "chaser"`, `kTarget = "target"`, `kMassKg = "masskg"`, `kInertia = "inertia"`, `kIxx = "ixx"`, `kIyy = "iyy"`, `kIzz = "izz"`.
   - Kinematics: `kInitialPosition = "initialposition"`, `kInitialVelocity = "initialvelocity"`, `kX = "x"`, `kY = "y"`, `kZ = "z"`, `kVx = "vx"`, `kVy = "vy"`, `kVz = "vz"`.
   - Platform: `kPlatform = "platform"`, `kFrameMapping = "framemapping"`.
   - Logging: `kLogging = "logging"`, `kEnabled = "enabled"`, `kLogDirectory = "logdirectory"`, `kLogFileName = "logfilename"`, `kLogDecimation = "logdecimation"`.
2. Expanded namespace `space_docking::dynamics::xml_values`:
   - Frame mapping: `kFrameMappingDefault = "default"`, `kFrameMappingFlipped = "flipped"`.
   - Boolean values: `kTrue = "true"`, `kFalse = "false"`, `kOne = "1"`, `kZero = "0"`.

### Where It Was Done
- [`include/space_docking/dynamics_engine/XmlTags.hpp`](file:///home/shruj/Documents/space_vde/include/space_docking/dynamics_engine/XmlTags.hpp)

---

## 3. Configuration File (`simulation_config.xml`)

### What Was Done
Converted all XML element tags to plain lowercase and formatted enum and boolean values to lowercase.

### Why It Was Done
Provides consistency across all configuration blocks (`spacecraft`, `platform`, `logging`) without requiring PascalCase tags.

### How It Was Done
Updated all tags in `config/simulation_config.xml`:
```xml
<simulationconfig>
    <spacecraft>
        <chaser>
            <masskg>50.0</masskg>
            <inertia>
                <ixx>5.0</ixx>
                <iyy>5.0</iyy>
                <izz>10.0</izz>
            </inertia>
            <initialposition>
                <x>-3000.0</x>
                <y>0.0</y>
                <z>0.0</z>
            </initialposition>
            <initialvelocity>
                <vx>0.05</vx>
                <vy>0.0</vy>
                <vz>0.0</vz>
            </initialvelocity>
        </chaser>
        ...
    </spacecraft>
    <platform>
        <framemapping>default</framemapping>
    </platform>
    <logging>
        <enabled>true</enabled>
        <logdirectory>logs</logdirectory>
        <logfilename>simulation_trajectory.csv</logfilename>
        <logdecimation>10</logdecimation>
    </logging>
</simulationconfig>
```

### Where It Was Done
- [`config/simulation_config.xml`](file:///home/shruj/Documents/space_vde/config/simulation_config.xml)

---

## 4. Parser Architecture & Case Insensitivity (`XmlConfigLoader.cpp`)

### What Was Done
Introduced two dedicated parsing helper functions in an anonymous namespace:
1. `findCaseInsensitive`: Scans XML for tags without case sensitivity.
2. `toLowerTrimmed`: Trims whitespace and converts string values to lowercase.
3. Updated `extractTagContent` to use case-insensitive matching while **preserving the exact casing of inner content**.
4. Added strict validation for `<framemapping>`, `<enabled>`, and `<logdecimation>`.

### Why It Was Done
1. **Robustness**: Users or external tools might write `<Chaser>`, `<chaser>`, or `<CHASER>`. A case-insensitive tag scanner ensures that the configuration loads regardless of tag casing.
2. **Filesystem Safety**: Linux filesystems are strictly case-sensitive. If an entire XML buffer or extracted content were converted to lowercase, a path like `Logs/Run1/trajectory.CSV` would be altered to `logs/run1/trajectory.csv`, causing `fopen`/`std::ofstream` to fail. Thus, inner content MUST preserve original casing.
3. **Defensive Validation**: Previously, entering an unrecognized value such as `<FrameMapping>FLIPD</FrameMapping>` silently fell back to `DEFAULT`, disguising misconfigurations. Now, invalid values produce an explicit `CONFIG_INVALID_VALUE` error.

### How It Was Done

#### 4.1 Case-Insensitive Tag Extraction
```cpp
size_t findCaseInsensitive(std::string_view haystack, std::string_view needle, size_t pos = 0)
{
    if (needle.empty()) return pos <= haystack.size() ? pos : std::string_view::npos;
    if (pos + needle.size() > haystack.size()) return std::string_view::npos;

    auto it = std::search(
        haystack.begin() + pos, haystack.end(),
        needle.begin(), needle.end(),
        [](char c1, char c2) {
            return std::tolower(static_cast<unsigned char>(c1)) ==
                   std::tolower(static_cast<unsigned char>(c2));
        });

    if (it == haystack.end()) return std::string_view::npos;
    return static_cast<size_t>(std::distance(haystack.begin(), it));
}
```
In `extractTagContent`:
- `openTag = "<" + tagName + ">"` and `closeTag = "</" + tagName + ">"` are searched via `findCaseInsensitive`.
- The returned substring `xml.substr(contentStart, end - contentStart)` slices the **original** `xml` string, preserving original character casing.

#### 4.2 Value Trimming & Lowercasing
```cpp
std::string toLowerTrimmed(std::string_view sv)
{
    const auto start = sv.find_first_not_of(" \t\r\n");
    if (start == std::string_view::npos) return "";
    const auto end = sv.find_last_not_of(" \t\r\n");
    sv = sv.substr(start, end - start + 1);

    std::string result;
    result.reserve(sv.size());
    for (char c : sv) {
        result.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
    return result;
}
```

#### 4.3 Strict Enum Validation (`<framemapping>`)
```cpp
const std::string rawMapping = extractTagContent(platformBlock, tags::kFrameMapping);
if (!rawMapping.empty())
{
    const std::string mappingStr = toLowerTrimmed(rawMapping);
    if (mappingStr == values::kFrameMappingFlipped)
    {
        outConfig.frameMapping = FrameMapping::FLIPPED;
    }
    else if (mappingStr == values::kFrameMappingDefault)
    {
        outConfig.frameMapping = FrameMapping::DEFAULT;
    }
    else
    {
        SD_LOG_ERROR(ErrorCode::CONFIG_INVALID_VALUE,
                     "Invalid <" << tags::kFrameMapping << "> value: '" << rawMapping
                     << "'. Expected '" << values::kFrameMappingDefault << "' or '"
                     << values::kFrameMappingFlipped << "'");
        return ErrorCode::CONFIG_INVALID_VALUE;
    }
}
```

#### 4.4 Strict Boolean Validation (`<enabled>`)
```cpp
const std::string rawEnabled = extractTagContent(loggingBlock, tags::kEnabled);
if (!rawEnabled.empty())
{
    const std::string enabledStr = toLowerTrimmed(rawEnabled);
    if (enabledStr == values::kTrue || enabledStr == values::kOne)
    {
        outConfig.loggingEnabled = true;
    }
    else if (enabledStr == values::kFalse || enabledStr == values::kZero)
    {
        outConfig.loggingEnabled = false;
    }
    else
    {
        SD_LOG_ERROR(ErrorCode::CONFIG_INVALID_VALUE,
                     "Invalid <" << tags::kEnabled << "> boolean value: '" << rawEnabled
                     << "'. Expected 'true', 'false', '1', or '0'");
        return ErrorCode::CONFIG_INVALID_VALUE;
    }
}
```

#### 4.5 Integer Decimation Validation (`<logdecimation>`)
```cpp
const std::string decimationStr = extractTagContent(loggingBlock, tags::kLogDecimation);
if (!decimationStr.empty())
{
    try
    {
        const unsigned long parsed = std::stoul(toLowerTrimmed(decimationStr));
        if (parsed == 0)
        {
            SD_LOG_ERROR(ErrorCode::CONFIG_INVALID_VALUE,
                         "Invalid <" << tags::kLogDecimation << "> value: 0 (must be >= 1 step)");
            return ErrorCode::CONFIG_INVALID_VALUE;
        }
        outConfig.logDecimation = static_cast<uint32_t>(parsed);
    }
    catch (...)
    {
        SD_LOG_ERROR(ErrorCode::CONFIG_INVALID_VALUE,
                     "Invalid <" << tags::kLogDecimation << "> integer: '" << decimationStr << "'");
        return ErrorCode::CONFIG_INVALID_VALUE;
    }
}
```

### Where It Was Done
- [`src/dynamics_engine/XmlConfigLoader.cpp`](file:///home/shruj/Documents/space_vde/src/dynamics_engine/XmlConfigLoader.cpp)

---

## 5. Verification & Testing

The implementation was built and validated against the automated test suite:

1. **Compilation**: Built with C++20 using `cmake -B build && cmake --build build` (zero compiler warnings, zero errors).
2. **Regression Testing**: Executed `ctest --test-dir build --output-on-failure`. All three test suites (`MathUtilsTest`, `ProtocolTest`, `DynamicsEngineBaseTest`) passed 100%.
3. **End-to-End Simulation**: Successfully parsed and verified `simulation_config.xml` loading Chaser (50.0 kg), Target (100.0 kg), relative coordinates, and logging parameters.
