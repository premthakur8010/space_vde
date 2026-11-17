#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENGINE_DIR="${SCRIPT_DIR}/space_vde"
UI_DIR="${SCRIPT_DIR}/ui-space-docking-system-refactored"

ENGINE_EXE="${ENGINE_DIR}/build/physics_engine"
UI_EXE="${UI_DIR}/build/vde_sim"

# Candidate search paths for the VDE Hardware Emulator
EMU_CANDIDATES=(
    "${SCRIPT_DIR}/../../emulator/docking-twin-emulator/build/HardwareEmulator"
    "${HOME}/Desktop/emulator/docking-twin-emulator/build/HardwareEmulator"
    "${SCRIPT_DIR}/../docking-twin-emulator/build/HardwareEmulator"
    "${SCRIPT_DIR}/docking-twin-emulator/build/HardwareEmulator"
)

EMU_EXE=""
for cand in "${EMU_CANDIDATES[@]}"; do
    if [ -f "${cand}" ]; then
        EMU_EXE="${cand}"
        break
    fi
done

echo "========================================================================="
echo "        Space Docking System - Full 3-Process HIL Launcher"
echo "========================================================================="
echo ""

if [ -z "${EMU_EXE}" ]; then
    echo "[WARNING] Hardware Emulator not found in default paths."
    echo "  Looked in:"
    for cand in "${EMU_CANDIDATES[@]}"; do
        echo "    - ${cand}"
    done
    echo "  If you have the emulator elsewhere, run it manually, or pass EMU_EXE=/path/to/HardwareEmulator"
    echo ""
    read -r -p "Launch Physics Engine + UI anyway? (y/n) " choice
    if [[ ! "$choice" =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

if [ ! -f "${ENGINE_EXE}" ]; then
    echo "[ERROR] Physics Engine not found at: ${ENGINE_EXE}"
    echo "Please build it first: cmake -B \"${ENGINE_DIR}/build\" -S \"${ENGINE_DIR}\" && cmake --build \"${ENGINE_DIR}/build\" -j"
    exit 1
fi

if [ ! -f "${UI_EXE}" ]; then
    echo "[ERROR] UI Visualizer not found at: ${UI_EXE}"
    echo "Please build it first: cmake -B \"${UI_DIR}/build\" -S \"${UI_DIR}\" && cmake --build \"${UI_DIR}/build\" -j"
    exit 1
fi

PIDS=()
cleanup() {
    echo ""
    echo "Shutting down simulation processes..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -SIGINT "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
        fi
    done
    echo "All processes stopped cleanly."
}
trap cleanup EXIT INT TERM

# 1. Launch Hardware Emulator if found
if [ -n "${EMU_EXE}" ] && [ -f "${EMU_EXE}" ]; then
    EMU_DIR="$(dirname "$(dirname "${EMU_EXE}")")"
    echo "[1/3] Launching VDE Hardware Emulator (500 Hz, RX :5006 / TX :5005)..."
    (cd "${EMU_DIR}" && exec "${EMU_EXE}") &
    PIDS+=($!)
    sleep 1.5
else
    echo "[1/3] Skipping Hardware Emulator (not found)."
fi

# 2. Launch Physics Engine
echo "[2/3] Launching Dynamics Engine (1000 Hz Physics + HIL UDP)..."
(cd "${ENGINE_DIR}" && exec "${ENGINE_EXE}" config/simulation_config.xml) &
PIDS+=($!)
sleep 1.5

# 3. Launch UI Visualizer
echo "[3/3] Launching 3D UI Visualizer (120 FPS OpenGL Client)..."
(cd "${UI_DIR}" && exec "${UI_EXE}")
