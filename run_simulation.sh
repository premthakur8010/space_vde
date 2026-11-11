#!/usr/bin/env bash
set -e

# Resolve absolute directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENGINE_DIR="${SCRIPT_DIR}/space_vde"
UI_DIR="${SCRIPT_DIR}/ui-space-docking-system-refactored"

ENGINE_EXE="${ENGINE_DIR}/build/physics_engine"
UI_EXE="${UI_DIR}/build/vde_sim"

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

echo "========================================================================="
echo "              Space Docking System - Simulation Launcher                 "
echo "========================================================================="
echo ""

echo "[1/2] Launching Dynamics Engine (1000 Hz Physics, UDP Port 5002 / 5001)..."
(cd "${ENGINE_DIR}" && exec "${ENGINE_EXE}" config/simulation_config.xml) &
ENGINE_PID=$!

cleanup() {
    echo ""
    echo "Stopping Physics Engine (PID: ${ENGINE_PID})..."
    kill -SIGINT "${ENGINE_PID}" 2>/dev/null || true
    wait "${ENGINE_PID}" 2>/dev/null || true
    echo "Done."
}
trap cleanup EXIT INT TERM

echo "Waiting for socket initialization..."
sleep 1.5

echo "[2/2] Launching 3D UI Visualizer (120 FPS OpenGL Client)..."
(cd "${UI_DIR}" && exec "${UI_EXE}")
