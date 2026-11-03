@echo off
title Space VDE - Simulation Launcher
echo =========================================================================
echo               Space Docking System - Simulation Launcher
echo =========================================================================
echo.

set SCRIPT_DIR=%~dp0
set ENGINE_DIR=%SCRIPT_DIR%space_vde
set UI_DIR=%SCRIPT_DIR%ui-space-docking-system-refactored

set ENGINE_EXE=%ENGINE_DIR%\build_win\physics_engine.exe
set UI_EXE=%UI_DIR%\build_win\vde_sim.exe

if not exist "%ENGINE_EXE%" (
    echo [ERROR] Physics Engine not found at:
    echo   %ENGINE_EXE%
    echo Please build it first.
    pause
    exit /b 1
)

if not exist "%UI_EXE%" (
    echo [ERROR] UI Visualizer not found at:
    echo   %UI_EXE%
    echo Please build it first.
    pause
    exit /b 1
)

echo [1/2] Launching Dynamics Engine (1000 Hz Physics, UDP 5001/5002)...
start "Space VDE - Physics Engine" cmd /c "cd /d "%ENGINE_DIR%" && "%ENGINE_EXE%" config\simulation_config.xml"

echo Waiting for socket initialization...
timeout /t 2 /nobreak > nul

echo [2/2] Launching 3D UI Visualizer (120 FPS OpenGL Client)...
start "Space VDE - 3D Visualizer" cmd /c "cd /d "%UI_DIR%" && "%UI_EXE%""

echo.
echo =========================================================================
echo Both applications are running!
echo   - Close the UI window or press Ctrl+C in the engine window to stop.
echo =========================================================================

