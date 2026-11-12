@echo off
title Space VDE + Hardware Emulator - HIL Simulation Launcher
echo =========================================================================
echo        Space Docking System - Full 3-Process HIL Launcher
echo =========================================================================
echo.

set SCRIPT_DIR=%~dp0
set ENGINE_DIR=%SCRIPT_DIR%space_vde
set UI_DIR=%SCRIPT_DIR%ui-space-docking-system-refactored

set ENGINE_EXE=%ENGINE_DIR%\build_win\physics_engine.exe
set UI_EXE=%UI_DIR%\build_win\vde_sim.exe

set EMU_DIR=%SCRIPT_DIR%..\..\emulator\docking-twin-emulator
if not exist "%EMU_DIR%\build\HardwareEmulator.exe" (
    set EMU_DIR=%USERPROFILE%\Desktop\emulator\docking-twin-emulator
)
set EMU_EXE=%EMU_DIR%\build\HardwareEmulator.exe

if not exist "%EMU_EXE%" (
    echo [ERROR] Hardware Emulator not found at:
    echo   %EMU_EXE%
    echo Please verify the emulator repo is built.
    pause
    exit /b 1
)

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

tasklist /FI "IMAGENAME eq HardwareEmulator.exe" 2>nul | find /I "HardwareEmulator.exe" >nul
if %errorlevel%==0 (
    echo [INFO] Closing existing HardwareEmulator.exe to avoid duplicate UDP :5006 binding...
    taskkill /IM HardwareEmulator.exe /F >nul 2>&1
    timeout /t 1 /nobreak >nul
)

echo [1/3] Launching VDE Hardware Emulator (500 Hz, RX :5006 / TX :5005)...
start "VDE Hardware Emulator" cmd /c "cd /d "%EMU_DIR%" && "%EMU_EXE%""

echo Waiting for emulator UDP socket initialization...
timeout /t 2 /nobreak > nul

echo [2/3] Launching Dynamics Engine (1000 Hz Physics + HIL UDP)...
start "Space VDE - Physics Engine" cmd /c "cd /d "%ENGINE_DIR%" && "%ENGINE_EXE%" config\simulation_config.xml"

echo Waiting for physics engine socket initialization...
timeout /t 2 /nobreak > nul

echo [3/3] Launching 3D UI Visualizer (120 FPS OpenGL Client)...
start "Space VDE - 3D Visualizer" cmd /c "cd /d "%UI_DIR%" && "%UI_EXE%""

echo.
echo =========================================================================
echo All 3 applications are running!
echo   1. VDE Hardware Emulator  (RX :5006, TX :5005)
echo   2. Space VDE Physics      (1000 Hz HIL loop)
echo   3. Space VDE 3D UI        (120 FPS OpenGL)
echo.
echo Tip: Chaser starts at -550 mm and approaches at 50 mm/s.
echo      Docking contact + load-cell rebound begins at ~T = 11 seconds!
echo      Close the windows or press Ctrl+C in Physics Engine to stop.
echo =========================================================================

