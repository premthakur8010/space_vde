# Space VDE - PowerShell Simulation Launcher
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

$EngineDir = Join-Path $ScriptDir "space_vde"
$UiDir     = Join-Path $ScriptDir "ui-space-docking-system-refactored"

$EngineExe = Join-Path $EngineDir "build_win\physics_engine.exe"
$UiExe     = Join-Path $UiDir "build_win\vde_sim.exe"

Write-Host "=========================================================================" -ForegroundColor Cyan
Write-Host "              Space Docking System - Simulation Launcher                 " -ForegroundColor Cyan
Write-Host "=========================================================================" -ForegroundColor Cyan

if (-not (Test-Path $EngineExe)) {
    Write-Error "Physics Engine executable not found at: $EngineExe"
    exit 1
}

if (-not (Test-Path $UiExe)) {
    Write-Error "UI Visualizer executable not found at: $UiExe"
    exit 1
}

Write-Host "`n[1/2] Starting Dynamics Engine (1000 Hz, XML-configured UDP)..." -ForegroundColor Green
Start-Process -FilePath "cmd.exe" -ArgumentList "/c cd /d `"$EngineDir`" && `"$EngineExe`" config\simulation_config.xml"

Start-Sleep -Seconds 2

Write-Host "[2/2] Starting 3D UI Visualizer (120 FPS OpenGL Client)..." -ForegroundColor Green
Start-Process -FilePath "cmd.exe" -ArgumentList "/c cd /d `"$UiDir`" && `"$UiExe`""

Write-Host "`nBoth systems are running!" -ForegroundColor Cyan
Write-Host "  - Close the visualizer window or press Ctrl+C in the physics console to stop." -ForegroundColor Gray

