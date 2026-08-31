@echo off
rem Run a camera test step from this folder.
rem Usage:  run.bat step1          (steps: step1 .. step6)
rem         run.bat step3 640 480  (extra args go to the script)
setlocal enabledelayedexpansion
if "%~1"=="" (
  echo Usage: run.bat step1 [step2..step6]
  pause
  exit /b 1
)
set "TARGET="
for %%f in ("%~dp0%~1_*.ps1") do set "TARGET=%%f"
if not defined TARGET (
  echo No script starting with "%~1" found in %~dp0
  pause
  exit /b 1
)
powershell -NoProfile -ExecutionPolicy Bypass -File "!TARGET!" %2 %3 %4 %5
echo.
pause
