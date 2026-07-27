@echo off
REM First-time install for SO-ARM101 Web Controller (Windows).
REM
REM Usage: scripts\install.bat
REM
REM Creates .\venv and installs Python deps locally. Nothing is
REM installed globally.

setlocal
cd /d "%~dp0.."

where python >nul 2>nul
if errorlevel 1 (
  echo Python 3 is required but not found in PATH.
  echo Install Python 3.10+ from https://www.python.org/downloads/
  exit /b 1
)

echo [install] creating venv\
python -m venv venv
if errorlevel 1 exit /b 1

echo [install] upgrading pip
call "venv\Scripts\python.exe" -m pip install --upgrade pip
if errorlevel 1 exit /b 1

echo [install] installing requirements
call "venv\Scripts\python.exe" -m pip install -r requirements.txt
if errorlevel 1 exit /b 1

echo.
echo [install] done. Run run.bat to start the controller.
exit /b 0
