@echo off
REM SO-ARM101 Controller launcher (Windows).
REM
REM Usage:
REM   run.bat                    -- default, http://127.0.0.1:8000
REM   run.bat --port 9000
REM   run.bat --serial COM4

setlocal
cd /d "%~dp0"

if not exist "venv\Scripts\python.exe" (
  echo venv not found -- running scripts\install.bat first
  call scripts\install.bat
  if errorlevel 1 goto :error
)

call "venv\Scripts\activate.bat"
python app.py %*
exit /b %errorlevel%

:error
exit /b 1
