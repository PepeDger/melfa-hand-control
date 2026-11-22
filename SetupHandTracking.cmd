@echo off
setlocal
cd /d "%~dp0"
rem Separate optional environment; the standard panel needs no extra packages.
if exist ".venv_hand\Scripts\python.exe" goto install
py -3.11 -m venv .venv_hand
if not errorlevel 1 goto install
if exist "%LocalAppData%\Programs\Python\Python311\python.exe" (
  "%LocalAppData%\Programs\Python\Python311\python.exe" -m venv .venv_hand
  if not errorlevel 1 goto install
)
echo Install Python 3.11 and run this file again.
pause
exit /b 1
:install
".venv_hand\Scripts\python.exe" -m pip install -r requirements-hand.txt
if errorlevel 1 goto failed
".venv_hand\Scripts\python.exe" hand_camera.py --check-model
if errorlevel 1 goto failed
echo Ready. Start the web panel, choose hand mode, then Open camera.
pause
exit /b 0
:failed
echo Setup failed. Read the error above and README_HAND.md.
pause
exit /b 1
