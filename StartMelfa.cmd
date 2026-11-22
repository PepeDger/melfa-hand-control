@echo off
setlocal
cd /d "%~dp0"
if not exist "app.py" goto missing_files
if not exist "config.json" goto missing_files

rem Prefer the known installation if present; otherwise use the Windows launcher
rem or Python on PATH. Each candidate must actually run Python 3.9 or newer.
if exist "%LocalAppData%\Programs\Python\Python311\python.exe" (
  "%LocalAppData%\Programs\Python\Python311\python.exe" -c "import sys; sys.exit(sys.version_info < (3, 9))" >nul 2>&1
  if not errorlevel 1 goto local_python
)
py -3 -c "import sys; sys.exit(sys.version_info < (3, 9))" >nul 2>&1
if not errorlevel 1 goto py_launcher
python -c "import sys; sys.exit(sys.version_info < (3, 9))" >nul 2>&1
if not errorlevel 1 goto path_python

echo Python 3.9+ was not found.
echo Install Python from https://www.python.org/downloads/windows/
echo Enable "Add python.exe to PATH", then reopen this file.
echo Read README.md or README.txt for setup and connection instructions.
pause
exit /b 1

:local_python
"%LocalAppData%\Programs\Python\Python311\python.exe" app.py %*
goto finished

:py_launcher
py -3 app.py %*
goto finished

:path_python
python app.py %*
goto finished

:missing_files
echo Project files are missing. Extract the WHOLE ZIP archive first.
echo Start this file from the extracted MelfaControl folder.
pause
exit /b 1

:finished
echo.
echo The panel has stopped. Keep this window open while using the panel.
echo Startup errors, if any, are shown above. See README.md or README.txt.
pause
