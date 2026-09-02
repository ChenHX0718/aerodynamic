@echo off
setlocal
set "PROJECT_ROOT=%~dp0"
set "VENV_DIR=%PROJECT_ROOT%.venv"
set "PYTHON_EXE=%VENV_DIR%\Scripts\python.exe"

echo ==========================================
echo OpenVSP Aero Tool Setup
echo ==========================================

where python >nul 2>nul
if errorlevel 1 (
    echo Python ................ FAIL
    echo Install 64-bit Python 3.11, then run setup.bat again.
    pause
    exit /b 1
)

python -c "import sys; raise SystemExit(0 if sys.version_info[:2] == (3, 11) else 1)"
if errorlevel 1 (
    echo Python 3.11 ........... FAIL
    echo OpenVSP 3.51.3 bindings in this package were verified with Python 3.11.
    pause
    exit /b 1
)
echo Python 3.11 ........... PASS

if not exist "%VENV_DIR%\" goto :create_venv
if not exist "%PYTHON_EXE%" goto :rebuild_venv
"%PYTHON_EXE%" --version >nul 2>nul
if errorlevel 1 goto :rebuild_venv
echo Existing environment ... PASS
goto :install

:rebuild_venv
echo Existing environment ... INVALID
echo Rebuilding virtual environment...
rmdir /s /q "%VENV_DIR%"
if exist "%VENV_DIR%\" goto :setup_failed

:create_venv
echo Creating virtual environment...
python -m venv "%VENV_DIR%"
if errorlevel 1 goto :setup_failed

:install

"%PYTHON_EXE%" -m pip install -r "%PROJECT_ROOT%requirements.txt"
if errorlevel 1 goto :setup_failed

"%PYTHON_EXE%" "%PROJECT_ROOT%run.py" check
if errorlevel 1 goto :setup_failed

echo.
echo SETUP COMPLETE
echo Double-click run_aero.bat or run: .venv\Scripts\python.exe run.py all
pause
exit /b 0

:setup_failed
echo.
echo SETUP FAILED
echo Review the error above, then run setup.bat again.
pause
exit /b 1
