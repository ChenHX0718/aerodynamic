@echo off
setlocal
set "PROJECT_ROOT=%~dp0"
set "PYTHON_EXE=%PROJECT_ROOT%.venv\Scripts\python.exe"

if not exist "%PYTHON_EXE%" (
    echo [ERROR] Virtual environment not found.
    echo Run setup.bat first.
    pause
    exit /b 1
)

"%PYTHON_EXE%" -u "%PROJECT_ROOT%run.py" check
if errorlevel 1 (
    echo.
    echo [ERROR] Environment check failed. See the messages above.
    pause
    exit /b 1
)

if not exist "%PROJECT_ROOT%results\numerical_convergence\production_numerical_settings.yaml" (
    echo.
    echo [ERROR] Production Numerical Settings not found.
    echo Run: .venv\Scripts\python.exe run.py numerical-convergence
    pause
    exit /b 1
)

"%PYTHON_EXE%" -u "%PROJECT_ROOT%run.py" all --config "%PROJECT_ROOT%config\aircraft.yaml"
set "RUN_STATUS=%ERRORLEVEL%"
echo.
if "%RUN_STATUS%"=="0" (
    echo Results: %PROJECT_ROOT%results\latest
    echo MATLAB: %PROJECT_ROOT%results\autotune\aircraft_aero.mat
) else (
    echo [ERROR] Aero database build failed. Open results\latest\run_summary.txt for details.
)
pause
exit /b %RUN_STATUS%
