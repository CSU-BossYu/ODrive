@echo off
REM ==========================================================================
REM FOC upper-computer launcher.
REM
REM Usage:
REM   run.bat           start backend in production mode (serves built dist)
REM   run.bat dev       start backend + Vite hot-reload frontend (development)
REM   run.bat build     rebuild frontend then start backend
REM
REM Prerequisites:
REM   - Python 3.10+ installed
REM   - Node.js 20+ installed (only needed for dev/build)
REM   - backend\.venv is auto-created on first run
REM ==========================================================================

setlocal
set "ROOT_DIR=%~dp0"
set "BACKEND_DIR=%ROOT_DIR%backend"
set "FRONTEND_DIR=%ROOT_DIR%frontend"
set "VENV_PY=%BACKEND_DIR%\.venv\Scripts\python.exe"
set "NODE_DIR=C:\Program Files\nodejs"

cd /d "%BACKEND_DIR%"

REM --- ensure backend venv exists ---
if not exist "%VENV_PY%" (
    echo [setup] creating backend venv...
    python -m venv "%BACKEND_DIR%\.venv"
    "%VENV_PY%" -m pip install --quiet --upgrade pip
    "%VENV_PY%" -m pip install -r "%BACKEND_DIR%\requirements.txt"
    echo [setup] backend deps installed.
)

REM --- dispatch on first argument ---
set "MODE=%~1"
if "%MODE%"=="" set "MODE=prod"
if /i "%MODE%"=="dev"   goto dev
if /i "%MODE%"=="build" goto build
goto prod

:build
echo [build] building frontend...
if not exist "%FRONTEND_DIR%\node_modules" (
    call "%NODE_DIR%\npm.cmd" --prefix "%FRONTEND_DIR%" install
)
call "%NODE_DIR%\npm.cmd" --prefix "%FRONTEND_DIR%" run build
if errorlevel 1 (
    echo [build] FAILED
    exit /b 1
)
echo [build] OK, frontend dist updated.
goto prod

:dev
echo [dev] starting backend on :8000 and Vite on :5173 ...
echo [dev] open http://127.0.0.1:5173 in your browser.
if not exist "%FRONTEND_DIR%\node_modules" (
    call "%NODE_DIR%\npm.cmd" --prefix "%FRONTEND_DIR%" install
)
start "FOC vite" "%NODE_DIR%\npm.cmd" --prefix "%FRONTEND_DIR%" run dev
"%VENV_PY%" -m odrive_can.app
goto end

:prod
echo [prod] serving built frontend from backend on http://127.0.0.1:8000
"%VENV_PY%" -m odrive_can.app
goto end

:end
endlocal
