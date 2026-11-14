@echo off
setlocal
cd /d "%~dp0"
if exist "Releases\20261111\Studio2416\Studio2416.exe" (
    start "" "Releases\20261111\Studio2416\Studio2416.exe" %*
    exit /b 0
)
if exist "App\Studio2416\Studio2416.exe" (
    start "" "App\Studio2416\Studio2416.exe" %*
    exit /b 0
)
where python >nul 2>nul
if errorlevel 1 (
    echo Python or packaged application not found.
    pause
    exit /b 1
)
python -c "import PySide6.QtWebEngineWidgets, serial" >nul 2>nul
if errorlevel 1 (
    python -m pip install -r requirements.txt
    if errorlevel 1 (
        pause
        exit /b 1
    )
)
python desktop_launcher.py %*
if errorlevel 1 pause