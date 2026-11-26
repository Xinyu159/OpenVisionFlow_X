@echo off
cd /d "%~dp0build\bin\Release"
if not exist ovf-studio.exe (
    echo [ERR] ovf-studio.exe not found - build first
    pause
    exit /b 1
)
start "" ovf-studio.exe
