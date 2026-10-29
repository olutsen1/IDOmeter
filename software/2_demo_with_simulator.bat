@echo off
cd /d "%~dp0"
chcp 65001 >nul
start "IDometer simulator" python simulator.py
timeout /t 1 >nul
python receiver.py --sim
pause
