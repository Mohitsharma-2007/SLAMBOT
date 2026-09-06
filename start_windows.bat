@echo off
title SLAM Bot — Windows Backend & WebApp Launcher
echo ==================================================
echo    SLAM BOT — Starting Windows Backend + WebApp
echo ==================================================

:: Free ports if in use
for /f "tokens=5" %%a in ('netstat -aon ^| findstr :8000 ^| findstr LISTENING') do taskkill /f /pid %%a 2>nul
for /f "tokens=5" %%a in ('netstat -aon ^| findstr :5173 ^| findstr LISTENING') do taskkill /f /pid %%a 2>nul

echo [1/2] Starting Python FastAPI Backend on port 8000...
start "SLAM Bot Backend" cmd /k "python backend\main.py"

timeout /t 2 /nobreak >nul

echo [2/2] Starting WebApp Dev Server on port 5173...
start "SLAM Bot WebApp" cmd /k "cd webapp && npm run dev"

echo.
echo ==================================================
echo SUCCESS: Windows Backend & WebApp launched!
echo Open http://localhost:5173 in your browser.
echo Next: Run 'bash start_wsl_ros.sh' in WSL to start ROS2.
echo ==================================================
