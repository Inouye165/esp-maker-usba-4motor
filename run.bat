@echo off
:: Check if start or stop is requested
if "%1"=="stop" goto stop
if "%1"=="start" goto start
if "%1"=="" goto start

:start
echo ==========================================
echo Starting ESP32-CAM Tank Cockpit Services
echo ==========================================

echo Starting Backend Server (Port 5000)...
start "Tankcam Backend" /D "%~dp0..\espcam-tankcam\server" cmd /k npm start

echo Starting Frontend Cockpit (Port 5173)...
start "Tankcam Frontend" /D "%~dp0..\espcam-tankcam\frontend" cmd /k npm run dev

echo.
echo Services started in separate terminal windows.
echo - Access Cockpit: http://localhost:5173
echo - Close the spawned command windows to stop the servers,
echo   or run: %0 stop
echo ==========================================
goto end

:stop
echo ==========================================
echo Stopping Tank Cockpit Services...
echo ==========================================
taskkill /FI "WINDOWTITLE eq Tankcam Backend*" /T /F >nul 2>&1
taskkill /FI "WINDOWTITLE eq Tankcam Frontend*" /T /F >nul 2>&1
echo Done. Services stopped.
echo ==========================================
goto end

:end
