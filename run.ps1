param (
    [string]$Action = "start"
)

$ServerDir = Join-Path $PSScriptRoot "..\espcam-tankcam\server"
$FrontendDir = Join-Path $PSScriptRoot "..\espcam-tankcam\frontend"

if ($Action -eq "start") {
    Write-Host "==========================================" -ForegroundColor Cyan
    Write-Host "Starting ESP32-CAM Tank Cockpit Services" -ForegroundColor Cyan
    Write-Host "==========================================" -ForegroundColor Cyan
    
    Write-Host "Starting Backend Server..." -ForegroundColor Green
    Start-Process cmd -ArgumentList "/k npm start" -WorkingDirectory $ServerDir
    
    Write-Host "Starting Frontend Cockpit..." -ForegroundColor Green
    Start-Process cmd -ArgumentList "/k npm run dev" -WorkingDirectory $FrontendDir
    
    Write-Host "`nServices started in separate terminal windows." -ForegroundColor Yellow
    Write-Host "- Cockpit URL: http://localhost:5173" -ForegroundColor Yellow
    Write-Host "- Close the command windows to stop the servers," -ForegroundColor Yellow
    Write-Host "  or run: .\run.ps1 stop" -ForegroundColor Yellow
    Write-Host "==========================================" -ForegroundColor Cyan
}
elseif ($Action -eq "stop") {
    Write-Host "==========================================" -ForegroundColor Red
    Write-Host "Stopping Tank Cockpit Services..." -ForegroundColor Red
    Write-Host "==========================================" -ForegroundColor Red
    
    taskkill /FI "WINDOWTITLE eq Tankcam Backend*" /T /F > $null 2>&1
    taskkill /FI "WINDOWTITLE eq Tankcam Frontend*" /T /F > $null 2>&1
    
    Write-Host "Done. Services stopped." -ForegroundColor Green
    Write-Host "==========================================" -ForegroundColor Red
}
