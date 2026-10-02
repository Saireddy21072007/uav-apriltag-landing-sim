@echo off
REM ===================================================================
REM   run2.bat  -  open the drone simulation. That is all you need.
REM
REM   Double-click this file. NetLogo opens with the model loaded and
REM   the world already built. Then:
REM
REM       press  GO            to start flying
REM       press  any number    to send the drone to that address
REM
REM   The right-hand panel is the drone's own camera, ray cast pixel by
REM   pixel through the same lens model the perception code uses.
REM ===================================================================
title Drone AirTag Landing - NetLogo
setlocal enabledelayedexpansion
cd /d "%~dp0"

echo.
echo   ==============================================================
echo     Vision-guided autonomous landing on AirTag addresses
echo     NetLogo simulation
echo   ==============================================================
echo.

if not defined NETLOGO_HOME set "NETLOGO_HOME=%LOCALAPPDATA%\NetLogo\NetLogo-6.4.0-64"

if not exist "%NETLOGO_HOME%\lib\app\netlogo-6.4.0.jar" (
    echo   [X] NetLogo 6.4.0 was not found at:
    echo         %NETLOGO_HOME%
    echo.
    echo   Get NetLogo-6.4.0-64.tgz from
    echo     https://github.com/NetLogo/NetLogo/releases/tag/v6.4.0
    echo   extract it, then either put it at the path above or set
    echo   NETLOGO_HOME to wherever you extracted it.
    echo.
    pause
    exit /b 1
)
echo   [ok] NetLogo found

set "JAVAW="
if defined JAVA_HOME if exist "%JAVA_HOME%\bin\javaw.exe" set "JAVAW=%JAVA_HOME%\bin\javaw.exe"
if not defined JAVAW if exist "C:\Program Files\Eclipse Adoptium\jdk-17.0.13.11-hotspot\bin\javaw.exe" set "JAVAW=C:\Program Files\Eclipse Adoptium\jdk-17.0.13.11-hotspot\bin\javaw.exe"
if not defined JAVAW for /f "delims=" %%j in ('where javaw 2^>nul') do if not defined JAVAW set "JAVAW=%%j"
if not defined JAVAW (
    echo   [X] No Java found. NetLogo 6.4 needs Java 17 or newer.
    echo       Get it from https://adoptium.net
    echo.
    pause
    exit /b 1
)
echo   [ok] Java found

if not exist "drone_airtag_landing.nlogo" (
    echo   [X] drone_airtag_landing.nlogo is missing from this folder.
    echo       Rebuild it with:   python build_nlogo.py
    echo.
    pause
    exit /b 1
)
echo   [ok] model found
echo.

set "OPTS=-Xmx2g -Dfile.encoding=UTF-8"
set "OPTS=%OPTS% -Dnetlogo.extensions.dir=%NETLOGO_HOME%\extensions"
set "OPTS=%OPTS% -Dnetlogo.models.dir=%NETLOGO_HOME%\models"
set "OPTS=%OPTS% --add-exports=java.base/java.lang=ALL-UNNAMED"
set "OPTS=%OPTS% --add-exports=java.desktop/sun.awt=ALL-UNNAMED"
set "OPTS=%OPTS% --add-exports=java.desktop/sun.java2d=ALL-UNNAMED"

echo   Opening NetLogo. The window takes 10-30 seconds the first time.
echo.
echo   When it appears:
echo      1.  press  GO
echo      2.  press any address number  (1 to 10)
echo.
echo   Orange = street    Blue = vehicle    Green = roof
echo.

start "NetLogo" "%JAVAW%" %OPTS% -classpath "%NETLOGO_HOME%\lib\app\*" org.nlogo.app.App --open "%~dp0drone_airtag_landing.nlogo"

ping -n 11 127.0.0.1 >nul 2>&1
tasklist /fi "imagename eq javaw.exe" | find /i "javaw.exe" >nul
if errorlevel 1 (
    echo   [X] NetLogo did not start. Run OPEN_NETLOGO.bat to see the error.
    echo.
    pause
) else (
    echo   [ok] NetLogo is open. You can close this window.
    ping -n 9 127.0.0.1 >nul 2>&1
)
endlocal
