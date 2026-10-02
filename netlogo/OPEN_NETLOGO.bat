@echo off
REM ===================================================================
REM  Double-click to open the drone model in NetLogo.
REM
REM  NetLogo lives in a hidden Windows folder (AppData), so this finds
REM  it for you. Nothing is installed system-wide: NetLogo is just an
REM  extracted folder, and its interface is pure Java, so it is started
REM  with the Java already on this machine.
REM ===================================================================
setlocal enabledelayedexpansion
cd /d "%~dp0"

echo.
echo   Opening the AirTag landing model in NetLogo
echo   -------------------------------------------

if not defined NETLOGO_HOME set "NETLOGO_HOME=%LOCALAPPDATA%\NetLogo\NetLogo-6.4.0-64"

if not exist "%NETLOGO_HOME%\lib\app\netlogo-6.4.0.jar" (
    echo.
    echo   [X] NetLogo was not found at:
    echo       %NETLOGO_HOME%
    echo.
    echo   Download NetLogo-6.4.0-64.tgz from
    echo     https://github.com/NetLogo/NetLogo/releases/tag/v6.4.0
    echo   extract it anywhere, then set NETLOGO_HOME to that folder.
    echo.
    pause
    exit /b 1
)
echo   [ok] NetLogo    : %NETLOGO_HOME%

set "JAVAW="
if defined JAVA_HOME if exist "%JAVA_HOME%\bin\javaw.exe" set "JAVAW=%JAVA_HOME%\bin\javaw.exe"
if not defined JAVAW if exist "C:\Program Files\Eclipse Adoptium\jdk-17.0.13.11-hotspot\bin\javaw.exe" set "JAVAW=C:\Program Files\Eclipse Adoptium\jdk-17.0.13.11-hotspot\bin\javaw.exe"
if not defined JAVAW for /f "delims=" %%j in ('where javaw 2^>nul') do if not defined JAVAW set "JAVAW=%%j"

if not defined JAVAW (
    echo.
    echo   [X] No Java found. NetLogo 6.4 needs Java 17 or newer.
    echo       Install Temurin 17 from https://adoptium.net
    echo.
    pause
    exit /b 1
)
echo   [ok] Java       : %JAVAW%

if not exist "%~dp0drone_airtag_landing.nlogo" (
    echo.
    echo   [X] drone_airtag_landing.nlogo is missing from this folder.
    echo       Rebuild it with:  python build_nlogo.py
    echo.
    pause
    exit /b 1
)
echo   [ok] model      : drone_airtag_landing.nlogo

set "OPTS=-Xmx2g -Dfile.encoding=UTF-8"
set "OPTS=%OPTS% -Dnetlogo.extensions.dir=%NETLOGO_HOME%\extensions"
set "OPTS=%OPTS% -Dnetlogo.models.dir=%NETLOGO_HOME%\models"
set "OPTS=%OPTS% --add-exports=java.base/java.lang=ALL-UNNAMED"
set "OPTS=%OPTS% --add-exports=java.desktop/sun.awt=ALL-UNNAMED"
set "OPTS=%OPTS% --add-exports=java.desktop/sun.java2d=ALL-UNNAMED"

echo.
echo   Starting NetLogo. The window takes 10-30 seconds to appear.
echo   You can close this box once it does.
echo.

start "NetLogo" "%JAVAW%" %OPTS% -classpath "%NETLOGO_HOME%\lib\app\*" org.nlogo.app.App --open "%~dp0drone_airtag_landing.nlogo"

ping -n 9 127.0.0.1 >nul 2>&1
tasklist /fi "imagename eq javaw.exe" | find /i "javaw.exe" >nul
if errorlevel 1 (
    echo   [X] NetLogo did not start. Run this to see the error:
    echo.
    echo       "%JAVAW:javaw.exe=java.exe%" %OPTS% -classpath "%NETLOGO_HOME%\lib\app\*" org.nlogo.app.App
    echo.
    pause
) else (
    echo   [ok] NetLogo is starting.
    echo.
    echo   In the window: press SETUP, then GO, then any address number.
    echo.
    ping -n 7 127.0.0.1 >nul 2>&1
)
endlocal
