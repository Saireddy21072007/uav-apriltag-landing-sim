@echo off
REM ===================================================================
REM  Run the NetLogo model with no GUI and write results to CSV.
REM
REM    run_headless.bat                 the "validate" experiment
REM    run_headless.bat myexperiment    any other BehaviorSpace one
REM ===================================================================
setlocal
cd /d "%~dp0"

echo.
echo   Headless run of the AirTag landing model
echo   ---------------------------------------

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

set "JAVA="
if defined JAVA_HOME if exist "%JAVA_HOME%\bin\java.exe" set "JAVA=%JAVA_HOME%\bin\java.exe"
if not defined JAVA if exist "C:\Program Files\Eclipse Adoptium\jdk-17.0.13.11-hotspot\bin\java.exe" set "JAVA=C:\Program Files\Eclipse Adoptium\jdk-17.0.13.11-hotspot\bin\java.exe"
if not defined JAVA for /f "delims=" %%j in ('where java 2^>nul') do if not defined JAVA set "JAVA=%%j"

if not defined JAVA (
    echo.
    echo   [X] No Java found. NetLogo 6.4 needs Java 17 or newer.
    echo.
    pause
    exit /b 1
)

set "EXPERIMENT=%~1"
if "%EXPERIMENT%"=="" set "EXPERIMENT=validate"

echo   NetLogo    : %NETLOGO_HOME%
echo   experiment : %EXPERIMENT%
echo   output     : %~dp0netlogo_results.csv
echo.
echo   Flying six addresses. This takes a minute or two...
echo.

set "OPTS=-Dfile.encoding=UTF-8 -Dnetlogo.extensions.dir=%NETLOGO_HOME%\extensions"
set "OPTS=%OPTS% --add-exports=java.base/java.lang=ALL-UNNAMED"
set "OPTS=%OPTS% --add-exports=java.desktop/sun.awt=ALL-UNNAMED"
set "OPTS=%OPTS% --add-exports=java.desktop/sun.java2d=ALL-UNNAMED"

"%JAVA%" %OPTS% -classpath "%NETLOGO_HOME%\lib\app\netlogo-6.4.0.jar" org.nlogo.headless.Main --model "%~dp0drone_airtag_landing.nlogo" --experiment %EXPERIMENT% --table "%~dp0netlogo_results.csv"

if errorlevel 1 (
    echo.
    echo   [X] the run failed - the error is above.
) else (
    echo.
    echo   [ok] done. Results written to netlogo_results.csv
)
echo.
pause
endlocal
