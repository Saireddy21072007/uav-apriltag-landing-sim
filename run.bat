@echo off
REM ===================================================================
REM  Vision-guided autonomous landing on addressed AirTag targets
REM
REM    run.bat                 launch the simulation straight away
REM    run.bat 5               start on address 5
REM    run.bat paper           fly with the base paper's PID alone
REM    run.bat record 3        record address 3 to an animated GIF
REM    run.bat evaluate        run every experiment and rebuild figures
REM    run.bat world           regenerate the pad textures and MuJoCo world
REM    run.bat doc             rebuild both review documents
REM    run.bat check           verify the environment without flying
REM    run.bat analyse         controllability, observability, lerp/slerp
REM    run.bat install         install the Python packages this needs
REM ===================================================================

setlocal enabledelayedexpansion
pushd "%~dp0"

REM --- never run stale bytecode -------------------------------------
REM Python validates a cached .pyc against the source's SIZE and its
REM mtime in whole SECONDS. Editing 0.40 to 0.45 in config.py changes
REM neither, so an edit made in the same second as the last run is
REM silently ignored: the world is rebuilt from the new number while the
REM code still runs the old one. Cheaper to never keep bytecode at all.
set "PYTHONDONTWRITEBYTECODE=1"
for /d /r "%~dp0drone_sim" %%d in (__pycache__) do @if exist "%%d" rd /s /q "%%d"

REM --- pause at the end only when double-clicked from Explorer -------
set "FROM_EXPLORER="
echo %cmdcmdline% | find /i "%~nx0" >nul 2>&1 && set "FROM_EXPLORER=1"

REM --- find Python --------------------------------------------------
set "PY="
where py >nul 2>&1 && set "PY=py -3"
if not defined PY (
    where python >nul 2>&1 && set "PY=python"
)
if not defined PY (
    echo.
    echo   Python was not found on this machine.
    echo   Install Python 3.11 or newer from https://www.python.org/downloads/
    echo   and tick "Add python.exe to PATH" during setup.
    goto :finish
)

REM --- dispatch -----------------------------------------------------
set "CMD=%~1"

if /i "%CMD%"=="install"  goto :install
if /i "%CMD%"=="check"    goto :check
if /i "%CMD%"=="analyse"  goto :analyse
if /i "%CMD%"=="analyze"  goto :analyse
if /i "%CMD%"=="world"    goto :world
if /i "%CMD%"=="doc"      goto :doc
if /i "%CMD%"=="evaluate" goto :evaluate
if /i "%CMD%"=="record"   goto :record
if /i "%CMD%"=="paper"    goto :paper
if /i "%CMD%"=="help"     goto :help
if /i "%CMD%"=="-h"       goto :help
if /i "%CMD%"=="/?"       goto :help

REM anything else is treated as a list of addresses for the demo
goto :demo


REM ==================================================================
:deps
REM Returns errorlevel 1 when a required package is missing.
%PY% -c "import mujoco, cv2, numpy, pupil_apriltags, matplotlib" >nul 2>&1
exit /b %errorlevel%


:assets
if exist "drone_sim\world\assets\world.xml" exit /b 0
echo   Building the MuJoCo world and the pad textures for the first time...
%PY% "drone_sim\world\build_world.py"
exit /b %errorlevel%


:ensure
call :deps
if errorlevel 1 (
    echo.
    echo   Some Python packages are missing.
    echo   Installing them now - this happens once and takes a couple of minutes.
    echo.
    call :install_quiet
    call :deps
    if errorlevel 1 (
        echo.
        echo   The install did not complete. Run:  run.bat install
        exit /b 1
    )
)
call :assets
exit /b %errorlevel%


:install_quiet
%PY% -m pip install --quiet --disable-pip-version-check mujoco pupil-apriltags opencv-python numpy matplotlib pillow
exit /b %errorlevel%


REM ==================================================================
:install
echo.
echo   Installing the Python packages the simulation needs...
%PY% -m pip install --upgrade --disable-pip-version-check mujoco pupil-apriltags opencv-python numpy matplotlib pillow
echo.
call :deps
if errorlevel 1 (echo   Something is still missing.) else (echo   All packages present.)
goto :finish


:check
echo.
echo   Checking the environment...
%PY% --version
call :deps
if errorlevel 1 (
    echo   MISSING PACKAGES - run:  run.bat install
    goto :finish
)
echo   Python packages OK
call :assets
if errorlevel 1 goto :finish
REM No percent signs in the Python below: cmd.exe eats them, and a mangled
REM format string turns a health check into a confusing traceback.
%PY% "drone_sim\selftest.py"
if errorlevel 1 goto :finish
echo.
echo   Ready. Run:  run.bat        to fly.
goto :finish


:analyse
call :deps
if errorlevel 1 (echo   Run:  run.bat install & goto :finish)
%PY% "drone_sim\analysis.py"
goto :finish


:world
call :deps
if errorlevel 1 (echo   Run:  run.bat install & goto :finish)
%PY% "drone_sim\world\build_world.py"
goto :finish


:doc
where node >nul 2>&1
if errorlevel 1 (
    echo   Node.js is not installed, so the document cannot be rebuilt.
    echo   The simulation itself does not need it.
    goto :finish
)
node "docs\make_docx.js"
node "docs\make_intro_method_docx.js"
goto :finish


:evaluate
call :ensure
if errorlevel 1 goto :finish
echo.
echo   Running every experiment. This takes roughly half an hour: every camera
echo   frame is really rendered and every marker really decoded.
echo.
%PY% "drone_sim\evaluate.py"
goto :finish


:record
call :ensure
if errorlevel 1 goto :finish
set "ADDR=%~2"
if not defined ADDR set "ADDR=3"
echo.
echo   Recording address %ADDR% to an animated GIF...
%PY% "drone_sim\record.py" %ADDR%
goto :finish


:paper
call :ensure
if errorlevel 1 goto :finish
shift
echo.
echo   Flying with the base paper's fixed-gain PID only - no pad-velocity
echo   feed-forward, no gain scheduling. Watch it lag behind a moving pad.
echo.
%PY% "drone_sim\panel.py" %1 --paper
goto :finish


:demo
call :ensure
if errorlevel 1 goto :finish
echo.
echo   ==============================================================
echo     AirTag-addressed autonomous landing
echo     One window opens: the world, the drone camera, and ten numbered
echo     landing sites. Press a number - or click it - and the drone flies
echo     there and lands. Works mid-flight too.
echo     [space] pause   [r] restart   [a][d] orbit   [w][s] zoom   [q] quit
echo   ==============================================================
%PY% "drone_sim\panel.py" %*
goto :finish


:help
echo.
echo   run.bat                 launch the simulation straight away
echo   run.bat 5               start on address 5
echo   run.bat paper 5         start on 5 with the base paper's PID alone
echo   run.bat record 3        record address 3 to an animated GIF
echo   run.bat evaluate        run every experiment and rebuild figures
echo   run.bat world           regenerate the pad textures and MuJoCo world
echo   run.bat doc             rebuild both review documents
echo   run.bat check           verify the environment without flying
echo   run.bat analyse         controllability, observability, lerp/slerp
echo   run.bat install         install the Python packages this needs
goto :finish


:finish
popd
if defined FROM_EXPLORER (
    echo.
    pause
)
endlocal
