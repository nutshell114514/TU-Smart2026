@echo off
setlocal
cd /d "%~dp0"
set "MPY_CROSS=%~dp0tools\mpy-cross-v1.20.0\extracted\mpy_cross-1.20.0.data\purelib\mpy_cross\mpy-cross.exe"

call :build_role follower 0 task_follow.py
if errorlevel 1 goto fail

call :build_role leader 1 task_leader.py
if errorlevel 1 goto fail

echo Build complete.
exit /b 0

:build_role
set "OUT_DIR=%~1"
set "ROLE_VALUE=%~2"
set "TASK_SRC=%~3"

if exist task.py (
    echo task.py already exists. Remove it before building.
    exit /b 1
)
if not exist "%TASK_SRC%" (
    echo Missing %TASK_SRC%.
    exit /b 1
)

python -c "from pathlib import Path; p=Path('config.py'); s=p.read_text(encoding='utf-8'); s=s.replace('_IS_LEADER = 0', '_IS_LEADER = %ROLE_VALUE%').replace('_IS_LEADER = 1', '_IS_LEADER = %ROLE_VALUE%'); p.write_text(s, encoding='utf-8')"
if errorlevel 1 exit /b 1

rem 不删除目录本身（目录被资源管理器/终端占用时 rmdir 会失败并留下空目录），只清空内容。
if not exist "%OUT_DIR%" mkdir "%OUT_DIR%"
if errorlevel 1 (
    echo [%OUT_DIR%] mkdir failed.
    exit /b 1
)
del /q "%OUT_DIR%\*" 2>nul

ren "%TASK_SRC%" task.py
if errorlevel 1 exit /b 1

"%MPY_CROSS%" -O3 -march=armv7emdp -o "%OUT_DIR%\base.mpy" base.py
if errorlevel 1 (
    echo [%OUT_DIR%] mpy-cross base.py failed.
    goto restore_task
)

"%MPY_CROSS%" -O3 -march=armv7emdp -o "%OUT_DIR%\task.mpy" task.py
if errorlevel 1 (
    echo [%OUT_DIR%] mpy-cross task.py failed.
    goto restore_task
)

copy /y main.py "%OUT_DIR%\main.py" >nul
if errorlevel 1 goto restore_task

"%MPY_CROSS%" -O3 -march=armv7emdp -o "%OUT_DIR%\config.mpy" config.py
if errorlevel 1 (
    echo [%OUT_DIR%] mpy-cross config.py failed.
    goto restore_task
)

"%MPY_CROSS%" -O3 -march=armv7emdp -o "%OUT_DIR%\debug_switch.mpy" debug_switch.py
if errorlevel 1 (
    echo [%OUT_DIR%] mpy-cross debug_switch.py failed.
    goto restore_task
)

ren task.py "%TASK_SRC%"
if errorlevel 1 exit /b 1
exit /b 0

:restore_task
ren task.py "%TASK_SRC%"
if errorlevel 1 exit /b 1
exit /b 1

:fail
echo Build failed.
exit /b 1
