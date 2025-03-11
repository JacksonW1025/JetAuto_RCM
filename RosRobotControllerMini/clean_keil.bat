@echo off

REM 递归清除Keil编译产生的文件

REM 获取当前目录
set CURRENT_DIR=%CD%

REM Keil编译产生的文件
echo Deleting output files...
for /r "%CURRENT_DIR%" %%F in (*.o *.obj *.d *.hex *.bin *.axf *.map *.crf *.htm *.lnp *.iex *.dep *.sct *.uvguix.* *.lst) do (
    echo Deleting "%%F"...
    del /q "%%F"
)

echo Cleanup complete.
choice /t 1 /d y /n >nul