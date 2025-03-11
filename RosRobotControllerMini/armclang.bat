@echo off

set "source_path=.\Middlewares\Third_Party\FreeRTOS\Source\portable\GCC"
set "destination_path=.\Middlewares\Third_Party\FreeRTOS\Source\portable\RVDS"

xcopy /E /I /Y "%source_path%" "%destination_path%"

choice /t 1 /d y /n >nul
