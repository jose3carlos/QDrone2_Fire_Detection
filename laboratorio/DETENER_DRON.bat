@echo off
REM Doble clic: detiene el stack del dron explorador del modo solo (quarc_run -q). SOLO con el dron en el piso.
REM En modo dual usa DETENER_DUAL.bat (detiene los dos).
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0iniciar_lab.ps1" -Modo solo -Detener
