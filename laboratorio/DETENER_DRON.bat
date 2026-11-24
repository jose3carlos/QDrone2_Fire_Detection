@echo off
REM Doble clic: detiene el stack del dron explorador (quarc_run -q). SOLO con el dron en el piso.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0iniciar_lab.ps1" -Modo solo -Detener
