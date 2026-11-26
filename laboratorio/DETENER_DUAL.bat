@echo off
REM Doble clic: detiene el stack de los DOS drones (quarc_run -q). SOLO con los dos en el piso.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0iniciar_lab.ps1" -Modo dual -Detener
