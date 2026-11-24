@echo off
REM Doble clic: modelo de tierra PID (solo explorador) + estacion web (UI) + quarc_run del dron 1 + camara.
REM Pregunta antes de lanzar el dron y la camara. Ver laboratorio\README.md
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0iniciar_lab.ps1" -Modo solo
