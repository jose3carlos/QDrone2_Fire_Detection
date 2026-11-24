@echo off
REM Doble clic: modo DUAL (2 drones). AUN NO VOLADO: falta el rigid body real del dron 2 en el .motive
REM y red.ip_dron2 en config\estacion_dual.json. Ver laboratorio\README.md
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0iniciar_lab.ps1" -Modo dual
