@echo off
REM QDrone2 Fire Detection - interfaz web (sin MATLAB)
REM Primera vez: crea .venv e instala dependencias. Luego solo arranca el servidor.
setlocal
cd /d "%~dp0"

if exist ".venv\Scripts\python.exe" goto run

set "PYEXE="
where py >nul 2>nul && (py -3.11 -c "import sys" >nul 2>nul && set "PYEXE=py -3.11")
if not defined PYEXE (where py >nul 2>nul && set "PYEXE=py -3")
if not defined PYEXE (python -c "import sys; assert sys.version_info>=(3,10)" >nul 2>nul && set "PYEXE=python")
if not defined PYEXE (
  echo No se encontro Python 3.10 o superior. Instalalo desde https://www.python.org/downloads/
  echo ^(recomendado 3.11, marcando "Add python.exe to PATH"^) y vuelve a ejecutar este archivo.
  pause
  exit /b 1
)

echo Creando entorno virtual con %PYEXE% ...
%PYEXE% -m venv .venv || goto fail
".venv\Scripts\python.exe" -m pip install --upgrade pip || goto fail
".venv\Scripts\python.exe" -m pip install -r requirements.txt || goto fail

if defined QUARC_DIR (
  if exist "%QUARC_DIR%python" (
    echo Instalando el SDK de Quanser desde "%QUARC_DIR%python" ...
    ".venv\Scripts\python.exe" -m pip install --find-links "%QUARC_DIR%python" quanser-api || echo AVISO: no se pudo instalar quanser-api; solo funcionaran las fuentes synthetic/folder/opencv.
  )
) else (
  echo AVISO: QUARC no esta instalado ^(QUARC_DIR vacio^). Solo funcionaran las fuentes synthetic/folder/opencv.
)

:run
start "" http://127.0.0.1:8080
".venv\Scripts\python.exe" -m app.server %*
goto :eof

:fail
echo.
echo Fallo la instalacion. Revisa el mensaje de arriba.
pause
exit /b 1
