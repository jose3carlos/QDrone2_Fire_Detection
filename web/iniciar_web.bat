@echo off
REM Estacion web 2x QDrone 2 (deteccion de tomas clandestinas y fuegos)
REM Uso:  iniciar_web.bat                      (fuente segun config\estacion.json -> web.fuente)
REM       iniciar_web.bat --fuente mock        (simulacion interna, sin MATLAB)
REM       iniciar_web.bat --fuente simulink    (modelo de tierra o QD2_Sim_Dual en tcpip://127.0.0.1:18700)
REM       iniciar_web.bat --fuente sim3d       (simulador MATLAB 3D, mismo puerto 18700)
REM       iniciar_web.bat --fuente simulink --host-puente 192.168.2.11
REM                                            (modelo de tierra en OTRA PC del laboratorio: la estacion corre en
REM                                             esta laptop y se conecta a tcpip://192.168.2.11:18700. Solo con
REM                                             --fuente simulink; no se guarda en estacion.json. En esa PC hace
REM                                             falta una regla de firewall de entrada TCP 18700 para esta laptop)
REM El modo de mision (dual o solo explorador) sale de config\estacion.json -> mision.modo / mision.dron_solo
REM El entorno virtual se crea FUERA de OneDrive: %USERPROFILE%\.venvs\qd2_web
setlocal
cd /d "%~dp0"
set "VENV=%USERPROFILE%\.venvs\qd2_web"
set "PY=%VENV%\Scripts\python.exe"

if exist "%PY%" goto check

set "PYEXE="
where py >nul 2>nul && (py -3.11 -c "import sys" >nul 2>nul && set "PYEXE=py -3.11")
if not defined PYEXE (where py >nul 2>nul && (py -3.10 -c "import sys" >nul 2>nul && set "PYEXE=py -3.10"))
if not defined PYEXE (where py >nul 2>nul && set "PYEXE=py -3")
if not defined PYEXE (python -c "import sys; assert sys.version_info>=(3,10)" >nul 2>nul && set "PYEXE=python")
if not defined PYEXE (
  echo No se encontro Python 3.10 o superior. Instalalo desde https://www.python.org/downloads/
  echo ^(marca "Add python.exe to PATH"^) y vuelve a ejecutar este archivo.
  pause
  exit /b 1
)
echo Creando entorno virtual en %VENV% con %PYEXE% ...
if not exist "%USERPROFILE%\.venvs" mkdir "%USERPROFILE%\.venvs"
%PYEXE% -m venv "%VENV%" || goto fail

:check
"%PY%" -c "import fastapi, uvicorn, cv2, numpy, psutil, multipart" >nul 2>nul && goto run
echo Instalando dependencias (solo la primera vez) ...
"%PY%" -m pip install --upgrade pip || goto fail
"%PY%" -m pip install -r requirements.txt || goto fail

:run
REM QD2_NO_BROWSER=1 evita abrir el navegador. El puerto sale de red.puerto_web de config\estacion.json
set "PORT=8080"
for /f %%P in ('"%PY%" -m app.server --imprimir-puerto %* 2^>nul') do set "PORT=%%P"
if not defined QD2_NO_BROWSER start "" cmd /c "timeout /t 4 >nul & start http://127.0.0.1:%PORT%"
"%PY%" -m app.server %*
goto :eof

:fail
echo.
echo Fallo la instalacion. Revisa el mensaje de arriba.
pause
exit /b 1
