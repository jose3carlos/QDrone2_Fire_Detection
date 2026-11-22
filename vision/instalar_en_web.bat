@echo off
REM OPCIONAL: instala torch (CUDA 12.8) + ultralytics en el venv de la estacion web para que el detector
REM YOLO corra en la GPU. NO hace falta para detectar fuegos y tomas: sin torch, vision/detector.py usa
REM vision/modelos/mejor.onnx con OpenCV (CPU, ~30 ms por imagen), que ya trae el venv de la estacion.
REM
REM Uso:  vision\instalar_en_web.bat                 (venv por defecto %USERPROFILE%\.venvs\qd2_web)
REM       vision\instalar_en_web.bat C:\ruta\al\venv
REM Descarga ~2.5 GB la primera vez.
setlocal
set "VENV=%~1"
if "%VENV%"=="" set "VENV=%USERPROFILE%\.venvs\qd2_web"
set "PY=%VENV%\Scripts\python.exe"
if not exist "%PY%" (
  echo No existe %PY%. Corre primero web\iniciar_web.bat una vez para crear el venv de la estacion.
  exit /b 1
)
echo Instalando torch + ultralytics en %VENV% ...
"%PY%" -m pip install --extra-index-url https://download.pytorch.org/whl/cu128 torch==2.11.0+cu128 torchvision==0.26.0+cu128 ultralytics==8.4.216 || goto fail
"%PY%" -c "import torch, ultralytics; print('torch', torch.__version__, '| CUDA:', torch.cuda.is_available(), '| ultralytics', ultralytics.__version__)" || goto fail
echo Listo. La estacion usara vision\modelos\mejor.pt con ultralytics (GPU si hay CUDA).
exit /b 0
:fail
echo Fallo la instalacion. La estacion sigue funcionando con mejor.onnx (OpenCV, CPU).
exit /b 1
