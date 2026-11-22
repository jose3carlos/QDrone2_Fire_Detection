> **Nota (versión 2 drones):** esta guía describe la estación anterior de **un solo dron** (Trayectory_Model, misión en Python). La estación actual de 2 drones se explica en `web/README.md` y `docs/ARQUITECTURA.md`. Se conserva como referencia histórica (Parte A y el streamer QDF1).

# QDrone2 Fire Detection — guía paso a paso

Esta guía tiene dos partes:

- **Parte A**: cómo correr el proyecto original (MATLAB/Simulink) con el QDrone2.
- **Parte B**: la versión web en Python de esta carpeta, que no necesita MATLAB.

---

## 0. Qué hace realmente el proyecto (léelo antes de volar)

`Trayectory_Model.slx` tiene esta cadena:

```
Waypoints ─► Trayectory SM ─► PoseD (Display)          ◄── Position Simulation = [0;0;1;0] (Constant)
                  │ startCamera
                  ▼
From Video Device (webcam "HD Webcam", 1280x720) ─► Video 3D Capture (640x480)
                  ─► Image Batch Capture (25 fotos en 3 s, 180x180 gris)
                  ─► FOMO Inference (DLL Edge Impulse; mayoría de 25) ─► ObjectDetected / Confidence
```

Hay cuatro puntos que importan para el QDrone2:

1. **La posición es una constante.** `Position Simulation` vale siempre `[0;0;1;0]`. El
   primer waypoint es (-1.5, -1.5), así que la FSM se queda en `MOVE` para siempre.
   Necesitas meterle la **pose real** del dron (OptiTrack / pila de vuelo de Quanser).
2. **El setpoint no va al dron.** `PoseD` termina en un bloque Display. Hay que enviarlo
   al controlador de posición del QDrone2.
3. **La cámara es una webcam de la PC** (`winvideo 1 (HD Webcam)`), no la del QDrone2.
4. **La inferencia es un DLL de Windows** que corre en simulación *Normal* en la PC. No se
   puede compilar con QUARC para el target `linux_qdrone2` (el propio README lo indica).
   Por eso el modelo corre en la PC y se comunica con el dron por red.

También: en `main` el `.slx` está guardado con **R2023b Update 10**, y **R2023a** no lo puede
abrir. En la rama `Quique` ya viene guardado en R2023a, y `rebuild_trayectory_model` lo
reconstruye en tu versión (instrucciones en `LEEME_R2023a.md`, en la raíz del proyecto).

Y un quinto punto: en el original, el `Trigger` de *Image Batch Capture* es una constante
`false` y `startCamera` no está conectado, así que la captura nunca se dispara. Con
`rebuild_trayectory_model('ConnectTrigger',true)` quedan conectados.

---

## Parte A — Proyecto original con MATLAB + QDrone2

### A1. Instalar (una sola vez)

1. MATLAB **R2023b**, o **R2023a** usando `rebuild_trayectory_model`. Necesitas Simulink,
   Image Acquisition Toolbox (y su support package *OS Generic Video Interface*) y
   Computer Vision Toolbox.
2. Compilador: *Home → Add-Ons →* **MATLAB Support for MinGW-w64 C/C++ Compiler**.
3. **QUARC** para R2023b, con licencia, y la red del QDrone2 configurada según el manual de
   Quanser. Pon la PC en la red del router del QDrone2; la IP del dron suele ser
   `192.168.2.x`.
4. Descomprime el proyecto en una ruta corta **sin espacios**, por ejemplo `C:\QDrone2_Fire_Detection`.

### A2. Compilar y probar el detector (MATLAB)

**Atajo (rama `Quique`):** en la carpeta del proyecto, `instalar` hace A2 y A3 de una vez:
selecciona MinGW, consigue CMake, compila, prueba y enlaza el modelo.

En la *Command Window*, con *Current Folder* = la carpeta del proyecto:

```matlab
setup_project                         % configura compilador, compila el DLL y la MEX
info = load_ei_model(); disp(info.labels)        % -> Fire
result = run_ei_inference(zeros(180,180,'uint8')) % status 2 = sin detección (correcto)
result = run_ei_inference('C:\imagenes\fuego.jpg')
close_ei_model
```

La primera compilación tarda varios minutos. Si falla, revisa `docs/TROUBLESHOOTING.md`.

### A3. Preparar el modelo Simulink

```matlab
setup_custom_model('Trayectory_Model')   % re-enlaza el DLL de ESTA computadora
open_system('Trayectory_Model')
```

En **R2023a** puedes usar el modelo de la rama `Quique`, que ya está guardado en R2023a, o
regenerarlo con `rebuild_trayectory_model('ConnectTrigger',true)`. Esa función reconstruye el
modelo y ya corre `setup_custom_model` al final.

En R2023a necesitas además **MinGW64** como compilador de `mex` (con Visual C++ el SDK no
compila) y un CMake 3.20 o más nuevo. `config.m` toma solo el de Visual Studio 2022
Community si lo tienes. Los detalles están en `LEEME_R2023a.md`.

El modelo trae rutas absolutas de la computadora original
(`C:\Ago-Dic26\VSCode\...\libei_inference_eb16cb243767a9c4559b.dll.a`).
`setup_custom_model` las reemplaza por las tuyas. Corre ese comando **siempre** que copies
el proyecto a otra máquina o cambies el ZIP del modelo.

### A4. Conectar el modelo al QDrone2

Estos bloques los aporta tu instalación de QUARC / QDrone2 Research Studio, no este proyecto:

| Qué | Bloque actual | Reemplázalo por |
|---|---|---|
| Pose medida `[X;Y;Z;Yaw]` | `Position Simulation` (Constant) | La pose real: un *Stream Client* de QUARC hacia tu modelo de localización/OptiTrack, o el mismo bloque que usa tu modelo de vuelo de Quanser. |
| Setpoint al dron | `PoseD` (Display) | Un *Stream Server*/*Client* de QUARC hacia el modelo de vuelo (commander) del QDrone2 que acepta `[X;Y;Z;Yaw]`. |
| Cámara | `From Video Device` (webcam) | La cámara que veas desde la PC. Abre el bloque y selecciona dispositivo y formato. Si es la del QDrone2, hace falta un stream de video a la PC (por ejemplo `drone/qdrone2_camera_streamer.py` de la Parte B). |

El bloque `Video 3D Capture` acepta cualquier resolución RGB y la reescala a 640×480, así
que no tienes que tocar el resto de la cadena.

### A5. Volar

1. Despega el QDrone2 y ponlo en *hover* con el procedimiento de Quanser (modelo de vuelo
   corriendo, localización activa).
2. En Simulink: modo **Normal**, **Fast Restart apagado** y **Simulation Pacing activado**
   (*Run → Simulation Pacing*, 1× tiempo real). Sin pacing, la FSM cuenta los 3 s de
   estabilidad en tiempo de simulación y no en tiempo real.
3. Pulsa **Run**. Haz doble clic en `Manual Switch` para pasar `startBool` a 1.
4. La FSM va a cada waypoint, verifica 3 s de estabilidad (error medio ≤ 0.10 m / 5°),
   dispara 25 fotos en 3 s y avanza. Mira `Status` (1 = capturando, 2 = lote listo,
   -2 = faltó un frame), `Confidence1` (mayoría de fuego) y `Confidence` (promedio en %).
5. Al terminar el último waypoint, `homeToggle` cae a 0 y el `Switch` manda `Home = [0;0;1;0]`.

---

## Parte B — Versión web (sin MATLAB)

Carpeta: `web/`. Hace lo mismo que el `Trayectory_Model`, en Python y con el
SDK de Quanser:

| Simulink | Python |
|---|---|
| DLL + MEX Edge Impulse (compilar con MinGW/CMake) | `app/fomo.py`: usa el `.tflite` que ya viene en el ZIP del modelo, con LiteRT. No se compila nada. |
| `prepare_image.m`, `VideoDeviceToQUARC`, `CaptureFOMOBatch` | Mismo reescalado *squash* con vecino más cercano y la misma luma 0.299/0.587/0.114 con `round` de MATLAB. |
| `Trayectory SM` | `app/mission.py: TrajectoryStateMachine`, portado línea por línea (Ts = 0.05 s, ventana de 3 s, tolerancias). |
| `Image Batch Capture` + `FOMO Inference` | `BatchCapture`: N fotos en T s, mayoría estricta y confianza promedio. |
| `Switch` / `Home` | Al completar la misión se comanda `home`. |
| `From Video Device` | `camera.source`: webcam, Quanser `VideoCapture`/`Video3D` o el stream del QDrone2. |
| `Position Simulation` | `pose.source`: `sim`, `qdrone2_stack` o `stream`. |
| `PoseD` (Display) | Mapa en vivo y, opcionalmente, un Stream Server de Quanser con el setpoint. |

### B1. Instalar (una sola vez)

1. Instala **Python 3.11**. Si tienes el *Python Install Manager* (`winget install 9NQ7512CXL7T`),
   abre una terminal nueva y ejecuta `py install 3.11`. Comprueba que funciona con
   `py -3.11 --version`. Sin el manager, usa el instalador de python.org y marca
   *Add python.exe to PATH*.
2. Haz doble clic en **`iniciar_web.bat`**. La primera vez:
   - crea `.venv` e instala `flask`, `numpy`, `opencv-python` y `ai-edge-litert`;
   - instala el **SDK de Quanser** desde `%QUARC_DIR%\python` (ya lo tienes en
     `C:\Program Files\Quanser\QUARC\python`).
3. Se abre `http://127.0.0.1:8080`. Después, `iniciar_web.bat` solo arranca el servidor.

Por defecto usa una **cámara sintética** y un **dron simulado**: puedes pulsar *Iniciar
misión* y ver el recorrido completo sin hardware. *Probar con una imagen…* hace lo mismo que
`run_ei_inference('foto.jpg')`.

Pruebas automáticas, desde la carpeta `web`:

```bash
.venv\Scripts\python.exe -m unittest discover -s tests -v
```

### B2. Elegir la cámara (`config.json → camera.source`)

| Valor | Cuándo usarlo |
|---|---|
| `synthetic` | Demo sin hardware. |
| `folder` | Recorre las fotos de `test_images/` (pon ahí fotos con y sin fuego). |
| `opencv` | Webcam de la PC (`opencv_index`). Equivale a la configuración actual del `.slx`. |
| `quanser_video` | Cámara 2D con el SDK de Quanser, `video://localhost:<quanser_video_id>`. |
| `quanser_realsense` | Stream COLOR de una RealSense con `quanser.multimedia.Video3D`. |
| `qdrone2_remote` | **Cámara del QDrone2**: recibe los frames de `drone/qdrone2_camera_streamer.py` en `remote_uri`. |

**Cámara del QDrone2, paso a paso:**

1. Copia `drone/qdrone2_camera_streamer.py` al dron:

   ```bash
   scp drone/qdrone2_camera_streamer.py nvidia@192.168.2.15:~/
   ```

   Usuario e IP dependen de tu dron; revisa el manual de Quanser.
2. En el dron (SSH), comprueba que el SDK de Quanser está disponible:

   ```bash
   python3 -c "import quanser.multimedia, quanser.communications"
   ```

3. Arranca el streamer. Prueba primero la RealSense y, si no abre, una cámara 2D por id:

   ```bash
   python3 qdrone2_camera_streamer.py --camera realsense
   ```

   ```bash
   python3 qdrone2_camera_streamer.py --camera video --id 0
   ```

4. En la PC, en `config.json`, pon `"source": "qdrone2_remote"` y
   `"remote_uri": "tcpip://<IP_DEL_DRON>:18800"`. Reinicia `iniciar_web.bat`.

No pude verificar qué id corresponde a cada cámara física del QDrone2 (frontal, inferior,
RealSense). Prueba los ids y mira la imagen en la página.

### B3. Elegir la pose (`config.json → pose.source`)

| Valor | Descripción |
|---|---|
| `sim` | Dron simulado que sigue el setpoint. Sirve para probar la misión completa. |
| `stream` | Se conecta a `stream_uri` y recibe 4 doubles `[X, Y, Z, Yaw]`. Úsalo si tu modelo QUARC de localización/OptiTrack publica la pose con un bloque *Stream Server*. Es la opción más limpia. |
| `qdrone2_stack` | Protocolo de `pal.products.qdrone2.QDrone2StreamStack` de Quanser: manda 1 double y recibe 47 doubles de `tcpip://<qdrone2_ip>:18491`. Esa clase no documenta qué índices son X, Y, Z, Yaw. Abre *Diagnóstico* en la página, mueve el dron a mano y anota qué posiciones del vector `raw` cambian. Luego ponlas en `"qdrone2_indices": [iX, iY, iZ, iYaw]`. Mientras no lo configures, la pose se considera inválida y la FSM no avanza (comportamiento seguro). |

### B4. Mandar el setpoint al dron (opcional)

Con `"setpoint_output": {"enabled": true, "uri": "tcpip://localhost:18600"}` la app
publica cada 50 ms 6 doubles `[X, Y, Z, Yaw, startCamera, fuegoDetectado]`. En tu modelo
de vuelo/commander de QUARC, agrega un **Stream Client** a `tcpip://<IP_DE_LA_PC>:18600`
que reciba `double[6]` y usa los 4 primeros como referencia de posición. Para conexiones
desde otra máquina, pon `"server": {"host": "0.0.0.0"}` y abre el puerto en el firewall.

> **Seguridad.** Prueba primero con `pose: sim` y `setpoint_output` apagado. Después usa la
> pose real con el dron en el suelo, sin hélices, y verifica en el mapa que los setpoints
> tienen sentido. Al final vuela con alguien listo para el paro de emergencia de Quanser.
> **Abortar → Home** manda `home`, pero no sustituye al paro de emergencia.

### B5. Usar la página

- **Detectar ahora**: una inferencia sobre el frame actual. Muestra la imagen 180×180 que
  ve el modelo, las celdas FOMO (naranja) y la detección elegida (rojo).
- **Detección en vivo**: dibuja la caja sobre el video unas 8 veces por segundo.
- **Waypoints**: edita la tabla, N y la duración, y pulsa *Guardar*. Solo se puede con la
  misión detenida.
- **Iniciar misión / Abortar → Home / Reiniciar**.
- **Resultados por waypoint**: fuego sí/no por mayoría, confianza promedio, positivas/N y
  la mejor foto. En el mapa, los waypoints con fuego se marcan en rojo.

### Diferencias conocidas con Simulink

- LiteRT (TensorFlow Lite) puede diferir en ±1 LSB de TFLite Micro del SDK. En casos
  límite, una confianza justo en el umbral puede cambiar. Para comparar, sube la misma foto
  en la página y en `run_ei_inference`.
- La captura usa el reloj de Windows (sin tiempo real duro). Si la cámara no entrega un
  frame nuevo a tiempo, el lote falla con `-2`, igual que `CaptureFOMOBatch`.
- En Simulink, `nextPoint` sube cuando terminó la captura y la inferencia sigue después.
  Aquí se infieren las N fotos (unos milisegundos) antes de avanzar, así que cada resultado
  queda asociado a su waypoint.
