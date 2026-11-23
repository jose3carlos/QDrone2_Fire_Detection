# Estación web — 2× QDrone 2 (tomas clandestinas y fuegos)

La misión vive en Simulink (o en el simulador MATLAB). Esta estación en Python **solo** hace de puente
(`:18700`), recibe el video del explorador (`:18800`), detecta, geolocaliza, publica los objetivos y muestra
todo en el navegador. El contrato completo está en `docs/ARQUITECTURA.md` (§2, §3.3, §3.4, §6).

## Arrancar

```bat
web\iniciar_web.bat                    :: fuente según config\estacion.json (web.fuente)
web\iniciar_web.bat --fuente mock      :: simulación interna de la misión completa, sin MATLAB
web\iniciar_web.bat --fuente simulink  :: modelo de tierra o QD2_Sim_Dual en tcpip://127.0.0.1:18700
web\iniciar_web.bat --fuente sim3d     :: simulador MATLAB 3D (mismo puerto 18700)
```

- La primera vez crea el entorno virtual en `%USERPROFILE%\.venvs\qd2_web` (fuera de OneDrive) e instala `requirements.txt`.
- Abre `http://127.0.0.1:<red.puerto_web>` (8080 por defecto). Por seguridad la web **solo escucha en esta PC**. Para abrirla desde otra máquina usa `--host 0.0.0.0` (no tiene contraseña).
- La fuente también se cambia en vivo desde el selector **Fuente** de la barra superior.
- Para que no se abra el navegador: `set QD2_NO_BROWSER=1`.

Sin el `.bat`: `cd web` y luego `python -m app.server --fuente mock`.

## Qué hay en la pantalla
- **Vista 3D** (three.js r160 desde CDN): arena 3×3, la N, los dos QDrone 2 (modelo low-poly con las 4 guardas, color por rol: cian = explorador, naranja = respondedor), consignas (◇), estelas, home (▢), huella y pirámide de la cámara del explorador, anillo `d_min_h` y anomalías (llama = fuego, válvula = toma; el anillo indica el estado: amarillo detectada, azul asignada, verde visitada). Si no hay internet para el CDN, sigue funcionando el mapa 2D.
- **Mapa 2D**: lo mismo visto desde arriba (x a la derecha, y hacia arriba), con coordenadas bajo el cursor.
- **Cámara del explorador** con las cajas del detector y la posición geolocalizada de cada una.
- **Controles**: START, PAUSE, RESUME, ABORT/RTH y LAND ALL (START, ABORT y LAND ALL piden confirmación). También se muestran Δh/Δz entre E y R y el indicador del SEPARATION GUARD.
- **Seguridad de los comandos**:
  - Solo se aceptan con telemetría fresca del puente. Si el puente está caído la página responde *"el comando NO se envió"* (HTTP 409): no se guarda para después.
  - Se mandan uno a la vez (cola) y el siguiente sale cuando Simulink devuelve el `cmd_seq_ack`. Así un ABORT seguido de LAND ALL llega completo.
  - Si no llega el ack en `web.cmd_timeout_s` (1.5 s) el comando **caduca**: se cambia `cmd_seq` con `cmd = 0` y nunca se ejecuta tarde (ni al reconectar).
  - ABORT y LAND ALL descartan los comandos normales en cola y reemplazan al que está en curso.
  - Cada resultado (recibido / caducado / descartado / reemplazado) sale como aviso y en **Eventos**.
- **Alerta de Simulink, y(55)**: barra bajo los controles con el código 0..10 y su texto (tabla en `docs/ARQUITECTURA.md` §3.3). Amarilla = aviso (1 homes cerca, 4 timeout de aterrizaje, 6 Takeoff abajo); roja = grave (las demás). Cada cambio queda en **Eventos**.
- **Estado de cada dron**: en PID es el *Flight State de tierra* del Equipo2 (0 Idle, 1 Armed, 2 Takeoff, 3 Land, 9 MS Stop), porque el dron PID no reporta su FSM; en MPC es la *FSM a bordo* (1..9; 7 y 8 en rojo). Se decide con y(3). El 9 (DISABLE / MS Stop) es el fin normal de un vuelo: solo sale en rojo si el dron debería estar en el aire (TAKEOFF, HOVER_READY, MISSION, RETURN_HOME, ABORT, PAUSED) o si hay alerta 9. Un y(55) NaN/inf sale como alerta grave (código interno −1, `protocol.ALERTA_NO_FINITA`), no como "nada"; un código > 10 o negativo sale como "alerta desconocida" (grave). La alerta 9 es global: en MPC la estación recuerda qué dron vio fallar (7/8, o 9 debiendo volar) en la corrida actual de Simulink y, con ese historial, solo ese sale en rojo; el 9 del dron sano al aterrizar queda como "fin de vuelo" (sin historial, los dos en rojo, como dice el contrato). La tarjeta de cada dron muestra la distancia a su home y **en home** (< `mision.tol_xy`): en LANDING cada dron baja en su sitio, así que da LAND ALL cuando los dos marquen *en home*.
- **Barra de avisos** (roja / amarilla): cámara sintética con un puente que no es el mock, detector que no detecta tomas, imagen girada sin `giro_en_dron`, geolocalización sin intrínsecos, actitud del explorador no disponible en misión (y(63) = 0: modelo de Simulink sin y(61:63), se geolocaliza nivelado), o modelo corriendo (y(3)) distinto de `version_control` de `estacion.json` (los comandos `quarc_run` de **Conexiones** lanzarían el binario del dron equivocado).
- **Pestañas**:
  - **Misión**: progreso de la N, cola, marcado y separación.
  - **Anomalías**: lista, alta manual haciendo clic en el mapa y estado del detector. La lista une lo que publicó la estación con lo que Simulink reporta en y(56:60), por id y por cercanía (un objetivo de Simulink con otro id a menos de `vision.radio_cluster_m` es el alias que guardó Simulink: una sola fila, con "= #N"), sin duplicar. El puente procesa cada mensaje aunque lleguen varios juntos, así no se pierde ningún objetivo rotando. Columna **Origen**: *estación*, *ambos* (publicado aquí y confirmado por Simulink) o *Simulink* (lo conoce Simulink pero no lo publicó esta estación; p. ej. el detector interno del simulador 3D). En el mapa 2D, los de solo Simulink llevan anillo punteado. Con la fuente mock hay una tarjeta **Pruebas con el mock** para forzar y(55) (muestra la alerta forzada actual) y agregar objetivos "de Simulink".
  - **Conexiones**: IPs de la PC por interfaz (resalta la antena Alfa *Wi-Fi 2*, RTL8811AU), IPs de los drones que se guardan en `estacion.json` y comandos `quarc_run` / `-URI_Host` con botón **Copiar**.
  - **Motive**: navega la carpeta de OptiTrack o sube un archivo `.cal` / `.motive`, y asigna los rigid bodies a los drones.
  - **Pista**: edita los vértices de la N (máximo 16, el límite de Simulink) o usa **Capturar vértice** con la pose en vivo de un dron. Simulink lee la pista al compilar o iniciar el modelo, así que después de guardar hay que volver a correr el modelo de tierra o el simulador.
  - **Eventos**.
- Se puede abrir una pestaña directamente desde la URL, por ejemplo `http://127.0.0.1:8080/#conexiones`.

## Fuentes de datos
| Fuente | Qué es |
|---|---|
| `mock` | `app/mock_sim.py` levanta un Stream Server TCP local que habla **exactamente** el protocolo §3.3 y simula la misión. Hace roles por menor x, despegue, la N a `z_E` (empieza por el extremo más cercano al home de E) y R atiende primero el objetivo más cercano, con `t_mark`. Los comandos siguen **las mismas reglas que `qd2_mission_step.m`**: primer contacto, cualquier cambio de `cmd_seq`, START solo en ASSIGN_ROLES, PAUSE solo en MISSION/RETURN_HOME. Incluye un guard PID con poses predichas, rodeos y desbloqueo, y pausa si se pierde el heartbeat. La cámara es virtual (render desde la pose del explorador) y el detector es "falso" por color, pero real sobre la imagen. Así se prueba toda la cadena imagen → caja → rayo → suelo → cluster → objetivo → Simulink → R lo visita. |
| `simulink` / `sim3d` | Cliente TCP de `web.host_puente:red.puerto_puente` (127.0.0.1:18700). Se reconecta solo. |

El mock también se puede correr suelto, para que el equipo de MATLAB pruebe contra él:
`python -m app.mock_sim --puerto 18700`.

El mock emite además, igual que el modelo real:
- y(55) = alerta. Rechaza START con alerta 1 si los homes están cerca (< 1.05 m en PID, < 0.8 m en MPC). Para probar la UI se puede forzar cualquier código: `--alerta 9`, o desde la UI/`POST /api/mock/alerta {"codigo": 9}` (`null` = alerta real).
- y(56:60) = un objetivo de su tabla por mensaje, rotando. Para simular objetivos que conoce Simulink y la estación no publicó: `--objetivo-interno X Y CLS` (se repite) o `POST /api/mock/objetivo_interno {"cls": 1, "x": 0.4, "y": -0.8}`.
- o+2 = Flight State de tierra en PID y FSM a bordo en MPC.
- Como `qd2_mission_step`, un id nuevo a menos de `vision.radio_cluster_m` de un objetivo que ya tiene queda como alias (no entra a la tabla ni a y(56:60)) y `visited_mask` lo marca cuando se visita el original.
- Despegue como `qd2_mission_step`: arma al entrar a TAKEOFF y da takeoff 2 s después. ABORT/LAND_ALL antes de despegar cancelan el despegue (sin takeoff: desarma y vuelve a ASSIGN_ROLES; con takeoff y nadie arriba de 0.15 m: bajan en su sitio hasta DONE).
- `mock.reset()` cierra la conexión, como un reinicio real de Simulink.

**Video** (`camara.fuente` en `estacion.json`):
- `dron`: se conecta a `ip_dron{explorer_id}:18800` y cambia solo si cambia el explorador.
- `sintetica`: cámara virtual que dibuja anomalías **inventadas**. Solo se permite con `mock` o `sim3d`. Con `simulink` queda bloqueada (aviso rojo), porque mandaría objetivos falsos a los drones reales. Para usarla a propósito con QD2_Sim_Dual, pon `web.permitir_sintetica_en_simulink: true`.
- `opencv:0`: webcam local, para probar el detector con las piezas en la mano.
- `carpeta`: imágenes de `web/datos/imagenes_prueba`.
- `ninguna`.

**Detector** (`web.detector`):
- `auto`: con cámara sintética usa el detector falso. Si no, usa `vision/detector.py` (YOLO11n) y, si falla, el FOMO original (respaldo, solo fuego, con aviso rojo).
  - En el venv `qd2_web` **no hace falta torch**: `vision/detector.py` corre `vision/modelos/mejor.onnx` con OpenCV (CPU; entrada 480×640, así que los cuadros de 640×480 entran sin relleno y salen las mismas cajas que con ultralytics; mediana ~28 ms por cuadro en la laptop). Probado aquí (22-nov-2026): `load_detector('auto', ...)` devuelve el detector `YOLO11n opencv/cpu (vision/detector.py)` y la pestaña **Anomalías** lo muestra como `vision/detector.py (mejor.pt)` (el nombre del modelo configurado en `vision.modelo`, aunque corra el `.onnx`).
  - Modelo actual (22-nov-2026): YOLO11n entrenado con ~10 000 imágenes **sintéticas** y afinado con **negativos** (1 500 fotos de COCO y 82 capturas de webcam, sin piezas) más 16 capturas de webcam con fuego; **todavía sin fotos del laboratorio**. Con el motor de la estación (OpenCV-ONNX, conf ≥ 0.5): fotos de COCO apartadas con "toma" falsa 28.8 % → 0 % y con "fuego" falso 12.2 % → 0.2 %; capturas de la cara sin piezas con alguna detección 44/50 → 0/50; fuego detectado en 56 capturas de webcam 50/56 → 49/56 (con el `.onnx` de 480×640; con el anterior de 640×640 eran 0.4 % y 47/56). mAP50 en la validación del entrenamiento = 0.9945 (`vision/modelos/metricas.json`). Detalle y advertencias en `vision/README.md` (§3 y "Negativos").
  - Para usar la GPU: `%USERPROFILE%\.venvs\qd2_web\Scripts\pip install ultralytics`, más torch con CUDA (ver `vision/README.md`).
  - `vision/modelos/mejor.pt`, `mejor.onnx` y `mejor.onnx.json` van en git (excepción en `vision/.gitignore`). Los respaldos `mejor_<fecha>.*` y los `candidato_*` no: si se quieren en otra PC, se copian con `vision/paquete.py`.
- También se puede forzar: `vision`, `fomo`, `falso` (solo con cámara sintética) o `ninguno`.
- El error de cada intento se muestra en la pestaña **Anomalías**.

**Filtros físicos de la estación** (`app/vision_service.py`, independientes del modelo). Con la webcam se vio que el YOLO sólo-sintético confundía caras y personas con "toma" (conf 0.5–0.9); el modelo actual se reentrenó con negativos y en las capturas de webcam ya no lo hace, pero los filtros siguen como segunda barrera. Antes de que una detección entre al cluster, la estación comprueba que *físicamente* pueda ser una pieza:
- **Tamaño real** (`vision.tam_max_m` = 0.35, `vision.tam_min_m` = 0.0 = desactivado): además del centro, se proyectan al piso (mismo plano z = 0.02 m) los puntos medios de los 4 lados de la caja; ancho y alto sobre el piso son las distancias izquierda–derecha y arriba–abajo, y se compara `max(ancho, alto)`. Las piezas reales miden 0.07–0.14 m (a 1.5 m, ~40 px de 640); una cara a 0.5–1 m de la cámara, proyectada al piso, sale de varios decímetros. Ojo, la pieza medida no mide 0.10 m: (1) la caja es paralela a la imagen, así que una pieza girada respecto a la cámara mide hasta L·√2; (2) las piezas son altas (fuegos 0.125 m, válvula 0.137 m sobre el piso) y en las orillas de la imagen la parte alta se proyecta al piso desplazada hacia afuera. Calculado con `CameraModel` (cajas 3D reales, 1.3–1.8 m, cualquier posición con la pieza completa en la imagen): caja ajustada ≤ 0.19 m; con caja 15 % holgada y 10° de inclinación sin corregir (modelo viejo sin y(61:63) o `vision.usar_actitud` = false) ≤ 0.24 m. Por eso el umbral es 0.35 y no 0.15 (prueba `test_piezas_reales_con_altura_en_las_orillas_no_se_descartan`). Si algún lado no corta el piso (rayo sobre el horizonte) también se descarta por tamaño (salvo con los dos filtros de tamaño desactivados): no se puede validar algo que no se puede medir, y con la cámara hacia abajo de este dron eso no le pasa a una pieza real sobre el tapete.
- **Toma cerca del tubo** (`vision.dist_max_tubo_toma_m` = 0.25): una toma va abrazada al PVC, así que si su punto en el piso está a más de 0.25 m de la polilínea `pista.vertices` (en el orden dado, tramos consecutivos: la N, no un polígono cerrado) se descarta. Para fuegos `vision.dist_max_tubo_fuego_m` = `null` (desactivado; un fuego puede estar fuera del tubo). Con menos de 2 vértices no aplica. **Depende de que los vértices estén medidos** (Capturar vértice en la pestaña **Pista**); con la estimación desde fotos, una toma real podría quedar "lejos del tubo". La geolocalización usa el roll y pitch del explorador (y(61:63), ver abajo). Sin ellos (modelo viejo) una inclinación desplaza el punto ~3–4 cm por grado a 1.5–1.8 m (5° → hasta ~0.2 m en las orillas) y se come el margen de 0.25 m: con un modelo viejo el dron debe explorar lento (casi sin inclinarse). Si se ven tomas reales descartadas "lejos del tubo", revisar vértices, la actitud (pestaña **Anomalías**, línea *Actitud*) y la prueba de signos del `docs/CHECKLIST_LABORATORIO.md` antes de subir el umbral.
- **Actitud del explorador** (`vision.usar_actitud` = true, `vision.inclinacion_max_deg` = 15, `vision.actitud_signos` = [1, 1], `vision.actitud_offset_deg` = [0, 0]): la pose con la que se geolocaliza incluye roll y pitch de y(61:62) cuando y(63) = 1, interpolados a la hora del frame igual que x, y, z, yaw (`PoseHistory`). La cuenta (`app/geo.py`): `R_wb = Rz(yaw)·Ry(pitch)·Rx(roll)` (roll > 0 = lado izquierdo arriba, pitch > 0 = nariz abajo; con la nariz abajo la cámara ve hacia **atrás**), cámara en `p + R_wb·offset_camara_m + b·(ẑ − R_wb·ẑ)` con orientación `R_wb·R_bc` (`b` = 0.07 m = −`optitrack.bias_z_dronN`: la pose corregida queda debajo del pivote de Motive y el offset gira alrededor del pivote). Con roll = pitch = 0 da exactamente lo mismo que antes. Si la inclinación total `acos(cos roll · cos pitch)` pasa de `inclinacion_max_deg` la detección se descarta con motivo **inclinación** (`null` = sin este descarte). `actitud_signos` (±1 por ángulo) y `actitud_offset_deg` (lo que marca el dron nivelado en el piso, se resta) corrigen un signo invertido o un rigid body creado inclinado sin tocar código. Con `usar_actitud` = false o sin actitud (y(63) = 0: modelo viejo, explorador sin rastreo) se geolocaliza nivelado y no se descarta por inclinación; la pestaña **Anomalías** lo dice en *Actitud* y la tarjeta del explorador muestra φ/θ crudos (y(61:62)) o "no disponible"; con signos/offset ajustados o `usar_actitud` = false agrega la fila *actitud usada*. La cámara sintética dibuja siempre con la actitud que manda el mock (la del empuje necesario para su aceleración, máx. ~14°, p99 ~8°), así que apagar `usar_actitud` con el mock muestra el error de suponer el dron nivelado.
- Las descartadas **no entran al cluster** (no hacen candidatos ni objetivos), pero se dibujan en la cámara en **gris** con el motivo (`DESCARTADA: tamano 0.71 m` / `lejos del tubo a 0.60 m` / `inclinacion 17 deg`), y la pestaña **Anomalías** muestra *Descartadas: N (tamaño a, tubo b, inclinación c)*, acumulado desde el último reinicio de objetivos. Igual que las *Observaciones*, solo se cuentan con la visión activa (MISSION/PAUSED y explorador sobre `web.z_min_deteccion`); sin pose del explorador no hay posición ni filtro.
- Los parámetros se leen en cada frame: se pueden ajustar en `estacion.json` con la estación corriendo (la estación relee el archivo cuando la página pide `/api/config`: al abrirla y cada 10 s). Clave ausente = valor por defecto (`app/config.py → VISION_FILTROS_DEFAULTS` y `ACTITUD_DEFAULTS`); `null` o ≤ 0 = filtro desactivado; un valor que no es número se ignora (se usa el de por defecto).
- El render del mock (`default_anomalies`, a 8 cm del tubo) pasa los filtros: recorriendo la N a 0.5–1.9 m con varios yaw, la pieza más grande midió 0.125 m y la toma más lejana quedó a 0.11 m del tubo. Pruebas: `tests/test_filtros_vision.py` y `tests/test_actitud.py`.

## En el dron explorador (Jetson)
```bash
scp web/drone/qdrone2_camera_streamer.py nvidia@<IP_DRON>:~/
ssh nvidia@<IP_DRON> "python3 ~/qdrone2_camera_streamer.py --camara auto"
```
- Por defecto usa `pyrealsense2`. Si no está, prueba el SDK de Quanser (`Video3D`) y después OpenCV/V4L2.
- Al arrancar imprime los intrínsecos reales de la D435i, listos para copiar en `camara.intrinsecos` de `estacion.json`. Ejemplo: `"intrinsecos": {"640x480": {"fx": 615.3, "fy": 615.1, "cx": 322.4, "cy": 238.9}}`.
  - La geolocalización toma fx, fy, cx, cy en este orden: `camara.intrinsecos` → `vision.intrinsecos_nominales` (fx = fy = 617.1 a 640×480, HFOV ≈ 55°) → `hfov_deg`/`vfov_deg`. Esto último solo pasa si no hay ninguno de los dos, y sale un aviso amarillo.
  - A 640×480 la FOV **no** es la 69.4×42.5 de 16:9.
- `--rotar 0|90|180|270` gira la imagen en el dron. Lo recomendado es no girarla ahí y poner el giro en `camara.rotacion_montaje`. Si la giras con `--rotar G`:
  - pon `camara.giro_en_dron: G` (la estación gira fx/fy/cx/cy);
  - resta G a `rotacion_montaje` (módulo 360), porque `rotacion_montaje` describe la imagen tal como llega.
- Convención: con 0, la parte de arriba de la imagen mira hacia adelante del dron.

## Configuración (`config/estacion.json`)
- Es la única fuente de verdad. Al arrancar **no se escribe nada**: los valores por defecto de `web` viven en memoria. Cuando guardas algo desde la página, el archivo conserva el formato, los números tal como estaban escritos (`0.20` sigue siendo `0.20`) y los finales de línea (CRLF). Así `git diff` solo muestra lo que cambió.
- Claves que agregó la estación (todas opcionales): `web.*` (fuente, host, detector, `cmd_timeout_s`, latencia de video, puertos QUARC, `permitir_sintetica_en_simulink`), `camara.intrinsecos`, `camara.giro_en_dron` y los filtros `vision.tam_max_m`, `vision.tam_min_m`, `vision.dist_max_tubo_toma_m`, `vision.dist_max_tubo_fuego_m` (ver **Filtros físicos de la estación**; Simulink no los lee).
- Valores existentes que modifica:
  - `red.ip_*`
  - `optitrack.*`
  - `pista.vertices`
  - `version_control`
- Nunca renombra ni borra claves.
- Antes de guardar vuelve a leer el archivo, para no pisar cambios de otras herramientas.

## Ids de los objetivos
Simulink busca los objetivos por id y no vuelve a mover uno ya visitado. Por eso la estación **no reinicia los ids mientras Simulink conserve su tabla**:
- Solo se usan ids **1..32** (los que caben en `visited_mask`). Si se acaban, la anomalía confirmada **no se publica** y sale un aviso rojo; el alta manual responde 400. El puente descarta cualquier id fuera de 1..32.
- Los ids nuevos también empiezan después de los que Simulink reporta en y(56:60).
- En cada conexión nueva el puente no manda objetivos hasta recibir la primera telemetría: si Simulink se reinició, la estación vacía su lista antes.
- La tabla se guarda en `web/datos/objetivos_estacion.json`. Si reinicias la estación a mitad de misión, la recupera.
- Los ids nuevos siempre empiezan después de los que Simulink ya conoce (`n_targets_known`, `visited_mask` y `responder_target_id`).
- Si Simulink se reinicia (su `t` vuelve a empezar, o se reconecta sin objetivos), la estación también empieza de cero.
- **Reiniciar lista** solo se permite mientras Simulink no tenga objetivos.

## Pruebas
```bat
cd web
%USERPROFILE%\.venvs\qd2_web\Scripts\python -m pytest -q tests
```
Son 167 pruebas (165 y 2 saltadas si no están `vision/datos/sintetico_smoke` ni el respaldo `mejor_640x640_*.onnx`, que no van a git); `tests/test_frontera_e2e.py` cubre lo que se corrigió en la prueba punta a punta (fallas por dron con la alerta 9 y aviso de versión del modelo distinta de la config); `tests/test_revision.py` cubre los defectos de la revisión y `tests/test_alertas_objetivos_sim.py` la alerta y(55), los objetivos y(56:60), la etiqueta de fsm según la versión, los ids 1..32, el ack con +0.5 del operador, la retención de objetivos al reconectar, las ráfagas de telemetría, el 9 como fin de vuelo, y(55) no finito, el alias de Simulink y la cancelación del despegue en el mock. Cubren:
- protocolo (índices base 1);
- comandos:
  - rechazo sin telemetría;
  - caducidad sin ack, sin ejecución al reconectar;
  - cola ABORT → LAND_ALL y prioridad de emergencia;
  - `cmd_seq` distinto del ack de una sesión anterior;
- intrínsecos de la config, imagen girada en el dron, y render con intrínsecos "reales" geolocalizado con los de la config (no circular);
- ids que sobreviven al reinicio de la estación y se reinician con Simulink, y bloqueo de *Reiniciar lista*;
- mock con las reglas de `qd2_mission_step`, y guard PID en 10 misiones aleatorias (homes, anomalías, momento de publicación y paso de tiempo);
- Origin ajeno rechazado en POST y en el WebSocket, validaciones, y cámara sintética bloqueada con `simulink`;
- puente: heartbeat a 20 Hz, `cmd_seq`/ack, rotación de ranuras y reconexión;
- misión mock completa: PID y MPC, pausa por heartbeat, ABORT y LAND ALL;
- geolocalización: ida y vuelta con yaw/montaje, huella y cadena imagen→suelo con error < 3 cm;
- actitud del explorador (`tests/test_actitud.py`): convención de ejes, nivelado idéntico al cálculo anterior, ida y vuelta mundo→píxel→suelo < 1 mm con roll/pitch de ±12°, yaw y montajes variados, casos a mano (pitch 5° nariz abajo → 0.135 m hacia atrás), el offset de la cámara girando alrededor del pivote de Motive (0.07 m sobre la pose corregida, `optitrack.bias_z_dronN`), y(61:63) en el protocolo, interpolación en `PoseHistory`, descarte por inclinación, `usar_actitud` = false, modelo viejo (y(63) = 0), signos configurables, actitud del mock y huella de la estación;
- clusters y persistencia de la configuración;
- API REST y WebSocket;
- video QDF1 entre el streamer y el cliente;
- detector FOMO.

## Prueba punta a punta (E2E)
`e2e/e2e_estacion.py` levanta esta estación real, la maneja por la API (START, objetivo manual, PAUSE, RESUME, LAND ALL en home), lee el WebSocket como la página, toma capturas con Chrome headless y revisa cada corrida. Se corre contra `QD2_Sim_Dual` (`simulink_qdrone2/sim/qd2_e2e_web.m`) o contra el simulador 3D (`simulador3d/sim3d_e2e_web.m`). También mide la actitud y(61:63) y el error de geolocalización contra la verdad de la cámara sintética; `--sin-actitud`, `--actitud-esperada si|no` e `--inclinacion-max` sirven para comparar con la geolocalización nivelada, probar un modelo viejo y ver el descarte por inclinación. Pasos exactos, resultados y paso a los drones reales: `docs/PRUEBA_E2E.md`.

## Archivos
| Archivo | Rol |
|---|---|
| `app/server.py` | FastAPI + WebSocket (`/ws` a 15 Hz), `Station` une todo |
| `app/protocol.py` | Empaquetado §3.3 (64/32 doubles) |
| `app/bridge.py` | Cliente del puente :18700 |
| `app/mock_sim.py` | Misión mock + Stream Server de prueba |
| `app/mock_world.py` | Anomalías simuladas, cámara virtual, detector falso por color |
| `app/geo.py` | Modelo de cámara, rayo → suelo, huella, clusters |
| `app/vision_service.py` | Detector enchufable + geolocalización + publicación |
| `app/cameras.py` | Fuentes de video (cliente QDF1, sintética, webcam, carpeta) |
| `app/netinfo.py` | IPs por interfaz y comandos `quarc_run` |
| `app/fomo.py` | Detector FOMO original (respaldo) |
| `app/static/` | UI (sin compilación): `index.html`, `app.js`, `escena3d.js`, `mapa2d.js`, `style.css` |
| `drone/qdrone2_camera_streamer.py` | Streamer QDF1 en el Jetson |
| `preflight_lab.py` | Revisión previa al laboratorio, de solo lectura: archivos de `versiones/`, binarios de los drones, ONNX, config (cámara, Motive, rigid bodies, pista, IPs), interfaces, puertos, Motive abierto y QUARC. Uso: `%USERPROFILE%\.venvs\qd2_web\Scripts\python web\preflight_lab.py [--version MPC] [--ping]` (ver `docs/CHECKLIST_LABORATORIO.md`) |

## Pendiente de verificar en el laboratorio
- Las opciones de `quarc_run`. Los comandos generados usan `-r -t tcpip://IP:17000 <modelo>.rt-linux_qdrone2 -uri tcpip://IP:17001?retries=10 -URI_Host tcpip://PC:1800i`. `?retries=10` es el valor por defecto del `_main.c` original. Según la documentación local de QUARC (`%QUARC_DIR%\quarc\doc\quarc_command_line_tools.html`), `-r` descarga y ejecuta y `-D` solo descarga; en la sintaxis son alternativas excluyentes, por eso se usa solo `-r`. La ayuda corta es `quarc_run -?` (con `-h` no imprime nada en la consola). El comando se corre en un `cmd` abierto en `simulink_qdrone2\binarios_dron`, donde están los `.rt-linux_qdrone2`.
- En el explorador, elegir la D435i de abajo con `--serie` (el QDrone 2 trae otra RealSense frontal): `rs-enumerate-devices -s` en el Jetson da los números de serie.
- Copiar a `camara.intrinsecos` los intrínsecos que imprime el streamer en el dron real.
- Que la PC del laboratorio tenga el `vision/modelos/mejor.onnx` (y `mejor.pt`) actual: van en git; `vision/modelos/mejor.onnx.json` debe decir `92e8dec0…` e `"imgsz": [480, 640]` (y `vision/detector.py` debe ser el de la misma versión: uno anterior sólo maneja `.onnx` de 640×640).
- Que el Stream Server de Simulink en :18700 mande 64 doubles a 50 Hz y lea 32 (los mismos ajustes de *most recent* que el original).
- La orientación real del soporte de la cámara (`rotacion_montaje`) y su offset respecto al pivote del rigid body.
