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
web\iniciar_web.bat --fuente simulink --host-puente 192.168.2.11
                                       :: modelo de tierra en OTRA PC (ver "Estación en la laptop")
```

- La primera vez crea el entorno virtual en `%USERPROFILE%\.venvs\qd2_web` (fuera de OneDrive) e instala `requirements.txt`.
- Abre `http://127.0.0.1:<red.puerto_web>` (8080 por defecto). Por seguridad la web **solo escucha en esta PC**. Para abrirla desde otra máquina usa `--host 0.0.0.0` (no tiene contraseña).
- La fuente también se cambia en vivo desde el selector **Fuente** de la barra superior.
- Para que no se abra el navegador: `set QD2_NO_BROWSER=1`.

Sin el `.bat`: `cd web` y luego `python -m app.server --fuente mock`.

## Modo solo explorador (un solo QDrone 2)
Se activa **solo por config**: `mision.modo = "solo_explorador"` y `mision.dron_solo = 1 | 2` (por defecto 1).
Cualquier otro valor (texto desconocido, `dron_solo` 0/3/1.5/"1"/true…) cae a **dual** con un aviso amarillo; dual
exige los dos drones rastreados y nunca despega con uno solo. El modelo de tierra lee el modo del `estacion.json`
de **su** PC al compilar (InitFcn): cambiarlo obliga a copiar el archivo y recompilar (Build), igual que la pista.

- **Telemetría:** y(64) = `dron_solo` (0 = dual o modelo viejo). En solo y(4) = dron solo desde INIT, y(5) = 0 y
  y(12) = y(13) = 0. `protocol.unpack_telemetry` agrega `dron_solo`, `modo_mision` y `y64_invalido` (y conserva
  `reserved` = y(64) crudo).
- **`snapshot.modo`:** `config` (`config.mision_modo`), `tel` (lo que dice y(64)), `efectivo` (el de la telemetría
  fresca, si no el de la config), `coincide`, `host_puente` y `remoto`.
- **START** responde **409** si la telemetría fresca y la config local no coinciden en `(modo, dron_solo)` (salvo con
  el mock). PAUSE, RESUME, ABORT y LAND_ALL nunca se bloquean.
- **Avisos:** rojo si y(64) no es 0/1/2, si y(64) ≠ y(4), si la config es solo y el modelo dual o si el dron solo es
  distinto; amarillo si la config es dual y el modelo solo, si `mision.modo`/`dron_solo` tienen basura, si el dron no
  usado está a la vista de OptiTrack o conectado, y si la pista compilada en tierra (y(7) = número de waypoints) no
  da lo mismo que la de esta estación (`mock_sim.contar_waypoints`, misma regla que `qd2_track_waypoints.m`).
- **UI:** chip "SOLO EXPLORADOR · dron N", confirmaciones de START/ABORT/LAND ALL con el dron, START deshabilitado si
  no coinciden, si el dron solo no está rastreado o fuera de ASSIGN_ROLES; Misión sin respondedor, visitados ni
  separación ("no aplica"); el otro dron en gris con **NO USADO** (en la Vista 3D, "fantasma" si OptiTrack lo ve);
  la huella de la cámara ya no depende de que haya respondedor. Conexiones no muestra el `quarc_run` del dron no usado.
- **Mock:** el dron solo sale de su home, el otro sin rastreo y quieto; sin guard ni respondedor; los objetivos se
  quedan "detectada" (y(53) = cuántos); fin = N + 2 s → RETURN_HOME → (`auto_land` o LAND ALL) → DONE.

## Estación en la laptop y modelo de tierra en la PC del laboratorio
`--host-puente IP` (solo CLI, nunca se guarda) conecta el puente a `tcpip://IP:18700`, donde corre el modelo de tierra
(QUARC ignora el host de un URI que escucha, así que `tcpip://localhost:18700` acepta a la laptop por la LAN). Solo se
permite con `--fuente simulink`: `sim3d` y `mock` siempre usan 127.0.0.1 (al cambiar de fuente en la UI queda en
Eventos). La web sigue escuchando solo en `127.0.0.1` de la laptop.

- `-URI_Host` de los drones: `red.ip_pc` si está; si no, el host remoto del puente; nunca la IP de la laptop.
- En la PC: regla de firewall de entrada TCP 18700 para la IP de la laptop. Si no llega telemetría, revisa eso y que
  el modelo tenga Start. El Stream Server atiende **un** cliente: no lo pruebes con `Test-NetConnection` con la
  estación conectada.
- **Dos copias de `estacion.json`:** la de la laptop es la maestra (con las rutas de Motive **de la PC**); cópiala a la
  PC antes de cada Build. Guardar la pista o la calibración en la UI cambia el archivo de la laptop (la página lo
  recuerda en las pestañas Pista y Motive).
- Revisión previa: `%USERPROFILE%\.venvs\qd2_web\Scripts\python web\preflight_lab.py --remoto 192.168.2.11`
  (no revisa Motive ni los puertos 18700/18001/18002 locales, exige `red.ip_pc` vacía o igual a la PC y no abre
  ninguna conexión hacia ella; en solo exige solo la IP del dron solo).

## Qué hay en la pantalla
- **Vista 3D** (three.js r160 desde CDN): arena 3×3, la N, los dos QDrone 2 (modelo low-poly con las 4 guardas, color por rol: cian = explorador, naranja = respondedor), consignas (◇), estelas, home (▢), huella y pirámide de la cámara del explorador, anillo `d_min_h` y anomalías (llama = fuego, válvula = toma; el anillo indica el estado: amarillo detectada, azul asignada, verde visitada). Si no hay internet para el CDN, sigue funcionando el mapa 2D.
- **Mapa 2D**: lo mismo visto desde arriba (x a la derecha, y hacia arriba), con coordenadas bajo el cursor.
- **Envolvente de consignas** (`mision.limites_x` / `mision.limites_y`, hoy `"limites_y": [-1.4, 1.2]`): límite efectivo de las consignas xy de los dos drones en Simulink (`qd2_limites_xy`, sin el margen de `limites_xy`). El mapa 2D y la vista 3D la dibujan en punteado violeta (el límite de vuelo `limites_xy` sigue en punteado ámbar y el piso en gris) y en **Anomalías** un objetivo fuera lleva "se marca desde (x, y)": R lo marca desde el punto más cercano dentro (en dual). Solo informa, no bloquea nada; avisa si la clave es inválida o si un vértice de la ruta queda fuera (el explorador no alcanzaría ese waypoint). El mock recorta igual en los ejes con envolvente. Simulink la toma al **recompilar**.
- **Cámara del explorador** con las cajas del detector y la posición geolocalizada de cada una.
- **Controles**: START, PAUSE, RESUME, ABORT/RTH y LAND ALL (START, ABORT y LAND ALL piden confirmación). También se muestran Δh/Δz entre E y R y el indicador del SEPARATION GUARD.
- **Seguridad de los comandos**:
  - Solo se aceptan con telemetría fresca del puente. Si el puente está caído la página responde *"el comando NO se envió"* (HTTP 409): no se guarda para después.
  - Se mandan uno a la vez (cola) y el siguiente sale cuando Simulink devuelve el `cmd_seq_ack`. Así un ABORT seguido de LAND ALL llega completo.
  - Si no llega el ack en `web.cmd_timeout_s` (1.5 s) el comando **caduca**: se cambia `cmd_seq` con `cmd = 0` y nunca se ejecuta tarde (ni al reconectar).
  - ABORT y LAND ALL descartan los comandos normales en cola y reemplazan al que está en curso.
  - Cada resultado (recibido / caducado / descartado / reemplazado) sale como aviso y en **Eventos**.
- **Alerta de Simulink, y(55)**: barra bajo los controles con el código 0..10 y su texto (tabla en `docs/ARQUITECTURA.md` §3.3). Amarilla = aviso (1 homes cerca, 4 timeout de aterrizaje, 6 Takeoff abajo); roja = grave (las demás). Cada cambio queda en **Eventos**. La alerta 2 (pérdida de rastreo) dice qué dron: con ABORT/LANDING/DONE, "Aterrizaje por pérdida de rastreo (dron N)" (en el aire, más de `mision.t_perdida_aterrizar_s` = 0.2 s sin rastreo: ese dron aterriza y no vuelve a despegar; en dual el otro se aparta, regresa y aterriza); con PAUSED (solo si ese valor es > 0.2 s), "misión en PAUSA". El número del dron no viaja en y(1..64): la estación lo deduce del dron con `tracked` = 0 (en solo explorador, solo el que vuela) y lo recuerda mientras dure la alerta. Un valor inválido de `mision.t_perdida_aterrizar_s`, o mayor que 0.2, sale como aviso amarillo.
- **Estado de cada dron**: en PID es el *Flight State de tierra* del Equipo2 (0 Idle, 1 Armed, 2 Takeoff, 3 Land, 9 MS Stop), porque el dron PID no reporta su FSM; en MPC es la *FSM a bordo* (1..9; 7 y 8 en rojo). Se decide con y(3). El 9 (DISABLE / MS Stop) es el fin normal de un vuelo: solo sale en rojo si el dron debería estar en el aire (TAKEOFF, HOVER_READY, MISSION, RETURN_HOME, ABORT, PAUSED) o si hay alerta 9. Un y(55) NaN/inf sale como alerta grave (código interno −1, `protocol.ALERTA_NO_FINITA`), no como "nada"; un código > 10 o negativo sale como "alerta desconocida" (grave). La alerta 9 es global: en MPC la estación recuerda qué dron vio fallar (7/8, o 9 debiendo volar) en la corrida actual de Simulink y, con ese historial, solo ese sale en rojo; el 9 del dron sano al aterrizar queda como "fin de vuelo" (sin historial, los dos en rojo, como dice el contrato). La tarjeta de cada dron muestra la distancia a su home y **en home** (< `mision.tol_xy`): en LANDING cada dron baja en su sitio, así que da LAND ALL cuando los dos marquen *en home*.
- **Barra de avisos** (roja / amarilla): cámara sintética con un puente que no es el mock, detector que no detecta tomas, imagen girada sin `giro_en_dron`, geolocalización sin intrínsecos, actitud del explorador no disponible en misión (y(63) = 0: modelo de Simulink sin y(61:63), se geolocaliza nivelado), o modelo corriendo (y(3)) distinto de `version_control` de `estacion.json` (los comandos `quarc_run` de **Conexiones** lanzarían el binario del dron equivocado).
- **Pestañas**:
  - **Misión**: progreso de la N, cola, marcado, separación y **Batería** (ver [Batería](#batería)).
  - **Anomalías**: lista, alta manual haciendo clic en el mapa y estado del detector. La lista une lo que publicó la estación con lo que Simulink reporta en y(56:60), por id y por cercanía (un objetivo de Simulink con otro id a menos de `vision.radio_cluster_m` es el alias que guardó Simulink: una sola fila, con "= #N"), sin duplicar. El puente procesa cada mensaje aunque lleguen varios juntos, así no se pierde ningún objetivo rotando. Columna **Origen**: *estación*, *ambos* (publicado aquí y confirmado por Simulink) o *Simulink* (lo conoce Simulink pero no lo publicó esta estación; p. ej. el detector interno del simulador 3D). En el mapa 2D, los de solo Simulink llevan anillo punteado. Con la fuente mock hay una tarjeta **Pruebas con el mock** para forzar y(55) (muestra la alerta forzada actual) y agregar objetivos "de Simulink".
  - **Conexiones**: IPs de la PC por interfaz (resalta la antena Alfa *Wi-Fi 2*, RTL8811AU), IPs de los drones que se guardan en `estacion.json` y comandos `quarc_run` / `-URI_Host` con botón **Copiar**.
  - **Motive**: navega la carpeta de OptiTrack o sube un archivo `.cal` / `.motive`, y asigna los rigid bodies a los drones.
  - **Pista**: el selector *Qué se edita* elige el arreglo que se edita, se llena con **Capturar vértice** (pose en vivo de un dron) y se guarda (`POST /api/config/pista` con `destino`; solo cambia ese arreglo de `estacion.json`):
    - **Ruta de vuelo (`pista.vertices` · requiere recompilar)**: máximo 16 vértices (el límite de Simulink). Simulink la lee al compilar o iniciar el modelo, así que después de **Guardar ruta** hay que volver a correr el modelo de tierra o el simulador. Debe quedar dentro de la zona con rastreo: un vértice fuera del límite de consignas (envolvente `mision.limites_x`/`limites_y` o, sin ella, `limites_xy` ∓ 0.15) sale en amarillo (aviso, no bloquea).
    - **Tubo real (`pista.tubos`) — pieza N / pieza nueva**: con el dron en el piso sobre cada extremo/esquina del tubo, en orden. **Guardar tubos** escribe todas las piezas (cada una con ≥ 2 puntos; una pieza vacía se descarta); no requiere recompilar (Simulink no lee `pista.tubos`).
  - **Eventos**.
- Se puede abrir una pestaña directamente desde la URL, por ejemplo `http://127.0.0.1:8080/#conexiones`.

## Fuentes de datos
| Fuente | Qué es |
|---|---|
| `mock` | `app/mock_sim.py` levanta un Stream Server TCP local que habla **exactamente** el protocolo §3.3 y simula la misión. Hace roles por menor x, despegue, la N a `z_E` (empieza por el extremo más cercano al home de E) y R atiende primero el objetivo más cercano, con `t_mark`. Los comandos siguen **las mismas reglas que `qd2_mission_step.m`**: primer contacto, cualquier cambio de `cmd_seq`, START solo en ASSIGN_ROLES, PAUSE solo en MISSION/RETURN_HOME. Incluye un guard PID con poses predichas, rodeos y desbloqueo, y pausa si se pierde el heartbeat. La pérdida de rastreo (`perder_rastreo`: tracked = 0 y pose NaN) sigue la lógica de `qd2_mission_step` portada línea a línea (sin PAUSED con 0.2 s; solo 7 → 8; dual 9 → 7 → 8 con el otro apartado ≥ 1.60 m; nunca más takeoff) y se compara con trazas de MATLAB (`tests/referencia_rastreo/`). La cámara es virtual (render desde la pose del explorador) y el detector es "falso" por color, pero real sobre la imagen. Así se prueba toda la cadena imagen → caja → rayo → suelo → cluster → objetivo → Simulink → R lo visita. |
| `simulink` / `sim3d` | Cliente TCP de `web.host_puente:red.puerto_puente` (127.0.0.1:18700). Se reconecta solo. |

El mock también se puede correr suelto, para que el equipo de MATLAB pruebe contra él:
`python -m app.mock_sim --puerto 18700`.

El mock emite además, igual que el modelo real:
- y(55) = alerta. Rechaza START con alerta 1 si los homes están cerca (< 1.05 m en PID, < 0.8 m en MPC). Para probar la UI se puede forzar cualquier código: `--alerta 9`, o desde la UI/`POST /api/mock/alerta {"codigo": 9}` (`null` = alerta real). Pérdida de rastreo: `--rastreo DRON T1 T2` (s de misión; se repite) o `MockServer.perder_rastreo(dron, duracion_s)`.
- y(56:60) = un objetivo de su tabla por mensaje, rotando. Para simular objetivos que conoce Simulink y la estación no publicó: `--objetivo-interno X Y CLS` (se repite) o `POST /api/mock/objetivo_interno {"cls": 1, "x": 0.4, "y": -0.8}`.
- o+2 = Flight State de tierra en PID y FSM a bordo en MPC.
- Como `qd2_mission_step`, un id nuevo a menos de `vision.radio_cluster_m` de un objetivo que ya tiene queda como alias (no entra a la tabla ni a y(56:60)) y `visited_mask` lo marca cuando se visita el original.
- Despegue como `qd2_mission_step`: arma al entrar a TAKEOFF y da takeoff 2 s después. ABORT/LAND_ALL antes de despegar cancelan el despegue (sin takeoff: desarma y vuelve a ASSIGN_ROLES; con takeoff y nadie arriba de 0.15 m: bajan en su sitio hasta DONE).
- `mock.reset()` cierra la conexión, como un reinicio real de Simulink.

**Video** (`camara.fuente` en `estacion.json`):
- `dron`: se conecta SIEMPRE a `ip_dron{camara.dron}:18800`. `camara.dron` (1 o 2, por defecto 1; otro valor → 1
  con aviso amarillo) es el dron que lleva la RealSense abajo. No sigue al explorador de turno: en ASSIGN_ROLES Simulink
  reevalúa los roles (el de menor x es el explorador) y el video saltaría de IP antes del START.
  - **Explorador ≠ cámara** (dual: y(4) de la telemetría fresca; solo explorador: `dron_solo`, también sin telemetría):
    aviso rojo *"El explorador es el dron N pero la cámara está en el dron M: reacomoda los drones (el explorador es el
    de menor x) o cambia camara.dron"* (en solo: vuela con ese dron o cambia `camara.dron`) y **START responde 409**
    con ese motivo (`CamaraDistinta`, mismo patrón que el modo distinto, que se revisa primero). PAUSE, RESUME, ABORT
    y LAND_ALL nunca se bloquean, y en vuelo no cambia nada. No aplica con el mock ni con `camara.fuente` distinta de
    `dron` (la cámara sintética sigue al explorador). `preflight_lab.py`: FALLA en solo si `dron_solo` ≠ `camara.dron`;
    en dual, INFO "coloca ese dron con MENOR x". En dual, con ASSIGN_ROLES y |x1 − x2| < 5 cm (`DX_ROLES_MIN`) también
    se rechaza START: Simulink reevalúa los roles en cada paso y el ruido podría darle el explorador al otro dron.
  - **Sin video:** con la visión activa (gate: MISSION/PAUSED, explorador rastreado sobre `web.z_min_deteccion`) y sin
    frames de la cámara hace más de 2 s (`SIN_VIDEO_S`), aviso rojo *"Sin video del explorador: no se detecta nada"*.
    Solo avisa (nada automático) y se va solo cuando vuelve el video; Eventos registra cuándo aparece y cuándo se va.
  - UI: la cabecera de la cámara dice `dron M (camara.dron)` (en rojo con `≠ explorador N`), la tarjeta de ese dron
    lleva la etiqueta **CÁMARA** y `snapshot.camara` trae `dron`, `explorador` y `coincide`.
- `sintetica`: cámara virtual que dibuja anomalías **inventadas**. Solo se permite con `mock` o `sim3d`. Con `simulink` queda bloqueada (aviso rojo), porque mandaría objetivos falsos a los drones reales. Para usarla a propósito con QD2_Sim_Dual, pon `web.permitir_sintetica_en_simulink: true`.
  - **Ruta y tubos** (como en el laboratorio del 24-nov): el explorador (mock o simulador) vuela `pista.vertices`, la ruta que compila Simulink, y la cámara sintética dibuja la tubería **real**: `geo.tubos_pista(pista)` = `pista.tubos` si la config los trae o, si no, la ruta partida en `pista.tramos_sin_tubo` (el mismo criterio del filtro de tomas y del mapa; sin huecos ni tubos es la ruta completa, como antes). Las anomalías son `web.mock_anomalias` o, si no hay, `mock_world.anomalias_mock`: con `pista.tubos` van **sobre los tubos reales**, 2 por pieza a 0.35 y 0.75 de su longitud alternando toma/fuego (con la pista del 24-nov: toma (0.46, 1.40), fuego (−0.91, 1.176), toma (0.962, −1.347) y fuego (−0.313, −1.476), en el eje del tubo y a 0.23/0.07/0.33/0.35 m de la ruta, orillas incluidas); sin `pista.tubos`, las 3 de siempre a 8 cm de la ruta. La e2e usa la misma verdad (`anomalias_mock`).
- `opencv:0`: webcam local, para probar el detector con las piezas en la mano.
- `carpeta`: imágenes de `web/datos/imagenes_prueba`.
- `ninguna`.

**Detector** (`web.detector`):
- `auto`: con cámara sintética usa el detector falso. Si no, usa `vision/detector.py` (YOLO11n) y, si falla, el FOMO original (respaldo, solo fuego, con aviso rojo).
  - Detector falso (`mock_world.ColorDetector`): pieza completa 0.90–0.99 (0.90 + 0.09·relleno de la caja: toma ~0.99, fuego ~0.97), como el detector real con las piezas del laboratorio (fuegos ≥ 0.93; una planta de hojas rojas ≤ 0.87); pieza cortada por el borde de la imagen ≤ 0.594 (igual que antes). Así la demo mock funciona con el `vision.confianza_min` = 0.9 de `config/estacion.json` sin un umbral aparte: antes daba 0.55 + 0.4·relleno, un fuego (disco, relleno ~π/4) salía en ~0.86 y con 0.9 solo se detectaban las tomas (2 de 4 con la pista del 24-nov). Las pruebas siguen fijando su propio umbral (0.5).
  - En el venv `qd2_web` **no hace falta torch**: `vision/detector.py` corre `vision/modelos/mejor.onnx` con OpenCV (CPU; entrada 480×640, así que los cuadros de 640×480 entran sin relleno y salen las mismas cajas que con ultralytics; mediana ~28 ms por cuadro en la laptop). Probado aquí (22-nov-2026): `load_detector('auto', ...)` devuelve el detector `YOLO11n opencv/cpu (vision/detector.py)` y la pestaña **Anomalías** lo muestra como `vision/detector.py (mejor.pt)` (el nombre del modelo configurado en `vision.modelo`, aunque corra el `.onnx`).
  - Modelo actual (22-nov-2026): YOLO11n entrenado con ~10 000 imágenes **sintéticas** y afinado con **negativos** (1 500 fotos de COCO y 82 capturas de webcam, sin piezas) más 16 capturas de webcam con fuego; **todavía sin fotos del laboratorio**. Con el motor de la estación (OpenCV-ONNX, conf ≥ 0.5): fotos de COCO apartadas con "toma" falsa 28.8 % → 0 % y con "fuego" falso 12.2 % → 0.2 %; capturas de la cara sin piezas con alguna detección 44/50 → 0/50; fuego detectado en 56 capturas de webcam 50/56 → 49/56 (con el `.onnx` de 480×640; con el anterior de 640×640 eran 0.4 % y 47/56). mAP50 en la validación del entrenamiento = 0.9945 (`vision/modelos/metricas.json`). Detalle y advertencias en `vision/README.md` (§3 y "Negativos").
  - Para usar la GPU: `%USERPROFILE%\.venvs\qd2_web\Scripts\pip install ultralytics`, más torch con CUDA (ver `vision/README.md`).
  - `vision/modelos/mejor.pt`, `mejor.onnx` y `mejor.onnx.json` van en git (excepción en `vision/.gitignore`). Los respaldos `mejor_<fecha>.*` y los `candidato_*` no: si se quieren en otra PC, se copian con `vision/paquete.py`.
- También se puede forzar: `vision`, `fomo`, `falso` (solo con cámara sintética) o `ninguno`.
- El error de cada intento se muestra en la pestaña **Anomalías**.

**Filtros físicos de la estación** (`app/vision_service.py`, independientes del modelo). Con la webcam se vio que el YOLO sólo-sintético confundía caras y personas con "toma" (conf 0.5–0.9); el modelo actual se reentrenó con negativos y en las capturas de webcam ya no lo hace, pero los filtros siguen como segunda barrera. Antes de que una detección entre al cluster, la estación comprueba que *físicamente* pueda ser una pieza:
- **Tamaño real** (`vision.tam_max_m` = 0.35, `vision.tam_min_m` = 0.0 = desactivado): además del centro, se proyectan al piso (mismo plano z = 0.02 m) los puntos medios de los 4 lados de la caja; ancho y alto sobre el piso son las distancias izquierda–derecha y arriba–abajo, y se compara `max(ancho, alto)`. Las piezas reales miden 0.07–0.14 m (a 1.5 m, ~40 px de 640); una cara a 0.5–1 m de la cámara, proyectada al piso, sale de varios decímetros. Ojo, la pieza medida no mide 0.10 m: (1) la caja es paralela a la imagen, así que una pieza girada respecto a la cámara mide hasta L·√2; (2) las piezas son altas (fuegos 0.125 m, válvula 0.137 m sobre el piso) y en las orillas de la imagen la parte alta se proyecta al piso desplazada hacia afuera. Calculado con `CameraModel` (cajas 3D reales, 1.3–1.8 m, cualquier posición con la pieza completa en la imagen): caja ajustada ≤ 0.19 m; con caja 15 % holgada y 10° de inclinación sin corregir (modelo viejo sin y(61:63) o `vision.usar_actitud` = false) ≤ 0.24 m. Por eso el umbral es 0.35 y no 0.15 (prueba `test_piezas_reales_con_altura_en_las_orillas_no_se_descartan`). Si algún lado no corta el piso (rayo sobre el horizonte) también se descarta por tamaño (salvo con los dos filtros de tamaño desactivados): no se puede validar algo que no se puede medir, y con la cámara hacia abajo de este dron eso no le pasa a una pieza real sobre el tapete.
- **Toma cerca del tubo** (`vision.dist_max_tubo_toma_m` = 0.25): una toma va abrazada al PVC, así que si su punto en el piso está a más de 0.25 m del tubo más cercano se descarta. Los tubos son `geo.tubos_pista(pista)`: `pista.tubos` (polilíneas de los tubos REALES, una por pieza; solo la estación los usa, Simulink no) o, sin ellos, la polilínea `pista.vertices` (en el orden dado, tramos consecutivos: la N, no un polígono cerrado) partida en `pista.tramos_sin_tubo`. En el laboratorio la ruta de vuelo está recortada a la zona con rastreo de OptiTrack y los tubos quedan 0.1–0.45 m más afuera: contra la ruta, las tomas de las orillas se descartaban. Para fuegos `vision.dist_max_tubo_fuego_m` = `null` (desactivado; un fuego puede estar fuera del tubo). Con menos de 2 vértices no aplica. **Depende de que los vértices estén medidos** (Capturar vértice en la pestaña **Pista**); con la estimación desde fotos, una toma real podría quedar "lejos del tubo". La geolocalización usa el roll y pitch del explorador (y(61:63), ver abajo). Sin ellos (modelo viejo) una inclinación desplaza el punto ~3–4 cm por grado a 1.5–1.8 m (5° → hasta ~0.2 m en las orillas) y se come el margen de 0.25 m: con un modelo viejo el dron debe explorar lento (casi sin inclinarse). Si se ven tomas reales descartadas "lejos del tubo", revisar vértices, la actitud (pestaña **Anomalías**, línea *Actitud*) y la prueba de signos del `docs/CHECKLIST_LABORATORIO.md` antes de subir el umbral.
- **Actitud del explorador** (`vision.usar_actitud` = true, `vision.inclinacion_max_deg` = 15, `vision.actitud_signos` = [1, 1], `vision.actitud_offset_deg` = [0, 0]): la pose con la que se geolocaliza incluye roll y pitch de y(61:62) cuando y(63) = 1, interpolados a la hora del frame igual que x, y, z, yaw (`PoseHistory`). La cuenta (`app/geo.py`): `R_wb = Rz(yaw)·Ry(pitch)·Rx(roll)` (roll > 0 = lado izquierdo arriba, pitch > 0 = nariz abajo; con la nariz abajo la cámara ve hacia **atrás**), cámara en `p + R_wb·offset_camara_m + b·(ẑ − R_wb·ẑ)` con orientación `R_wb·R_bc` (`b` = 0.07 m = −`optitrack.bias_z_dronN`: la pose corregida queda debajo del pivote de Motive y el offset gira alrededor del pivote). Con roll = pitch = 0 da exactamente lo mismo que antes. Si la inclinación total `acos(cos roll · cos pitch)` pasa de `inclinacion_max_deg` la detección se descarta con motivo **inclinación** (`null` = sin este descarte). `actitud_signos` (±1 por ángulo) y `actitud_offset_deg` (lo que marca el dron nivelado en el piso, se resta) corrigen un signo invertido o un rigid body creado inclinado sin tocar código. Con `usar_actitud` = false o sin actitud (y(63) = 0: modelo viejo, explorador sin rastreo) se geolocaliza nivelado y no se descarta por inclinación; la pestaña **Anomalías** lo dice en *Actitud* y la tarjeta del explorador muestra φ/θ crudos (y(61:62)) o "no disponible"; con signos/offset ajustados o `usar_actitud` = false agrega la fila *actitud usada*. La cámara sintética dibuja siempre con la actitud que manda el mock (la del empuje necesario para su aceleración, máx. ~14°, p99 ~8°), así que apagar `usar_actitud` con el mock muestra el error de suponer el dron nivelado.
- Las descartadas **no entran al cluster** (no hacen candidatos ni objetivos), pero se dibujan en la cámara en **gris** con el motivo (`DESCARTADA: tamano 0.71 m` / `lejos del tubo a 0.60 m` / `inclinacion 17 deg`), y la pestaña **Anomalías** muestra *Descartadas: N (tamaño a, tubo b, inclinación c)*, acumulado desde el último reinicio de objetivos. Igual que las *Observaciones*, solo se cuentan con la visión activa (MISSION/PAUSED y explorador sobre `web.z_min_deteccion`); sin pose del explorador no hay posición ni filtro.
- Los parámetros se leen en cada frame: se pueden ajustar en `estacion.json` con la estación corriendo (la estación relee el archivo cuando la página pide `/api/config`: al abrirla y cada 10 s). Clave ausente = valor por defecto (`app/config.py → VISION_FILTROS_DEFAULTS` y `ACTITUD_DEFAULTS`); `null` o ≤ 0 = filtro desactivado; un valor que no es número se ignora (se usa el de por defecto).
- El render del mock (`default_anomalies`, a 8 cm del tubo) pasa los filtros: recorriendo la N a 0.5–1.9 m con varios yaw, la pieza más grande midió 0.125 m y la toma más lejana quedó a 0.11 m del tubo. Pruebas: `tests/test_filtros_vision.py` y `tests/test_actitud.py`. Con la pista del laboratorio (ruta + `pista.tubos`) las 4 anomalías sobre los tubos reales pasan el filtro y se geolocalizan a < 5 mm en la misión mock (`tests/test_tubos_reales.py`); la toma de la orilla −y, contra la ruta, saldría "lejos del tubo" (0.33 m).

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

## Registro de vuelos (`web/datos/vuelos`)
Los tubos del laboratorio se mueven (espacio compartido): una toma real puede quedar a más de
`vision.dist_max_tubo_toma_m` del tubo configurado y la estación la descarta "lejos del tubo". Para corregir
`pista.tubos` con datos, cada vuelo deja `web/datos/vuelos/AAAA-MM-DD/vuelo_HHMMSS_<fuente>.jsonl` (JSON Lines,
`app/registro_vuelo.py`; no se ve en la UI, solo la ruta y las líneas en la pestaña Anomalías):
- 1.ª línea `cabecera`: fecha, fuente, modo, explorador, ruta, tubos efectivos, tramos sin tubo, filtros, cámara;
- una línea `obs` por observación con conf ≥ `vision.confianza_min` y punto en el piso: aceptada (`motivo` null) o
  descartada (tamaño, lejos del tubo, inclinación), con `xy`, `dist_tubo_m`, el tubo más cercano (pieza, tramo,
  punto y desplazamiento con signo, + = izquierda del sentido del tramo), tamaño, pose y actitud. Las de confianza
  menor solo se cuentan;
- una línea `bateria` por segundo y por dron activo (`t`, `dron`, `V`, `fsm`, `estado_mision`, `z`; V = media de
  1 s de `battery_V`), solo si el valor es finito: en PID con el modelo del Equipo2 no hay;
- última línea `resumen`: conteos por clase y motivo, n bajo umbral, duración, objetivos finales y `bateria` por dron
  (`v_inicial`, `v_final`, `v_min_suavizada`, `n_muestras`, o null si nunca hubo dato).

El vuelo se abre cuando el gate de la visión se activa (MISSION/PAUSED con el explorador rastreado y sobre
`web.z_min_deteccion`) y se cierra al salir de MISSION/PAUSED, al reiniciar objetivos o al detener la estación (una
pausa no lo parte). Sin observaciones solo queda archivo si hubo > 5 s de gate. Por defecto se registra solo con
`--fuente simulink` (vuelo real); `web.registro_vuelos` (`"auto"`, `true`, `false`) o `--registro-vuelos auto|si|no`
lo cambian. Con un `estacion.json` de prueba se escribe junto a ese archivo, y pytest nunca escribe en `web/datos`.
**`web/datos/vuelos` sí va a git** (el resto de `web/datos` no): así la laptop y la PC del laboratorio leen los
mismos vuelos.

`herramientas/ajustar_tubos.py` propone `pista.tubos` a partir de esos registros: agrupa las tomas aceptadas y
rechazadas por lejanía (mismo radio y mínimo de observaciones que la estación), mide el desplazamiento de cada
cluster respecto al tubo/tramo más cercano de la config actual y mueve cada tramo la mediana ponderada (≥ 0.05 m y
≥ 2 clusters o uno con ≥ 8 observaciones); los vértices compartidos son la intersección de los tramos movidos y un
cluster más allá de un extremo alarga ese extremo. Nunca toca `pista.vertices` (la ruta de vuelo).
```bat
%USERPROFILE%\.venvs\qd2_web\Scripts\python web\herramientas\ajustar_tubos.py                    &rem todos los vuelos: solo reporte
%USERPROFILE%\.venvs\qd2_web\Scripts\python web\herramientas\ajustar_tubos.py web\datos\vuelos\2026-11-25 --ultimos 2 --svg tubos.svg
%USERPROFILE%\.venvs\qd2_web\Scripts\python web\herramientas\ajustar_tubos.py --aplicar          &rem reescribe SOLO pista.tubos (+ estacion.json.bak)
```
Avisa si los vuelos se registraron con tubos distintos (el cálculo siempre es contra la config actual, con las xy
registradas). Opciones: `--config`, `--umbral`, `--min-clusters`, `--n-fuerte`, `--max-desp`, `--margen`.

## Batería
Pestaña **Misión → Batería** (`app/bateria.py`), por dron activo (en solo explorador, solo el dron solo): voltaje
actual, gráfica de los últimos 5 min (medias de 1 s; banda cian = en el aire), estado (en reposo / armado / bajo
carga), caída desde el despegue (contra el último dato en reposo) y una estimación **informativa**
"≈ N min hasta 14.0 V a este ritmo" (recta del voltaje en vuelo del último minuto; solo con > 20 s en el aire).
Las dos líneas de la gráfica son referencias que ya existían, **no límites de la estación**:
- **14.0 V: aterrizaje automático a bordo (0.3 s)**: en todos los modelos del dron (Equipo2 PID y MPC), "Check
  Battery level" enclava low_battery con V ≤ 14.0 V durante ≥ 0.3 s; en vuelo pasa a AUTOLAND y en tierra no arma.
  (14.8 V es el voltaje *nominal* de una LiPo 4S, 3.7 V/celda; no hay ningún corte a 14.8 V.)
- **15.5 V: mínimo recomendado para despegar (checklist)**: recomendación de `docs/CHECKLIST_LABORATORIO.md`, nada
  automático.

La estación **no actúa por la batería**: no manda comandos, no levanta alertas ni avisos y no cambia la misión
(`tests/test_bateria.py` lo revisa). El mock simula ~16.4 V en reposo, ~0.9 V de caída con carga y ~0.008 V/s en
vuelo, sin reaccionar a ella, y solo la manda en MPC (en PID manda NaN, como el real). **En PID no hay dato en vivo**: el modelo de tierra pone NaN porque el dron PID del
Equipo2 solo devuelve el eco del timestamp; la UI lo dice y la batería se revisa después del vuelo con los logs a
bordo y `simulink_qdrone2/bateria/qd2_bateria.m`. En MPC llega en vivo y queda en el registro de vuelo.

## Configuración (`config/estacion.json`)
- Es la única fuente de verdad. Al arrancar **no se escribe nada**: los valores por defecto de `web` viven en memoria. Cuando guardas algo desde la página, el archivo conserva el formato, los números tal como estaban escritos (`0.20` sigue siendo `0.20`) y los finales de línea (CRLF). Así `git diff` solo muestra lo que cambió.
- Claves que agregó la estación (todas opcionales): `web.*` (fuente, host, detector, `cmd_timeout_s`, latencia de video, puertos QUARC, `permitir_sintetica_en_simulink`), `camara.dron` (dron con la RealSense, por defecto 1), `camara.intrinsecos`, `camara.giro_en_dron` y los filtros `vision.tam_max_m`, `vision.tam_min_m`, `vision.dist_max_tubo_toma_m`, `vision.dist_max_tubo_fuego_m` (ver **Filtros físicos de la estación**; Simulink no los lee).
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
Son 229 pruebas (227 y 2 saltadas si no están `vision/datos/sintetico_smoke` ni el respaldo `mejor_640x640_*.onnx`, que no van a git); `tests/test_frontera_e2e.py` cubre lo que se corrigió en la prueba punta a punta (fallas por dron con la alerta 9 y aviso de versión del modelo distinta de la config); `tests/test_revision.py` cubre los defectos de la revisión y `tests/test_alertas_objetivos_sim.py` la alerta y(55), los objetivos y(56:60), la etiqueta de fsm según la versión, los ids 1..32, el ack con +0.5 del operador, la retención de objetivos al reconectar, las ráfagas de telemetría, el 9 como fin de vuelo, y(55) no finito, el alias de Simulink y la cancelación del despegue en el mock. Cubren:
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
- pérdida de rastreo (`tests/test_rastreo_perdida.py`): texto de la alerta 2 y "(dron N)" en la UI, validación de
  `mision.t_perdida_aterrizar_s` igual que MATLAB, mock sin PAUSED (solo 7 → 8, dual 9 → 7 → 8), nunca más takeoff
  con RESUME/START, sin alerta 4, PAUSED con un umbral mayor, rampa de despegue, TAKEOFF sin takeoff y LANDING, y
  el mock contra 4 trazas de `qd2_mission_step` (MATLAB, `tests/referencia_rastreo/generar_referencia_rastreo.m`).
  Estados y tiempos de eventos coinciden; las posiciones no siempre: el mock no hace el dwell de 0.3 s por waypoint
  de MISSION, así que con una pérdida tras una MISSION larga el explorador aterriza en otro sitio (hasta ~0.5 m);
- geolocalización: ida y vuelta con yaw/montaje, huella y cadena imagen→suelo con error < 3 cm;
- actitud del explorador (`tests/test_actitud.py`): convención de ejes, nivelado idéntico al cálculo anterior, ida y vuelta mundo→píxel→suelo < 1 mm con roll/pitch de ±12°, yaw y montajes variados, casos a mano (pitch 5° nariz abajo → 0.135 m hacia atrás), el offset de la cámara girando alrededor del pivote de Motive (0.07 m sobre la pose corregida, `optitrack.bias_z_dronN`), y(61:63) en el protocolo, interpolación en `PoseHistory`, descarte por inclinación, `usar_actitud` = false, modelo viejo (y(63) = 0), signos configurables, actitud del mock y huella de la estación;
- clusters y persistencia de la configuración;
- registro de vuelos (`tests/test_registro_vuelo.py`): `geo.tubo_cercano` igual a `dist_tubos`, formato de las
  líneas, apertura/cierre por gate y estado de la misión, flush, error de disco, habilitación y que pytest no
  escriba en `web/datos`; `tests/test_ajustar_tubos.py`: tubo 2 desplazado 0.2 m hacia y- → corrimiento correcto
  (±3 cm) sin mover el tubo 1, alargue de un extremo y `--aplicar` sobre una copia (solo `pista.tubos`, CRLF);
- batería (`tests/test_bateria.py`): medias de 1 s solo con valores finitos, caída desde el reposo, estimación
  con > 20 s en el aire, panel "sin dato" en PID y en modo solo, líneas `bateria` y resumen del registro, mock
  descendente que no reacciona, y que ningún comando, alerta, aviso ni cambio de misión dependa de `battery_V`;
- API REST y WebSocket;
- video QDF1 entre el streamer y el cliente;
- detector FOMO;
- modo solo explorador (`tests/test_solo_explorador.py`): tabla de `mision.modo`/`dron_solo`, y(64) y telemetría
  vieja, mock solo hasta DONE con el dron 1 y con el dron 2 sin tocar al otro, ABORT/pausa/auto_land, mock dual
  idéntico byte a byte al de `HEAD` (PID y MPC), avisos y START 409, aviso de pista distinta, `--host-puente`
  (sin abrir sockets), comandos con PC remota, `preflight_lab.py --remoto` y `snapshot.modo`;
- cámara del explorador (`tests/test_camara_dron.py`): tabla de `camara.dron`, video conectado a `ip_dron{camara.dron}`
  aunque cambie el explorador, START 409 con explorador ≠ cámara (dual y solo) y START ok cuando coinciden, ABORT/LAND
  /PAUSE nunca bloqueados, sin bloqueo con el mock o la cámara sintética, aviso "Sin video" que aparece y se va
  (también con el mock real, donde nunca sale) y `preflight_lab.py`;
- ruta y tubos reales separados (`tests/test_tubos_reales.py`): anomalías por defecto sobre `pista.tubos` (orillas
  incluidas) y sin tubos idénticas a las de antes, la cámara sintética (también la de `Station`) dibuja los tubos
  reales y no la ruta, confianza del detector falso (completa ≥ 0.9, cortada < 0.6), la toma de la orilla pasa el
  filtro con los tubos y se descarta contra la ruta, y la misión mock solo con la pista del lab (ruta de 36
  waypoints) con la cámara sintética y la visión en el lazo hasta DONE: 4/4 objetivos a < 5 cm del real y del tubo,
  0 descartes, con umbral 0.5 y 0.9.

`tests/conftest.py` hace las pruebas independientes del día: `cfg_path` es una copia de la config en **dual** con la
N de 4 vértices y `cfg_path_solo` una en solo (dron 1) con la pista de 7 vértices del 23-nov (a las dos se les quitan
`pista.tubos`, que son de otra ruta); `cfg_path_lab` es una en solo (dron 1) con la pista del lab del 24-nov
(`PISTA_LAB`: ruta, hueco en el tramo 4 y tubos reales).

## Prueba punta a punta (E2E)
`e2e/e2e_estacion.py` levanta esta estación real, la maneja por la API (START, objetivo manual, PAUSE, RESUME, LAND ALL en home), lee el WebSocket como la página, toma capturas con Chrome headless y revisa cada corrida. Se corre contra `QD2_Sim_Dual` (`simulink_qdrone2/sim/qd2_e2e_web.m`) o contra el simulador 3D (`simulador3d/sim3d_e2e_web.m`). También mide la actitud y(61:63) y el error de geolocalización contra la verdad de la cámara sintética; `--sin-actitud`, `--actitud-esperada si|no` e `--inclinacion-max` sirven para comparar con la geolocalización nivelada, probar un modelo viejo y ver el descarte por inclinación. Pasos exactos, resultados y paso a los drones reales: `docs/PRUEBA_E2E.md`. `--modo solo [--dron-solo N]` corre el modo solo explorador en la copia de la config (exige y(64) = N, el otro dron en el piso y ningún objetivo visitado) y `--fuente mock` lo corre contra el mock interno, sin MATLAB (p. ej. `python e2e\e2e_estacion.py --fuente mock --corridas solo --modo solo --velocidad-mock 2`).

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
| `app/registro_vuelo.py` | Registro JSON Lines de las observaciones (y la batería) de cada vuelo (`web/datos/vuelos`) |
| `app/bateria.py` | Batería por dron para la UI y el registro (solo visualización; nada actúa por ella) |
| `herramientas/ajustar_tubos.py` | Propone (y con `--aplicar` escribe) `pista.tubos` a partir de los registros |
| `app/cameras.py` | Fuentes de video (cliente QDF1, sintética, webcam, carpeta) |
| `app/netinfo.py` | IPs por interfaz y comandos `quarc_run` |
| `app/fomo.py` | Detector FOMO original (respaldo) |
| `app/static/` | UI (sin compilación): `index.html`, `app.js`, `escena3d.js`, `mapa2d.js`, `style.css` |
| `drone/qdrone2_camera_streamer.py` | Streamer QDF1 en el Jetson |
| `preflight_lab.py` | Revisión previa al laboratorio, de solo lectura: archivos de `versiones/`, binarios de los drones, ONNX, config (cámara, Motive, rigid bodies, pista, IPs), interfaces, puertos, Motive abierto y QUARC. Uso: `%USERPROFILE%\.venvs\qd2_web\Scripts\python web\preflight_lab.py [--version MPC] [--ping] [--remoto IP_PC]` (ver `docs/CHECKLIST_LABORATORIO.md`) |

## Pendiente de verificar en el laboratorio
- Las opciones de `quarc_run`. Los comandos generados usan `-r -t tcpip://IP:17000 <modelo>.rt-linux_qdrone2 -uri tcpip://IP:17001?retries=10 -URI_Host tcpip://PC:1800i`. `?retries=10` es el valor por defecto del `_main.c` original. Según la documentación local de QUARC (`%QUARC_DIR%\quarc\doc\quarc_command_line_tools.html`), `-r` descarga y ejecuta y `-D` solo descarga; en la sintaxis son alternativas excluyentes, por eso se usa solo `-r`. La ayuda corta es `quarc_run -?` (con `-h` no imprime nada en la consola). El comando se corre en un `cmd` abierto en `simulink_qdrone2\binarios_dron`, donde están los `.rt-linux_qdrone2`.
- En el explorador, elegir la D435i de abajo con `--serie` (el QDrone 2 trae otra RealSense frontal): `rs-enumerate-devices -s` en el Jetson da los números de serie.
- Copiar a `camara.intrinsecos` los intrínsecos que imprime el streamer en el dron real.
- Que la PC del laboratorio tenga el `vision/modelos/mejor.onnx` (y `mejor.pt`) actual: van en git; `vision/modelos/mejor.onnx.json` debe decir `92e8dec0…` e `"imgsz": [480, 640]` (y `vision/detector.py` debe ser el de la misma versión: uno anterior sólo maneja `.onnx` de 640×640).
- Que el Stream Server de Simulink en :18700 mande 64 doubles a 50 Hz y lea 32 (los mismos ajustes de *most recent* que el original).
- La orientación real del soporte de la cámara (`rotacion_montaje`) y su offset respecto al pivote del rigid body.
