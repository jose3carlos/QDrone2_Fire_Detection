# Simulador 3D de 2 QDrone 2 (`simulador3d/`)

Simulador en tiempo real escrito en MATLAB puro. Reemplaza al simulador 2D del compañero (`referencias/qdrone_sim_realtime_companero.m`).

- **Requisitos:** MATLAB R2022a o más nuevo. No usa Simulink ni toolboxes.
- **Probado en:** R2025a, en esta laptop.

## Qué hace

**Drones.** Dibuja 2 QDrone 2 en 3D con una malla procedural (`sim3d_malla_qdrone2.m`) basada en las medidas del manual de Quanser: 442.58 × 490.8 × 135.53 mm.
- Los motores van en (±101.6, ±127) mm: tabla 1 de `referencias/quanser/qdrone2_thrust_parametrization.pdf`, los mismos `X_MOTOR`, `Y_MOTOR` de `impresion3d/cad/soporte_camara.py`.
- Los aros son concéntricos con los motores, con R = 119/107 mm (exterior/interior; hay que medirlos en el laboratorio).
  - Con el largo y el ancho del manual, el radio exterior sale 119.7 mm en x y 118.4 mm en y: unos aros circulares no pueden dar las dos medidas a la vez.
  - Con 119 mm, la malla mide 441.2 × 492.0 × 135.53 mm (−1.4 / +1.2 / 0 mm).
- La malla incluye 4 aros de carbono dobles con postes, hélices de 7" que giran, motores, el Jetson con su ventilador, la batería y la RealSense D435 al frente, mirando hacia abajo. La RealSense queda dentro del contorno de los aros delanteros.
- El dron gira alrededor del pivote de Motive, que queda `-bias_z` (0.07 m) arriba de la pose.
- El **dron 1 es azul** y el **dron 2 es naranja**. La etiqueta de cada uno muestra su rol: EXPLORADOR o RESPONDEDOR.

**Escena.**
- **Marco de la arena** (`sim3d_marco.m`, la misma regla que `web/app/static/marco.js` de la UI): el piso es la caja que contiene los tubos reales, la ruta de vuelo, los homes y `mision.limites_xy`, más 0.25 m por lado, redondeada hacia afuera a múltiplos de 0.5 m. Con la pista de hoy (tubos hasta y = −1.52) queda de 4 × 4 m (±2). Sin datos, ±1.5 como antes.
- Retícula alineada a los múltiplos (el origen OptiTrack cae en una línea): fina cada 0.25 m, marcada cada 0.5 m y más marcada cada 1 m. Borde del piso gris; `mision.limites_xy` (el límite de vuelo de Simulink) en punteado ámbar. Los límites de los ejes, los postes de las cámaras, las vistas general y cenital y el mini mapa siguen al marco.
- Ejes OptiTrack: +x adelante (rojo), +y izquierda (verde), +z arriba.
- Los **tubos reales** y la **ruta de vuelo** por separado, como la UI (`escena3d.js` y el mapa 2D):
  - `pista.vertices` es la ruta de vuelo que compila Simulink (recortada a la zona donde OptiTrack rastrea). Se dibuja como línea celeste punteada en el piso.
  - `pista.tubos` son los tubos reales en el piso, una polilínea por pieza (en el lab, más afuera que la ruta). Se dibujan como tubo PVC blanco de Ø 26.7 mm. Sin `pista.tubos`, los tubos son la ruta partida en `pista.tramos_sin_tubo` (comportamiento anterior). Las reglas son las de `geo.tubos_pista` (`sim3d_tubos.m`).
  - El mini mapa y la vista cenital de las gráficas finales muestran lo mismo: tubos en blanco o gris y ruta celeste punteada.
- Fuegos y tomas cargados de los **STL del proyecto** (`impresion3d/stl` y la válvula armada de `impresion3d/renders/_mallas_uso`). Si no están, se usan figuras simples.

**Misión.** Usa la **misma librería** que Simulink (`simulink_qdrone2/lib`):
1. `qd2_mission_step`
2. `qd2_separation_guard`, solo en la versión PID
3. El limitador por norma (`qd2_rate_limit(...,1)`)
4. `qd2_onboard_step`, que es el PID original o el MPC
5. `qd2_downwash`
6. `qd2_quad_plant`

Arriba de `qdrone2_sim3d.m` se elige la versión:
- **PID:** los drones nunca quedan uno encima del otro.
- **MPC:** vuelo apilado, con compensación de downwash.

**Cámara y detector.** Se dibujan el frustum y la huella de la cámara del explorador.
- La huella es lo que ve la cámara **real**: roll, pitch y yaw de la planta, pivote de Motive y error del soporte.
- La cámara usa los mismos intrínsecos y extrínsecos que la estación (`web/app/geo.py`), incluido el pivote de Motive (`optitrack.bias_z_dronN`).
- El **detector simulado** funciona así:
  1. Proyecta cada objeto con la actitud real del dron. Suma un error del soporte de 0.8° y −0.6° (`sesgo_montaje_deg`), que solo gira la cámara.
  2. Aplica probabilidad de detección, ruido de píxel, confusión de clase y falsos positivos.
  3. Geolocaliza como la estación (`VisionService.process`):
     - con la pose medida y el roll y pitch que le llegan en y(61:62);
     - les aplica `vision.actitud_signos` y `actitud_offset_deg`;
     - proyecta con R = Rz(yaw)·Ry(pitch)·Rx(roll) y el pivote de Motive sobre el plano z = 0.02 m.

     Geolocaliza nivelado en dos casos: sin actitud (`actitud_en_tel = false`) o con `vision.usar_actitud = false`. Si la inclinación pasa de `vision.inclinacion_max_deg` (15°), descarta la observación.

     La estación no conoce el error del soporte, así que ese error sí queda en la posición (~26 mm a 1.5 m).
  4. Filtra por distancia al **tubo real** más cercano, como `VisionService.filtrar` con `dist_tubos`: una toma a más de `vision.dist_max_tubo_toma_m` (0.25 m) de `C.tubos` se descarta por "lejos del tubo". Los fuegos solo se filtran si `dist_max_tubo_fuego_m` no es `null`. Los umbrales siguen las reglas de `filtros_vision`. El filtro de tamaño no se simula, porque el detector simulado no tiene cajas.
  5. Agrupa con las mismas reglas del `Clusterer`: radio de 0.25 m y mínimo de 3 observaciones.
- `R.met` guarda, entre otras cosas:
  - los cuadros geolocalizados con actitud y los nivelados (`det_con_actitud`, `det_nivelado`);
  - las descartadas por inclinación (`det_desc_incl`);
  - la inclinación máxima (`det_incl_max_deg`);
  - el error de cada observación real que entró al cluster (`err_obs`);
  - las descartadas por "lejos del tubo" (`det_desc_tubo`, y por objeto real `det_desc_tubo_obj`), las observaciones de cada objeto que entraron al cluster (`det_obs_obj`), la distancia al tubo de cada observación aceptada (`det_dist_tubo_obs`), el estado de cada objeto real (`estado_obj`) y si los tubos salen de `pista.tubos` (`tubos_reales`).

  El HUD muestra la actitud con la que se geolocaliza, en la línea "geolocalización".
- Capturas de una misión PID con esta geolocalización y la malla nueva, en `capturas/`:
  - `actitud_pid_t040.0_vista1.png`: vista general;
  - `actitud_pid_t040.1_vista4.png`: primer plano del dron 1;
  - `actitud_pid_t070.0_vista3.png`: sigue al explorador.

**Pantalla.**
- Estelas, consignas, la "burbuja" de separación d_min_h/2 (versión PID) y la línea entre drones con dh y dz.
- HUD con:
  - estado de misión, estación (interna o Python), alertas;
  - por dron: rol, z, estado de la FSM a bordo y batería;
  - waypoint del explorador;
  - objetivo de R y porcentaje de marca;
  - distancias;
  - tabla de objetivos (detectado / ASIGNADO / VISITADO) con su error contra el objeto real.
- Mini mapa cenital.
- Al final, gráficas como las del original: posiciones y consignas, banderas, timers, y distancia horizontal y vertical.

**Puente con la UI web.** Sirve el protocolo §3.3 en `tcpip://127.0.0.1:18700`:
- Manda 64 doubles a 50 Hz.
- Lee 32 doubles: START, PAUSE, RESUME, ABORT, LAND_ALL, heartbeat y, si `use_vision_python = true`, los objetivos de Python.
- La UI web se conecta igual que a Simulink.

## Cómo correrlo

1. Abre MATLAB en la carpeta del repositorio.
2. Abre `simulador3d/qdrone2_sim3d.m`.
3. Ajusta la sección **OPCIONES** y presiona **Run (F5)**.

```matlab
cd simulador3d
qdrone2_sim3d                 % con la figura; espera START (botón, tecla "s" o la UI web)
```

**Controles de la figura:**

| Botón | Tecla | Acción |
|---|---|---|
| START | `s` | Inicia la misión |
| PAUSE | `p` | Pausa |
| RESUME | `r` | Reanuda |
| ABORT/RTH | `a` | Regresa a home |
| LAND ALL | `l` | Aterriza los dos |
| Vista | `v` | Cambia la vista: general, cenital, sigue al explorador o primer plano del dron 1 |
| x0.5 / x2 | `-` / `+` | Velocidad |
| Salir | `q` | Termina |

Con la estación interna, después de `RETURN_HOME` se manda LAND_ALL automático a los 4 s (opción `auto_land_all`).

**Opciones principales** (todas en `sim3d_opciones.m`):

| Opción | Valores |
|---|---|
| `version` | `'PID'` o `'MPC'` |
| `escenario` | JSON de `escenarios/` (objetos con `x`/`y` o sobre un tubo real con `tubo`/`s_m`/`lado_m`), `'aleatorio'` (sobre los tubos reales, con `n_fuegos`, `n_tomas` y `semilla`) o `'web'` (los objetos de la cámara sintética de la UI) |
| `tiempo_real`, `velocidad` | `false` = lo más rápido posible. Con la UI conectada también funciona: el heartbeat se mide en reloj |
| `semilla_ruido` | Semilla del ruido de sensores, planta y detector (flujo propio: con o sin figura da lo mismo) |
| `usar_tcp`, `puerto`, `backend_tcp` | Servir el protocolo §3.3 (por defecto `true`); puerto `[]` = `red.puerto_puente` de la config (18700); `'auto'`, `'java'` o `'tcpserver'` |
| `use_vision_python` | Objetivos de la estación en lugar del detector simulado |
| `auto_start` | Segundos para un START automático |
| `comandos` | `[t cmd]` programados |
| `perdida_rastreo` | `[dron t1 t2]`: el dron llega sin rastreo (`tracked = 0`, pose NaN) en [t1, t2). Más de 0.2 s en el aire = aterriza (ver "Pérdida de rastreo") |
| `trk_a_bordo` | `true` (por defecto) = el stack a bordo también ve `isTracking = 0` y a los 0.2 s pasa a 7 (rampa de bajada) y luego a 9, como el binario del Equipo2; `false` = sigue volando con su estimación (baja por autoland cuando tierra le quita el takeoff) |
| `actitud_en_tel` | `false` = emula un modelo de Simulink viejo: y(61:63) = 0 (la estación y el detector interno geolocalizan nivelado; la estación muestra el aviso amarillo) |
| `usar_actitud`, `inclinacion_max_deg` | Detector interno: `[]` = lo de `config/estacion.json` (`vision.usar_actitud`, `vision.inclinacion_max_deg`); `usar_actitud = false` = nivelado; `inclinacion_max_deg` 0 o `Inf` = sin descarte |
| `sesgo_montaje_deg` | `[roll pitch]` del error del soporte de la cámara que la estación no conoce (`[0.8 -0.6]`) |
| `mostrar` | `false` = sin ventana |
| `capturas` y `capturas_vistas` | PNG en `capturas/` |
| `config` | `[]` = `config/estacion.json`; un struct ya decodificado para usar otra config (pruebas, demo) |
| `modo_mision`, `dron_solo` | `[]` = lo de la config (`mision.modo`, `mision.dron_solo`); `'dual'` o `'solo_explorador'` y 1 o 2 para forzarlo |
| `dron_ausente` | Solo explorador: `'fuera'` (el otro dron sin rastreo, oculto en la figura) o `'piso'` (rastreado en su home, en gris "NO USADO") |

**Modo solo explorador.** Un solo QDrone (`dron_solo`) recorre la pista con la misma librería que tierra (`p.modo_mision = 1`): el otro nunca recibe arm/takeoff, no hay respondedor (los objetos quedan "detectado"), el guard deja pasar todo y la telemetría lleva y(64) = `dron_solo`. La figura muestra `[PID · SOLO]`, la línea "modo: SOLO EXPLORADOR" en el HUD, la huella de la cámara del explorador y sin burbuja ni línea de distancia entre drones. Demo con la pista del laboratorio:

```matlab
cd simulador3d
opc_sim3d = struct('modo_mision', 'solo_explorador', 'dron_solo', 1, ...
    'escenario', 'escenarios/escenario_solo_lab.json');   % ruta y tubos: los de config/estacion.json
qdrone2_sim3d
```

**Escenario del laboratorio** (`escenarios/escenario_solo_lab.json`, vuelos del 24-nov). No copia la pista: la ruta (`pista.vertices`: 7 vértices, 7.96 m, 36 waypoints) y los tubos (`pista.tubos`: 2 piezas, lado +y en y ~1.37-1.41 y lado −y en y ~−1.41..−1.52) salen de `config/estacion.json`. Los objetos se colocan **sobre los tubos reales** con `"tubo"`, `"s_m"` (o `"s"`) y `"lado_m"` (ver la cabecera de `sim3d_escenario.m`), así que siguen a la config si se miden de nuevo los tubos.
- Home del explorador en el centro (0, 0), donde despegó y aterrizó en los vuelos reales.
- 7 objetos: 4 tomas sobre los tubos y 3 fuegos a 7.5-8 cm de ellos. Dos tomas están en las orillas, como las que el vuelo 6 descartaba por "lejos del tubo": (−0.84, 1.37), a 0.26 m de la ruta, y (0.20, −1.45), a 0.37 m. Hay un fuego en (−1.03, −1.59), cerca del que vio el vuelo 5 en (−1.18, −1.52).
- Misión PID con la config real (`confianza_min` 0.9, `dist_max_tubo_toma_m` 0.25): DONE en 102.1 s simulados, 36/36 waypoints, 7/7 objetos detectados con error de 23 a 49 mm, 0 observaciones de objetos reales descartadas por tubo y 0 falsos publicados.
- Con la misma posición de los objetos pero **sin** `pista.tubos` (filtro contra la ruta, como antes de b65603f), la toma a 0.37 m no se publica y la de 0.26 m pierde la mayoría de sus observaciones (prueba 20).

**Pérdida de rastreo.** Igual que tierra (`qd2_mission_step`, `mision.t_perdida_aterrizar_s` = 0.2 s en la config): si un dron en el aire pasa **más de 0.2 s** sin rastreo, ya no hay PAUSED. Ese dron queda en falla 2 (alerta 2), se le baja el takeoff en ese mismo paso y nunca se le vuelve a dar, aunque el rastreo regrese o llegue RESUME o START.
- **Solo explorador:** LANDING en su sitio y DONE (estados 5 → 7 → 8).
- **Dual:** el otro se aparta a su altura, regresa a su home y aterriza (5 → 9 → 7 → 8; el HUD muestra "ABORT · reacción a falla").
- **0.2 s o menos:** no pasa nada.
- **A bordo** (`trk_a_bordo = true`): a los 0.2 s el dron pasa a 7 (actitud nivelada, sin control de posición, empuje bajando a 1.75 N/s, sigue con la velocidad que traía) y a 9 (motores apagados) al llegar a 7.38 N. `R.met.t_trk7` guarda el instante del 7 por dron; la línea del dron en el HUD dice ERROR. No se modela viento ni deriva extra: la deriva es solo la velocidad que traía.
- **HUD:** la alerta 2 dice "pérdida de rastreo: aterriza dron N (sin takeoff)" o, en PAUSED, "pausa por pérdida de rastreo".
- **PAUSED por rastreo** solo ocurre con `mision.t_perdida_aterrizar_s` mayor que 0.2 (válido hasta 2.0) y una pérdida más corta que ese valor. Para probar PAUSED/RESUME, usa una config con 0.5 s (en `opc.config`, sin tocar `config/estacion.json`), `perdida_rastreo = [d 40 40.3]` y `trk_a_bordo = false`. Con `trk_a_bordo = true` el dron ya cae a los 0.2 s mientras tierra solo pausa: es justo el riesgo del dron PID real.

Corrida de referencia (prueba 22, sin tiempo real, N de las pruebas, PID):
- **Solo explorador, dron 1 sin rastreo en [40, 43) s a 1.50 m:** takeoff 0 a los 40.18 s (0.2 s después del último cuadro con rastreo), estado 7 a bordo a los 40.20 s, en el suelo a los 42.24 s a 0.16 m de donde se perdió, 9 a los 44.44 s, DONE con alerta 2. El RESUME a los 46 s no hace nada.
- **Lo mismo con `trk_a_bordo = false`:** baja por autoland y toca el suelo a los 43.72 s.
- **Dual, dron 2 (R, a 0.50 m) sin rastreo en [40, 45) s:** 5 → 9 → 7 → 8 y alerta 2. El dron 1 se aparta y aterriza; la distancia horizontal mínima con los dos arriba de 0.2 m fue 1.63 m.
- **MPC:** misma secuencia; la misión ve la FSM 7 a bordo un paso después, pero la falla 2 ya está marcada, así que la alerta queda en 2.

**Umbral de confianza en las pruebas.** El detector simulado da confianzas de 0.55 a 0.95. `vision.confianza_min` de la config real es 0.9 (el detector real separa una planta de hojas rojas, ≤ 0.87, de los fuegos, ≥ 0.93) y no se cambia. Las misiones lo usan tal cual: con 15 cuadros por segundo pasan suficientes observaciones (151 en la misión del lab). Solo la prueba de un cuadro sin ruido (16) fija su propio umbral de 0.5, porque cuenta las 12 observaciones de un solo cuadro. Fallaba desde 853cd9d, cuando la config pasó a 0.9.

Con la UI web: `web\iniciar_web.bat --fuente sim3d` (la estación toma el modo de la telemetría, y(64)).

**Sin ventana** (para pruebas o en `-batch`), desde MATLAB:

```matlab
addpath simulador3d
R = sim3d_correr(struct('mostrar', false, 'tiempo_real', false, 'usar_tcp', false, 'auto_start', 3, ...
    'capturas', [10 30 60], 'capturas_vistas', [1 2 4]));
R.met            % métricas: estado final, dh mínima, objetivos, errores...
```

> **QUARC y `-batch`.** Al arrancar, QUARC abre la ventana modal "Tip of the Day", que bloquea MATLAB en `-batch`. Usa `matlab -sd <carpeta> -batch "..."`, donde `<carpeta>` tiene un `startup.m` que solo hace `error('fin')`. Ver `simulink_qdrone2/tests/README.md`.

## Con la UI web

1. En MATLAB, corre `qdrone2_sim3d` con `opc.usar_tcp = true` (es el valor por defecto).
2. En otra terminal: `web\iniciar_web.bat --fuente sim3d`.
3. En la UI, pulsa START: el simulador lo recibe y responde con el ack.

Notas:
- **Heartbeat en tiempo de reloj.** Python manda a 20 Hz de *reloj* aunque el simulador corra a x16 o sin tiempo real. Por eso el simulador cuenta a Python como vivo si llegó un heartbeat nuevo en los últimos `t_hb_python` = 0.5 s de reloj; mientras tanto, el heartbeat que ve la misión sube en cada paso. Puedes acelerar (botón x2, hasta x16) con la UI conectada sin que la misión se pause sola.
- Si la estación se cae durante MISSION, la misión pasa a PAUSED como en el sistema real. A x1 la pausa llega ~1.5 s después (0.5 s de reloj + 1 s de `hb_timeout`; en el real es ~1 s). A los 3 s de reloj sin paquetes, la estación interna retoma el heartbeat y puedes dar RESUME desde la figura.
- Si el **primer** paquete de Python ya trae un comando, no se ejecuta ni se confirma (ack = `cmd_seq` − 1): BridgeClient lo reporta como *caducado*, no como *recibido*.
- La telemetría manda `tracked` y la pose **medida** (la que usa la misión). Con `perdida_rastreo`, el dron llega con `tracked = 0` y la pose en NaN; la estación convierte NaN en `null` y no la usa. En y(61:63) va el roll y pitch del explorador (ver "Notas y diferencias con el contrato").

**Objetivos en la UI.** La estación une sus objetivos con los que el simulador reporta en y(56:60) (columna *Origen* de la tabla: estación / ambos / Simulink). Hay dos formas de usarlo:

| Modo | Opciones del simulador | Qué ves en la UI |
|---|---|---|
| Coherente con la UI | `use_vision_python = true`, `escenario = 'web'` y en la UI la cámara **sintética** (`camara.fuente = "sintetica"`) | El simulador pone los mismos objetos que dibuja la cámara sintética de la UI: `web.mock_anomalias` o, si no existe, `default_anomalies` de `mock_world.py`. La estación los detecta, los publica y el simulador los visita, así que los ids y los estados (asignada / visitada) coinciden. |
| Detector del simulador (por defecto) | `use_vision_python = false` | La misión usa los objetivos del detector simulado. La UI los muestra con origen *Simulink* (anillo punteado en el mapa 2D), leídos de y(56:60). Si además la cámara de la UI es sintética, la estación también publicaría los suyos (ids independientes): deja que publique solo uno. |

## Prueba punta a punta con la estación web

`sim3d_e2e_web` corre PID y MPC en tiempo real, sin ventana, con escenario `'web'` y `use_vision_python = true`; `web/e2e/e2e_estacion.py --fuente sim3d --corridas PID,MPC` maneja la misión desde la API de la UI. Pasos y resultados en `docs/PRUEBA_E2E.md`.

## Pruebas

```matlab
cd simulador3d
res = probar_sim3d();        % ~4 min; guarda PNG en capturas/ (probar_sim3d(false): sin capturas)
```

Son 27 pruebas. `probar_sim3d(false)` corre 26, porque se salta "misma corrida con y sin figura"; en esta laptop tarda ~43 s. Las pruebas que no son del modo solo usan la config real con `mision.modo = "dual"` y la N de 4 vértices fija, sin `pista.tubos` ni `tramos_sin_tubo` (el tubo es la N entera), así que no dependen del modo ni de la pista del día. Las del modo solo usan la ruta y los tubos de `config/estacion.json`. Cubren:
- **Malla:** medidas del manual, motores y aros concéntricos.
- **Contra la estación:** cámara y clusters contra `geo.py`, y escenario `'web'` contra `mock_world.py`, con la N y con la pista real (anomalías sobre `pista.tubos`, `anomalias_mock`).
- **Tubos contra la estación:** `tubos_pista`, `tramos_sin_tubo` y `dist_tubos` con 16 pistas (con y sin `pista.tubos`, huecos, valores inválidos), los umbrales `dist_max_tubo_toma_m`/`dist_max_tubo_fuego_m` (8 variantes) y la decisión "lejos del tubo" de `VisionService.filtrar` (60 casos), todos sin diferencias.
- **Geolocalización con actitud** contra `geo.py` y `pose_para_geo` (casos con actitud de la referencia).
- **Detector interno** en un cuadro sin ruido con el dron inclinado 5°: ida y vuelta exacta con actitud, error nivelado y con el error del soporte, y descarte por inclinación.
- **Misiones:** PID, MPC y aleatoria; PID con `usar_actitud = false` y con `inclinacion_max_deg = 1.5`.
- **Solo explorador:** ruta y tubos reales del laboratorio (7 vértices, 36 waypoints, 2 tubos) con `escenario_solo_lab.json` y el dron 2 fuera de OptiTrack. Comprueba: DONE, recorrido completo (los mismos waypoints que `qd2_track_waypoints` con la ruta de la config), dron 2 sin arm/takeoff y en z = 0, y(64) = 1, los 7 objetos con un objetivo de su clase, ninguna observación de un objeto real descartada por tubo y 0 visitados.
- **Sin `pista.tubos`:** los mismos objetos con el filtro contra la ruta: la toma de la orilla a 0.37 m se pierde.
- **Sin `pista`:** `C.tubos` queda vacío y no se filtra, como la estación; la N por defecto solo se dibuja y sirve para colocar objetos. Con solo `pista.tubos`, los tubos son esos.
- **Pérdida de rastreo en el aire** (prueba 22, ver "Pérdida de rastreo"): solo explorador con `trk_a_bordo` en `true` y en `false`, una pérdida de 0.15 s que no hace nada, y dual PID.
- **Dron 2 volando:** con el dron 1 rastreado en el piso debajo de la N, el guard nunca se activa y la misión llega a DONE.
- **Detector y telemetría:** el detector solo en MISSION/PAUSED; la telemetría y(61:63) (actitud del explorador) y su ausencia con `actitud_en_tel = false`, que también deja el detector interno nivelado.
- **Marco de la arena** (prueba 24): con la pista real los tubos (también los de y = −1.52) quedan dentro del piso con ≥ 0.25 m de margen, el marco es múltiplo de 0.5 m, casos fijos (sin datos = ±1.5, borde exacto, home fuera de `limites_xy`) y la figura usa ese marco en el piso y los ejes.
- **Resto:** la misma corrida con y sin figura, el heartbeat de Python a x16, el primer paquete con START, tiempo real y costo de cuadro.

En una terminal, con el simulador corriendo y `usar_tcp = true`:

```bat
python simulador3d\prueba_cliente_puente.py --segundos 45 --start-en 2 --comando 28 PAUSE --comando 33 RESUME
python simulador3d\prueba_cliente_puente.py --con-bridge --segundos 50 --objetivo 0.0 -0.9
```

El segundo comando usa el `BridgeClient` real de `web/app`.

Comprobaciones extra del cliente (modo socket):

| Opción | Qué comprueba |
|---|---|
| `--comando-sim T NOMBRE` | Manda el comando cuando el tiempo **simulado** llega a T |
| `--prohibir-pausa-hb` | Falla si hay un PAUSED con una alerta distinta de 2 (no depende de que haya PAUSED por rastreo) |
| `--rastreo D T1 T2` | El dron D llega con `tracked = 0` y pose NaN en [T1, T2) |
| `--verificar-56` | y(56:60) rota por todos los objetivos |
| `--start-primer-paquete` | Un START en el primer paquete no se ejecuta ni se confirma |
| `--objetivo X Y [CLS]` | Publica un objetivo; se puede repetir |
| `--puerto` | Para no chocar con otro simulador abierto |

Ejemplo a x16 en modo dual (en MATLAB: `modo_mision = 'dual'`, `velocidad = 16`, `auto_start = 3`, `auto_land = 1`, `perdida_rastreo = [2 40 43]`):

```bat
python simulador3d\prueba_cliente_puente.py --start-en -1 --prohibir-pausa-hb --rastreo 2 40 43 --verificar-56 --segundos 30
```

El dron 2 pierde el rastreo 3 s, así que a los 0.2 s aterriza y el otro se aparta, regresa y aterriza (MISSION → ABORT → LANDING → DONE, alerta 2, sin PAUSED). Un RESUME (`--comando-sim 46 RESUME`) ya no hace nada. Para probar PAUSED → RESUME con el puente, usa una config con `mision.t_perdida_aterrizar_s = 0.5`, `perdida_rastreo = [2 40 40.3]` y `trk_a_bordo = false`, y en el cliente `--rastreo 2 40 40.3 --comando-sim 42 RESUME`.

`pruebas/generar_referencia_estacion.py` regenera con el código de la estación los casos de referencia:
- cámara nivelada y clusters;
- casos con actitud: píxel → suelo y huella con roll y pitch de hasta 20°, y pivote;
- `pose_para_geo` con 9 variantes de `vision`/`optitrack` (54 casos): `usar_actitud`, signos, offsets, límites de inclinación (null, 0, texto, `[]`), arreglos con tipos mezclados (`[true, -1]`, `[-30, "a"]`) y `bias_z`. Cada caso lleva también su bloque `vision` como texto JSON (`vision_json`), porque `jsondecode` no distingue `null` de `[]`;
- tubos (`tubos`): `tubos_pista`, `tramos_sin_tubo` y `dist_tubos` con 16 pistas (cada una como texto JSON, `pista_json`), umbrales de `filtros_vision` y `VisionService.filtrar` con la cámara real.

Los casos con actitud se generan después de los demás con el mismo generador aleatorio, así que los nivelados no cambian. Los de tubos usan un generador aparte (semilla 24) y van al final. `anomalias_web` usa la N fija de las pruebas, no la pista del día. Al regenerar, todos los casos anteriores quedan idénticos. Necesita numpy, por ejemplo la venv `qd2_web`.

## Archivos

| Archivo | Qué es |
|---|---|
| `qdrone2_sim3d.m` | Script principal (opciones arriba) |
| `sim3d_correr.m` | Motor: bucle en tiempo real, misión, planta, detector, TCP y registro |
| `sim3d_opciones.m` | Todas las opciones con su valor por defecto |
| `sim3d_config.m` | Lee `config/estacion.json`: misión, pista, cámara, visión y puerto. Los campos de actitud de `vision` siguen las reglas de `web/app/config.py` elemento por elemento, también con tipos mezclados |
| `sim3d_leer_json.m` | `jsondecode` de `estacion.json` que trata una lista en `vision.inclinacion_max_deg`, `dist_max_tubo_toma_m` o `dist_max_tubo_fuego_m` (`[]`, `[0.3]`) como la estación (no es número → valor por defecto) y no como `null` (sin filtro) |
| `sim3d_tubos.m` | Tubos reales y distancia a ellos con las reglas de `geo.tubos_pista`, `tramos_sin_tubo` y `dist_tubos` |
| `sim3d_escenario.m` | Objetos y homes (JSON, sobre los tubos reales de la config o aleatorio sobre ellos) |
| `sim3d_malla_qdrone2.m` | Malla procedural del QDrone 2 |
| `sim3d_mallas_objetos.m` | STL de fuegos y válvula (reducidos, con caché en `tempdir`) |
| `sim3d_escena.m` | Figura 3D, HUD, mini mapa, botones y teclas |
| `sim3d_marco.m` | Marco de la arena (piso y retícula), misma regla que `web/app/static/marco.js` |
| `sim3d_camara.m` | Geometría de la cámara (igual a `web/app/geo.py`) |
| `sim3d_detector.m` | Detector simulado y clusters de la estación |
| `sim3d_estacion.m` | Estación interna, mezcla con Python y traducción de `cmd_seq`/ack |
| `sim3d_puente.m` | Servidor TCP §3.3 (java.nio; `tcpserver` opcional) |
| `sim3d_graficas.m` | Gráficas de la corrida |
| `probar_sim3d.m` | Pruebas sin ventana |
| `prueba_cliente_puente.py` | Cliente de prueba del puente (Python estándar) |
| `sim3d_e2e_web.m` | Prueba E2E con la estación web real (`docs/PRUEBA_E2E.md`) |
| `escenarios/escenario_base.json` | 3 fuegos y 2 tomas sobre la N por defecto |
| `escenarios/escenario_solo_lab.json` | Solo explorador en el laboratorio: 4 tomas y 3 fuegos sobre los tubos reales de la config; home en (0, 0) |

## Notas y diferencias con el contrato

- **TCP.** `tcpserver` es de la **Instrument Control Toolbox**: no está en MATLAB base ni en esta laptop. Por eso el servidor usa `java.nio` (así lo dice ya `ARQUITECTURA.md` §2), que viene con MATLAB R2022a–R2025a sin toolboxes. `backend_tcp = 'tcpserver'` está disponible si la PC tiene esa toolbox, pero no se probó aquí.
- **Telemetría 56–60 (contrato §3.3).** En cada mensaje va un objetivo de la tabla de la misión, rotando: `[id clase x y estado]`, con estado 1 = pendiente, 2 = asignado, 3 = visitado. La estación los une con los suyos por id y por cercanía (alias).
- **o+2 según la versión (contrato §3.3).** En PID va el **Flight State de tierra** (Flight Plan del Equipo2: 0 Idle, 1 Armed, 2 Takeoff, 3 Land, 9 MS Stop), emulado con arm/takeoff; en MPC, la FSM a bordo (1..9). Antes de la revisión E2E (22-nov) iba la FSM a bordo también en PID.
- **FSM a bordo hacia la misión (MPC).** Como tierra MPC, la misión recibe `fsm_a_bordo` (1 paso de retardo) y aplica las reglas de falla a bordo (alerta 9). La única falla a bordo que emula el simulador es el 7 por pérdida de rastreo (`trk_a_bordo`); para las demás (`falla_fsm`, enlace caído) usa `QD2_Sim_Dual`.
- **Telemetría 61–63 (actitud del explorador).** y(61) = roll, y(62) = pitch (rad) y y(63) = 1 si son válidos, con la actitud **real** de la planta (`X(7:8)` de `qd2_quad_plant`, sin ruido, misma convención que OptiTrack: R = Rz(yaw)·Ry(pitch)·Rx(roll), roll > 0 baja el lado derecho, pitch > 0 baja la nariz). Sin explorador o con `perdida_rastreo` del explorador va 0 0 0 y la estación geolocaliza nivelada. y(64) sigue en 0. El detector interno del simulador (`sim3d_detector`, paso 3) usa esa misma actitud con la corrección de la estación (ver "Cámara y detector").
- **y(55) = alerta.** Como en la librería (`tel.alerta` de `qd2_mission_step`: 0-4, con la 2 = pérdida de rastreo que aterriza ese dron; en MPC también la 9 si la FSM a bordo simulada llega a 7/8, aunque con una pérdida de rastreo la falla 2 se marca primero y queda la 2; nunca la 10, porque no le pasa `proto_mal`). El simulador no tiene capa del operador, así que nunca emite 5-8. El HUD muestra el texto de cada código (1-10) como `ALERTAS` de `web/app/protocol.py`.
- **`cmd_seq`.** El simulador numera sus comandos internos (botones y Python) y devuelve a Python el ack con el `cmd_seq` de Python.
- **Detector.** Solo procesa cuadros en MISSION o PAUSED, con el explorador rastreado y por encima de `web.z_min_deteccion` (0.4 m), igual que `_vision_gate` de la estación. En pausas largas pueden acumularse falsos positivos, porque el `Clusterer` no tiene decaimiento; la estación real tiene el mismo riesgo.
- **Filtro de tubo.** Igual que en la estación: una toma confundida con fuego (`p_confusion`) no pasa por el filtro de tubo, porque `dist_max_tubo_fuego_m` es `null`. Si la config no tiene `pista.vertices` ni `pista.tubos`, la estación no tiene tubos y no filtra; el simulador tampoco (`C.tubos` vacío) y usa la N por defecto de Simulink (`C.ruta`) solo para dibujar y colocar objetos.
- **Escenario `'web'`.** Repite `anomalias_mock` de `web/app/mock_world.py`. Con `pista.tubos` válidos, las anomalías van sobre los tubos reales: toma, fuego, ... a 35 y 75 % de cada pieza. Sin ellos van junto a la ruta, como antes. Con la pista real y el detector interno, una misión solo explorador PID detecta las 4 (36/36 waypoints, 0 descartadas por tubo).
- **Referencia y config.** `anomalias_web_pista_real`, los casos de tubos de la config real y los de cámara salen de `config/estacion.json`. Si cambian la pista, los tubos o la cámara, hay que regenerar `referencia_estacion.json`.
- **Caché de mallas.** `tempdir/qdrone2_sim3d_mallas_v2.mat` guarda el nombre, el tamaño y la fecha de cada STL; si cambian, se vuelven a leer.
- **Sin modelar:** latencia de comunicación, retardo puro de actitud y ruido en roll y pitch. Las pruebas son algo optimistas.
- **Pivote.** La z = 0 del dron es el suelo, con las patas apoyadas. Esa z es la del punto corregido de Motive, `pivote + bias_z`.
  - La malla y la cámara giran alrededor del pivote, `-bias_z` = 0.07 m más arriba (promedio de `bias_z_dron1/2`, como `geo.pivote_de_config`).
  - La huella usa el `offset_camara_m` de la config, igual que la estación.
  - La lente dibujada sale de la malla, no de `offset_camara_m`.
