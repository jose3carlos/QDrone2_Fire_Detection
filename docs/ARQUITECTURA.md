# Arquitectura: 2× QDrone 2 para detección de tomas clandestinas y fuegos

Este documento es el **contrato** entre todas las partes del sistema (Simulink, Python, visión, web y simulador).
Si cambias un puerto, un índice de mensaje o el nombre de una función, actualiza este archivo primero.

## 1. Misión
- **Arena:** 3 × 3 m centrada en (0,0). Marco RSF (Research Studio Frame): +x adelante, +y izquierda, +z arriba, yaw = 0 mirando a +x.
- **Pista:** tubería PVC 3/4" SCH40 (diámetro exterior 26.7 mm) en forma de **N invertida**. Se describe con una polilínea configurable (`config/estacion.json → pista.vertices`). El valor por defecto es una estimación tomada de las fotos y se mide en el laboratorio desde la UI (ver §6).
- **Anomalías:** `fuego` (clase 0) y `toma` (toma clandestina / válvula, clase 1). Son piezas impresas en 3D que se colocan sobre o junto a la tubería.
- **Roles.** Se asignan automáticamente al arrancar con OptiTrack:
  - El dron con **menor x** (atrás) es el **EXPLORADOR (E)**. Recorre la N a `z_E = 1.5 m` con la RealSense D435i mirando hacia abajo y detecta anomalías.
  - El dron con **mayor x** (adelante) es el **RESPONDEDOR (R)**. Hace hover a `z_R = 0.5 m` desde el inicio. Cuando se publica una anomalía, va a su (x, y) en paralelo con E, hace hover `t_mark = 3 s` encima y la marca como visitada en el mapa de la UI.
  - **Home** = posición (x, y) donde estaba cada dron al arrancar (detectado, no fijo).
- **Fin de misión:** E terminó la N, la cola de objetivos de R está vacía y R está libre. Entonces ambos regresan a su home, hacen hover y aterrizan cuando el operador lo indica (o automáticamente si `auto_land = true`).

### Modo solo explorador (un solo QDrone, solo por configuración)
- **Activación:** `config/estacion.json → mision.modo = "solo_explorador"` y `mision.dron_solo = 1 | 2` (por defecto 1). Cualquier otro valor, o la clave ausente, es el modo **dual** de siempre (con texto inválido se avisa). En tierra es el parámetro `P` (`p.modo_mision`, `p.dron_solo`), **no ajustable**: cambiar el modo obliga a recompilar (Build), igual que la pista.
- **Qué hace:** el dron `dron_solo` es el EXPLORADOR desde INIT (`explorer_id = dron_solo`, `responder_id = 0`); espera **solo su** rastreo (1 s estable), START sin chequeo de homes, despega a `z_E`, recorre la N igual, los objetivos se registran y quedan "detectados" (sin respondedor: nunca "visitados"); al terminar la N + `t_fin_gracia` regresa directo a su home. LAND_ALL / `auto_land` / falla a bordo: aterriza **en su sitio**.
- **El otro dron ("ausente"):** nunca recibe arm ni takeoff y su consigna es su última pose en el piso `[x y 0 0]`, en todos los estados. Su rastreo, FSM a bordo, protocolo y enlace se ignoran (sin alertas 2/9/10 ni bloqueo de START). El SEPARATION GUARD deja pasar las consignas (no hay con quién separarse).
- **Telemetría:** y(64) = `dron_solo` (0 en dual), y(5) = 0, y(12) = y(13) = 0 (§3.3).
- **Validado en PID** (librería, simulador 3D y el modelo de tierra PID en SIL). En MPC funciona igual en la librería, pero no se ha validado (qd2_cargar_config avisa).

### Dos versiones
| | Versión **PID** | Versión **MPC** |
|---|---|---|
| Control de posición a bordo | PID original del Equipo2 (`COMMANDER (ANGLE MODE)`); el binario del dron no se toca | **MPC a bordo** que sustituye al PID de posición dentro de `COMMANDER`; resto del stack igual |
| Regla de separación | **Nunca uno encima del otro**: distancia horizontal ≥ `d_min_h` (por defecto 0.9 m) entre poses **y** entre consignas. Lo vigila el `SEPARATION GUARD` en tierra | Se permite apilarse. Restricción vertical `Δz ≥ dz_min` (por defecto 0.8 m), más compensación de downwash en el MPC del dron de abajo (recibe la pose del otro dron) |
| Mensaje PC→dron | 16 doubles (igual que el original) | 24 doubles (16 originales + 8 extra, §3.2) |
| Mensaje dron→PC | 1 double (eco del timestamp) | 8 doubles (telemetría, §3.2) |

## 2. Componentes
```
            ┌────────────── PC de tierra (Windows, QUARC) ───────────────┐
OptiTrack ─►│ QD2_MissionCtrl_Dual_{PID|MPC}.slx  (copia extendida del   │
 (Motive    │  MissionCtrl del Equipo2; 1 kHz; único lector de OptiTrack)│
  API, 2 RB)│   ├ Localization - Optitrack (RB 1 y 2)                    │
            │   ├ ROLE & HOME ASSIGNMENT                                 │
            │   ├ MISSION SUPERVISOR (E: N; R: cola de objetivos)        │
            │   ├ SEPARATION GUARD (PID)  /  STACKING INFO (MPC)         │
            │   ├ Communication D1 ⇄ :18001 ─────────────► QDrone 2 #1   │
            │   ├ Communication D2 ⇄ :18002 ─────────────► QDrone 2 #2   │
            │   └ PYTHON BRIDGE  Stream Server :18700 ◄──┐               │
            │                                            │               │
            │ Estación Python (web/)  ───────────────────┘  cliente TCP  │
            │   ├ bridge.py   (telemetría 64 doubles / comandos 32)       │
            │   ├ vision/     (YOLO11n + geolocalización)  ◄── video :18800 ◄── RealSense del EXPLORADOR
            │   └ server.py   (FastAPI + WebSocket, UI en :8080)          │
            └─────────────────────────────────────────────────────────────┘
```
- **Simulación.** `QD2_Sim_Dual.slx` usa la misma librería de misión, pero cambia OptiTrack y los drones por un **modelo de planta** (dinámica de cuadricóptero, controlador PID o MPC equivalente y modelo de downwash). También expone el mismo Stream Server :18700, así que la UI web funciona igual en simulación y en real.
- **Simulador MATLAB 3D.** `simulador3d/qdrone2_sim3d.m` es la mejora de la simulación del compañero. Es un script autónomo en tiempo real con 2 QDrone 2 en 3D (malla procedural fiel a las medidas del manual), las mismas funciones de misión, cámara virtual y detección simulada. Habla el mismo protocolo :18700 con un servidor TCP propio (`sim3d_puente.m`: `java.nio`, sin toolboxes; `tcpserver` solo si la PC tiene la Instrument Control Toolbox), para que la UI pueda mostrarlo sin Simulink.

## 3. Protocolos (todos: doubles IEEE-754 little-endian, tamaño fijo, semántica "most recent")

### 3.1 PC ⇄ dron, versión PID (sin cambios respecto al Equipo2)
Mensaje PC→dron de 16 doubles: `[arm, takeoff, estop, joystick_issue(0), isTracking, x y z roll pitch yaw (medidos), xd yd zd yawd, timestamp]`.
Respuesta dron→PC: 1 double, el eco del timestamp. Cada dron se lanza con `-URI_Host tcpip://<IP_PC>:1800i`, donde `i` = 1, 2.

### 3.2 PC ⇄ dron, versión MPC
PC→dron, 24 doubles:
- Índices 1–16: iguales que en PID.
- 17–19: posición del otro dron `x y z`.
- 20–22: velocidad del otro dron `vx vy vz`.
- 23: `mpc_mode` (0 = PID de respaldo, 1 = MPC).
- 24: **etiqueta de protocolo**: `2` = "protocolo MPC v1" (la manda siempre el modelo de tierra MPC). El código acordado admitía `0` o `2`; desde la revisión del 22-nov el PROTOCOL CHECK del dron es **estricto** y solo acepta `2`. Consecuencia: tierra MPC y `QD2_DroneStack_MPC` deben ser de la **misma construcción** (un MissionCtrl MPC viejo que manda 0 no arma el dron).

Dron→PC, 8 doubles:
- 1: eco del timestamp.
- 2: estado de la **FSM a bordo** (1–9, tabla de abajo).
- 3: batería (V).
- 4: estado del MPC: 0 ok; 1 FISTA no convergió (el comando se aplica igual); 2 entrada inválida (pasa el PIV); 3 inactivo (FSM fuera de 4–6); 4 se eligió el PIV y el MPC corre en sombra.
- 5–7: comando `[T roll pitch]`.
- 8: **etiqueta de protocolo = 2** ("protocolo MPC v1"), para que tierra compruebe con quién habla.

**FSM a bordo** (índice 2 de la respuesta; es la del DroneStack del Equipo2):

| Código | Estado | |
|---|---|---|
| 1 | IDLE | |
| 2 | ARMED | |
| 3 | TAKEOFF_THROTTLE (rampa de empuje) | |
| 4 | TAKEOFF_HEIGHT (subiendo) | |
| 5 | FLIGHT | |
| 6 | AUTOLAND | |
| 7 | EMERGENCY STOP: error (pérdida de rastreo o de enlace); rampa de bajada | **falla** |
| 8 | EMERGENCY ABORT | **falla** |
| 9 | DISABLE (motores apagados) | **fin normal** de un vuelo (1→2→3→4→5→6→9); **falla** solo si el dron debería estar volando (takeoff = 1, o en TAKEOFF armado), p. ej. por FLIGHT SAFETY |

**Validación de tramas (PROTOCOL CHECK a bordo).** Una trama PC→dron solo se acepta si los índices 1–5 (arm, takeoff, e-stop, joystick_issue, isTracking) y 23 (`mpc_mode`) valen exactamente 0 o 1, el índice 24 vale 2 y el timestamp (16) es finito y ≥ 0.
- **Una trama inválida cuenta como pérdida de enlace:** se repite la última trama válida con el timestamp congelado, la "Detect Communication Loss" original declara Communication Issue a los 0.5 s y la FSM pasa a 7 (Error), igual que un enlace caído de verdad.
- Sin ninguna trama válida quedan los ceros iniciales (arm = 0, isTracking = 0): el dron no arma.
- Así se detecta un MissionCtrl de 16 doubles emparejado por error con este binario (las tramas quedan desalineadas y se descartan).

**Vigilancia en tierra (modelo MPC).** Tierra publica en la telemetría §3.3, y(55):
- **alerta 9** si algún dron armado reporta FSM a bordo 7 u 8, o 9 con takeoff = 1 (o en TAKEOFF armado). Un 9 después de aterrizar es el fin normal del vuelo y **no** es alerta (así lo implementa `qd2_mission_step`);
- **alerta 10** (protocolo MPC incorrecto) si la respuesta de un dron no trae `y(8) = 2` o su FSM está fuera de 1..9 durante más de 0.5 s.
- El MPC LINK CHECK de tierra entrega `Onboard Status [4] = [fsm1 fsm2 cod1 cod2]` (cod 0 bien, 1 protocolo incorrecto, 2 enlace perdido tras haber estado conectado; con cod 2 la FSM sale como 0). Un dron que debía volar (takeoff = 1) y pierde el enlace cuenta como **falla a bordo: alerta 9** (a bordo ya pasó a 7 a los 0.5 s).
- Antes de START, un dron desarmado con FSM 7/8/9 no es alerta; si se da START, en TAKEOFF ya armado se detecta, se cancela el despegue sin dar takeoff y queda la alerta 9. El protocolo incorrecto sí bloquea START (alerta 10).

### 3.3 Puente Simulink ⇄ Python (`tcpip://localhost:18700`)
Simulink es el **Stream Server** y la estación Python es el cliente. Simulink envía 64 doubles a 50 Hz y Python envía 32 doubles a 20 Hz.

**Telemetría, Simulink → Python (64 doubles, índices base 1):**

| Índice | Campo |
|---|---|
| 1 | `t` (s) |
| 2 | `mission_state`: 0 INIT, 1 WAIT_TRACKING, 2 ASSIGN_ROLES, 3 TAKEOFF, 4 HOVER_READY, 5 MISSION, 6 RETURN_HOME, 7 LANDING, 8 DONE, 9 ABORT, 10 PAUSED |
| 3 | `version` (1 = PID, 2 = MPC) |
| 4 | `explorer_id` (1 o 2) |
| 5 | `responder_id` (1 o 2) |
| 6 | `explorer_wp_idx` |
| 7 | `explorer_wp_total` |
| 8 | `responder_target_id` (0 = ninguno) |
| 9 | `n_targets_known` |
| 10 | `n_targets_visited` |
| 11 | `guard_active` (1 si el SEPARATION GUARD está reteniendo a alguno) |
| 12 | `dist_horizontal` E–R (m) |
| 13 | `dist_vertical` E–R (m) |
| 14 | `explorer_done` |
| 15–32 | Bloque del dron 1 (ver abajo) |
| 33–50 | Bloque del dron 2 (mismo formato) |
| 51 | `visited_mask`: bit k-1 = objetivo k visitado (k ≤ 32) |
| 52 | `marking_progress` (0–1) |
| 53 | `queue_length` |
| 54 | `cmd_seq_ack`: último `cmd_seq` **recibido** (no "ejecutado"; ver abajo) |
| 55 | `alerta` (0–10, tabla de abajo) |
| 56–60 | Un objetivo de la tabla de Simulink por mensaje, rotando: `[id clase x y estado]` (ver abajo) |
| 61 | `explorer_roll`: roll del **explorador** (rad) |
| 62 | `explorer_pitch`: pitch del **explorador** (rad) |
| 63 | `explorer_att_ok`: 1 si y(61:62) son válidos (actitud medida y explorador rastreado), 0 si no |
| 64 | `dron_solo`: 0 = modo dual (o modelo de tierra anterior); 1 o 2 = modo solo explorador con ese dron. Firma: `explorer_id` = 1 o 2 con `responder_id = 0` (nunca ocurre en dual); entonces y(12) = y(13) = 0 |

**Actitud del explorador, y(61:63).** Sale de la MISMA "Corrected Pose Data [x y z roll pitch yaw isTracking]" de OptiTrack con la que se llenan x, y, z, yaw del explorador en y(o+3…o+6) (`d(4)`, `d(5)`; el dron que va en y(61:62) se elige por `explorer_id`, igual que pose). Si el explorador no está rastreado, si no hay explorador o si algún ángulo no es finito: y(61:63) = 0. Convención (ángulos de Euler ZYX intrínsecos, regla de la mano derecha, cuerpo FLU: +x nariz, +y izquierda, +z arriba):

`R_mundo←cuerpo = Rz(yaw) · Ry(pitch) · Rx(roll)`

- roll > 0: sube el lado izquierdo y baja el derecho (el dron acelera hacia −y).
- pitch > 0: baja la nariz (el empuje se inclina hacia +x, el dron acelera hacia adelante).

La convención se sacó de la cadena de QUARC del modelo del Equipo2 ("Convention Correction" → "Prevent Gimbal Lock", Euler YZX → matriz → Euler ZYX relativo) y los signos se comprobaron con un vuelo real del Equipo2 (aceleración hacia adelante vs pitch: correlación +0.88; hacia la izquierda vs roll: −0.85). Es la misma de `qd2_quad_plant.m` y de `rot_zyx` del simulador 3D. Falta la prueba en el laboratorio con el rigid body actual (`docs/CHECKLIST_LABORATORIO.md`); si un signo saliera al revés se corrige con `vision.actitud_signos` en `config/estacion.json`, sin tocar código. Un modelo de Simulink viejo manda y(61:63) = 0: la estación lo detecta por y(63) y geolocaliza nivelada, como antes (§7).

Bloque de dron (18 doubles, offset `o` = 14 para el dron 1 y `o` = 32 para el dron 2):

| Índice | Campo |
|---|---|
| o+1 | `tracked` |
| o+2 | `fsm`: su significado depende de la versión y(3), ver abajo |
| o+3…o+6 | `x y z yaw` medidos |
| o+7…o+10 | `x y z yaw` deseados (los que se envían al dron) |
| o+11…o+13 | `vx vy vz` |
| o+14 | `battery_V` (NaN si no hay) |
| o+15 | `home_x` |
| o+16 | `home_y` |
| o+17 | `role` (1 = explorador, 2 = respondedor, 0 = sin asignar) |
| o+18 | `comm_ok` |

**`fsm` (o+2) según la versión.** Se distingue con y(3):
- **PID (y(3) = 1): Flight State de tierra** del Flight Plan del Equipo2, porque el binario PID no reporta su FSM. Códigos: 0 Idle, 1 Armed, 2 Takeoff (en vuelo), 3 Land, 9 MS Stop (paro). Los demás no se usan.
- **MPC (y(3) = 2): FSM a bordo** que reporta el dron (índice 2 de su respuesta, §3.2): 1–9. 7 y 8 son fallas; 0 = el dron todavía no responde.
- **El código 9 (DISABLE a bordo / MS Stop de tierra) es el fin normal de un vuelo** en las dos versiones. La UI solo lo marca como falla si el dron debería estar en el aire (`mission_state` = TAKEOFF, HOVER_READY, MISSION, RETURN_HOME, ABORT o PAUSED) o si tierra publicó la alerta 9.
  - La alerta 9 es **global** (no dice qué dron falló). Por eso, en MPC, la estación recuerda qué dron vio fallar en la corrida actual de Simulink (FSM 7/8, o 9 debiendo volar). Con ese historial, con la alerta 9 solo sale en rojo **ese** dron; el 9 del dron sano al aterrizar es su "fin de vuelo". Sin historial (la estación se conectó después de la falla) se aplica la regla de arriba a los dos. El historial se borra cuando Simulink se reinicia.
- **Los simuladores siguen esta tabla igual que los modelos de tierra:** en PID, `QD2_Sim_Dual` y el simulador 3D emulan el Flight Plan de tierra del Equipo2 (0/1/2/3/9) a partir de los disparos arm/takeoff (y e-stop / Communication Issue en `QD2_Sim_Dual`); en MPC mandan la FSM a bordo simulada.

**`alerta` (y(55)).** Códigos acordados entre los dos equipos. Se reporta **una** a la vez: la del operador (5–8) tiene prioridad sobre la del supervisor (1–4) y también sobre la 9 y la 10 de tierra MPC (así lo hace el TELEMETRY PACK de `construir_modelos.m`; falta que los dos equipos lo acuerden formalmente).

| Código | Significado | Origen | Gravedad (UI) |
|---|---|---|---|
| 0 | Nada | — | — |
| 1 | Homes demasiado cerca: START rechazado | Supervisor | aviso |
| 2 | Pérdida de rastreo (> 0.2 s sin rastreo válido). En el aire, con `mision.t_perdida_aterrizar_s` = 0.2 s (valor por defecto), ese dron **aterriza** en ese mismo paso (takeoff = 0, nunca más takeoff aunque vuelva el rastreo) y, en dual, el otro se aparta, regresa y aterriza (estado 9 → 7 → 8, o 7 → 8 en solo; ver abajo): **no hay PAUSED**. Solo si se configura un valor mayor, una pérdida entre 0.2 s y ese valor da PAUSED, los dos en hover | Supervisor | grave |
| 3 | Timeout de despegue (`t_takeoff_max`) | Supervisor | grave |
| 4 | Timeout de aterrizaje (`t_land_max`, por paso) | Supervisor | aviso |
| 5 | E-stop del operador | Operador | grave |
| 6 | Switch Takeoff/Land abajo: LAND ALL ordenado | Operador | aviso |
| 7 | Switch Arm abajo: desarma y LAND ALL | Operador | grave |
| 8 | Respaldo del operador: el supervisor no aceptó el LAND ALL en 2 s, o pasaron 90 s con el switch abajo; se soltó el takeoff | Operador | grave |
| 9 | Falla a bordo de un dron: FSM a bordo 7 u 8, o 9 con takeoff = 1, o enlace perdido con takeoff = 1 (solo versión MPC; ver §3.2) | Tierra MPC | grave |
| 10 | Protocolo MPC incorrecto: respuesta del dron sin `y(8) = 2` o FSM fuera de 1..9 durante > 0.5 s | Tierra MPC | grave |

Un código > 10 o negativo se muestra como "alerta desconocida" (grave). Un y(55) **no finito** (NaN o inf) **no** es "nada": la estación lo convierte en el código interno −1 (`protocol.ALERTA_NO_FINITA`) y lo muestra como alerta grave "y(55) no es un número" (telemetría corrupta o TELEMETRY PACK mal cableado).

**Objetivo rotando (y(56:60)).** En cada mensaje va **un** objetivo de la tabla de misión de Simulink: `[id clase x y estado]`, con estado 1 = pendiente, 2 = asignado, 3 = visitado. Rota por toda la tabla, así que con N objetivos cada uno se repite cada N mensajes (≤ 32 × 20 ms = 0.64 s). `id = 0` = ranura vacía; solo valen ids 1..32 (los de `visited_mask`). Así la estación ve los objetivos que conoce Simulink aunque no los haya publicado ella (por ejemplo, el simulador 3D con su detector interno). La estación:
- procesa **cada** mensaje de telemetría que llega, también cuando llegan varios juntos en una ráfaga (si solo leyera el último, con ráfagas regulares algunos objetivos no se verían nunca);
- une su tabla con la de Simulink **por id** (mismo id = mismo objetivo; no lo duplica) y **por cercanía**: Simulink guarda como *alias* (sin agregarlo a su tabla) un id nuevo a menos de `radio_cluster` (`vision.radio_cluster_m`, 0.25 m) de un objetivo que ya tiene, sin mirar la clase. Por eso un objetivo de Simulink con otro id a menos de `radio_cluster` de uno de la estación es el mismo objeto: se muestra una sola fila (la de la estación, con "= #id de Simulink");
- usa `visited_mask` y `responder_target_id` para el estado, porque llegan en cada mensaje, y de y(56:60) solo toma "visitado" (es permanente);
- olvida los objetivos de Simulink cuando `n_targets_known = 0`, cuando Simulink se reinicia, o si con telemetría fresca no vuelven a aparecer en 5 s;
- nunca asigna un id nuevo menor o igual que los que ya conoce Simulink.

**`cmd_seq_ack` (y(54)) significa "recibido", no "ejecutado".**
- Simulink confirma **todo** cambio de `cmd_seq` con heartbeat vivo, aunque el comando no aplique en el estado actual (START con los homes cerca, PAUSE fuera de MISSION, RESUME sin rastreo…). Para saber si se ejecutó hay que mirar `mission_state` y `alerta`.
- **Primer contacto:** con el primer heartbeat, el `cmd_seq` que llegue solo se confirma y **no se ejecuta**. Así un START viejo de la estación no hace despegar a los drones.
- **Petición del operador (modelos de tierra):** el OPERATOR COMMAND MERGE inyecta un LAND ALL durante **un paso (1 ms a 1 kHz)** con `heartbeat + 0.5` y `cmd_seq + 0.5`. En el paso siguiente vuelve al `cmd_seq` real de Python con `cmd = 0`. Consecuencias:
  - y(54) puede valer `seq + 0.5` durante ~1 ms. La estación **no redondea** y(54) (redondear 1.5 daría 2 y confirmaría por error otro comando).
  - Un comando de Python que estaba en curso queda confirmado sin ejecutarse.

**Comandos y detecciones, Python → Simulink (32 doubles):**

| Índice | Campo |
|---|---|
| 1 | `heartbeat` (contador creciente) |
| 2 | `cmd`: 0 nada, 1 START, 2 ABORT (regresar a home y hover), 3 LAND_ALL, 4 PAUSE, 5 RESUME |
| 3 | `cmd_seq` (incrementa con cada comando nuevo) |
| 4 | `n_targets` (total de objetivos publicados, ≤ 32) |
| 5–19 | 3 ranuras de objetivo, cada una `[id, clase, x, y, confianza]`. Python las rota entre todos los objetivos conocidos; Simulink actualiza la entrada por `id`. Un `id` = 0 indica ranura vacía. **Ids válidos: 1..32** |
| 20 | `z_explorer` (0 = usar valor por defecto) |
| 21 | `z_responder` (0 = usar valor por defecto) |
| 22 | `t_mark` (0 = usar valor por defecto) |
| 23 | `d_min_h` (0 = usar valor por defecto) |
| 24–32 | Reservado |

- Si no llega un heartbeat durante más de 1 s en MISSION, el supervisor pasa a **PAUSED**: ambos drones quedan en hover en su sitio.
- **Pérdida de rastreo en vuelo (24-nov).** Más de `mision.t_perdida_aterrizar_s` sin rastreo válido en el aire (0.2 s por defecto = `t_perdida_trk`; válido de 0.2 a 2.0 s, si no se usa 0.2 con aviso): ese dron queda en falla con código 2 (alerta 2; `tel.falla` va al bus Mission Info 30/31 de Simulink, **no** a la telemetría y(1..64): la estación sabe qué dron fue por su `tracked` = 0 en la trama) y se le baja takeoff en ese paso; nunca se le vuelve a dar. Motivo: sin pose el stack a bordo no sostiene el hover (vuelos 2-4 del 24-nov: derivó ~0.5 m y cayó fuera de la arena); con `isTracking = 0` sostenido 0.2 s la FSM a bordo del Equipo2 ya pasa a 7 (rampa de bajada sin control de posición, **irreversible**). Tierra le reenvía al dron el `isTracking` de OptiTrack, así que ve la pérdida antes que el dron: por eso el umbral es el **mismo** 0.2 s, sin margen (con 0.3 s, una pérdida de 0.2-0.3 s o intermitente dejaba la misión en PAUSED, con RESUME posible, y un dron que ya caía). Un valor mayor solo sirve con un stack que sostenga el hover sin rastreo. Pérdidas ≤ 0.2 s no hacen nada. En dual el otro dron hace la reacción de falla a bordo (§3.2 / `qd2_mission_step`): se aparta a su altura a ≥ `d_min_h + margen + 0.05 + 0.5 m` (deriva) de la última pose del perdido, regresa a su home (si queda lejos del perdido) y aterriza; DONE. Si el perdido está encima del otro y cerca, su takeoff se baja cuando el otro ya salió de debajo. Si durante esa reacción el que pierde el rastreo es el sano, aterriza en su sitio; al dron en falla se le baja takeoff también, salvo que esté encima del sano y cerca: entonces conserva takeoff y consigna hasta ver al sano en el suelo o `t_falla_sep_max` (8 s). Solo explorador: aterriza en su sitio. En la rampa de despegue sin nadie en el aire: despegue cancelado. En LANDING: sigue, sin alerta 4 por no verlo en el suelo. Alerta: sigue siendo la 2 (la 9/10 tienen prioridad).
- **Ids 1..32.** `visited_mask` solo representa 32 ids. La estación nunca asigna un id > 32: si se acaban, la anomalía confirmada **no se publica** y la UI muestra un aviso rojo. El puente también descarta cualquier id fuera de 1..32.
- **Overrides 20–23.** Solo se leen **antes de START**; durante el vuelo quedan fijos. Están acotados: `d_min_h` solo puede aumentar (hasta 1.5 m); `t_mark` entre 0.5 y 30 s; alturas entre 0.3 m y `z_max`; en MPC además `z_R ≤ z_max − dz_min` y `z_E ≥ z_R + dz_min`.
- **Reconexión.** En cada conexión nueva, la estación no manda objetivos hasta recibir la primera telemetría. Así, si Simulink se reinició (tabla vacía, `t` vuelve a 0), la estación vacía su lista antes de mandarle objetivos de la corrida anterior.

### 3.4 Video (dron explorador → PC)
`web/drone/qdrone2_camera_streamer.py` corre en el Jetson. Captura la RealSense D435i en color a 640×480 y 15 fps, opcionalmente con profundidad, y la sirve en `tcp :18800` con el protocolo QDF1 ya existente (cabecera `<4sBxxxHHI` + JPEG). El montaje mira hacia abajo y la rotación se configura en `camera.rotacion_montaje`.

## 4. Librería de algoritmos (MATLAB, compatible con codegen, sin toolboxes)
Ubicación: `simulink_qdrone2/lib/`. Estas funciones se usan desde bloques **MATLAB Function** (no Stateflow: los modelos originales solo usan MATLAB Function) y también desde el simulador 3D. Todas deben compilar con `coder` en R2022a, sin `containers.Map`, sin cell arrays dinámicos y con tamaños fijos.

Las firmas exactas están en `simulink_qdrone2/lib/README.md`; aquí va el resumen.

| Función | Firma (resumen) |
|---|---|
| `qd2_params()` · `[p, aviso] = qd2_params_from_config(...)` | Struct con todos los valores por defecto (alturas, `d_min_h`, `dz_min`, `t_mark`, tolerancias, límites de la arena, ganancias, parámetros del MPC, `modo_mision` = 0 dual / 1 solo explorador y `dron_solo` = 1 o 2). La segunda lo llena con `config/estacion.json` (solo MATLAB, no codegen); `aviso` explica un `mision.modo` / `mision.dron_solo` inválido (se usa dual) o un `mision.t_perdida_aterrizar_s` fuera de [0.2, 2.0] s (se usa 0.2 s). |
| `qd2_assign_roles(p1, p2, tracked1, tracked2)` | Devuelve `[explorer_id, responder_id, home1(2), home2(2), ok]`. La menor x es el explorador. Requiere los 2 rastreados de forma estable durante 1 s. |
| `qd2_track_waypoints(vertices, spacing, z [, n_vertices])` | Densifica la polilínea de la N en waypoints `[128x4]` (tamaño fijo) y un conteo. |
| `s = qd2_mission_init(p)` · `[s, sp_E, sp_R, tel] = qd2_mission_step(s, in, p)` | Supervisor de misión (FSM de §3.3). `s` es un struct de tamaño fijo con la tabla de 32 objetivos. `tel` trae los campos de §3.3, incluida `alerta`. |
| `[spE_o, spR_o, guard_active, gs] = qd2_separation_guard(pE, pR, spE, spR, p [, gs, ctx])` | Versión PID. Ajusta las consignas para mantener la distancia horizontal ≥ `d_min_h` entre poses y entre consignas. Prioridad: E sigue explorando; R espera en un punto seguro. Si R está marcando y E se acerca, E espera. |
| `y = qd2_rate_limit(y_prev, u, rate, dt [, xy_norma])` | Limitador de consignas `[x y z yaw]`. Con `xy_norma = 1` limita x,y **por la norma** (camino recto). Ver el requisito de abajo. |
| `[u, mst, info] = qd2_mpc_position(x_est, ref, other, p, mst)` | MPC lineal a bordo, 50 Hz, horizonte N = 20. Modelo de doble integrador con actitud de primer orden. QP con cotas en las entradas (roll, pitch, empuje), resuelto con FISTA de iteraciones fijas. Incluye observador de perturbación en z (offset-free) y *feedforward* de downwash si el otro dron está encima. `u = [T roll pitch yaw_rate status]`. |
| `y64 = qd2_telemetry_pack(...)` | Arma los 64 doubles de §3.3 (y(55) = alerta, y(56:60) = objetivo rotando, y(61:63) = roll, pitch y validez de la actitud del explorador, y(64) = `dron_solo`). |
| `[est, t_mal] = qd2_enlace_mpc(t_mal, t, tel1, tel2, lb, cs, p, version)` | MPC LINK CHECK de tierra (§3.2): valida la respuesta de 8 doubles de cada dron y entrega `Onboard Status [4] = [fsm1 fsm2 cod1 cod2]`. |
| `qd2_quad_plant(x, u, dist, dt)` · `qd2_onboard_step(...)` | Planta y emulación del stack a bordo para simulación: traslación 3D, actitud de primer orden, ruido y downwash. |
| `qd2_downwash(p_low, p_high)` | Fuerza vertical y ruido sobre el dron de abajo en función de Δxy y Δz. |

**Requisitos de la librería y de quien la usa (contrato):**
- **Limitador de consignas por norma.** Después del SEPARATION GUARD, la consigna x,y se limita con `qd2_rate_limit(..., 1)` dentro de un bloque MATLAB Function. **No** se usa el `Rate Limiter2` original, que limita cada eje por separado: con él, un movimiento diagonal sigue un camino en "L", mientras que el guard y la planeación del regreso suponen una recta. En pruebas, con el limitador por eje 1 de 30 escenarios bajó a 0.878 m (< `d_min_h` = 0.9 m); por norma, el mínimo fue 1.065 m en 90 escenarios. Aplica a los modelos de tierra, a `QD2_Sim_Dual` y al simulador 3D.
- **START rechazado si los homes están cerca:** a menos de `d_min_h + guard_margen` (1.05 m) en PID, o de `d_home_min_mpc` (0.8 m) en MPC. Se pone la alerta 1 y el comando se confirma (ack) pero no se ejecuta. Hay que separar los drones y volver a dar START.
- **ABORT o LAND_ALL antes de despegar cancelan el despegue** (TAKEOFF sin ningún dron por encima de `z_despegado` = 0.15 m). Nunca se da un takeoff nuevo:
  - si takeoff aún no se dio, se desarma y se vuelve a ASSIGN_ROLES (se puede dar START otra vez);
  - si ya se dio (rampa de empuje a bordo), se baja takeoff de los dos y se termina en DONE;
  - si algún dron ya despegó, ABORT y LAND_ALL funcionan normal.
- **Overrides 20–23 de Python:** solo antes de START y con los límites de §3.3.
- **Ids de objetivo 1..32:** la tabla tiene 32 entradas y `visited_mask` solo representa ids 1..32. Un id > 32 se visitaría pero no aparecería en la máscara; por eso la estación nunca los manda.
- **Timeouts:** `t_takeoff_max` = 30 s (alerta 3) y `t_land_max` = 30 s por paso de LANDING (alerta 4).

## 5. Simulink (copias que extienden los modelos del Equipo2)
- **Originales intactos** en `simulink_qdrone2/equipo2_original/`.
- **Modelos nuevos:**
  - `simulink_qdrone2/pid/QD2_MissionCtrl_Dual_PID.slx`: tierra, versión PID. El dron usa el binario original del Equipo2.
  - `simulink_qdrone2/mpc/QD2_MissionCtrl_Dual_MPC.slx`: tierra, versión MPC.
  - `simulink_qdrone2/mpc/QD2_DroneStack_MPC.slx`: dron, versión MPC.
  - `simulink_qdrone2/sim/QD2_Sim_Dual.slx`: simulación SIL; cambia entre PID y MPC con una constante.
  - `simulink_qdrone2/lib/QD2_Mission_Lib.slx`: librería con los subsistemas nuevos.
- **Formato:** se respeta religiosamente el acomodo y el estilo de los originales (nombres de subsistema en MAYÚSCULAS, colores, áreas, Goto/From, posiciones). Lo nuevo se agrega en espacio libre, siguiendo la misma retícula y convención.
- **Requisitos de los modelos de tierra** (detalle en `simulink_qdrone2/pid/README.md`):
  - **SETPOINT RATE LIMITER por norma** (`qd2_rate_limit(..., 1)` en un MATLAB Function) entre el SEPARATION GUARD (o STACKING INFO) y los drones. La cadena original Waypoint → Saturation2 → Rate Limiter2 solo queda para el modo MANUAL del Equipo2 (Command Source = 1, solo dron 1).
  - **OPERATOR SAFETY LAYER DUAL + OPERATOR COMMAND MERGE:** los switches del operador piden LAND ALL al supervisor (alertas 5–8) y el merge lo inyecta con `cmd_seq + 0.5` durante 1 ms (§3.3).
  - **TELEMETRY PACK:** y(55) = alerta (la del operador tiene prioridad) e y(56:60) = objetivo rotando, igual que el simulador 3D.
  - **Versión MPC:** manda la etiqueta 2 en el índice 24 (§3.2) y publica las alertas 9 y 10.
- **Versiones.** Los scripts `simulink_qdrone2/construir_modelos.m` (librería y modelos de tierra PID y MPC) y `simulink_qdrone2/construir_dron_y_sim.m` (`QD2_DroneStack_MPC` y `QD2_Sim_Dual`) generan todo de forma programática a partir de las copias. Después exportan con `ExportToVersion` a `simulink_qdrone2/versiones/{R2022a,R2022b,R2023a,R2023b,R2025a}/`.
- **Configuración.** `qd2_cargar_config.m` lee `config/estacion.json` (rutas de Motive, IDs de rigid body, IPs, pista, alturas, modo de misión) en `InitFcn`. Es la única fuente de verdad, compartida con la UI. Imprime el modo (`MODO SOLO EXPLORADOR -> dron N …` o `modo dual (2 drones)`).
- **Modo solo explorador sin regenerar modelos.** Vive en las funciones `.m` de `lib/` y en el struct `P` (`QD2P`), que ya llega como parámetro a MISSION SUPERVISOR, SEPARATION GUARD y SETPOINT RATE LIMITER; el TELEMETRY PACK lo deduce de `explorer_id`/`responder_id` (Mission Info). Ningún `.slx` (tierra PID/MPC, `QD2_Sim_Dual`, `QD2_DroneStack_MPC`, librería, `versiones/`) se regeneró, así que tierra MPC y dron MPC siguen siendo de la misma construcción. La PC compila los mismos `.slx` y toma las `.m` nuevas.

## 6. UI web (`web/`) y configuración (`config/estacion.json`)
- **Mapa en tiempo real (2D y 3D):** arena 3×3, polilínea de la N, ambos drones con su rol, sus consignas y su estela, la huella de la cámara del explorador, las anomalías (fuego o toma) con su estado (detectada → asignada → visitada) y coordenadas, y el feed de cámara si existe.
- **Conexiones:**
  - La IP de la PC se detecta por interfaz; se recomienda la de la antena Alfa ("Wi-Fi 2").
  - Las IPs de dron 1 y dron 2 se editan en la UI, quedan pendientes hasta llenarlas, y tienen botón **copiar**.
  - La UI genera los comandos `quarc_run` / `-URI_Host`, listos para copiar o ejecutar.
- **Calibración de Motive:** se eligen los archivos `.cal` y `.motive` (navegador de la carpeta de OptiTrack o subida de archivo) y se mapean los IDs de rigid body a dron 1 y 2. Se guarda en `estacion.json` y Simulink lo lee al construir.
- **Medición de la pista:** botón "capturar vértice" que guarda la (x, y) actual de un rigid body elegido (por ejemplo, un dron apagado colocado sobre cada extremo de tubo).
- **Controles:** START, PAUSE, RESUME, ABORT/RTH, LAND ALL, y selección de versión PID/MPC (solo informativa; la versión la define el modelo que esté corriendo).
- **Alerta y(55):** barra visible bajo los controles con el código y su texto en español: amarilla = aviso (1, 4, 6), roja = grave (las demás). Cada cambio queda en Eventos.
- **Objetivos:** la tabla y el mapa unen los que publicó la estación con los que Simulink reporta en y(56:60), por id y por cercanía (alias de Simulink, §3.3) (columna *Origen*: estación / ambos / Simulink, con "= #N" si Simulink lo tiene con otro id; en el mapa 2D, anillo punteado = solo Simulink).
- **Estado de cada dron:** "Flight State (tierra)" en PID y "FSM a bordo" en MPC, según y(3). Las fallas se marcan en rojo; el 9 al final de un vuelo no es falla, y con la alerta 9 solo sale en rojo el dron que la estación vio fallar (§3.3). La tarjeta de cada dron muestra la distancia a su home y "en home" cuando está a menos de `mision.tol_xy`: en LANDING cada dron baja **en su sitio**, así que el operador da LAND ALL cuando los dos están en home.
- **Prueba punta a punta:** `docs/PRUEBA_E2E.md` (estación real contra `QD2_Sim_Dual` y contra el simulador 3D, y el paso a los drones reales).
- **Modos de fuente:**
  - `simulink`: puente :18700 con el modelo real o con `QD2_Sim_Dual`.
  - `sim3d`: el simulador MATLAB.
  - `mock`: generador Python interno para probar la UI sin MATLAB. Emite y(55) (y puede forzar cualquier alerta 0..10; la tarjeta de pruebas muestra la que está forzada) e y(56:60), incluidos objetivos "internos" que la estación no publicó. También emite y(61:63): la actitud del explorador que hace falta para dar la aceleración de su planta, con la convención de §3.3. Guarda alias como Simulink (§3.3). Sigue las reglas de §4 para el despegue: takeoff 2 s después de armar, y ABORT/LAND_ALL antes de despegar cancelan el despegue.
  - Las tres fuentes pasan por el mismo cliente del puente, así que y(55) e y(56:60) se leen igual en todas.

## 7. Visión
- **Modelo:** YOLO11n (Ultralytics) con 2 clases: `fuego` y `toma`. Entrena en la RTX 4060 (laptop) y apunta a una RTX 5070 de 8 GB.
- **Datos:**
  - Sintéticos renderizados en Blender: piso de tapete oscuro, tubos PVC blancos, las piezas impresas (STL de `impresion3d/stl`), distractores (plantas, carritos, cilindros), cámara cenital entre 0.3 y 1.8 m con los intrínsecos de la D435i color (VFOV 42.5°; el HFOV de 69.4° solo vale en 16:9, a 640×480 es ~55° con fx ≈ fy ≈ 617 px) e iluminación aleatoria.
  - Negativos (fotos sin piezas, etiqueta vacía) contra falsos positivos: 1 500 fotos de COCO val2017 y capturas de webcam (`vision/negativos_coco.py`; `vision/README.md`, "Negativos").
  - Fotos reales que se capturen en el laboratorio (flujo de etiquetado incluido).
- **Geolocalización:** el centro de cada bbox se convierte en un rayo de cámara (intrínsecos y extrínsecos del soporte). Ese rayo se rota con la pose OptiTrack completa del explorador, interpolada a la hora del frame: x, y, z, yaw y, cuando y(63) = 1, roll y pitch de y(61:62) (§3.3). Luego se interseca con el plano z = 0.02 m. La cuenta está en `web/app/geo.py`:
  - `R_wb = Rz(yaw) · Ry(pitch) · Rx(roll)` (`R_mundo_cuerpo`, la única función que fija la convención).
  - Orientación de la cámara: `R_wc = R_wb · R_bc`, con `R_bc` el montaje hacia abajo más `rotacion_montaje`.
  - Posición de la cámara: `p_c = p + R_wb · offset_camara_m + b · (ẑ − R_wb · ẑ)`. El offset también se inclina, y gira alrededor del **pivote de Motive**, no del punto de la pose: la Corrected Pose Data es `pivote + bias_z · ẑ` (Bias [0 0 bias_z] en z del mundo, Measurement Conditioning), un punto que se queda `b = −bias_z` (0.07 m, `optitrack.bias_z_dronN`) debajo del pivote aunque el dron se incline. `offset_camara_m` se mide con el dron nivelado desde ese punto. Nivelado, el término de `b` es exactamente 0; ignorarlo dejaría un error de ≈ b·sin(inclinación) (~12 mm a 10°).
  - Sin la corrección, 5° de inclinación a 1.5 m movían el punto unos 0.13 m, y 10° unos 0.26 m (hasta 0.37 m en las esquinas). Eso se comía el margen de los filtros de tubo (0.25 m) y de tamaño (0.35 m). Ojo con el signo: con la nariz abajo la cámara ve hacia **atrás** del dron.
  - Parámetros en `config/estacion.json` → `vision`:
    - `usar_actitud` (true). Con false se supone el dron nivelado, como antes, y no se descarta por inclinación.
    - `inclinacion_max_deg` (15). Inclinación total `acos(cos roll · cos pitch)`. Si se pasa, la detección se descarta con el motivo "inclinación": se ve en gris en la UI, se cuenta en el estado y no entra al cluster. null lo desactiva.
    - `actitud_signos` ([1, 1] = [roll, pitch]) y `actitud_offset_deg` ([0, 0]). Corrigen un signo invertido o un rigid body creado un poco inclinado, sin tocar código.
  - Si llega y(63) = 0 (modelo de Simulink viejo, explorador sin rastreo), se geolocaliza nivelado. La estación lo indica en la pestaña Anomalías, tarjeta Visión (línea "Actitud: no disponible …"), en la fila *actitud (cruda)* de la tarjeta del explorador ("no disponible (y(63) = 0)") y con un aviso amarillo durante la misión. La tarjeta del explorador muestra y(61:62) tal como llegan; si `actitud_signos` / `actitud_offset_deg` no son los neutros, o con `usar_actitud` = false, agrega la fila *actitud usada* con la que entra a la geolocalización.
  - Las observaciones se agrupan (radio 0.25 m, mínimo 3 detecciones) y el resultado se publica como objetivo con posición promedio.
- **Entrenar una vez:** `vision/entrenar.py` guarda `vision/modelos/mejor.pt` junto con métricas (`vision/modelos/metricas.json`) y exporta `mejor.onnx`. La estación usa el modelo guardado y **no reentrena** salvo que se pida. Sin torch (venv `qd2_web`), la estación corre `mejor.onnx` con OpenCV en CPU.
- **Modelo actual (22-nov-2026):** el modelo sintético (80 épocas con ~10 000 imágenes, `sintetico_10k` + `sintetico_smoke`; mAP50 = 0.995 en validación sintética) afinado 25 épocas con negativos (1 500 fotos de COCO y 82 capturas de webcam sin piezas) más 16 capturas de webcam con fuego. Con el motor de la estación (OpenCV-ONNX, conf ≥ 0.5): fotos de COCO apartadas con "toma" falsa 28.8 % → 0 % y con "fuego" falso 12.2 % → 0.4 %; capturas de la cara sin piezas con alguna detección 44/50 → 0/50; fuego detectado en 56 capturas de webcam 50/56 → 47/56. mAP50 en la validación del entrenamiento = 0.9945. Todavía **sin fotos del laboratorio** (cámara cenital sobre el tapete): falta capturarlas y etiquetarlas (`vision/README.md` §3, "Negativos" y §4).
