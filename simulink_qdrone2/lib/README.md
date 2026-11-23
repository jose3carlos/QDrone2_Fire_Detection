# Librería de misión QDrone 2 (`simulink_qdrone2/lib`)

Funciones MATLAB compatibles con codegen (tamaños fijos, sin cell arrays, sin strings, sin `containers.Map`) para los bloques **MATLAB Function** de los modelos de tierra, del dron (MPC) y de la simulación, y también para el simulador 3D. Implementa el contrato de `docs/ARQUITECTURA.md` §4. Unidades SI en todo.

Probado en R2025a (MATLAB + Simulink): 32/32 pruebas pasan (22-nov-2026; 30 tras la revisión de la reacción a falla a bordo, más `test_actitud_y61` y `test_ext_mode_args`), y un modelo Simulink generado por código compila y simula todas las funciones (ver `../tests/README.md` y la sección "Resultados" al final). Solo usa funciones disponibles en R2022a, pero en R2022a no se probó.

## Cómo usarla en un bloque MATLAB Function

```matlab
function [spE, spR, ctx, tel64] = fcn(t, pose1, pose2, trk1, trk2, py32)
persistent s p
if isempty(p)
    p = qd2_params();          % o un parámetro del bloque con qd2_params_from_config()
    s = qd2_mission_init(p);
end
in.t = t; in.pose1 = pose1; in.pose2 = pose2;          % [x y z yaw]
in.tracked1 = trk1; in.tracked2 = trk2; in.py = py32; in.version = 1;
[s, spE, spR, tel] = qd2_mission_step(s, in, p);
ctx = tel.ctx_guard;
```

- Declara los tamaños de las salidas del bloque (por ejemplo `spE` `[1 4]`, `ctx` `[1 6]`, `tel64` `[64 1]`). Así lo hace `../tests/qd2_prueba_simulink.m`.
- Para usar los valores de `config/estacion.json` (vértices de la N, alturas, `d_min_h`…), construye `QD2P = qd2_params_from_config();` en `InitFcn` y pásalo al bloque como parámetro. Esta función solo corre en MATLAB, no en codegen.
- La primera compilación de un bloque que llama a estas funciones tarda unos 2 minutos.

## Funciones públicas (firmas exactas)

| Función | Firma |
|---|---|
| Parámetros | `p = qd2_params()` · `p = qd2_params_from_config([ruta_json o struct])` |
| Roles | `[explorer_id, responder_id, home1, home2, ok] = qd2_assign_roles(p1, p2, tracked1, tracked2)` |
| Waypoints | `[wp, n] = qd2_track_waypoints(vertices, spacing, z [, n_vertices])` → `wp` 128x4 `[x y z yaw]` |
| Supervisor | `s = qd2_mission_init(p)` · `[s, sp_E, sp_R, tel] = qd2_mission_step(s, in, p)` |
| Separación (PID) | `gs = qd2_guard_init()` · `[spE_o, spR_o, guard_active, gs] = qd2_separation_guard(pE, pR, spE, spR, p [, gs, ctx])` |
| MPC | `mst = qd2_mpc_init(p)` · `[u, mst, info] = qd2_mpc_position(x_est, ref, other, p, mst)` → `u = [T roll pitch yaw_rate status]` |
| Telemetría | `y64 = qd2_telemetry_pack(t, tel, version, guard_active, pose1, pose2, sp1, sp2, vel1, vel2, aux[, att])`. `att` (opcional) = `[roll1 pitch1 roll2 pitch2]` (rad, por dron) de la Corrected Pose Data `d(4:5)`; llena **y(61)** = roll y **y(62)** = pitch del explorador, y **y(63)** = 1 si son válidos (explorador 1 o 2, rastreado y ángulos finitos; si no, 0 0 0). Sin `att` (llamadores viejos), y(61:63) = 0. y(64) reservado (0) |
| Enlace MPC (tierra) | `[est, t_mal] = qd2_enlace_mpc(t_mal, t, tel1, tel2, lb, cs, p, version)` → `est = [fsm1 fsm2 cod1 cod2]` (cod 0 bien, 1 protocolo incorrecto, 2 enlace perdido; `t_mal` = `[t_mal1 t_mal2 visto1 visto2]`, inicia en `[-1 -1 0 0]`; se acepta `[-1 -1]`) |
| Planta | `x = qd2_quad_plant(x, u, dist, dt [, p.planta])` con `x` 10x1 `[x y z vx vy vz roll pitch yaw T]` |
| Downwash | `[F, sigma] = qd2_downwash(p_low, p_high [, p.dw])` |
| Simulación a bordo | `ob = qd2_onboard_init(p)` · `[u, ob] = qd2_onboard_step(ob, arm, takeoff, ref, x_est, other, p, dt, modo)` |
| PID original | `[u, st] = qd2_pid_position(x_est, ref, st, p, dt)` |
| Utilidades | `qd2_rate_limit`, `qd2_ruta_desvio`, `qd2_dist_punto_segmento`, `qd2_dist_segmentos`, `qd2_wrap_pi`, `qd2_mpc_build` |

**Entrada `in` de `qd2_mission_step`:**
- `t`: tiempo en s.
- `pose1`, `pose2`: `[x y z yaw]` de cada dron.
- `tracked1`, `tracked2`.
- `py`: los 32 doubles de Python (§3.3).
- `version`: 1 = PID, 2 = MPC.
- **Opcionales** (versión MPC; si no existen, el supervisor se comporta exactamente como antes; probado: mismas consignas y estados sin los campos que con FSM desconocida):
  - `fsm_a_bordo` [1x2]: FSM a bordo de cada **dron** (índice 2 de la respuesta de 8 doubles, validada con `qd2_enlace_mpc`). 0 o NaN = desconocida.
  - `proto_mal` [1x2]: código del enlace de `qd2_enlace_mpc`. 1 = ese dron responde con protocolo MPC incorrecto desde hace más de `p.t_proto_mal` (0.5 s) estando conectado; 2 = enlace perdido (eco viejo ≥ 0.5 s o `Communication_Status` = 1) después de haber estado conectado. Con 2, la FSM llega como 0: la última respuesta retenida por el Stream Server no dice nada del estado actual.

**Salida `tel`:**
- Campos de §3.3: `mission_state`, `explorer_id`, `responder_id`, `explorer_wp_idx`, `explorer_wp_total`, `responder_target_id`, `n_targets_known`, `n_targets_visited`, `explorer_done`, `visited_mask`, `marking_progress`, `queue_length`, `cmd_seq_ack`.
- Home y rol de cada dron: `home1`, `home2`, `role1`, `role2`.
- Por **dron**: `sp1`, `sp2` (consignas), `arm` [1x2], `takeoff` [1x2], `estop` [1x2].
- Para el guard: `r_marking` y `ctx_guard`, que es `[r_marking, prioridad_R, d_min_h, solo_respaldo, look_x, look_y]`.
- Valores efectivos tras los overrides: `z_E`, `z_R`, `d_min_h`, `t_mark`.
- Estado del enlace: `hb_ok`, `pausa_por_hb`, `pausa_por_trk`.
- Seguridad: `alerta` y `home_ok`. `alerta` (índice **55** del mensaje de 64, códigos acordados por los dos equipos): 0 nada, 1 homes demasiado cerca (START rechazado), 2 pérdida de rastreo, 3 timeout de despegue, 4 timeout de aterrizaje, **9 falla a bordo de un dron** (FSM a bordo 7 u 8, o 9 / enlace perdido debiendo volar; versión MPC), **10 protocolo MPC incorrecto**. Los códigos 5-8 (E-stop, Takeoff abajo, Arm abajo, respaldo del operador) los pone la capa del operador en tierra, no esta librería; en el TELEMETRY PACK de los modelos de tierra la del operador prevalece sobre todas las de esta librería (también sobre 9 y 10).
- Falla a bordo: `falla` [1x2] por dron (0, 9 o 10), `falla_fase` (0 ninguna, 1 apartarse, 2 regresar, 3 aterriza el sano, 4 espera al dron en falla en el suelo) y `escape` [1x2] por dron (1 = el limitador de consigna debe usar `p.falla_rate_xy`).
- `obj_rot` [1x5] = `[id clase x y estado]`: un objetivo de la tabla que rota cada `p.t_rot_obj` (0.02 s); `qd2_telemetry_pack` lo pone en **y(56:60)** (estado 1 pendiente, 2 asignado, 3 visitado; `id` = 0 con la tabla vacía). Igual que el simulador 3D.
- **y(61:63)** no sale de `tel`: `qd2_telemetry_pack` toma roll y pitch del argumento `att` con el mismo criterio de rol que la pose (`explorer_id`). Convención de OptiTrack (Corrected Pose Data, Tait-Bryan ZYX intrínseca): R = Rz(yaw)·Ry(pitch)·Rx(roll) sobre el cuerpo FLU; roll > 0 baja el lado derecho, pitch > 0 baja la nariz (la misma de `qd2_quad_plant`). La estación la usa para geolocalizar con la inclinación del explorador; con y(63) = 0 geolocaliza nivelada.

**MPC:**
- `x_est = [x y z vx vy vz roll pitch yaw]`.
- `ref = [x y z yaw]`.
- `other = [x y z vx vy vz]` del otro dron (índices 17–22 del mensaje MPC; con `NaN` no hay feedforward).
- `info` trae `res` (residuo de FISTA por eje), `d_hat`, `a_ff` y `T_vert`.

## Comportamiento del supervisor

- **Arranque.** `INIT` → `WAIT_TRACKING`: ambos drones rastreados de forma continua durante 1 s. Luego `ASSIGN_ROLES`, que se reevalúa hasta START: el de menor x es el explorador y los home son las posiciones (x, y) de cada dron.
- **START.**
  - Al pulsarlo quedan fijos los roles y los home, y se genera la N empezando por el extremo más cercano al home de E.
  - **Se rechaza (alerta 1) si los home están demasiado cerca:** a menos de `d_min_h + guard_margen` (1.05 m) en la versión PID, o de `d_home_min_mpc` (0.8 m) en la MPC. Así nunca despegan uno junto al otro. El comando se confirma con `cmd_seq_ack` pero no se ejecuta; hay que separar los drones y volver a dar START.
  - `TAKEOFF`: arm, y 2 s después takeoff. Una vez dado, takeoff no se vuelve a bajar salvo por ABORT/LAND_ALL.
  - `HOVER_READY` dura 2 s y después empieza `MISSION`.
  - **Timeout de despegue** (`t_takeoff_max` = 30 s desde takeoff, alerta 3): si algún dron despegó (z > 0.15 m) pasa a `ABORT` (hover en home hasta LAND_ALL); si ninguno despegó, se cancela en tierra.
- **ABORT / LAND_ALL durante `TAKEOFF` sin ningún dron en el aire** (z ≤ `z_despegado` = 0.15 m). Nunca se da un takeoff nuevo:
  - Si takeoff aún no se dio: arm = takeoff = 0 y se vuelve a `ASSIGN_ROLES` (a bordo ARMED → IDLE, sin error). Se puede dar START otra vez.
  - Si takeoff ya se dio (rampa de empuje a bordo): se baja takeoff de los dos a la vez (`LANDING` paso 4) y luego `DONE`. Ojo: en el stack original, soltar trigger_2 en `TAKEOFF_THROTTLE` es un Error, así que el dron hace la rampa de emergencia en tierra y **el modelo a bordo termina**. Hay que relanzarlo.
  - Si algún dron ya despegó: ABORT y LAND_ALL normales.
- **Overrides de Python (20–23).** Solo se leen antes de START; durante el vuelo quedan fijos. Están acotados:
  - `d_min_h` solo puede **aumentar**, hasta 1.5 m. El guard aplica la misma regla.
  - `t_mark` entre 0.5 y 30 s.
  - Alturas entre 0.3 m y `z_max`.
  - En la versión MPC: `z_R ≤ z_max − dz_min` y `z_E ≥ z_R + dz_min`.
- **E (explorador).**
  - Avanza por los waypoints cuando está dentro de `tol_xy`/`tol_z` durante `t_dwell_wp`.
  - Al terminar la N vuelve a su home y **cede la prioridad** a R.
- **R (respondedor).**
  - Cola de objetivos: siempre elige el más cercano.
  - Hace hover `t_mark` sobre cada objetivo y lo marca como visitado. `marking_progress` va de 0 a 1.
  - Cuando está libre espera sobre el último objetivo.
- **Objetivos.**
  - Tabla de 32. Se actualiza por `id`.
  - Un `id` nuevo a menos de `radio_cluster` de otro objetivo (sin mirar la clase) se registra como **alias** (duplicado): no entra a la tabla ni sale en y(56:60) (ARQUITECTURA §3.3).
  - `visited_mask` usa el **id de Python** (bit id−1) e incluye los duplicados.
- **Comandos.**
  - Un comando se aplica cuando cambia `cmd_seq`. Se confirma (`cmd_seq_ack`) aunque no aplique en el estado actual.
  - Con el primer heartbeat, el `cmd_seq` que llegue solo se confirma y **no se ejecuta**. Así un START viejo de la estación no hace despegar a los drones.
- **Heartbeat.** Más de 1 s sin heartbeat en `MISSION` lleva a `PAUSED`: los dos drones quedan en hover en su sitio. Para salir hace falta `RESUME`.
- **Pérdida de rastreo (OptiTrack).** Rastreo válido significa `tracked = 1` y pose finita. Una pose NaN se reemplaza por la última pose finita, así que nunca entra NaN a la FSM ni sale en una consigna.
  - En `TAKEOFF` (con takeoff ya dado), `HOVER_READY`, `MISSION`, `RETURN_HOME` y `ABORT`: si un dron lleva más de 0.2 s sin rastreo válido, los dos pasan a `PAUSED` (alerta 2) y quedan en hover en su sitio. El dron perdido entra en emergencia a bordo. El otro no se mueve hacia una pose fantasma.
  - `RESUME` solo se acepta cuando los dos vuelven a estar rastreados. Desde la pausa también funcionan `ABORT` y `LAND_ALL`.
  - En `TAKEOFF` antes de dar takeoff: se desarma y se vuelve a `WAIT_TRACKING`.
  - En `LANDING` el aterrizaje sigue su curso.
- **Regreso a home (`RETURN_HOME` / `ABORT`).**
  - Las trayectorias nunca se cruzan: si los segmentos rectos quedan a ≥ `d_min_h + margen`, los dos regresan a la vez.
  - Si no, va primero el dron cuyo home no está bloqueado por el otro, con un desvío si hace falta. El segundo sale cuando el primero ya llegó.
  - Si los dos homes están bloqueados, R se estaciona en un punto libre.
  - `ABORT` se queda en hover en home hasta `LAND_ALL`.
- **Aterrizaje.**
  - R aterriza primero. E aterriza en su sitio si queda a ≥ `d_min_h` de R; si no, en su home o en el punto libre más cercano.
  - `DONE` llega con los dos en el suelo y pone `arm = 0`.
  - **Timeout por paso** (`t_land_max` = 30 s, alerta 4). Si R no se ve en el suelo (z ≤ 0.08 m), por ejemplo porque el pivot de OptiTrack quedó alto, E baja igual cuando ya está en su punto, que está a ≥ `d_min_h + margen` del de R. Lo mismo vale para E en el paso 2.
  - Ojo: a bordo, el fin del autoland exige z ≤ 0.05 m. Hay que revisar el pivot del rigid body en el laboratorio.
- **Versión MPC.** Se fuerza `z_E ≥ z_R + dz_min`.
- **Falla a bordo (solo con las entradas opcionales, versión MPC).** La versión PID no tiene FSM a bordo (el dron solo devuelve el eco), así que ahí nada cambia.
  - **Dron en falla:** armado y con FSM a bordo 7 (Error) u 8 (E-stop); o 9 (motores apagados) con takeoff = 1 (o armado en TAKEOFF); o **enlace perdido** (código 2) con takeoff = 1 (o armado en TAKEOFF): a bordo, la "Detect Communication Loss" lo lleva a 7 a los 0.5 s aunque tierra ya no lo vea; o protocolo MPC incorrecto sostenido. En LANDING, el 9 (o el modelo que termina) de un dron al que ya se le bajó takeoff es el fin normal del autoland y no cuenta.
  - **Antes de START** (estados 0-2), como dice ARQUITECTURA §3.2: un dron **desarmado** con FSM 7/8/9 **no** es alerta (un 9 después de aterrizar es el fin normal). Si se da START, en TAKEOFF ya armado se detecta, se cancela el despegue **sin dar takeoff** (vuelve a ASSIGN_ROLES desarmado) y queda la alerta 9 mientras ese dron siga en 7/8/9 (hay que relanzar su modelo). El protocolo incorrecto sí rechaza START (alerta 10) y la alerta se limpia cuando se corrige.
  - **En TAKEOFF sin nadie en el aire:** como ABORT antes de despegar (si takeoff no se dio, se desarma y vuelve a ASSIGN_ROLES; si ya se dio, los dos bajan takeoff y DONE).
  - **En el aire** (TAKEOFF con alguien despegado, HOVER_READY, MISSION, RETURN_HOME, ABORT, PAUSED) **y en LANDING mientras R todavía espera a su altura** (ver abajo): reacción, alerta 9 (10 si solo es el protocolo):
    1. **Apartarse** (estado 9): el dron sano va EN RECTA, a su altura (`z_E` o `z_R`), a un punto a ≥ `d_min_h + margen + 0.05` (1.10 m) del dron en falla, dentro de la arena. Si estaban apilados sale de debajo/encima en ese mismo paso, con límite xy `p.falla_rate_xy` = **1.5 m/s** (`tel.escape`; el normal es 1/3 m/s; con 1.0 m/s, cuando E caía a la altura de R la distancia horizontal era 0.852 m < `d_min_h`). El dron en falla conserva su consigna y su takeoff. Si el dron en falla se desplaza, el punto se recalcula.
    2. **Regresar** (estado 9): cuando ya está a ≥ `d_min_h` (o tras `t_falla_sep_max` = 8 s), al dron en falla se le baja takeoff (si aún volara, autoland en su sitio: ya no tiene a nadie debajo). El sano va a su home, con desvío alrededor del dron en falla (`qd2_ruta_desvio`); si su home queda a < 1.10 m del dron en falla, se queda donde está.
    3. **Aterriza el sano** (estado 7): takeoff = 0 en su punto.
    4. Espera a que el dron en falla esté en el suelo (o `t_land_max`) → **DONE** (arm = 0).
  - **Nunca** se vuelve a dar takeoff al dron en falla (solo puede bajar). Durante la reacción no se pausa por rastreo ni por heartbeat; solo se atiende LAND_ALL (el sano aterriza en cuanto está apartado; el estado pasa a 7 para que la capa del operador lo vea aceptado).
  - **Los dos en falla:** aterrizaje ordenado normal (R primero, E a ≥ `d_min_h`), por si aún obedecen.
  - **LANDING (paso 1) con los drones apilados** (versión MPC, p. ej. LAND_ALL de Python o del switch Takeoff del operador con E encima de R): **R no baja** mientras E esté a < `d_min_h` en horizontal de su punto; espera a su altura con takeoff = 1 (`s.land_r_baja` = 0). Así, si E falla en ese momento, R sigue en vuelo y la reacción completa lo saca de debajo. Cuando E ya está a ≥ `d_min_h`, R baja (autoland) y una falla posterior solo se avisa (alerta 9/10) y al dron en falla se le baja takeoff. En la PID (nunca apilados) R baja de inmediato como antes.
- **Objetivos: solo ids 1..32.** `visited_mask` y y(56) solo representan los ids de Python del 1 al 32, así que un objetivo con id < 1 o > 32 se **ignora** (no entra a la tabla ni sale en y(56:60)).

## SEPARATION GUARD (versión PID)

- **Prioridad de E.** E (o R, cuando E ya terminó) avanza salvo que el otro esté a menos de `d_min_h + m` de su segmento [pose, consigna]. En ese caso espera; solo se congela x,y y la z sigue libre.
- **Movimiento de R.** R va a su consigna solo si su camino recto queda a ≥ `d_min_h + 2m` de la zona de E. Esa zona incluye 2 m de la N por delante de E (`ctx(5:6)`).
- **Cuando el camino de R no está libre.**
  - R espera en un punto seguro.
  - Si ese punto deja de ser seguro, se evade sin acercarse a la posición actual de E.
  - Si no encuentra a dónde evadirse, se queda quieto.
- **R marcando.** Mientras R marca, E espera.
- `m = guard_margen = 0.15 m`.
- **Entradas inválidas.** Con una pose NaN usa la última pose finita; con una consigna NaN, la última salida. Las salidas nunca son NaN.
- **Override de `d_min_h`** (`ctx(3)`): solo se usa si es mayor que `p.d_min_h`.
- **`ctx`** puede tener 4 o 6 elementos, o faltar (llamada con 5 argumentos).
- **REQUISITO para el modelo PID (`QD2_MissionCtrl_Dual_PID`).** Después del guard hay que limitar la consigna en x,y **por la norma**: `qd2_rate_limit(..., 1)` dentro de un bloque MATLAB Function. **No** hay que usar el `Rate Limiter2` original, que limita cada eje por separado. Con limitación por eje los movimientos diagonales siguen un camino en "L", mientras que el guard y la planeación del regreso suponen una recta. Ya está en el contrato (`docs/ARQUITECTURA.md` §4 y §5) y en los modelos (SETPOINT RATE LIMITER).
  - Limitador por norma: 90 escenarios aleatorios, mínimo 1.065 m.
  - Limitador por eje: 1 de 30 escenarios bajó a 0.878 m (< `d_min_h`). La prueba `test_mision_pid_rl_por_eje` lo vigila.
- **Colocación inicial.** Coloca los drones a ≥ 1.2 m (`d_min_h + 2m`) al arrancar. Con menos de 1.05 m la misión rechaza START.

## MPC a bordo

- **Modelo.** Por eje, doble integrador con actitud de primer orden (`tau_att = 0.12 s`), discretizado a `Ts = 0.02 s`.
- **Horizonte y costo.** `N = 20`, con costo terminal de Riccati.
- **Cotas.** Roll/pitch ±0.35 rad; empuje entre 5 y 30 N.
- **Solver.** FISTA con 40 iteraciones fijas por eje, reinicio adaptativo y arranque en caliente.
- **Eje z.**
  - Observador de perturbación con polo doble a 6 rad/s.
  - Feedforward de downwash con `qd2_downwash` y la pose/velocidad del otro dron.
  - Compensación de inclinación.
- **Yaw.** Control P como el original: 7.5 (rad/s)/rad.
- **Identificación con el vuelo real del Equipo2** (`../tests/qd2_identificar_actitud.m`):
  - Actitud: retardo de ~0.10 s más un lag ≤ 0.02 s. El primer orden equivalente da 0.09–0.095 s.
  - Empuje medio en FLIGHT: **13.80 N**, contra el trim de 14.754 N del modelo. El observador absorbe esa diferencia: el error final es 0.0000 m con observador y 0.039 m sin él.
- **Limitaciones del modelo de simulación.**
  - `qd2_quad_plant` modela la actitud solo como primer orden (`tau_att = 0.12 s`), sin el retardo puro de ~0.10 s que se identificó. Por eso las pruebas de misión son algo optimistas. En una sonda aparte, con retardo de 0.10 s, el MPC no tuvo sobrepaso; a 0.14 s aparece una oscilación de unos 3 mm.
  - La fuerza de downwash `F0 = 5.9 N` es un **supuesto** (unos 20 % del peso a 1 m), no una medición. Hay que medirla en el laboratorio.

## Resultados (última corrida, R2025a, esta laptop, 22-nov-2026, tras la revisión): 30/30 (hoy 32/32, ver arriba)

- **Misión PID** (limitador por norma):
  - Distancia horizontal mínima medida: 1.135 m. Entre consignas: 1.200 m. Tras el rate limiter: 1.109 m.
  - 12 escenarios aleatorios: mínimo 1.120 m.
- **Misión PID con el `Rate Limiter2` original (por eje)**, 6 escenarios: [1.13 1.23 1.20 **0.878** 1.25 1.08] m. Uno queda por debajo de `d_min_h` = 0.9. Por eso es **obligatorio limitar por norma**.
- **Misión MPC:** 15.8 s de vuelo apilado con error en z de R de 0.015 m. FISTA: 0.22 ms por paso de media (p99 0.43 ms), status siempre 0.
- **Apilado, error máximo en z de R:**

  | Configuración | dz = 1.0 m | dz = 0.8 m |
  |---|---|---|
  | ff + observador | 0.016 m | 0.017 m |
  | sin ff ni observador | 0.157 m | 0.172 m |
  | PID original | 0.090 m | 0.099 m |

- **ABORT / LAND_ALL antes de despegar:**
  - A 0.5 s del START: `ASSIGN_ROLES`, y takeoff nunca vale 1.
  - Durante la rampa: `DONE` con z máxima 0.000 m.
  - Ya despegados: `ABORT`, luego `LAND_ALL` y `DONE`.
- **Homes a 0.45 m:** START rechazado, z máxima 0.000 m, alerta 1 en y(55).
- **Pérdida de rastreo del dron 2 en t = 40 s:**
  - `PAUSED` en t = 40.20 s; el dron rastreado deriva 0.064 m.
  - `RESUME` sin rastreo se ignora; con rastreo vuelve a `MISSION`, y la misión termina en `DONE`.
- **Timeouts:**
  - Despegue: nadie despegó, se cancela y llega a `DONE`. Si R no llega a su altura, `ABORT`.
  - Aterrizaje con pivot alto: `DONE` 33.8 s después de `LAND_ALL`, con alerta 4.
- **Simulink:** 8 bloques MATLAB Function compilados y 60 s simulados en 175 s. Estados 0–6 en la versión 1 y en la 2; distancia horizontal mínima 1.319 m.
- **Falla a bordo (pruebas nuevas, lazo cerrado MATLAB con el stack emulado y la rampa de emergencia del estado 7):**
  - `qd2_enlace_mpc`: respuesta válida → FSM; sin etiqueta y(8) = 2, FSM 10, 2.5 o 0 estando conectado → protocolo incorrecto a los 0.51 s; nunca conectado → nada; **enlace perdido después de haber estado conectado** (respuesta retenida con FSM 5, eco de 0.5 s o más) → FSM 0 y código 2 a los 0.50 s; al volver el enlace → FSM otra vez y código 0.
  - Antes de START: FSM 7, 8 o 9 con el dron desarmado → **sin alerta**; con START, ya armado en TAKEOFF → cancelado sin takeoff, ASSIGN_ROLES desarmado, alerta 9 hasta que el dron vuelve a 1; luego START despega normal. Protocolo incorrecto → START rechazado (alerta 10). Sin los campos opcionales, salida idéntica a FSM desconocida (diferencia 0).
  - **R falla DEBAJO de E** (apilados, dh 0.022 m, dz 1.00 m): alerta 9 al paso siguiente (0.02 s); E se aparta a 1.498–1.501 m de altura; dh 0.463 m a 1 s, 1.044 m a 2 s, ≥ d_min_h a los 1.58 s; E aterriza en su home; DONE 12.2 s después.
  - **E falla ENCIMA de R:** R recibe la orden en 0.02 s y se aparta a 0.503–0.511 m de altura; dh 0.664 m a 1 s, 1.195 m a 2 s, ≥ d_min_h a los 1.30 s. Cuando E (cayendo) llega a menos de 0.4 m de la altura de R, la distancia horizontal es **0.982 m** ≥ d_min_h (con 1.0 m/s era 0.852 m; con el límite normal de 1/3 m/s, 0.441 m). La prueba ahora exige ≥ `d_min_h`. DONE 7.7 s después.
  - **Falla en LANDING apilado** (LAND_ALL 1 s después de apilarse, falla 0.2 s después): sin falla, la distancia horizontal mínima mientras R baja con E más arriba es 1.090 m (R esperó a que E se apartara). E falla encima: hay reacción (antes no), distancia 3D mínima **0.988 m** (antes 0.378 m), dh al cruzar 1.017 m, DONE 8.8 s después. R falla debajo: 3D mínima 0.993 m, dh al cruzar 0.953 m, DONE 14.3 s después.
  - **Enlace perdido en los dos sentidos** (E apilado encima, `enlace_off`): a bordo FSM 7 a los 0.5 s; tierra da alerta 9 a los 0.48 s por el código 2; dh cuando E cae a la altura de R 0.880 m (E no tiene deriva horizontal en este caso; la prueba exige ≥ d_min_h − 0.05); DONE.
  - **Protocolo** (dron 2 pierde la etiqueta en MISSION a los 40 s): alerta 10 a los 40.52 s; el sano se aparta, regresa y aterriza; DONE a los 50.7 s; dh mínima ya separados 0.901 m.
  - En todos: al dron en falla nunca se le vuelve a subir takeoff y los dos terminan en el suelo.
  - y(56:60): con 3 objetivos, 6 mensajes seguidos enviaron los ids [7 9 4 7 9 4]; con ids 33, 40 y 32 publicados solo entra y sale el 32.
  - y(61:63) (`test_actitud_y61`, 22-nov): con explorador 1 o 2 se toma la actitud de ese dron. Con el explorador no rastreado, NaN/Inf en sus ángulos, sin explorador o con la llamada de 11 argumentos sale 0 0 0. El resto del vector no cambia con `att`.
- **Simulink/codegen** (`qd2_prueba_simulink`, también con `qd2_enlace_mpc` y los campos opcionales): compila y simula 60 s en 175 s de reloj.
