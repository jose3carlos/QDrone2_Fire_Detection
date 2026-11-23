# Dron MPC a bordo y simulación SIL dual

Este README cubre tres cosas:
- `mpc/QD2_DroneStack_MPC.slx`: el modelo a bordo, versión MPC.
- `sim/QD2_Sim_Dual.slx`: la simulación de los dos drones.
- `construir_dron_y_sim.m`: el script que genera ambos modelos.

Contrato: `docs/ARQUITECTURA.md`. Librería de algoritmos: `lib/` (`qd2_*.m`). Bloques de misión: `lib/QD2_Mission_Lib.slx`, que genera `construir_modelos.m`.

## Construir todo

```matlab
cd simulink_qdrone2
construir_dron_y_sim                  % dron + sim + capturas + versiones
construir_dron_y_sim(struct('sim', false))   % solo el dron
```

- Cada corrida parte de **copias**: el original del Equipo2 no se modifica.
- La salida va a tres lugares:
  - `mpc/QD2_DroneStack_MPC.slx` y `sim/QD2_Sim_Dual.slx`;
  - `versiones/{R2022a,R2022b,R2023a,R2023b,R2025a}/` (ExportToVersion);
  - `capturas/*.png`.
- La simulación usa los bloques de `lib/QD2_Mission_Lib.slx`. Si ese archivo no existe, primero corre `construir_modelos('verificar', false, 'exportar', false, 'capturas', false)`.
- **Ojo con QUARC en `-batch`**: la ventana "Tip of the Day" bloquea MATLAB. Para evitarlo, usa un `startup.m` que haga `error('fin')`; está explicado en `tests/README.md`. `construir_dron_y_sim` agrega por su cuenta la carpeta `QUARC/quarc` al path.
- `saved_data/` (logs `To Host File` de ~16 MB cada uno) está en `.gitignore`.

## 1. `QD2_DroneStack_MPC.slx` (a bordo)

Es una copia de `QD2_DroneStack_PID_2021a` con el mismo layout. Se le añadió lo siguiente:

- **COMMANDER (ANGLE MODE), área nueva "5. MPC Position Controller (50 Hz)":**
  - `qd2_mpc_position` corre en un bloque MATLAB Function a 50 Hz, entre dos `Rate Transition`:
    - de 1 kHz a 50 Hz es determinista;
    - de 50 Hz a 1 kHz es solo integridad, para que la salida llegue en cuanto termina el cálculo.
  - El modelo ya era MultiTasking, así que el MPC corre en su propio hilo y no frena el lazo de 1 kHz.
  - **Parámetros precalculados:** `qd2_params()` se evalúa al **compilar** (`coder.const(@feval, 'qd2_params')`). El Jetson no calcula la Riccati, `expm` ni los waypoints en tiempo real; el código generado lleva el struct como constante.
  - **Masa:** el MPC toma el trim del mismo "Drone Weight" que el PIV. Si ese peso no es 1.504 kg (por ejemplo, con un soporte de cámara), en tierra (FSM 1-2) se reconstruye **una vez** el modelo completo del MPC con `qd2_mpc_build`: trim, masa, matrices del eje z y cotas. Basta con cambiar "Drone Weight" en la raíz.
  - **Switch PID/MPC**: está justo después del mapeo XYZYaw→TRPY original, en el hueco entre las áreas 2 y 3/4.
    - El comando del MPC y la bandera "usar MPC" llegan por `From` locales (`MPC_Cmd`, `MPC_Use`) desde el área 5.
    - El MPC se usa solo si `mpc_mode = 1` (índice 23 del paquete), la FSM está en 4–6 y la entrada es válida.
    - En cualquier otro caso pasa el **PIV original** sin cambios.
  - FSM, THROTTLE SWITCHBOX (trim, despegue y aterrizaje), saturaciones, E-stop y Stabilizer quedan intactos.
  - El MPC entrega su comando en el mismo formato que el PIV: `[T−trim, roll, pitch, yaw_rate]`.
  - El MPC se reinicia al entrar a TAKEOFF_HEIGHT, igual que el integrador del PIV, y al pasar de PIV a MPC en vuelo.
  - **Estado MPC** (telemetría):
    - 0: ok.
    - 1: FISTA no convergió; el comando se aplica igual.
    - 2: entrada inválida; pasa el PIV.
    - 3: inactivo (FSM fuera de 4–6).
    - 4: se eligió el PIV; el MPC corre en sombra.
  - La regla `Δz ≥ dz_min` de la versión MPC **la garantiza tierra** (consignas de la librería de misión). `qd2_mpc_position` no la impone como restricción a bordo; solo usa la pose del otro dron para el feedforward del downwash.
- **Communication**: recibe **24 doubles** y envía **8** (ARQUITECTURA §3.2):
  - Recibe: `default_value = zeros(1,24)`. Los índices 17–24 van al Goto global `QD2_Datos_MPC`.
  - **PROTOCOL CHECK** (bloque naranja junto a la salida `rcv`): valida cada trama.
    - Se acepta solo si arm, takeoff, e-stop, joystick_issue, isTracking (índices 1–5) y `mpc_mode` (23) valen exactamente 0 o 1, **el índice 24 vale 2** (etiqueta "protocolo MPC v1") y el timestamp (16) es finito y ≥ 0.
    - **Estricto desde esta revisión:** antes aceptaba 24 = 0 o 2 porque tierra mandaba 0. Ahora `QD2_MissionCtrl_Dual_MPC` manda siempre 2 (STACKING INFO), así que se acepta solo 2. Es seguro porque una trama rechazada nunca arma el dron: con un MissionCtrl MPC viejo (24 = 0) el dron simplemente no arma (sin trama válida quedan arm = 0 e isTracking = 0). Consecuencia: **el dron y la tierra MPC deben ser de la misma construcción** (las carpetas `versiones/` se regeneraron juntas). Probado con `tests/qd2_probar_protocolo.m` (ver Resultados).
    - Si la trama no es válida, se repite la última válida. El timestamp se congela y la "Detect Communication Loss" original declara Communication Issue a los 0.5 s → Error → estado 7, igual que una pérdida de enlace real.
    - Sin ninguna trama válida quedan los ceros iniciales (arm = 0, isTracking = 0): el dron no arma y su FSM se detiene como el original sin tierra.
    - Esto protege contra emparejar este binario con un MissionCtrl de 16 doubles: las tramas quedan desalineadas y se descartan.
  - Envía: `[eco ts, FSM, batería (V), estado MPC, T, roll, pitch, 2]`. El índice 8 (en §3.2, la etiqueta de protocolo) lleva **2 = etiqueta "protocolo MPC v1"**, para que tierra pueda comprobar con quién habla.
- **Raíz**: solo hay dos Goto globales nuevos y una nota:
  - `QD2_Bateria_V`, en lugar del Terminator de "Battery Level";
  - `QD2_Actitud_Estimada`.
  - Los puertos de COMMANDER y Communication no cambiaron; las señales nuevas viajan por Goto/From.
- `InitFcn`: agrega `lib/` y `equipo2_original/` (por `Motor_Mapping_7_Inch`) buscando hacia arriba. Funciona igual desde `mpc/` que desde `versiones/RXXXX/`.
- **Importante:** úsalo **solo** con `QD2_MissionCtrl_Dual_MPC` y su ExtModeMexArgs, que es el mismo `-URI_Host tcpip://<IP_PC>:1800i`.

**Verificación en esta laptop:**
- Update Diagram del modelo completo: OK. Compila los bloques MATLAB Function y revisa tiempos de muestreo, tamaños y lazos.
- La compilación con QUARC (`quarc_linux_qdrone2`) **no se pudo probar**: aquí no hay targets de QUARC. Se comprobó que la configuración del target (STF, ExtModeMexArgs y opciones de QUARC) quedó igual que en el original.
- **Prueba en lazo cerrado del COMMANDER real:** `sim/qd2_probar_dron_mpc.m`.
  - Copia el subsistema COMMANDER del modelo MPC tal cual y lo cierra con la planta `qd2_quad_plant` a 1 kHz. La caja negra `To Host File` de la copia se comenta y la simulación corre en `tempdir`: no deja archivos en el repositorio.
  - El Pose Switchbox se emula con los límites [vel, acel] de los sigmoides originales (lento en TAKEOFF_HEIGHT, rápido en FLIGHT).
  - **La estimación es ideal** (estado verdadero, sin ruido ni latencia): sus errores de z son una cota optimista. La simulación dual sí usa OptiTrack con ruido y latencia.
- **PROTOCOL CHECK con datos reales:** se probó el código exacto del bloque con los 154183 paquetes del vuelo real del Equipo2 (ver Resultados).

## 2. `QD2_Sim_Dual.slx` (simulación SIL)

**Abrir y correr:**
```matlab
cd simulink_qdrone2/sim
open_system('QD2_Sim_Dual')     % el PreLoadFcn agrega lib/ y sim/ al path
```

- El `PreLoadFcn` busca `lib/` desde la carpeta actual y desde `which('QD2_Sim_Dual')`, así que los enlaces a `QD2_Mission_Lib` se resuelven al abrir el modelo desde `sim/` (doble clic en el Current Folder). Si lo abres desde otra carpeta, haz antes `addpath('<...>/simulink_qdrone2/lib')`. Las copias de `versiones/` no dependen de la librería.
- **VERSION** (constante roja): 1 = PID, 2 = MPC.
- **COMMAND SOURCE** (constante naranja):
  - **1 = estación simulada (por defecto):** START automático, heartbeat y LAND_ALL. Pulsas Run y la misión corre sola. La telemetría sale igual por :18700, así que la UI web puede mirar.
  - 2 = estación Python (la UI web) por `tcpip://localhost:18700`. Pon 2 antes de Run para mandar comandos desde la UI; sin cliente la misión espera en ASSIGN_ROLES.
- Simulation Pacing está activo, así que corre en tiempo real.

**Sin interfaz, más rápido que tiempo real:**
```matlab
R = qd2_correr_sim(1);   % PID
R = qd2_correr_sim(2);   % MPC
R = qd2_correr_sim(1, struct('override', struct('home1', [-1 0.6])));          % otro escenario
R = qd2_correr_sim(1, struct('override', struct('trk_off', [2 40 45])));       % pérdida de rastreo
R = qd2_correr_sim(2, struct('override', struct('comm_off', [2 60 62])));      % enlace caído
R = qd2_correr_sim(2, struct('override', struct('pose_onboard', 0)));          % estimación ideal
R = qd2_correr_sim(2, struct('override', struct('falla_apilado', 1)));         % falla a bordo del de ARRIBA apilado
R = qd2_correr_sim(2, struct('override', struct('falla_apilado', 2)));         % falla a bordo del de ABAJO apilado
R = qd2_correr_sim(2, struct('override', struct('falla_fsm', [2 60])));        % falla a bordo del dron 2 en t = 60 s
R = qd2_correr_sim(2, struct('override', struct('proto_mal', [2 60 400])));    % el dron 2 pierde la etiqueta y(8) = 2
R = qd2_correr_sim(2, struct('override', struct('comm_off', [2 60 400], 'resp_off', [2 60 400])));  % enlace caído en los 2 sentidos
R = qd2_correr_sim(2, struct('puerto', 18751));                               % puerto de prueba (sin chocar con la E2E en 18700)
```
Con una falla, `qd2_correr_sim` imprime además una línea `FALLA a bordo` (alerta 9, distancias a 1 y 2 s, cuándo queda a ≥ d_min_h, distancia cuando el que cae llega a la altura del otro, estados y alturas finales) y la guarda en `R.falla`.
`qd2_correr_sim` restaura VERSION, SOURCE, pacing y StopTime al terminar: el modelo no queda modificado.

**Probar el puente sin la UI:** pon COMMAND SOURCE = 2, corre el modelo y en otra terminal ejecuta `python prueba_puente.py`.

**Prueba punta a punta con la estación web real:** `qd2_e2e_web` corre PID, MPC y MPC con falla a bordo en tiempo real (COMMAND SOURCE = 2, puerto 18700) y `web/e2e/e2e_estacion.py --fuente simulink` maneja la misión desde la API de la UI. Pasos y resultados en `docs/PRUEBA_E2E.md`.

### Qué contiene
- **0. Configuración.**
- **1. QDRONE 2 #1/#2 (SIM)**, a 500 Hz:
  - FSM a bordo, con PIV original o MPC (`qd2_onboard_step`), más la planta `qd2_quad_plant`;
  - **estimación como el dron real:** pose de OptiTrack (100 Hz, ruido de 2 mm, retenida, con latencia `lat_pose` = 15 ms) y velocidad con el filtro complementario original (acelerómetro + derivada filtrada de OptiTrack, wn = 50 rad/s, Kp3 = 4). Roll/pitch salen de la planta (IMU).
  - **errores del stack original:**
    - isTracking = 0 durante 0.2 s → estado 7 (rampa de empuje a 1.75 N/s, actitud nivelada) y, con ≤ 7.38 N, estado 9 (motores apagados): el dron cae;
    - enlace PC→dron caído (`comm_off`) 0.5 s → estado 7; 1 s armado → 9 (FLIGHT SAFETY corta los motores);
    - E-stop → 9.
  - batería simulada.
  - **DOWNWASH COUPLING**: `qd2_downwash` en los dos sentidos, con turbulencia y un factor real/modelo de 1.3.
- **2. OPTITRACK (SIM)**, a 100 Hz:
  - ruido de 2 mm;
  - mismo formato `[x y z roll pitch yaw isTracking]` que el Measurement Conditioning; lo usan la misión en tierra **y** los drones;
  - roll y pitch son los **reales** de la planta (`x(7)`, `x(8)` de `qd2_quad_plant`, sin ruido), así que el TELEMETRY PACK enlazado manda en y(61:63) la actitud real del explorador y y(63) = 1 (0 durante `trk_off` del explorador). `qd2_correr_sim` lo resume en `R.att_frac_valida`, `R.att_max_deg` y `R.att_err_max` (y(61:62) contra `x(7:8)` de la planta del explorador; 22-nov: error 0 en PID y MPC, inclinación máxima 1.8° y 3.1°);
  - pérdidas de rastreo programables (`trk_off`).
- **3. Misión**: bloques **enlazados** de `QD2_Mission_Lib`, los mismos de los modelos de tierra:
  - ROLE & HOME, MISSION SUPERVISOR, SEPARATION GUARD (solo se usa en PID), OPERATOR SAFETY LAYER, SETPOINT RATE LIMITER (por norma, con el límite de escape `P.falla_rate_xy` del dron que se aparta de uno en falla), VELOCITY ESTIMATE, STACKING INFO, TELEMETRY PACK y **MPC LINK CHECK**.
  - **DRONE LINK (sim, 8 doubles)** (abajo del área 3): arma la respuesta de 8 doubles de cada dron como la manda `QD2_DroneStack_MPC` (`[eco, FSM, batería, estado MPC, T, 0, 0, 2]`) a partir de la FSM simulada a bordo. Pasa por el **mismo** `MPC LINK CHECK` del modelo de tierra MPC y, con 1 paso de retardo (`Onboard Status z-1`), llega a la entrada `Onboard Status [4]` del MISSION SUPERVISOR. Con VERSION = 1 (PID) el MPC LINK CHECK entrega 0: igual que en el laboratorio, donde el binario PID solo devuelve el eco.
  - Fallas a bordo programables (`QD2_SIM`): `falla_fsm` [dron t] y `falla_apilado` (1 = falla el de arriba, 2 = el de abajo, tras 1 s apilados) llevan la FSM simulada a 7 (rampa de empuje y luego 9, como el stack original); `proto_mal` [dron t1 t2] quita la etiqueta y(8) = 2 de la respuesta simulada; `resp_off` [dron t1 t2] corta la respuesta dron → PC (el DRONE LINK retiene la última y el loopback crece; con `comm_off` en el mismo intervalo es un enlace caído en los dos sentidos). `comm_off` y `trk_off` siguen igual (también acaban en 7/9 y ahora tierra lo ve en la versión MPC).
- **DRONE STATUS (sim link)** (para el TELEMETRY PACK): o+2 de la telemetría según la versión, igual que los modelos de tierra (ARQUITECTURA §3.3). En **PID** emula el **Flight Plan de tierra** del Equipo2 (0 Idle, 1 Armed, 2 Takeoff, 3 Land, 9 MS Stop; misma tabla de transiciones que `Subsystem` del original) con los disparos efectivos `TRIG1/TRIG2` y el comm issue simulado; en **MPC** pasa la FSM a bordo simulada (1..9). Antes de la revisión E2E (22-nov) mandaba la FSM a bordo también en PID y la UI mostraba "Flight State: 5 · estado 5".
- **4. PYTHON BRIDGE**: Stream Server en :18700 que manda 64 doubles a 50 Hz y recibe 32.
  - **Por qué no se usó el bloque Stream Server de QUARC:** en modo Normal no abre el socket. Se comprobó: la salida `state` queda en 0 y `netstat` no muestra el puerto en escucha.
  - `tcpserver` tampoco sirve, porque requiere Instrument Control Toolbox, que no está.
  - Por eso el puente es `QD2PuenteTCP.m`: un System object con **Java NIO no bloqueante**, en ejecución interpretada, compatible con R2022a o posterior.
    - Si el cliente deja de leer (UI congelada, laptop suspendida), las tramas de telemetría se descartan enteras (como "send most recent" de QUARC) y la simulación sigue. A los 5 s sin poder enviar, cierra ese cliente para que otro pueda conectarse.
  - **SIMULATED DETECTOR**: la cámara cenital del explorador publica los objetivos cuando acumulan 3 observaciones. Si la estación Python no publica objetivos, sus ranuras las llena este detector.
- **5. Scopes y registro**: To Workspace de `qd2_tel`, `qd2_x1`, `qd2_x2`, `qd2_st1` y `qd2_st2` (`st` = [FSM, batería, estado MPC, empuje, comm issue]).

### Parámetros
- `qd2_sim_parametros.m` crea `QD2_SIM` en el InitFcn. Ahí están homes, objetivos, ruido, `trk_off`, `comm_off`, `lat_pose`, `pose_onboard`, `dw_factor`, el detector, `t_start` y el puerto (sale de `estacion.json`).
- `QD2P = qd2_params_from_config()` toma de `config/estacion.json` la pista, las alturas, `d_min_h` y lo demás.
- Para cambiar un escenario sin tocar el modelo, define `QD2_SIM_OVERRIDE` en el workspace base.

### Limitaciones
- **El stack a bordo es una emulación (`qd2_onboard_step`):** no incluye el Pose Switchbox (sigmoides). El Stabilizer se aproxima con una actitud de primer orden (`tau = 0.12 s`), sin el retardo de unos 0.10 s identificado. La IMU es ideal (sin ruido ni sesgo).
- **Enlace PC↔dron:** sin latencia; solo cortes programados (`comm_off`).
- **Tierra reacciona a la FSM del dron solo en la versión MPC** (en PID el dron real solo devuelve el eco). La respuesta dron → PC solo se corta con `resp_off`. Con el enlace caído en los dos sentidos tierra no ve el 7/9 del dron, pero el MPC LINK CHECK declara enlace perdido (código 2) y el supervisor lo trata como falla (alerta 9).
- **Ids del detector:** el detector simulado y una estación Python que publique objetivos usarían ids independientes. En simulación, deja que publique solo uno de los dos.
- **Esta laptop:** el puerto 18700 lo pueden ocupar el simulador 3D o el mock de la web. En ese caso el puente avisa y la simulación sigue sin él.

## Archivos
| Archivo | Qué es |
|---|---|
| `../construir_dron_y_sim.m` | Genera el dron MPC y la sim, toma las capturas y exporta las versiones |
| `qd2_construir_sim.m` | Construye `QD2_Sim_Dual.slx`; lo llama el script anterior |
| `qd2_sim_parametros.m` | Parámetros `QD2_SIM` |
| `QD2PuenteTCP.m` | Stream Server TCP no bloqueante (System object) del PYTHON BRIDGE |
| `qd2_correr_sim.m` | Corrida sin interfaz con métricas |
| `qd2_e2e_web.m` | Prueba E2E: corridas en tiempo real manejadas por la estación web (`docs/PRUEBA_E2E.md`) |
| `qd2_probar_dron_mpc.m` | Lazo cerrado del COMMANDER real del modelo MPC |
| `prueba_puente.py` | Cliente mínimo de estación (biblioteca estándar de Python) |

## Resultados

Última corrida: R2025a, en esta laptop, 22-nov-2026 (después de la revisión). Logs en el scratchpad de la sesión (`dsfix/t*.txt`).

### Lazo cerrado del COMMANDER real (`qd2_probar_dron_mpc`, estimación ideal)
En todos los escenarios la FSM pasa por 1→2→3→4→5→6→9 y el modelo se detiene solo (Threshold y Stop originales). Error de z: transitorio = [15, 20) s; asentado = [25, 30) s.

| Escenario | z transitorio | z asentado | Escalón xy 0.58 m: sobrepaso / asent. 3 cm | z con el otro dron encima (downwash ×1.3) |
|---|---|---|---|---|
| MPC | 0.0000 m | 0.0000 m | 0.000 m / 1.97 s | **0.017 m** |
| PIV original | 0.101 m | 0.010 m | 0.012 m / 3.49 s | 0.117 m |
| MPC + 100 ms de retardo de actitud | 0.0000 m | 0.0000 m | 0.000 m / 2.14 s | 0.017 m |
| PIV + 100 ms | 0.101 m | 0.010 m | 0.012 m / 3.27 s | 0.117 m |
| PIV→MPC en vuelo (t = 25 s) | — | — | 0.000 m / 1.97 s | 0.017 m; error z máx. 0.012 m en ±1 s del cambio |
| MPC, masa 1.65 kg ("Drone Weight" = 1.65) | 0.0000 m | 0.0000 m | 0.000 m / 1.96 s | 0.017 m |
| PIV, masa 1.65 kg | 0.107 m | 0.012 m | 0.012 m / 3.49 s | 0.118 m |

La comparación justa en hover es la columna "asentado" (PIV 0.010 m contra MPC 0.000 m); el 0.10 m del PIV es el transitorio después del despegue.

### Misión completa en `QD2_Sim_Dual` (`qd2_correr_sim`, estación simulada, sin pacing, estimación por OptiTrack)
| | PID | MPC |
|---|---|---|
| Completada (DONE) | sí, a los 121.9 s | sí, a los 91.8 s |
| Estados | 0→1→2→3→4→5→6→7→8 | 0→1→2→3→4→5→6→7→8 |
| Waypoints de la N | 35/35 | 35/35 |
| Objetivos visitados | 3 de 3 (mask 7) | 3 de 3 (mask 7) |
| Distancia horizontal mínima con los 2 en el aire | **1.153 m** (consignas 1.129 m; guard activo 61 % de MISSION) | 0.024 m (se apilan, está permitido) |
| Error z del dron de abajo apilado | — | **0.0082 m máx., 0.0041 m RMS** (7.3 s apilados); con estimación ideal: 0.0075 / 0.0040 m |
| FSM a bordo | 1→2→3→4→5→6→9 en los dos | igual |

### Fallas emuladas
- **Pérdida de rastreo del dron 2 en [40, 45) s (PID):** la misión pasa a PAUSED a los 40.20 s (alerta 2). El dron 2 entra a estado 7 a los 40.22 s (estaba a 0.50 m), llega a 9 a los 44.44 s y queda en el suelo. El dron 1 sigue en hover a 1.50 m. La misión se queda en PAUSED: así se comportaría el stack real.
- **Enlace caído (MPC):** dron 1 durante 0.4 s → sin error (umbral 0.5 s). Dron 2 durante 2 s desde los 60 s → estado 7 a los 60.50 s y 9 a los 61.00 s (FLIGHT SAFETY).

### Falla a bordo (22-nov-2026, `qd2_correr_sim`, versión MPC, puerto de prueba 18751)
Corridas reales en R2025a, estación simulada, sin pacing. La nominal no cambió: PID DONE a los 121.9 s (dh mínima 1.153 m), MPC DONE a los 91.8 s (apilados 7.3 s, error z del de abajo 0.0082 m), alerta 0 en las dos.

| Escenario (`override`) | Falla | Alerta | dh a 1 s / 2 s | ≥ d_min_h a los | dh cuando el que cae llega a \|dz\| < 0.4 m del otro | Estados después | Final |
|---|---|---|---|---|---|---|---|
| `falla_apilado = 1` (falla E, ENCIMA de R) | dron 1, t = 38.22 s, dh 0.034 m, dz 1.00 m | 9 a los 0.02 s | 0.525 / 1.117 m | 1.48 s | **0.944 m** (R ya salió de debajo) | 5→9→7→8 | DONE 44.3 s, z final [0 0] |
| `falla_apilado = 2` (falla R, DEBAJO de E) | dron 2, t = 38.22 s, dh 0.034 m | 9 a los 0.02 s | 0.547 / 1.029 m | 1.52 s | nunca (E sigue a 1.5 m) | 5→9→7→8 | DONE 48.0 s, z final [0 0] |
| `proto_mal = [2 60 400]` | etiqueta perdida en t = 60 s | 10 | — | — | — | …→9→7→8 | DONE 75.6 s |
| `comm_off = resp_off = [2 60 400]` (enlace caído en los 2 sentidos, revisión) | FSM real del dron 2: 7 en 60.50 s, luego 9; tierra no la ve | 9 a los 0.50 s (código 2) | 1.139 / 1.185 m | ya separados | nunca (no estaban apilados) | 5→9→7→8 | DONE 74.8 s, z final [0 0] |
| `comm_off = [2 60 62]` | dron 2: 7 a los 60.50 s | 9 a los 0.02 s | 1.141 / 1.185 m | ya separados (1.014 m) | — | 5→9→7→8 | DONE 74.8 s |
| `trk_off = [2 40 45]`, **PID** | dron 2: 7 a los 40.22 s | 2 (PAUSED) | — | — | — | 10 | sigue en PAUSED (PID no ve la FSM a bordo, como antes) |

- **Revisión (22-nov):** `P.falla_rate_xy` subió de 1.0 a **1.5 m/s**. Repetidas: `falla_apilado = 1` → falla t = 38.22 s, alerta 9 a los 0.02 s, dh 0.689 / 1.112 m a 1 / 2 s, ≥ d_min_h a los 1.26 s, **dh al cruzar 1.037 m** (antes 0.944 m), estados 5→9→7→8, DONE 44.0 s. `falla_apilado = 2` → ≥ d_min_h a los 1.32 s, DONE 47.8 s. Nominal PID y MPC sin cambios (DONE 121.9 s / 91.8 s, dh mín 1.153 m, error z apilado 0.0082 m).
- Mientras se aparta, el dron sano mantiene su altura (lo verifica `tests/test_qd2_lib.m`: z de R 0.50–0.51 m, de E 1.50 m) y usa el límite xy de escape (1.5 m/s). En el lazo cerrado MATLAB, la distancia cuando E cae a la altura de R es 0.982 m con 1.5 m/s, 0.852 m con 1.0 m/s y 0.441 m con el límite normal de 1/3 m/s.
- Al dron en falla nunca se le vuelve a subir takeoff; dh mínima con los dos en el aire una vez separados: 0.904–0.907 m.

### PROTOCOL CHECK estricto (24 = 2), `tests/qd2_probar_protocolo.m` (código del bloque tal cual, 22-nov-2026)
| Flujo de entrada | Tramas | Aceptadas |
|---|---|---|
| Tierra MPC actual (índice 24 = 2) | 5000 | 5000 |
| Tierra MPC vieja (índice 24 = 0) | 5000 | 0 (el dron no arma) |
| Tierra PID de 16 doubles leída en unidades de 24 | 3333 | 0 |
| Binarios = 0.5 o timestamp NaN | 5000 | 0 |

En ningún caso salieron arm/takeoff/e-stop distintos de 0 o 1. La tabla siguiente es de la revisión anterior (PROTOCOL CHECK que aceptaba 24 = 0 o 2); no se volvió a correr con el log real.

### PROTOCOL CHECK con los 154183 paquetes del vuelo real del Equipo2 (revisión anterior)
| Flujo de entrada | Tramas | Aceptadas | Timestamp congelado |
|---|---|---|---|
| Tierra MPC correcta (24, índice 24 = 0) | 154183 | 100 % | 0 |
| Tierra MPC con etiqueta (índice 24 = 2) | 154183 | 100 % | 0 |
| **Tierra PID de 16 doubles leída en unidades de 24** | 102788 | 33 (0.03 %) | 102755 (racha de 102723) |
| Mismo flujo con pose y triggers = 0 (peor caso) | 102788 | 50 % | 50 % |

- Las 33 tramas aceptadas son de los primeros 0.1 s del log y todas tienen desfase 0: sus 16 primeros doubles son un paquete completo y correcto, con `mpc_mode` = 0 (PID).
- En ningún caso salieron arm o takeoff con valores distintos de 0 o 1.
- Con el emparejamiento equivocado el timestamp queda congelado: la FSM original pasa a Communication Issue y luego a Error.

### Puente
- **Cliente real de la web** (`web/app/bridge.BridgeClient`, PID, SOURCE = 2, tiempo real): START → ack 1; PAUSE → PAUSED, ack 2; RESUME → MISSION, ack 3; LAND_ALL → LANDING, ack 4; DONE a los 113.8 s simulados en 118.7 s de reloj. 0 reconexiones.
- **Cliente que se conecta y nunca lee:** la simulación no se bloquea (60 s simulados en 18.7 s). Descarta 250 tramas y a los 5 s cierra al cliente. El cliente vio EOF.

### Versiones exportadas
- Las 10 (dos modelos × R2022a, R2022b, R2023a, R2023b, R2025a) cargan en R2025a. `Simulink.MDLInfo` da la versión correcta y todas las copias del dron incluyen el PROTOCOL CHECK.
- El `QD2_Sim_Dual` exportado a R2022a corre la misión MPC completa (0→…→8, 3 visitados).
- No se ha abierto en un R2022a real.
