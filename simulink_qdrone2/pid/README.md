# Modelos de tierra para 2 QDrone 2 (PID y MPC)

Los dos modelos de tierra son **copias extendidas** del modelo del Equipo2 `equipo2_original/QD2_MissionCtrl_NoJoystick_2021a.slx`. El original no se toca. Se generan con un script: no hay que editarlos a mano.

| Archivo | Qué es |
|---|---|
| `pid/QD2_MissionCtrl_Dual_PID.slx` | Tierra, versión PID: 16 doubles por dron y SEPARATION GUARD |
| `mpc/QD2_MissionCtrl_Dual_MPC.slx` | Tierra, versión MPC: 24 doubles enviados y 8 recibidos por dron, STACKING INFO |
| `lib/QD2_Mission_Lib.slx` | Librería con los subsistemas nuevos. También la usa la simulación (`sim/QD2_Sim_Dual.slx`) |
| `construir_modelos.m` | Genera todo lo anterior, verifica, exporta y toma capturas |
| `qd2_cargar_config.m` | Lee `config/estacion.json`. La llama el `InitFcn` de los modelos |
| `qd2_ext_mode_args_slx.m` | Lee el `ExtModeMexArgs` guardado en un `.slx` (configuración activa) sin cargarlo. Lo usan `construir_modelos` y `test_ext_mode_args_slx` |
| `versiones/R20xx/` | Exportaciones a R2022a, R2022b, R2023a y R2023b, más la copia nativa de R2025a. Cada carpeta lleva su propia `QD2_Mission_Lib.slx` |
| `capturas/` | Capturas del original (`original_*.png`) y de los modelos nuevos (`dual_*.png`) |
| `pid/qd2_probar_tierra.m` | Prueba funcional SIL del modelo de tierra completo, sin hardware, con casos del operador |
| `pid/qd2_probar_operador.m` | Pruebas unitarias de la lógica de seguridad (27 casos) |

## Regenerar

```matlab
cd simulink_qdrone2
construir_modelos            % ~1.5 min: librería, PID, MPC, Update Diagram, capturas y exportación
construir_modelos('exportar', false, 'capturas', false)   % versión rápida
```

Si MATLAB corre con `-batch`, la ventana "QUARC Tip of the Day" lo bloquea (ver `tests/README.md`).

## Abrir en el laboratorio

1. Abre el `.slx` de tu versión de MATLAB. Si es R2022a, usa `versiones/R2022a/`.
   - El `PreLoadFcn` agrega solo las rutas `simulink_qdrone2`, `lib` y la carpeta del modelo.
   - El `PostLoadFcn` y el `InitFcn` llaman a `qd2_cargar_config`, que crea `QD2CFG` y `QD2P`.
2. `config/estacion.json` es la única fuente de verdad:
   - `optitrack.rigid_body_dron1` y `optitrack.rigid_body_dron2`: van a `rigid_body_ids` del bloque OptiTrack Trackables. **Si son iguales, `qd2_cargar_config` da error** y el modelo no arranca.
   - `optitrack.bias_z_dron1` y `optitrack.bias_z_dron2`: van al Bias de cada Measurement Conditioning.
   - `optitrack.archivo_cal` y `optitrack.archivo_motive`: el `InitFcn` los escribe en el bloque OptiTrack. Si están vacíos, se conservan las rutas del Equipo2.
   - `red.puerto_*`: van a `ExtModeMexArgs` (`'-w -d %d -uri %u -URI_Host … -URI_Host2 …'`, entre comillas como el original del Equipo2) y al PYTHON BRIDGE. **El valor también tiene que ir guardado entre comillas en el `.slx`**: QUARC lo lee al abrir el modelo y al empezar el build, antes del `InitFcn` (sin comillas: `This statement is incomplete.` y sin pestaña HARDWARE; `docs/HALLAZGOS_PC_LAB_23NOV.md` fila 1). Todos los `.slx` de tierra ya lo traen así (23-nov-2026); `construir_modelos` lo comprueba al guardar y exportar (`qd2_ext_mode_args_slx`) y `tests/test_qd2_lib.m` (`test_ext_mode_args_slx`) lo vigila.
   - `mision.*` y `pista.vertices`: van a `QD2P`.
   - El `.motive` debe definir **los dos** rigid bodies (el del Equipo2 del 20-nov solo tiene uno).
   - **Motive tiene que estar cerrado** al correr el modelo: OptiTrack Trackables usa la API de Motive con el `.cal` y el `.motive`, no NatNet.
   - **Todo esto se aplica al compilar.** Si cambias `estacion.json` (rutas de Motive, rigid bodies, bias, pista, `mision.*`), vuelve a hacer el build QUARC; conectar al binario anterior no relee el JSON. El `InitFcn` deja el modelo modificado (rutas de Motive y, si cambiaron los puertos, `ExtModeMexArgs`); guardarlo o no da igual para el JSON, porque se vuelve a aplicar en cada build, pero el `ExtModeMexArgs` guardado debe seguir entre comillas (ver arriba).
3. Compila con QUARC y conecta con Monitor & Tune, igual que el original. **Antes del primer vuelo haz un build QUARC completo**: aquí solo se pudo hacer Update Diagram y la simulación SIL (ver abajo). Orden: build → Connect → Start de tierra **antes** de lanzar los drones (el dron se detiene solo si a ~1.2 s no recibe `isTracking = 1`).
4. Arranca la estación Python (`web/`). Se conecta como cliente a `tcpip://localhost:18700`.
5. Lanza el modelo de cada dron con `quarc_run` desde `simulink_qdrone2/binarios_dron/` (comandos en la pestaña **Conexiones** de la UI; ver `binarios_dron/README.md`):
   - Dron 1: `-URI_Host tcpip://<IP_PC>:18001`.
   - Dron 2: `-URI_Host tcpip://<IP_PC>:18002`.
   - En la versión PID es el binario original del Equipo2 (`binarios_dron/QD2_DroneStack_PID_2021a.rt-linux_qdrone2`). En la versión MPC es `QD2_DroneStack_MPC`, compilado para `quarc_linux_qdrone2` con la carpeta actual de MATLAB en `binarios_dron`.
   - **Switch `MPC Mode`** (solo tierra MPC, naranja, junto a STACKING INFO): `PID Backup` (0) manda `mpc_mode = 0` y el dron vuela con el PIV original; `MPC` (1) usa el MPC a bordo. Su `InitFcn` lo deja en MPC en cada arranque. Para el primer vuelo de un dron MPC ponlo en PID Backup.
6. Switches del operador (área 1) en modo **MISIÓN** (Command Source = 0, por defecto):
   - **Arm/Disarm** y **Takeoff/Land** arriba = **PERMISO**. Los drones se arman y despegan cuando se da START en la UI (takeoff siempre ≥ 1 s después de armar).
   - **Bajar Takeoff/Land en vuelo** = el supervisor recibe un **LAND ALL** (aterrizaje ordenado: R aterriza primero y E se aparta). Ya **no** corta el takeoff de los dos de golpe, porque cada dron aterrizaba en su x,y y en la versión MPC E caía encima de R. Funciona aunque la estación Python esté caída.
     - Respaldo crudo (alerta 8): si el supervisor no acepta en 2 s, o pasan 90 s con el switch abajo, se suelta el takeoff, pero **nunca** el de un dron que tiene al otro debajo a menos de `d_min_h`.
   - **Arm/Disarm abajo** en vuelo: desarma (como el original) y pide LAND ALL. Ojo: a bordo, soltar el arm en vuelo es un **Error** (estado 7: rampa de bajada sin control de posición; con poco empuje el modelo del dron se detiene y puede caer), no un aterrizaje. Para aterrizar se usa Takeoff/Land.
   - **Emergency Stop**: corta los motores de los dos al instante (los drones **caen**) y pide LAND ALL. Solo para evitar un choque con personas o cosas.
   - **No detengas el modelo de tierra con un dron en el aire**: el dron pierde el enlace, pasa a la rampa (7) y a 1 s FLIGHT SAFETY corta los motores.
   - **Command Source** (área 9, naranja): 0 = misión con 2 drones; 1 = modo MANUAL del Equipo2 (solo el dron 1, Waypoint Trajectories original, takeoff ≥ 1 s después de armar).
     - Solo cambia **en tierra**: los dos Flight State en 0 (Idle), los dos desarmados y **los switches Arm y Takeoff abajo**. Un cambio hecho en vuelo queda pendiente y nunca dispara arm/takeoff solo. Como el Flight Plan se queda en 3/9 después de un vuelo, para cambiar de modo después de volar hay que reiniciar el modelo.
7. y(55) de la telemetría (`alerta`, códigos acordados): 0 nada, 1 homes cerca, 2 pérdida de rastreo, 3 timeout de despegue, 4 timeout de aterrizaje, 9 falla a bordo de un dron (solo MPC), 10 protocolo MPC incorrecto (solo MPC) (supervisor); 5 E-stop, 6 Takeoff/Land abajo, 7 Arm abajo, 8 respaldo crudo (operador, tiene prioridad sobre todas las del supervisor, también sobre 9 y 10). y(56:60) = un objetivo por mensaje, rotando: `[id clase x y estado]` (1 pendiente, 2 asignado, 3 visitado). y(61:63) = roll y pitch (rad) del **explorador** y válido (0/1), de la misma Corrected Pose Data [7] que ya entra al TELEMETRY PACK por las entradas 4 y 5 (`d(4:5)`, convención OptiTrack R = Rz(yaw)·Ry(pitch)·Rx(roll): roll > 0 baja el lado derecho, pitch > 0 baja la nariz). No hizo falta ningún bloque ni puerto nuevo: solo cambió el código del MATLAB Function (`att = [d1(4) d1(5) d2(4) d2(5)]`). y(63) = 0 sin explorador, con el explorador sin rastreo o con ángulos no finitos; y(64) sigue en 0. `qd2_probar_tierra` lo verifica (`R.att_y63_ok`, `R.att_cero_sin_valido`, `R.att_err_max`, `R.att_n_valido` = muestras válidas E1/E2/no válidas). Por defecto el explorador es el dron 1; con `opt.homes = [0.9 -0.3; -1.0 0.3]` el explorador es el dron 2, y junto con `trk2_off` se prueba que y(63) = 0 cuando el **explorador** pierde el rastreo: `qd2_probar_tierra('PID', 120, false, 0, struct('homes', [0.9 -0.3; -1.0 0.3], 'trk2_off', [40 45]))`.

## Versión MPC: falla a bordo y protocolo (MPC LINK CHECK)

- **PC → dron, índice 24 = 2** (etiqueta "protocolo MPC v1", la pone STACKING INFO). El `PROTOCOL CHECK` de `QD2_DroneStack_MPC` ahora es **estricto**: solo acepta tramas con 24 = 2. Un MissionCtrl MPC viejo (24 = 0) o uno PID (16 doubles) no puede armar el dron MPC. **Hay que usar juntos** el modelo de tierra y el del dron de la misma construcción (las 5 carpetas de `versiones/` se regeneraron juntas).
- **Dron → PC:** bloque nuevo `MPC LINK CHECK` (área 9, a la derecha del OPERATOR SAFETY LAYER). Para cada dron valida la respuesta de 8 doubles: `y(8) == 2` y FSM `y(2)` entera en 1..9. Si el dron está "supuestamente conectado" (loopback < 0.5 s y `Communication_Status` = 0) y responde mal **más de 0.5 s seguidos** → protocolo incorrecto (alerta 10) y se le trata como **dron no disponible**. Ejemplo: un binario PID (eco de 1 double) leído de 8 en 8 da y(8) = un timestamp.
- **Enlace perdido:** si el dron deja de estar conectado (eco viejo ≥ 0.5 s o `Communication_Status` = 1) después de haber estado conectado, el código es 2 y la FSM se reporta como 0 (la respuesta retenida por el Stream Server no vale). Un dron que debía volar (takeoff = 1) sin enlace se trata como dron en falla (alerta 9): a bordo, la "Detect Communication Loss" ya lo lleva a 7 a los 0.5 s.
- Su salida `Onboard Status [4] = [fsm1 fsm2 cod1 cod2]` (fsm 0 = desconocida; cod 1 protocolo, 2 enlace perdido) pasa por un `Unit Delay` (rompe el lazo con los Triggered Subsystem de Communication Issue) y llega por `Goto/From Onboard_Status` a la **entrada 6 nueva** del MISSION SUPERVISOR. En el modelo PID esa entrada es una constante `[0;0;0;0]` (el dron solo devuelve el eco).
- **Reacción** (`qd2_mission_step`, ver `lib/README.md`): un dron en vuelo con FSM a bordo 7/8/9, sin enlace (o con protocolo incorrecto sostenido) → alerta 9 (10); el sano se aparta en horizontal ≥ `d_min_h` a su altura **antes de cualquier otra cosa** (si estaban apilados sale de inmediato, límite xy 1.5 m/s), luego regresa a su home y aterriza; DONE. Al dron en falla nunca se le vuelve a dar takeoff. **LAND_ALL con los drones apilados:** R espera a su altura hasta que E está a ≥ `d_min_h` en horizontal (antes bajaba debajo de E); si uno falla mientras tanto, hay reacción completa. **Antes de START:** un dron desarmado con FSM 7/8/9 no es alerta (ARQUITECTURA §3.2); con START, en TAKEOFF ya armado se cancela el despegue sin dar takeoff (alerta 9). El protocolo incorrecto sí bloquea START (alerta 10).
- **SETPOINT RATE LIMITER** tiene una 4.ª entrada `Mission Info [40]` y el parámetro `dron` (1/2): mientras ese dron se aparta de un dron en falla (`Mission Info(37 + dron)` = 1) su límite xy es `P.falla_rate_xy` (1.5 m/s). Cada limitador tiene su reloj y sus `From` alineados con los puertos; las consignas llegan con un solo quiebre (sin cruces).
- `Mission Info` pasó de 32 a **40** elementos: 30-31 falla por dron, 32 fase de la reacción, 33-37 objetivo rotando para y(56:60), 38-39 `escape` por dron.

## Qué se agregó (nada original se movió: 246 bloques del original, 0 movidos, 0 faltantes, 0 cambios de color; verificado por script)

Lo nuevo está en el **área numerada que le corresponde**:

- **Raíz**
  - `Communication` pasó a llamarse `Communication QDrone 1` y sigue en el mismo lugar.
  - `Communication QDrone 2` en :18002, con el argumento `-URI_Host2`.
  - `PYTHON BRIDGE` (naranja): Stream Server en :18700. Envía 64 doubles a 50 Hz y recibe 32.
  - Nota roja debajo de las instrucciones del Equipo2: en modo misión los pasos 10-13 cambian.
  - El dron 2 y Python cruzan el borde del subsistema con Goto/From **globales** (`QDrone2_Data_In`, `QDrone2_Communication_Issue`, `QDrone2_Timestamp_Received`, `Python_Telemetry`, `Python_Commands`), para no cambiar los puertos ni el tamaño del subsistema cyan.
- **Área 2**: `OptiTrack Trackables` con `rigid_body_ids = QD2CFG.optitrack.rb_ids`; selectores `QDrone 1 [1:7]` y `QDrone 2 [8:14]`.
- **Área 1**: el dron 1 y su Flight Plan reciben los disparos efectivos (From `Triggers_QDrone1`).
- **Área 4**: `Command Source Switch` elige la consigna del dron 1 (misión, o la cadena original Waypoint → Saturation2 → Rate Limiter2).
- **Flight Plan (`Subsystem`)**: bloque `AND Stop Allowed` antes del `Delay` de 3 s.
- **Áreas nuevas**
  - **7**: localización del QDrone 2.
  - **8**: comunicación, signal check y Flight Plan del QDrone 2.
  - **9**: ROLE & HOME ASSIGNMENT, MISSION SUPERVISOR, SEPARATION GUARD (PID) o STACKING INFO (MPC), SETPOINT RATE LIMITER por norma; fila inferior: OPERATOR SAFETY LAYER (bloque `OPERATOR SAFETY LAYER DUAL` de la librería), MPC LINK CHECK (solo MPC) y OPERATOR COMMAND MERGE; fila 3: MISSION END CHECK.
  - **10**: velocidades, Drone Status (con la alerta del operador), TELEMETRY PACK y log `mission_dual_data`.

**Librería `QD2_Mission_Lib`**: todos los bloques son subsistemas **atómicos con paso de muestreo discreto `Ts`** (parámetro de máscara; en tierra `qc_get_step_size`, en la simulación 0.01). `OPERATOR SAFETY LAYER` (5 entradas / 3 salidas) se conserva para `QD2_Sim_Dual`; los modelos de tierra usan `OPERATOR SAFETY LAYER DUAL` (9/4) y `OPERATOR COMMAND MERGE`.

**Corrección del paro automático.** El original paraba el modelo de tierra 3 s después de Land, con el dron todavía bajando. Ahora el `Stop with Message` solo actúa con `Stop Allowed` (MISSION END CHECK):
- **Los dos** drones en Land / MS Stop **y** rastreados en el suelo (z < 0.10 m) 3 s seguidos, o 60 s **seguidos sin rastreo** (respaldo).
- Un dron rastreado con z ≥ 0.10 m **nunca** permite el paro.
- Además se mantiene el Delay original de 3 s.
- Con Emergency Stop el Flight Plan original se queda en el estado 2 (así está en el original), así que el modelo **no** se detiene solo: hay que pararlo a mano.

## Verificado en esta laptop (R2025a)

- **Update Diagram sin errores** de los dos modelos; exportaciones R2022a-R2023b "OK".
- **Compilación (`model([],[],[],'compile')`)**: ya no falla en los MATLAB Function (antes: "continuous sample time" en OPERATOR SAFETY LAYER). Ahora llega exactamente a los mismos bloqueos que el original del Equipo2 en esta laptop: `Model Argument` (sin argumentos de external mode) y, con Communication comentado, la licencia de OptiTrack.
- **`qd2_probar_operador`**: 27/27 casos OK (22-nov-2026, con la librería nueva).
- **`qd2_probar_tierra`** (SIL; compila y simula todo menos OptiTrack, los Stream Servers y los sinks QUARC). Opciones nuevas para la versión MPC: `falla` = `[dron t]` (el stub de ese dron reporta FSM 7, y 9 a los 4 s, y el dron cinemático cae) y `proto_mal` = `[dron t1 t2]` (respuesta sin la etiqueta y(8) = 2) y `enlace_off` = `[dron t]` (enlace caído en los dos sentidos: el stub retiene la última respuesta y el dron cae a los 0.5 s). El stub MPC responde con la etiqueta 2. `R.lb2` = loopback del dron 2. Ojo: el loopback del modelo se mide con el bloque `Time` del Equipo2 (tiempo de reloj); como la prueba corre ~7 veces más rápido que el tiempo real, 0.5 s de loopback son ~4.5 s simulados.
- **Bloques originales:** script de verificación sobre los dos modelos: 246 bloques del original, 0 movidos, 0 faltantes, 0 cambios de color; ningún From sin Goto ni Goto sin From; 0 solapes entre los bloques de las áreas 9-10 (80 en PID, 107 en MPC). Update Diagram OK en los dos.

Corridas del 22-nov-2026 (R2025a):

| Corrida | Estados | DONE | Alertas y(55) | Despegues por dron | Resultado |
|---|---|---|---|---|---|
| PID nominal | 0→…→8 | 83.94 s | 0 | [1 1] | dh mínima en el aire 1.214 m; Stop Allowed 85.90 s, nunca con un dron en el aire |
| MPC nominal | 0→…→8 | 75.32 s | 0 | [1 1] | Stop Allowed 77.28 s; enviado a los dos drones índice 24 = 2 |
| MPC, `falla = [2 40]` (R) | …5→9 (40.02 s)→7 (44.56)→8 | 50.12 s | 0, 9 | [1 1] (a R nunca se le volvió a dar takeoff) | E se apartó y aterrizó en su home; los dos en el suelo; Stop Allowed 51.06 s |
| MPC, `proto_mal = [2 40 230]` | …5→9 (40.52 s)→7→8 | 50.30 s | 0, 10 | [1 1] | alerta 10 a los 0.52 s de perder la etiqueta (umbral 0.5 s + muestreo del registro); los dos en el suelo |
| MPC, `enlace_off = [2 40]` (22-nov, revisión) | …5→9 (44.52 s)→7→8 | 54.36 s | 0, 9 | [1 1] | alerta 9 cuando el loopback del dron 2 pasa de 0.5 s (44.5 s simulados = ~0.5 s de reloj, ver arriba); antes de la corrección tierra se quedaba con FSM 5 y no reaccionaba; los dos en el suelo; Stop Allowed 55.32 s, nunca con un dron en el aire |

## NO verificado aquí

- **Build con QUARC (`quarc_win64.tlc`) y external mode.** Esta laptop no tiene ese target.
- **Hardware real**: OptiTrack con 2 rigid bodies, los 3 Stream Servers y los drones; comportamiento del PYTHON BRIDGE cuando Python se desconecta.
- **Apertura en R2022a real.** Solo se exportó.
- **Escribir las rutas `.cal`/`.motive` con `set_param` desde el InitFcn durante el build QUARC.** Con Update Diagram sí funciona: el 22-nov, con un JSON de prueba en R2025a, el `InitFcn` escribió las dos rutas en OptiTrack Trackables y Update Diagram terminó sin error. Si en el build fallara, se escriben a mano en el bloque (área 2) y se guarda el modelo.
- **El enlace MPC real**: que la respuesta de 8 doubles del dron llegue con la etiqueta y(8) = 2 y la FSM real; que el loopback del Signal Check sea < 0.5 s (criterio de "conectado" del MPC LINK CHECK); y la reacción a una falla a bordo con drones reales (solo se probó en SIL y simulación).
- **Enlace perdido en vuelo = falla (alerta 9), resuelto en la revisión del 22-nov.** Falta comprobar en el laboratorio que el loopback y `Communication_Status` del Equipo2 no se disparen solos con el enlace bien (un falso "enlace perdido" en vuelo aterrizaría la misión).
