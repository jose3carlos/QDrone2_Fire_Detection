# Prueba punta a punta (E2E): estación web ⇄ Simulink / simulador 3D

Esta prueba usa **la estación web real**: el mismo `web/app/server.py` que abre el operador, con su puente :18700, su WebSocket y su API HTTP. La estación se prueba contra dos simuladores:

- `QD2_Sim_Dual` (Simulink, en tiempo real), en PID, en MPC y en MPC con una falla a bordo;
- el simulador 3D (`simulador3d`), en PID y en MPC.

La misión se maneja como lo haría el operador en la UI: START, objetivo manual, PAUSE, RESUME y LAND ALL con los dos drones en home. Se verifica:

- la telemetría que llega a la página por el WebSocket;
- los objetivos publicados, confirmados por Simulink en y(56:60) y visitados;
- y(55);
- el regreso a home y DONE.

Contrato: `docs/ARQUITECTURA.md`. Última corrida: **22-nov-2026, R2025a, en esta laptop**. Todos los resultados de abajo son de corridas reales.

## 1. Qué hace falta

| Qué | Dónde |
|---|---|
| venv de la estación | `%USERPROFILE%\.venvs\qd2_web`. Lo crea `web\iniciar_web.bat` la primera vez. |
| MATLAB R2022a o más nuevo | Simulink para `QD2_Sim_Dual`; el simulador 3D no necesita toolboxes. |
| Chrome (opcional) | Para las capturas. Sin Chrome, la prueba corre igual (`--sin-capturas`). |
| Puerto 18700 libre | Ni el mock, ni otro simulador, ni otro modelo de tierra pueden estar usándolo. Para revisarlo: `netstat -ano \| findstr :18700`. |

> **QUARC y `matlab -batch`.** La ventana "QUARC Tip of the Day" bloquea MATLAB. Para evitarlo, arranca con `matlab -sd <carpeta> -batch "..."`, donde `<carpeta>` tiene un `startup.m` que solo hace `error('fin')`. Desde el escritorio de MATLAB basta con cerrar la ventana.

**Archivos de la prueba:**

| Archivo | Qué hace |
|---|---|
| `web/e2e/e2e_estacion.py` | Levanta la estación real en el puerto 8090 con una **copia** de `estacion.json`. Lee `/ws` como la página y manda los comandos por `/api/cmd` y `/api/objetivos/manual`. Toma capturas con Chrome headless y revisa cada corrida (código de salida 0 = pasa). |
| `simulink_qdrone2/sim/qd2_e2e_web.m` | Corre `QD2_Sim_Dual` en tiempo real con COMMAND SOURCE = 2: corridas `pid`, `mpc` y `falla` (falla a bordo del dron 2 en t = 45 s). |
| `simulador3d/sim3d_e2e_web.m` | Corre el simulador 3D en tiempo real, sin ventana, con escenario `'web'` y `use_vision_python = true`: corridas `PID` y `MPC`. |

**La copia de la config.** La prueba **no toca** `config/estacion.json`: escribe `estacion_e2e.json` en la carpeta de salida. En la copia:

- la cámara es la **sintética**: anomalías inventadas de `mock_world` (2 fuegos y 1 toma sobre la N), que detecta el detector falso;
- con la fuente `simulink` se pone `web.permitir_sintetica_en_simulink = true`, **solo** porque `QD2_Sim_Dual` es una simulación. La UI muestra una barra roja que lo recuerda.

**Nunca uses esa copia con los drones reales.**

## 2. Prueba automática contra `QD2_Sim_Dual` (PID, MPC y falla a bordo)

Abre dos terminales. El orden no importa: la estación se reconecta sola.

**Terminal 1, MATLAB:**
```matlab
cd simulink_qdrone2/sim
R = qd2_e2e_web();               % pid, mpc y falla; unos 9 min en tiempo real
```
Sin escritorio:
```bat
matlab -sd C:\ruta\carpeta_con_startup_fin -batch "cd('<repo>\simulink_qdrone2\sim'); R = qd2_e2e_web();"
```

**Terminal 2, estación:**
```bat
cd web
%USERPROFILE%\.venvs\qd2_web\Scripts\python e2e\e2e_estacion.py --fuente simulink --corridas pid,mpc,falla
```

**Qué hace la estación en cada corrida.** La corrida termina sola 3 s después de DONE. La estación detecta la siguiente porque el tiempo de Simulink vuelve a 0, y en ese momento vacía su lista de objetivos.

1. START, 2 s después de ASSIGN_ROLES.
2. Un objetivo manual, toma en (0.80, −0.50), 4 s después de entrar a MISSION.
3. PAUSE a los 15 s de MISSION y RESUME 5 s después. En la corrida `falla` no hay pausa.
4. LAND ALL cuando los dos drones marcan **en home** (< `mision.tol_xy`, 0.12 m) durante 2 s.

**Salida.** Va a `%TEMP%\qd2_e2e\`; se cambia con `--salida`.

| Archivo | Contenido |
|---|---|
| `e2e_simulink.log` | Bitácora: estados, alertas, FSM, objetivos, comandos y eventos de la estación |
| `resumen_simulink.json` | Resumen de cada corrida |
| `servidor_simulink.log` | Log de la estación |
| `capturas/*.png` | Capturas de la UI |

Al final, la bitácora imprime `RESUMEN <corrida>: PASA/FALLA` y la lista de fallas.

**Comprobaciones por corrida:**
- llega a DONE;
- alertas: ninguna, salvo la 9 en `falla`;
- todos los objetivos publicados salen como **ambos**, es decir, Simulink los confirmó en y(56:60);
- todos quedan visitados, salvo en `falla`;
- los dos drones quedan en el suelo, a < 0.15 m de su home (salvo en `falla`);
- todos los comandos llegan como "recibido" (ack) y responden HTTP 200;
- el WebSocket llega a ≥ 10 Hz;
- la UI no marca una falla en una corrida sin falla.

## 3. Prueba automática contra el simulador 3D (PID y MPC)

```matlab
cd simulador3d
R = sim3d_e2e_web();                                     % PID y MPC, unos 5 min
R = sim3d_e2e_web({'PID'}, struct('mostrar', true));     % con la ventana 3D, para verlo
```
```bat
cd web
%USERPROFILE%\.venvs\qd2_web\Scripts\python e2e\e2e_estacion.py --fuente sim3d --corridas PID,MPC
```
- **Escenario `'web'`:** el simulador pone los mismos objetos que dibuja la cámara sintética de la UI. Así los ids y los estados coinciden.
- **Detector:** con `use_vision_python = true` el detector interno del simulador no publica. Publica solo la estación.

## 4. La misma prueba a mano, desde la UI

Sirve para verlo con tus ojos o para enseñarlo.

1. **Config de prueba.** Copia `config\estacion.json` a, por ejemplo, `%TEMP%\estacion_sim.json`. En la copia pon `"camara": {"fuente": "sintetica", ...}` y, solo para `QD2_Sim_Dual`, `"web": {"permitir_sintetica_en_simulink": true}`.
2. **Estación:**
   ```bat
   cd web
   %USERPROFILE%\.venvs\qd2_web\Scripts\python -m app.server --config %TEMP%\estacion_sim.json --fuente simulink
   ```
   Abre `http://127.0.0.1:8080`. Con el simulador 3D usa `--fuente sim3d`.
3. **Simulación:** elige una de las dos.
   - **`QD2_Sim_Dual`:**
     - abre `simulink_qdrone2/sim/QD2_Sim_Dual.slx`;
     - pon VERSION en 1 (PID) o 2 (MPC) y COMMAND SOURCE = 2;
     - si no quieres que publique el detector simulado: `QD2_SIM_OVERRIDE.objetivos = zeros(0,3);` en el workspace;
     - para la falla a bordo: `QD2_SIM_OVERRIDE.falla_fsm = [2 45];`;
     - pulsa Run. Simulation Pacing ya está activo.
   - **Simulador 3D:** en `qdrone2_sim3d.m` pon `escenario = 'web'`, `use_vision_python = true` y `usar_tcp = true`, y córrelo.
4. **En la UI:**
   1. **START.** Confirma el diálogo.
   2. Revisa los indicadores de la barra superior. Deben decir "Puente 50 Hz", "Video ~13 fps", "Modelo PID/MPC" y el estado de la misión.
   3. **Anomalías → Clic en el mapa 2D…** para agregar un objetivo manual.
   4. Prueba **PAUSE** y **RESUME**.
   5. Espera a RETURN_HOME. Cuando las dos tarjetas de **Drones** digan **en home**, pulsa **LAND ALL**.
   6. Revisa la pestaña **Eventos**.

## 5. Resultados (22-nov-2026)

### 5.1 `QD2_Sim_Dual` en tiempo real + estación real

Corrida final después de las correcciones. Estación real (`app.server`, fuente `simulink`), Simulink R2025a con pacing.

| | PID | MPC | MPC + falla a bordo del dron 2 en t = 45 s |
|---|---|---|---|
| Resultado de `e2e_estacion.py` | **PASA** | **PASA** | **PASA** |
| Estados | 0→1→2→3→4→5→10→5→6→7→8 | igual | 0→1→2→3→4→5→**9**→7→8 |
| DONE | 142.5 s (163.5 s de reloj) | 96.5 s (102.8 s de reloj) | 52.9 s |
| Objetivos (los 4 salen como "ambos") | 4/4 visitados: #1 toma (−0.21, −0.06) en 27.7 s; #2 toma manual (0.80, −0.50) en 101.8 s; #4 fuego (−0.20, −1.12) en 115.1 s; #3 fuego (0.60, 1.28) en 127.0 s | 4/4 visitados: 24.1, 41.4, 55.3 y 61.9 s | 2 de 3 visitados; el #3 quedó asignado al abortar |
| y(55) | 0 todo el tiempo | 0 | **9** a los 45.06 s (la FSM a bordo pasó a 7 a los 45.00 s) |
| Separación | dh mínima en el aire 1.150 m (consignas 1.104 m); guard activo 86 % de MISSION | apilados 3.1 s; dh mínima 0.073 m; error z del de abajo 0.0074 m máx. / 0.0036 m RMS | E ≥ `d_min_h` a 1.54 s de la falla |
| Estado del dron en la UI | Flight State de tierra 0→1→2→3→9 ("Paro (MS Stop) (fin de vuelo)", sin rojo) | FSM a bordo 1→2→3→4→5→6→9 ("(fin de vuelo)", sin rojo) | Dron 2: **7 "EMERGENCIA: rampa (error)"** y luego **9 "(falla)"**, en rojo. Dron 1: 6→9 "(fin de vuelo)", **sin rojo** |
| Aterrizaje (pos. final vs. home) | d1 (−1.02, 0.02) vs. (−1.00, 0.00); d2 (1.02, −0.28) vs. (1.00, −0.30) | los dos a ≤ 0.01 m de su home | E aterrizó en (−0.51, −0.68). Su home (−1, 0) queda a ~0.94 m del dron caído en (−0.07, 0.11), así que la librería lo baja donde quedó. R quedó en su punto de caída. Los dos con z = 0. |

**En las tres corridas:**
- **Comandos:** START, PAUSE, RESUME y LAND ALL respondieron HTTP 200 y salieron como "recibido por Simulink" (ack igual al `cmd_seq`).
- **Puente:** 50 Hz de telemetría y 20 Hz de comandos, con 0 reconexiones durante cada corrida. Entre corridas la estación detectó "Simulink se reinició" y vació su lista.
- **WebSocket:** 13.2 a 13.4 Hz.

### 5.2 Simulador 3D en tiempo real + estación real (escenario `'web'`)

| | PID | MPC |
|---|---|---|
| Resultado | **PASA** | **PASA** |
| Estados | 0→1→2→3→4→5→10→5→6→7→8 | igual |
| DONE | 142.9 s | 96.96 s |
| Objetivos | 4/4 visitados: 3 reales detectados de 3 + el manual. Todos salen como "ambos" | 4/4 visitados |
| dh mínima en el aire | 1.157 m | 0.072 m (apilados) |
| y(55) | 0 | 0 |
| Estado del dron en la UI | Flight State de tierra 0→1→2→3→9 (con la corrección; antes era la FSM a bordo) | FSM a bordo 1→…→6→9 "(fin de vuelo)" |
| Paquetes de Python recibidos / comandos | 2912 / 4 | 1992 / 4 |

### 5.3 Pruebas automáticas después de las correcciones
- `web`: **122 passed**. Son las 117 anteriores, más 5 nuevas en `web/tests/test_frontera_e2e.py`.
- `QD2_Sim_Dual` reconstruido:
  - Update Diagram OK.
  - El bloque nuevo no se solapa con ningún otro bloque (0 solapes).
  - Prueba funcional del Flight Plan con arm/takeoff `[0 1 2 2 3 9]`: lo esperado.
  - En MPC pasa la FSM a bordo.
  - Exportado a R2022a, R2022b, R2023a, R2023b y R2025a. Las 5 copias abren con el bloque de 5 entradas.
- Simulador 3D, `probar_sim3d(false)` después del cambio: **13 de 13 pasan**. Entre ellas: misión PID (DONE 120.1 s, dh mínima 1.165 m), misión MPC apilada (dz mínima 0.996 m), heartbeat a x16, primer paquete con START y tiempo real (factor 1.000).

### 5.4 Capturas (`docs/capturas_e2e/`, revisadas)

| Archivo | Qué muestra |
|---|---|
| `simulink_pid_mision_en_curso.png` | PID en MISSION. Vista 3D con la pirámide de la cámara, mapa 2D con el anillo `d_min_h`, "GUARD ACTIVO" y tarjeta "Flight State (tierra): 2 · Despegue / en vuelo" con la distancia a home |
| `simulink_pid_tabla_objetivos.png` | Primer objetivo visitado y la tabla: #1 "visitada", #2* (manual) "asignada", #3 "detectada", todos con origen "ambos" |
| `simulink_pid_done.png` | DONE: 4 visitados, 35/35 waypoints, "en home" y "Paro (MS Stop) (fin de vuelo)" |
| `simulink_mpc_mision_en_curso.png`, `simulink_mpc_tabla_objetivos.png` | MPC ("MPC · apilable") y la tabla con el manual visitado |
| `simulink_falla_alerta9.png` | Barra roja "ALERTA 9 Falla a bordo de un dron", estado ABORT y R en el suelo |
| `simulink_falla_done.png` | DONE con la alerta 9. Dron 1: "DESHABILITADO (fin de vuelo)", sin rojo |
| `sim3d_PID_mision_en_curso.png`, `sim3d_PID_tabla_objetivos.png`, `sim3d_MPC_mision_en_curso.png`, `sim3d_MPC_done.png` | Lo mismo con el simulador 3D |
| `antes_simulink_pid_flight_state_mal.png` | **Antes** de la corrección: "Flight State (tierra): 5 · estado 5" en PID (defecto 1) |

## 6. Defectos encontrados en la frontera y corregidos

1. **`QD2_Sim_Dual` y el simulador 3D mandaban la FSM a bordo en o+2 también en PID** (contrato §3.3: en PID va el Flight State de tierra).
   - **Síntoma en la UI:** "Flight State (tierra): 1 Armado" ya en INIT, "3 Aterrizaje (Land)" durante el despegue y "estado 4/5/6". Ver la captura `antes_…`.
   - **Corrección en `QD2_Sim_Dual`:** el bloque `DRONE STATUS (sim link)` (`sim/qd2_construir_sim.m`) tiene ahora 5 entradas: ST1, ST2, VERSION, TRIG1 y TRIG2.
     - En PID emula el Flight Plan del Equipo2, con la misma tabla de transiciones que `Subsystem` del original (la saqué del `.slx` original).
     - En MPC pasa la FSM a bordo.
     - Reconstruido y exportado a las 5 versiones.
   - **Corrección en el simulador 3D:** `sim3d_correr.m` hace lo mismo, sin e-stop ni pérdida de enlace porque no los emula. Además, en MPC le pasa `fsm_a_bordo` a `qd2_mission_step`, como tierra MPC.
2. **Con la alerta 9, el dron sano salía en rojo como "DESHABILITADO (falla)"** al aterrizar después de la falla del otro. La alerta 9 es global y la regla de `fsm_info` marcaba cualquier 9.
   - **Corrección:** la estación (`web/app/server.py`, `protocol.fsm_info(..., falla_vista)`) recuerda qué dron vio fallar en la corrida actual de Simulink. Con ese historial, solo ese dron sale en rojo.
   - Sin historial (la estación se conectó después de la falla) se mantiene la regla del contrato. El historial se borra cuando Simulink se reinicia.
   - Contrato §3.3 actualizado. Pruebas en `test_frontera_e2e.py`.
3. **El operador no tenía cómo saber cuándo dar LAND ALL.** En LANDING cada dron baja **en su sitio**. En la primera corrida, un LAND ALL dado a < 0.15 m del home (todavía en camino) dejó al respondedor a 0.12–0.13 m de su home.
   - **Corrección:** la tarjeta de cada dron muestra "a X m" de su home o **"en home"** (< `mision.tol_xy`). El script de la prueba espera 2 s con los dos en home.
4. **La configuración podía no coincidir con el modelo que corre.** La E2E corrió tierra MPC con `version_control = "PID"` en la config, y la pestaña Conexiones generaba los `quarc_run` del **binario PID**.
   - **Corrección:** aviso amarillo en la barra cuando y(3) ≠ `version_control`. Prueba en `test_frontera_e2e.py`.
5. **Documentación desactualizada:**
   - `simulador3d/README.md` decía que la UI no lee y(56:60) y llamaba "propuesta" a 56–60;
   - `docs/ARQUITECTURA.md` §3.2 seguía con "el dron acepta 0 o 2", que dejó de ser cierto en la revisión de tierra MPC. Lo actualicé, junto con `Onboard Status [4]`, el enlace perdido como alerta 9 y la falla antes de START.

**Observaciones (no son defectos de frontera):**
- **En PID el respondedor tardó 74 s en llegar al objetivo manual.** Estaba asignado a los 27.7 s y se visitó a los 101.8 s. El SEPARATION GUARD lo retuvo mientras E pasaba cerca (guard activo 86 % de MISSION). Es correcto según la regla de separación, pero alarga la misión.
- **En la corrida con falla, E aterrizó a ~0.6 m de su home.** Es la regla de la librería: si el home queda a menos de `d_min_h + 0.05` del dron caído, aterriza donde está.
- **El log de la estación muestra `ConnectionResetError [WinError 10054]`** cuando Chrome headless cierra el WebSocket de golpe. Es ruido de asyncio en Windows y no afecta.
- **Los textos de los eventos dicen "recibido por Simulink"** aun con la fuente `sim3d`. Es solo cosmético.

## 7. Día de laboratorio: de la simulación a los drones reales

La lista imprimible, con tiempos y criterios de **seguir / no seguir** en cada vuelo, es [`CHECKLIST_LABORATORIO.md`](CHECKLIST_LABORATORIO.md). Aquí va el resumen. Esta sección se corrigió el 22-nov-2026 (revisión de completitud para el laboratorio).

**Antes de ir (en la PC del laboratorio, sin drones)**
1. **Copia el proyecto a una ruta corta fuera de OneDrive** (por ejemplo `C:\QD2`). Con rutas largas el build pasa de 260 caracteres y falla (pasó en la regresión del 22-nov). Incluye lo que no está en git: `vision/modelos/mejor.pt`, `mejor.onnx` y `mejor.onnx.json`, y `simulink_qdrone2/binarios_dron/QD2_DroneStack_PID_2021a.rt-linux_qdrone2` (binario original del Equipo2, md5 `00d4d46e…`).
2. **Revisión automática:** `%USERPROFILE%\.venvs\qd2_web\Scripts\python web\preflight_lab.py` desde la raíz. Solo lee y dice qué falta: modelos de `versiones/`, binarios, ONNX, config, IPs, puertos, Motive abierto y QUARC.
3. **Prueba de la máquina.** `qd2_e2e_web` **no sirve en R2022a**: carga `sim/QD2_Sim_Dual.slx` y `lib/QD2_Mission_Lib.slx`, que están guardados en R2025a. En R2022a haz la prueba a mano de la sección 4 con la copia de `versiones/R2022a/`: en MATLAB `cd simulink_qdrone2\versiones\R2022a` **antes** de `open_system('QD2_Sim_Dual.slx')`. Así se usa la librería R2022a de esa carpeta (el `PreLoadFcn` del simulador agrega `lib/`, donde está la de R2025a). En R2025a o más nuevo, `qd2_e2e_web({'pid'})` sí sirve.
4. En `web`: `%USERPROFILE%\.venvs\qd2_web\Scripts\python -m pytest -q tests` (122 passed). La primera vez, `iniciar_web.bat` necesita internet (Wi-Fi interna) para crear el venv.

**Configuración real (`config/estacion.json`, desde la UI con la fuente `simulink`)**
1. **`camara.fuente = "dron"`.** No debe existir `web.permitir_sintetica_en_simulink`, o debe ser `false`. La barra roja de "CÁMARA SINTÉTICA" **no debe aparecer**.
2. **Pestaña Misión → versión:** `version_control` = PID o MPC, la del modelo de tierra que vas a correr. Si no coincide, sale el aviso amarillo y los `quarc_run` de Conexiones lanzarían el binario equivocado.
3. **Conexiones:** la IP de la PC (antena Alfa "Wi-Fi 2", conectada a la red de los drones) y la IP del dron 1 y del dron 2. La Wi-Fi interna queda para internet. Si se pierde internet al conectar la Alfa, sube la métrica de la Alfa (checklist, etapa 2).
4. **Motive (calibración del día):**
   - en Motive calibra y crea **dos** rigid bodies con IDs distintos;
   - exporta el `.cal` y el `.motive`; el `.motive` debe tener **los dos** rigid bodies (el del Equipo2 del 20-nov tiene uno);
   - **cierra Motive**: el bloque OptiTrack Trackables usa la API de Motive y no puede haber otro proceso con las cámaras;
   - en la pestaña **Motive** de la UI elige (o sube) esos dos archivos y asigna los rigid bodies. Si `archivo_cal`/`archivo_motive` quedan vacíos, el modelo sigue usando las rutas del Equipo2 del 20-nov.
5. **Cualquier cambio de `estacion.json` que lea Simulink (Motive, rigid bodies, bias, pista, `mision.*`) exige un build QUARC nuevo del modelo de tierra.** El `InitFcn` lo aplica al compilar; conectar a un binario ya compilado no lo vuelve a leer. Verificado el 22-nov en R2025a con Update Diagram: el `InitFcn` sí escribe las rutas `.cal`/`.motive` en el bloque. El build QUARC completo no se probó.
6. **Pista:** "Capturar vértice" usa la pose que manda Simulink. Hace falta el modelo de tierra **corriendo** (con la calibración nueva) y la estación conectada. Con un dron apagado sobre cada extremo de tubo pulsa "Capturar vértice", guarda y **vuelve a compilar** el modelo de tierra.
7. **Anomalías → Visión:** el detector debe decir `vision/detector.py (mejor.pt)` (o el modelo afinado con fotos reales, `vision/README.md` §4). **No** debe salir el aviso rojo "no detecta tomas" ni `fomo (Edge Impulse) — respaldo, solo fuego`; eso significa que falta el `.onnx`.

**Lanzar (en este orden: tierra antes que los drones)**
1. **Modelo de tierra:** en R2022a abre el de `versiones/R2022a/` (`QD2_MissionCtrl_Dual_PID.slx` o `_MPC.slx`). Haz el **build QUARC completo** (target `quarc_win64`), **Connect** y **Start** (Monitor & Tune). El build no se ha probado en la laptop porque no tiene el target.
2. **Estación:** `web\iniciar_web.bat --fuente simulink`. Comprueba "Puente 50 Hz", "Modelo PID/MPC" igual a `version_control` y los dos drones rastreados.
3. **Drones:** abre un `cmd` **en `simulink_qdrone2\binarios_dron`** (`quarc_run` busca el ejecutable en la carpeta actual) y pega los comandos `quarc_run` de **Conexiones**.
   - **PID:** `QD2_DroneStack_PID_2021a.rt-linux_qdrone2`, que ya está en esa carpeta.
   - **MPC:** `QD2_DroneStack_MPC.rt-linux_qdrone2`. Se compila en la PC del laboratorio con la carpeta actual de MATLAB en `binarios_dron` (abre `versiones/R2022a/QD2_DroneStack_MPC.slx`, target `quarc_linux_qdrone2`). Tiene que ser de **la misma construcción** que tierra MPC: el dron solo acepta el índice 24 = 2.
   - Si a ~1.2 s de lanzarlo tierra no le manda `isTracking = 1`, el dron se detiene solo. Por eso tierra va primero.
   - Los comandos usan `-r`: según la documentación local de QUARC (`%QUARC_DIR%\quarc\doc\quarc_command_line_tools.html`) descarga y ejecuta; `-D` (solo descargar) es una alternativa excluyente. La ayuda corta es `quarc_run -?`.
4. **Cámara del explorador.** El dron con la D435i hacia abajo va siempre en la **menor x**.
   1. Copia el streamer: `scp web/drone/qdrone2_camera_streamer.py nvidia@<IP_explorador>:~/`.
   2. El QDrone 2 trae su propia RealSense frontal. Mira los números de serie con `ssh nvidia@<IP> "rs-enumerate-devices -s"` y lanza el streamer con `--serie <serie de la D435i de abajo>`. Sin `--serie`, `--camara auto` puede abrir la frontal.
   3. Copia a `camara.intrinsecos` los intrínsecos que imprime al arrancar y reinicia la estación.

**Pruebas en seco, con los drones desarmados (switches Arm/Takeoff abajo)**
- **Telemetría:** los dos drones rastreados y la misión en ASSIGN_ROLES. Cada dron apoyado en el piso debe marcar z ≈ 0 (±0.02 m), porque el autoland exige z ≤ 0.05 m. Si no, corrige `optitrack.bias_z_dronN` y recompila.
- **Ejes:** mueve un dron a mano hacia +x y hacia +y y revisa que la UI lo siga en el mismo sentido, con yaw ≈ 0 mirando a +x.
- **Homes:** a ≥ 1.05 m (PID) o ≥ 0.8 m (MPC). Si no, START da la alerta 1.
- **Estado de los drones:** PID: "Flight State (tierra): 0 · Reposo (Idle)"; MPC: "FSM a bordo: 1 · IDLE", **sin alerta 10**.
- **Video:** "Video ~15 fps" y cajas sobre una pieza impresa puesta debajo del explorador con la mano.
- **Comandos:** START con los switches abajo **no despega**. Revisa en Eventos que el comando sale "recibido" y revisa la capa del operador (`simulink_qdrone2/pid/README.md`, "Abrir en el laboratorio" paso 6).

**Vuelos (detalle y criterios en la checklist)**
1. **Cada dron solo**, en PID, con Command Source = 1 (modo MANUAL del Equipo2, solo dron 1) y Command Enable = 0 (hover sobre su home). Para volar el dron 2 solo se intercambian en la config los rigid bodies y las IPs de dron 1 y dron 2, se recompila tierra y se lanza con el comando de "dron 1". Después se deshace el cambio y se recompila otra vez.
2. **Los dos en PID** (misión): primero sin anomalías, luego con anomalías.
3. **MPC:** cada dron solo (Command Source = 1), primero con el switch `MPC Mode` en PID Backup y después en MPC. Luego la misión MPC.
4. **Seguridad:**
   - **Takeoff/Land abajo** = LAND ALL ordenado (alerta 6). Es la forma normal de abortar desde tierra.
   - **Arm abajo** en vuelo (alerta 7): a bordo es un **Error** (estado 7, rampa de bajada sin control de posición), **no** un aterrizaje. Solo como emergencia.
   - **E-stop** (alerta 5): corta los motores al instante y el dron **cae**. Solo si va a golpear a alguien o algo.
   - **ABORT/RTH** y **LAND ALL** desde la UI.
   - **Nunca pares el modelo de tierra con un dron en el aire**: al perder el enlace el dron pasa a la rampa y, un segundo después, FLIGHT SAFETY corta los motores.
5. **Guarda** después de cada vuelo: la pestaña **Eventos**, `web/datos/objetivos_estacion.json` y los logs `saved_data` de Simulink.

**Pendiente de verificar en el laboratorio** (los informes de cada parte tienen el detalle):
- **Build:** que tierra y dron de la misma construcción compilen con QUARC en R2022a.
- **`quarc_run -r`:** probarlo contra el dron real.
- **MPC:** que el loopback y `Communication_Status` reales no den un "enlace perdido" falso (un enlace perdido en vuelo es la alerta 9 y aterriza la misión); el escape a 1.5 m/s con el MPC real; el LAND ALL con los drones apilados.
- **Parámetros reales de la cámara:** `rotacion_montaje` y el offset del soporte.
- **YOLO con fotos reales** (`vision/README.md` §4).

## 8. Pendientes de esta prueba
- **No se probó con hardware, QUARC ni OptiTrack reales.** Todo es SIL: `QD2_Sim_Dual` y el simulador 3D.
- **La falla a bordo solo se probó en `QD2_Sim_Dual`.** El simulador 3D no emula 7/8, solo pasa la FSM a bordo a la misión.
- **La prueba usa la cámara sintética con el detector falso.** No probé YOLO dentro de la E2E.
- **No se volvieron a tomar las capturas de `simulink_qdrone2/capturas/`** de `QD2_Sim_Dual`. El bloque `DRONE STATUS` ahora es más alto y tiene 5 entradas.
- **Prioridad de las alertas 9/10 frente a las del operador:** lo implementado (TELEMETRY PACK de `construir_modelos.m`) es que la del operador (5–8) prevalece también sobre 9 y 10; falta que los dos equipos lo acuerden formalmente.
- **YOLO con fotos reales:** el modelo actual (`vision/modelos/mejor.onnx`) solo vio imágenes sintéticas (mAP50 0.995 en validación sintética). Falta capturar y etiquetar fotos del laboratorio (`vision/README.md` §4).
