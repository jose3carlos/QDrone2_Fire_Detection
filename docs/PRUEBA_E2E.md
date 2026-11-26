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
| `simulador3d/sim3d_e2e_web.m` | Corre el simulador 3D en tiempo real, sin ventana, con escenario `'web'` y `use_vision_python = true`: corridas `PID` y `MPC`. Con `struct('actitud_en_tel', false)` emula un modelo de Simulink viejo (y(61:63) = 0). |

**Opciones de `e2e_estacion.py` para la actitud y(61:63)** (la sección 5.5 las usa):

| Opción | Qué hace |
|---|---|
| `--actitud-esperada si` | Exige y(63) = 1 en todos los mensajes de MISSION y que la UI muestre la actitud del explorador (`actitud_explorador.cruda`). |
| `--actitud-esperada no` | Exige y(63) = 0 siempre y el aviso amarillo "Actitud del explorador no disponible" (modelo viejo). |
| `--sin-actitud` | `vision.usar_actitud = false` en la copia de la config: geolocalización nivelada, para comparar. La cámara sintética sigue dibujando con la actitud real. |
| `--inclinacion-max G` | `vision.inclinacion_max_deg = G` en la copia (p. ej. 1.5 para ver el descarte por inclinación con los ~3° de la simulación). |

En todas las corridas el script compara cada detección que entra al cluster y cada objetivo publicado (sin el manual) con la **verdad** de la cámara sintética: `mock_world.anomalias_mock(cfg)` de la copia de la config, lo mismo que dibuja la estación (`web.mock_anomalias` si está; si no, `default_anomalies` de la pista: con `pista.tubos`, 2 anomalías por pieza **sobre los tubos reales**, alternando toma y fuego; sin `pista.tubos`, las 3 de antes sobre la ruta `pista.vertices`). Imprime `ACTITUD <corrida>` y `GEOLOCALIZACIÓN <corrida>` al final y lo guarda en `resumen_<fuente>.json` (`actitud`, `geolocalizacion`).

**La copia de la config.** La prueba **no toca** `config/estacion.json`: escribe `estacion_e2e.json` en la carpeta de salida. En la copia:

- la cámara es la **sintética**: anomalías inventadas de `mock_world.anomalias_mock` (sin `pista.tubos`: 2 fuegos y 1 toma sobre la ruta; con `pista.tubos`: 2 por pieza sobre los tubos reales, p. ej. 4 con las 2 piezas del 24-nov), que detecta el detector falso. Los resultados de las tablas de abajo son de antes de `pista.tubos` (verdad 3/3);
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
- la UI no marca una falla en una corrida sin falla;
- cada objetivo publicado por la estación (sin el manual) queda a ≤ `vision.radio_cluster_m` (0.25 m) de una anomalía verdadera de su clase;
- con `--actitud-esperada`, lo de y(63) descrito arriba.

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
- `web`: **122 passed** (cifra de esa fecha; hoy son 167, con las pruebas de actitud y(61:63) y del ONNX 480×640). Son las 117 anteriores, más 5 nuevas en `web/tests/test_frontera_e2e.py`.
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

### 5.5 Con la actitud del explorador y(61:63) (22-nov-2026, después del commit de la corrección por inclinación)

Misma prueba (estación real `app.server` en :8090, puente :18700, tiempo real, R2025a), ahora con y(61:63) llenos: `QD2_Sim_Dual` y el simulador 3D mandan el roll y pitch **reales** de la planta del explorador. Todas las corridas con `--actitud-esperada si`, salvo la del modelo viejo (`no`). Error = distancia a la anomalía verdadera de la misma clase; "por observación" es cada detección que entró al cluster; "por objetivo" es la posición publicada al llegar a DONE (3 objetivos: 2 fuegos y la toma; el manual no cuenta).

| Corrida | Resultado | DONE (t sim) | Objetivos | y(63) = 1 en MISSION | \|roll\| / \|pitch\| máx. en la UI | Frames con actitud / nivelados | Descartadas por inclinación | Error por observación: mediana / p95 / máx. | Error por objetivo (máx.) |
|---|---|---|---|---|---|---|---|---|---|
| `QD2_Sim_Dual` PID | **PASA** | 141.3 s | 4/4 visitados, verdad 3/3 | 1360/1360 | 1.63° / 2.40° | 1285 / 0 | 0 | 6.4 / 31 / 57 mm (n = 528) | 8.9 mm |
| `QD2_Sim_Dual` PID, `--sin-actitud` | **PASA** | 136.5 s | 4/4, verdad 3/3 | 1336/1336 | 1.61° / 2.68° | 0 / 1266 | 0 | 23.4 / 57 / 84 mm (n = 548) | 11.7 mm |
| `QD2_Sim_Dual` MPC | **PASA** | 96.8 s | 4/4, verdad 3/3 | 689/689 | 2.62° / 3.54° | 864 / 0 | 0 | 8.2 / 34 / 50 mm (n = 328) | 4.0 mm |
| `QD2_Sim_Dual` MPC, `--sin-actitud` | **PASA** | 96.3 s | 4/4, verdad 3/3 | 685/685 | 2.63° / 3.36° | 0 / 863 | 0 | 46.9 / 81 / 104 mm (n = 327) | 8.9 mm |
| Simulador 3D PID | **PASA** | 141.7 s | 4/4, verdad 3/3 | 1409/1409 | 1.61° / 1.82° | 1306 / 0 | 0 | 7.1 / 30 / 120 mm (n = 546) | 8.1 mm |
| Simulador 3D PID, `--sin-actitud` | **PASA** | 137.6 s | 4/4, verdad 3/3 | 1326/1326 | 1.58° / 2.59° | 0 / 1287 | 0 | 22.0 / 56 / 85 mm (n = 554) | 12.5 mm |
| Simulador 3D MPC | **PASA** | 96.9 s | 4/4, verdad 3/3 | 693/693 | 2.41° / 3.15° | 867 / 0 | 0 | 8.2 / 36 / 53 mm (n = 323) | 6.1 mm |
| Simulador 3D MPC, `--sin-actitud` | **PASA** | 97.1 s | 4/4, verdad 3/3 | 689/689 | 2.33° / 3.10° | 0 / 880 | 0 | 36.5 / 69 / 99 mm (n = 336) | 11.2 mm |
| Simulador 3D MPC, `--inclinacion-max 1.5` | **PASA** | 97.0 s | 4/4, verdad 3/3 | 679/679 | 2.42° / 2.99° | 870 / 0 | **131** | 10.3 / 37 / 174 mm (n = 194) | 4.1 mm |
| Simulador 3D MPC, **modelo viejo** (`actitud_en_tel = false`, `--actitud-esperada no`) | **PASA** | 97.1 s | 4/4, verdad 3/3 | 0/693 (y(63) = 0 siempre) | — (la tarjeta no muestra actitud) | 0 / 879 | 0 | 7.8 / 30 / 49 mm (n = 332) | 5.8 mm |

**En todas las corridas:**
- Estados 0→1→2→3→4→5→10→5→6→7→8 (con PAUSE/RESUME), **sin alertas** (y(55) = 0 todo el tiempo), los 4 objetivos salen como "ambos" y quedan visitados, y los dos drones aterrizan en su home.
- Los únicos mensajes con y(63) = 0 en las corridas con actitud son los de INIT/WAIT_TRACKING, cuando todavía no hay explorador (p. ej. 27 de 2029 en `QD2_Sim_Dual` PID).
- `qd2_correr_sim` compara y(61:62) con `x(7:8)` de la planta del explorador: error **0** en PID y MPC, y(63) = 1 en el 98.6 % / 98.0 % de los pasos, inclinación máxima 2.41° / 3.64°.
- Avisos: el rojo de la cámara sintética (esperado) y, en las corridas MPC, el amarillo de "el modelo es MPC pero `version_control` = PID" (esperado: la config real dice PID). En la corrida del modelo viejo aparece además el amarillo "Actitud del explorador no disponible (y(63) = 0)" y Visión dice "no disponible (y(63) = 0 …): geolocalización nivelada".

**Lo que dicen los números:**
- **La corrección por inclinación funciona punta a punta.** Con inclinaciones de solo 2–3.5°, el error por observación baja de 22–47 mm (mediana) a 6–8 mm, y el p95 de 56–81 mm a 30–36 mm. La mejora es mayor en MPC, que se inclina más.
- **Por objetivo la diferencia es menor** (máx. 4–9 mm con actitud contra 9–12.5 mm sin ella), porque el promedio del cluster cancela parte del error de un lado y del otro de la N. Con inclinaciones mayores (viento, maniobras bruscas, el dron real) la diferencia crecería: 5° ≈ 0.13 m a 1.5 m.
- **El descarte por inclinación funciona en vivo.** Con el límite bajado a 1.5°, la estación descartó 131 detecciones (n = 194 entraron al cluster contra ~330 normalmente) y la misión terminó igual, con los 3 objetos publicados en el mismo orden que sin el límite. Con el límite real (15°) no se descartó nada: la simulación no pasa de 3.8°.
- **El modelo viejo sigue funcionando** nivelado, con su aviso. En esa corrida la cámara sintética también dibuja nivelada (no tiene la actitud), así que su error no dice nada sobre la física; solo prueba que la estación no depende de y(61:63).
- Los máximos por observación de 120 mm y 174 mm son casos aislados (el p95 queda en 30–37 mm). No investigué de qué frame salieron; no afectaron a ningún objetivo publicado.

**Observación (no es defecto de la actitud):** en PID, con `--sin-actitud`, el respondedor visitó la toma central a los **88 s** en lugar de a los **27.7 s**, en las tres corridas niveladas (dos de `QD2_Sim_Dual` y una del simulador 3D). Una corrida repetida con la telemetría grabada muestra que el SEPARATION GUARD estuvo activo desde la publicación (t = 16.7 s) hasta t = 83 s (salvo en PAUSED), mientras E recorría la N: R se retiró a ~0.4 m de su home (1.28, −0.58) y solo avanzó hacia la toma cuando E se alejó (a (0.14, 0.16) entre 70 y 78 s, con el guard todavía activo). La toma se había publicado en (−0.204, −0.034); con actitud se publica en (−0.21, −0.06). Es la misma sensibilidad del guard ya anotada en §6 (el objetivo manual de 74 s): en PID, unos centímetros en la posición del objetivo pueden cambiar mucho el tiempo de visita.

Bitácoras de estas corridas: `e2e_<fuente>.log` y `resumen_<fuente>.json` en la carpeta `--salida` de cada una (no están en el repositorio). Las capturas de estas corridas no se copiaron a `docs/capturas_e2e/`; en ellas la tarjeta del explorador muestra "actitud (cruda)" con la inclinación.

### 5.6 Pruebas automáticas después de la E2E con actitud
- `web`: **163 passed** (después, con `web/tests/test_detector_onnx.py` del ONNX 480×640: 167).
- Simulador 3D, `probar_sim3d(false)`: **15 de 15 pasan** (antes 14; después, con la geolocalización con actitud del detector interno: 18 de 18, ver `simulador3d/README.md`). La nueva es la 14: con `actitud_en_tel = false`, 1500 mensajes (1399 con explorador) salen con y(61:63) = 0 aunque la planta llegó a 1.68°.
- **Después de la revisión (22-nov-2026, `ExtModeMexArgs` entre comillas, Conexiones y lectura de `vision` en `sim3d_config`):** `web` 167 passed; `probar_sim3d(false)` 18 de 18; `runtests('test_qd2_lib')` 32 de 32; E2E simulador 3D MPC con actitud (`--actitud-esperada si`) **PASA**: DONE a 97.0 s, 4/4 visitados, verdad 3/3, y(63) = 1 en 682/682 mensajes de MISSION, 872 frames con actitud, 0 descartadas, error por observación mediana 8.9 mm (p95 35 mm, máx. 68 mm, n = 322), por objetivo máx. 6.1 mm.

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
6. **(E2E con actitud) `e2e_estacion.py` se caía con el aviso de actitud no disponible.** Con la salida redirigida a un archivo, Windows usa cp1252, y el texto del aviso amarillo ("5° de inclinación ≈ 0.13 m") tiene "≈", que no existe en cp1252: `print()` lanzaba `UnicodeEncodeError` al entrar a MISSION y la prueba del modelo viejo se cortaba (el simulador se quedaba en PAUSED sin heartbeat).
   - **Corrección:** `sys.stdout.reconfigure(errors="replace")` al inicio del script. La bitácora `.log` ya se escribía en UTF-8 y no cambia.
   - Se repitió la corrida del modelo viejo: **PASA** (tabla de §5.5).

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
4. En `web`: `%USERPROFILE%\.venvs\qd2_web\Scripts\python -m pytest -q tests`: **0 failed** (387 passed el 26-nov; algunas salen *skipped* si la copia no trae `vision\datos\sintetico_smoke` ni el respaldo `mejor_640x640_*.onnx`; conteo vigente en `web/README.md`). La primera vez, `iniciar_web.bat` necesita internet (Wi-Fi interna) para crear el venv.

**Configuración real (`config/estacion.json` en solo explorador; `config/estacion_dual.json` en dual, con la estación arrancada con `--config config\estacion_dual.json`; desde la UI con la fuente `simulink`)**

Lo que se mide en el día (Motive, bias, pista, cámara, visión) se guarda en la config con la que corre la estación. Para el dual hay que copiarlo a `estacion_dual.json` (checklist, Etapa 8-0): nada sincroniza los dos archivos, y `web\preflight_lab.py --config config\estacion_dual.json` avisa si difieren.
1. **`camara.fuente = "dron"`.** No debe existir `web.permitir_sintetica_en_simulink`, o debe ser `false`. La barra roja de "CÁMARA SINTÉTICA" **no debe aparecer**.
2. **Pestaña Misión → versión:** `version_control` = PID o MPC, la del modelo de tierra que vas a correr. Si no coincide, sale el aviso amarillo y los `quarc_run` de Conexiones lanzarían el binario equivocado.
3. **Conexiones:** la IP de la PC (antena Alfa "Wi-Fi 2", conectada a la red de los drones) y la IP del dron 1 y del dron 2. La Wi-Fi interna queda para internet. Si se pierde internet al conectar la Alfa, sube la métrica de la Alfa (checklist, etapa 2).
4. **Motive (calibración del día):**
   - en Motive calibra y crea **dos** rigid bodies con IDs distintos;
   - exporta el `.cal` y el `.motive`; el `.motive` debe tener **los dos** rigid bodies (el del Equipo2 del 20-nov tiene uno);
   - **cierra Motive**: el bloque OptiTrack Trackables usa la API de Motive y no puede haber otro proceso con las cámaras;
   - en la pestaña **Motive** de la UI elige (o sube) esos dos archivos y asigna los rigid bodies. Si `archivo_cal`/`archivo_motive` quedan vacíos, el modelo sigue usando las rutas del Equipo2 del 20-nov.
5. **Cualquier cambio de `estacion.json` que lea Simulink (Motive, rigid bodies, bias, pista, `mision.*`) exige un build QUARC nuevo del modelo de tierra.** El `InitFcn` lo aplica al compilar; conectar a un binario ya compilado no lo vuelve a leer. Verificado el 22-nov en R2025a con Update Diagram: el `InitFcn` sí escribe las rutas `.cal`/`.motive` en el bloque. El build QUARC completo no se probó.
6. **Pista:** "Capturar vértice" usa la pose que manda Simulink. Hace falta el modelo de tierra **corriendo** (con la calibración nueva) y la estación conectada. Para los tubos reales elige *Qué se edita* = "Tubo real (pista.tubos)", pon un dron apagado en el piso sobre cada extremo/esquina del tubo, pulsa "Capturar vértice" y "Guardar tubos" (no requiere recompilar). Para la ruta de vuelo ("Ruta de vuelo (pista.vertices)") los vértices deben quedar dentro de la zona con rastreo (envolvente): guarda y **vuelve a compilar** el modelo de tierra.
7. **Anomalías → Visión:** el detector debe decir `vision/detector.py (mejor.pt)` (o el modelo afinado con fotos reales, `vision/README.md` §4). **No** debe salir el aviso rojo "no detecta tomas" ni `fomo (Edge Impulse) — respaldo, solo fuego`; eso significa que falta el `.onnx`.

**Lanzar (en este orden: tierra antes que los drones)**
1. **Modelo de tierra:** en R2022a abre el de `versiones/R2022a/`: `QD2_MissionCtrl_Dual_PID.slx` (solo explorador, `estacion.json`), `QD2_MissionCtrl_Dual_PID_2D.slx` (dual PID, `estacion_dual.json`; o `laboratorio\INICIAR_DUAL.bat`) o `_MPC.slx`. Haz el **build QUARC completo** (target `quarc_win64`), **Connect** y **Start** (Monitor & Tune). El build no se ha probado en la laptop porque no tiene el target.
2. **Estación:** `web\iniciar_web.bat --fuente simulink` (dual: `... --config ..\config\estacion_dual.json`, relativa a `web` porque el `.bat` se cambia a esa carpeta, o la ruta completa; sin eso START sale rechazado porque el modo no coincide con el del modelo). Comprueba "Puente 50 Hz", "Modelo PID/MPC" igual a `version_control` y los dos drones rastreados.
3. **Drones:** abre un `cmd` **en `simulink_qdrone2\binarios_dron`** (`quarc_run` busca el ejecutable en la carpeta actual) y pega los comandos `quarc_run` de **Conexiones** **de uno en uno**, dentro de la prueba de correspondencia (ver *Pruebas en seco*): primero solo el del dron 1.
   - **PID:** `QD2_DroneStack_PID_2021a.rt-linux_qdrone2`, que ya está en esa carpeta.
   - **MPC:** `QD2_DroneStack_MPC.rt-linux_qdrone2`. Se compila en la PC del laboratorio con la carpeta actual de MATLAB en `binarios_dron` (abre `versiones/R2022a/QD2_DroneStack_MPC.slx`, target `quarc_linux_qdrone2`). Tiene que ser de **la misma construcción** que tierra MPC: el dron solo acepta el índice 24 = 2.
   - Si a ~1.2 s de lanzarlo tierra no le manda `isTracking = 1`, el dron se detiene solo. Por eso tierra va primero.
   - Los comandos usan `-r`: según la documentación local de QUARC (`%QUARC_DIR%\quarc\doc\quarc_command_line_tools.html`) descarga y ejecuta; `-D` (solo descargar) es una alternativa excluyente. La ayuda corta es `quarc_run -?`.
4. **Cámara del explorador.** El dron con la D435i hacia abajo va siempre en la **menor x**.
   1. Copia el streamer y su relanzador (los dos en la misma carpeta): `scp web/drone/qdrone2_camera_streamer.py web/drone/correr_streamer.sh nvidia@<IP_explorador>:~/`.
   2. Mira los números de serie con `ssh nvidia@<IP> "rs-enumerate-devices -s"` y lanza: `ssh nvidia@<IP> "bash ~/correr_streamer.sh --camara auto --serie <serie de la RealSense que mira hacia abajo>"`. Queda en segundo plano, sobrevive al cierre de ssh y se relanza si muere; `--estado` muestra el log y `--detener` lo para (comandos con la IP en **Conexiones**). Si `rs-enumerate-devices -s` lista más de una RealSense, sin `--serie` `--camara auto` puede abrir otra.
   3. Copia a `camara.intrinsecos` los intrínsecos que imprime al arrancar y reinicia la estación.

**Pruebas en seco, con los drones desarmados (switches Arm/Takeoff abajo)**
- **Telemetría:** los dos drones rastreados y la misión en ASSIGN_ROLES. Cada dron apoyado en el piso debe marcar z ≈ 0 (±0.02 m), porque el autoland exige z ≤ 0.05 m. Si no, corrige `optitrack.bias_z_dronN` y recompila.
- **Ejes:** mueve un dron a mano hacia +x y hacia +y y revisa que la UI lo siga en el mismo sentido, con yaw ≈ 0 mirando a +x.
- **Homes:** a ≥ 1.05 m (PID) o ≥ 0.8 m (MPC). Si no, START da la alerta 1. Solo lo revisa un modelo dual (`_PID_2D` o MPC); en solo explorador no aplica. Homes duales PID sugeridos: checklist, Etapa 8-pre.
- **Correspondencia:** con los dos binarios detenidos, lanza solo el `quarc_run` de "dron 1", mueve a mano el dron que pitó y comprueba que en la UI se mueve el dron 1 (luego lanza el 2 y lo mismo). Si se mueve el otro, el rigid body o la IP están cruzados: no volar (checklist, Etapa 6). En dual se repite siempre después de poner el rigid body real del dron 2 y `red.ip_dron2` (checklist, Etapa 8-pre).
- **Estado de los drones:** PID: "Flight State (tierra): 0 · Reposo (Idle)"; MPC: "FSM a bordo: 1 · IDLE", **sin alerta 10**.
- **Video:** "Video ~15 fps" y cajas sobre una pieza impresa puesta debajo del explorador con la mano.
- **Comandos:** START con los switches abajo **no despega**. Revisa en Eventos que el comando sale "recibido" y revisa la capa del operador (`simulink_qdrone2/pid/README.md`, "Abrir en el laboratorio" paso 6).

**Vuelos (detalle y criterios en la checklist)**
1. **Cada dron solo**, en PID, con Command Source = 1 (modo MANUAL del Equipo2, solo dron 1) y Command Enable = 0 (hover sobre su home). Para volar el dron 2 solo se intercambian en la config los rigid bodies y las IPs de dron 1 y dron 2, se recompila tierra y se lanza con el comando de "dron 1". Después se deshacen los dos cambios (rigid bodies **e** IPs), se recompila otra vez y se repite la prueba de correspondencia.
2. **Los dos en PID** (misión, modelo `_PID_2D` con `estacion_dual.json`): primero la prueba en seco dual, luego sin anomalías y con anomalías (checklist, Etapa 8).
3. **MPC:** cada dron solo (Command Source = 1), primero con el switch `MPC Mode` en PID Backup y después en MPC. Luego la misión MPC.
4. **Seguridad:**
   - **Takeoff/Land abajo** = LAND ALL ordenado (alerta 6). Es la forma normal de abortar desde tierra.
   - **Arm abajo** en vuelo (alerta 7): a bordo es un **Error** (estado 7, rampa de bajada sin control de posición), **no** un aterrizaje. Solo como emergencia.
   - **E-stop** (alerta 5): corta los motores al instante y el dron **cae**. Solo si va a golpear a alguien o algo.
   - **ABORT/RTH** y **LAND ALL** desde la UI.
   - **Nunca pares el modelo de tierra con un dron en el aire**: al perder el enlace el dron pasa a la rampa y, un segundo después, FLIGHT SAFETY corta los motores.
5. **Guarda** después de cada vuelo: la pestaña **Eventos**, `web/datos/objetivos_estacion.json` (en dual, `web/datos/objetivos_estacion_dual.json`) y los logs `saved_data` de Simulink.

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
- **La E2E con actitud no valida los signos de roll y pitch del dron real.** La cámara sintética dibuja con la misma convención con la que la estación proyecta (`geo.R_mundo_cuerpo`), así que la prueba confirma que la cadena y(61:63) → interpolación → proyección → cluster es coherente, no que OptiTrack/QUARC manden los ángulos con ese signo. Eso sigue siendo la prueba del libro en el laboratorio (`CHECKLIST_LABORATORIO.md`). Por código, la planta (`qd2_quad_plant`: R = Rz·Ry·Rx, pitch > 0 acelera hacia +x, roll > 0 hacia −y) y el mock usan la misma convención.
- **Inclinaciones chicas.** Las simulaciones no pasan de 3.8°; con el límite real de 15° no se descartó ninguna detección. El descarte solo se ejercitó bajando el límite a 1.5°.
- **No se volvieron a tomar las capturas de `simulink_qdrone2/capturas/`** de `QD2_Sim_Dual`. El bloque `DRONE STATUS` ahora es más alto y tiene 5 entradas.
- **Prioridad de las alertas 9/10 frente a las del operador:** lo implementado (TELEMETRY PACK de `construir_modelos.m`) es que la del operador (5–8) prevalece también sobre 9 y 10; falta que los dos equipos lo acuerden formalmente.
- **YOLO con fotos reales:** el modelo actual (`vision/modelos/mejor.onnx`) solo vio imágenes sintéticas (mAP50 0.995 en validación sintética). Falta capturar y etiquetar fotos del laboratorio (`vision/README.md` §4).
