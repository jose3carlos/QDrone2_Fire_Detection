# Sistema dual: 2× QDrone 2 para detectar tomas clandestinas y fuegos (índice)

Este es el punto de entrada del **sistema nuevo** (rama `Quique`). El `README.md` de la raíz y casi todo lo que hay en la raíz (`Trayectory_Model.slx`, `instalar.m`, `models/`, `native/`, `matlab/`, `simulink/`…) son del proyecto anterior de un dron con FOMO; se conservan, pero no forman parte del sistema dual.

**Qué hace.** Dos QDrone 2 rastreados con OptiTrack en una arena de 3 × 3 m con una tubería en N invertida:

- **Explorador (E)**: el de menor x. Recorre la N a 1.5 m con la RealSense D435i mirando hacia abajo. La estación detecta fuegos y tomas con YOLO11n, los geolocaliza y los publica.
- **Respondedor (R)**: el de mayor x. Vuela a 0.5 m, va a cada anomalía, hace hover 3 s encima y la marca como visitada.
- **Versión PID**: el binario original del Equipo2 a bordo; nunca uno encima del otro (distancia horizontal ≥ `d_min_h` = 0.9 m, la vigila el SEPARATION GUARD en tierra).
- **Versión MPC**: MPC de posición a bordo (`QD2_DroneStack_MPC`); se permite apilarse con Δz ≥ `dz_min` = 0.8 m y compensación de downwash.

**El contrato** entre todas las partes (puertos, índices de los mensajes, alertas, reglas de seguridad) es [`ARQUITECTURA.md`](ARQUITECTURA.md). Si algo de este índice no coincide con el contrato, manda el contrato.

## 1. Qué hay en cada carpeta

| Carpeta | Qué es | README |
|---|---|---|
| `config/estacion.json` | Única fuente de configuración (IPs, puertos, Motive, pista, alturas, visión). La leen Simulink, el simulador 3D y la estación web | — (ARQUITECTURA §5–§6) |
| `simulink_qdrone2/equipo2_original/` | Modelos originales del Equipo2 (`QD2_MissionCtrl_NoJoystick_2021a`, `QD2_DroneStack_PID_2021a`). **No se tocan** | — |
| `simulink_qdrone2/lib/` | Librería MATLAB de misión (`qd2_*.m`, compatible con codegen) y `QD2_Mission_Lib.slx` | [`lib/README.md`](../simulink_qdrone2/lib/README.md) |
| `simulink_qdrone2/pid/` | Tierra versión PID: `QD2_MissionCtrl_Dual_PID.slx` y sus pruebas SIL | [`pid/README.md`](../simulink_qdrone2/pid/README.md) (cubre también tierra MPC) |
| `simulink_qdrone2/mpc/` | Tierra versión MPC (`QD2_MissionCtrl_Dual_MPC.slx`) y dron MPC (`QD2_DroneStack_MPC.slx`) | tierra: `pid/README.md`; dron: [`sim/README.md`](../simulink_qdrone2/sim/README.md) §1 |
| `simulink_qdrone2/sim/` | Simulación SIL `QD2_Sim_Dual.slx` (planta, OptiTrack y stack a bordo simulados; mismo puerto :18700) | [`sim/README.md`](../simulink_qdrone2/sim/README.md) §2 |
| `simulink_qdrone2/tests/` | Pruebas de la librería (`correr_pruebas`), del PROTOCOL CHECK y lazos cerrados MATLAB | [`tests/README.md`](../simulink_qdrone2/tests/README.md) |
| `simulink_qdrone2/versiones/` | Exportaciones R2022a, R2022b, R2023a, R2023b y R2025a (tierra, dron y sim **de la misma construcción**) | `pid/README.md`, `sim/README.md` |
| `simulink_qdrone2/binarios_dron/` | Ejecutables de los drones para `quarc_run`: el binario PID original del Equipo2 y, una vez compilado en el laboratorio, el MPC | [`binarios_dron/README.md`](../simulink_qdrone2/binarios_dron/README.md) |
| `simulink_qdrone2/construir_*.m` | Generan todos los `.slx` por script (no se editan a mano) | `pid/README.md`, `sim/README.md` |
| `simulador3d/` | Simulador MATLAB 3D en tiempo real (sin Simulink ni toolboxes), misma librería y mismo puerto :18700 | [`simulador3d/README.md`](../simulador3d/README.md) |
| `web/` | Estación Python: puente :18700, video :18800, visión, geolocalización y UI web en :8080. `web/drone/` = streamer de la cámara para el Jetson; `web/e2e/` = prueba punta a punta | [`web/README.md`](../web/README.md) |
| `vision/` | YOLO11n (fuego / toma): generador sintético en Blender, entrenamiento, evaluación, etiquetado de fotos reales y `modelos/mejor.onnx` | [`vision/README.md`](../vision/README.md) |
| `impresion3d/` | Piezas impresas (fuegos y válvulas) con placas 3MF, STL, STEP y CAD | [`impresion3d/README.md`](../impresion3d/README.md) |
| `docs/` | Contrato, prueba E2E, diagrama de flujo y este índice | — |
| `contexto/`, `referencias/` | Análisis de los repositorios originales y referencias (simulador 2D del compañero, toolboxes de la laptop) | — |

## 2. Cómo arrancar cada cosa

**Estación web sin MATLAB (mock).**
```bat
web\iniciar_web.bat --fuente mock
:: abre http://127.0.0.1:8080 y pulsa START
```

**Simulación Simulink `QD2_Sim_Dual` con la UI.**
```matlab
cd simulink_qdrone2/sim
open_system('QD2_Sim_Dual')   % VERSION 1 = PID / 2 = MPC; COMMAND SOURCE = 2 para mandar desde la UI
```
Luego `web\iniciar_web.bat --fuente simulink` y Run. Sin interfaz: `R = qd2_correr_sim(1);` (PID) o `qd2_correr_sim(2)` (MPC).

**Simulador 3D con la UI.**
```matlab
cd simulador3d
qdrone2_sim3d                 % opciones arriba del script; espera START
```
Luego `web\iniciar_web.bat --fuente sim3d`.

**Laboratorio (drones reales).** Sigue la checklist imprimible [`CHECKLIST_LABORATORIO.md`](CHECKLIST_LABORATORIO.md) (orden, tiempos y criterios de seguir / no seguir). En corto:
1. Copia el proyecto a una ruta corta fuera de OneDrive (p. ej. `C:\QD2`) y corre `%USERPROFILE%\.venvs\qd2_web\Scripts\python web\preflight_lab.py`.
2. Motive: calibra, crea los **dos** rigid bodies, exporta `.cal` y `.motive` y **cierra Motive**. Vincúlalos en la pestaña **Motive** de la UI.
3. Abre `simulink_qdrone2/pid/QD2_MissionCtrl_Dual_PID.slx` o `mpc/QD2_MissionCtrl_Dual_MPC.slx` (en R2022a, el de `versiones/R2022a/`), haz el **build QUARC**, Connect y Start. Después de cualquier cambio de `estacion.json` hay que **volver a compilar**.
4. `web\iniciar_web.bat --fuente simulink`.
5. Lanza cada dron con el `quarc_run` de la pestaña **Conexiones** (`-URI_Host tcpip://<IP_PC>:18001` / `:18002`), desde un `cmd` abierto en `simulink_qdrone2\binarios_dron`. PID: binario original del Equipo2 (ya está ahí). MPC: `QD2_DroneStack_MPC.rt-linux_qdrone2`, compilado en el laboratorio, de la misma construcción que tierra.
6. En el explorador (el de menor x): `ssh nvidia@<IP> "python3 ~/qdrone2_camera_streamer.py --camara auto --serie <serie de la D435i de abajo>"`.
7. Detalle de cada paso: [`PRUEBA_E2E.md`](PRUEBA_E2E.md) §7.

**Visión (solo si se reentrena o se prueba el detector).**
```bat
%USERPROFILE%\.venvs\qdrone2_vision\Scripts\python.exe vision\detector.py --imagen alguna_foto.jpg
%USERPROFILE%\.venvs\qdrone2_vision\Scripts\python.exe vision\evaluar.py --datos vision\datos\sintetico_smoke
```
La estación no necesita torch: corre `vision/modelos/mejor.onnx` con OpenCV.

**Pruebas.**
```matlab
% cada línea, desde la raíz del repositorio
cd simulink_qdrone2/tests; correr_pruebas; cd ../..          % librería: 30 pruebas
cd simulink_qdrone2/pid;   qd2_probar_operador; cd ../..     % capa del operador: 27 casos
cd simulador3d;            res = probar_sim3d(false); cd ..  % simulador 3D: 13 pruebas
```
```bat
cd web && %USERPROFILE%\.venvs\qd2_web\Scripts\python -m pytest -q tests   :: 122 pruebas
```
Con `matlab -batch`, la ventana "QUARC Tip of the Day" bloquea MATLAB: usa `matlab -sd <carpeta> -batch "..."` con un `startup.m` que solo haga `error('fin')` (ver `simulink_qdrone2/tests/README.md`).

## 3. Mapa de documentos

| Documento | Para qué |
|---|---|
| [`ARQUITECTURA.md`](ARQUITECTURA.md) | **Contrato**: misión, protocolos PC⇄dron (16/1 y 24/8 doubles), puente Simulink⇄Python (64/32 doubles), alertas y(55) 0..10, objetivos y(56:60), librería, modelos, UI y visión |
| [`PRUEBA_E2E.md`](PRUEBA_E2E.md) | Prueba punta a punta de la estación real contra `QD2_Sim_Dual` y el simulador 3D, defectos corregidos y resumen del paso a los drones reales (§7) |
| [`CHECKLIST_LABORATORIO.md`](CHECKLIST_LABORATORIO.md) | **Checklist imprimible del día de laboratorio**: copia, Motive, builds, red, cámara, fotos reales, prueba en seco, vuelos uno por uno, PID, MPC y abortos, con tiempos y criterios de seguir / no seguir |
| `simulink_qdrone2/binarios_dron/README.md` | Binarios `.rt-linux_qdrone2` de los drones y opciones de `quarc_run` |
| [`diagrama_flujo.mmd`](diagrama_flujo.mmd) | Diagrama de flujo Mermaid de la misión (se pega tal cual en Lucid o en mermaid.live) |
| `simulink_qdrone2/lib/README.md` | Firmas exactas de la librería, comportamiento del supervisor (falla a bordo, aterrizaje apilado, START rechazado, ABORT antes de despegar) y resultados |
| `simulink_qdrone2/pid/README.md` | Modelos de tierra PID y MPC: cómo abrirlos en el laboratorio, switches del operador, MPC LINK CHECK, qué se agregó al layout del Equipo2 y qué falta verificar |
| `simulink_qdrone2/sim/README.md` | Dron MPC a bordo (PROTOCOL CHECK estricto, índice 24 = 2) y simulación `QD2_Sim_Dual` con sus fallas programables |
| `simulink_qdrone2/tests/README.md` | Qué cubre cada prueba MATLAB |
| `simulador3d/README.md` | Simulador 3D: opciones, controles, uso con la UI y pruebas |
| `web/README.md` | Estación web: arranque, pantalla, fuentes de datos, cámara, detector, config, ids 1..32 y pruebas |
| `vision/README.md` | Datos sintéticos, entrenamiento, métricas del modelo actual y flujo de fotos reales |
| `impresion3d/README.md` | Impresión y armado de fuegos y válvulas |
| `web/GUIA_QDRONE2.md` | Histórico: guía de la estación anterior de un dron |

## 4. Estado de validación (22-nov-2026)

Todo lo que sigue se probó **en software (SIL) en la laptop de desarrollo, con MATLAB R2025a**. Nada se ha volado todavía con el sistema dual. Los resultados detallados están en los README de cada parte; aquí solo va el resumen.

**Probado en SIL**

| Qué | Dónde está el resultado |
|---|---|
| Librería: 30/30 pruebas (misiones PID y MPC, guard, MPC contra `quadprog`, downwash, ABORT/LAND_ALL antes de despegar, homes cerca, timeouts, pérdida de rastreo, falla a bordo apilada, enlace perdido, protocolo, y(56:60), codegen en Simulink) | `simulink_qdrone2/lib/README.md` "Resultados" |
| Modelos de tierra PID y MPC: Update Diagram, 246 bloques originales sin mover, `qd2_probar_operador` 27/27, `qd2_probar_tierra` (nominal PID y MPC, falla a bordo, protocolo, enlace caído) | `simulink_qdrone2/pid/README.md` |
| Dron MPC: COMMANDER real en lazo cerrado con la planta; PROTOCOL CHECK estricto (solo índice 24 = 2) con tramas sintéticas | `simulink_qdrone2/sim/README.md` §1 y "Resultados" |
| `QD2_Sim_Dual`: misión completa PID y MPC, fallas a bordo (apilados, protocolo, enlace caído en los dos sentidos), pérdida de rastreo | `simulink_qdrone2/sim/README.md` "Resultados" |
| Simulador 3D: 13/13 pruebas sin capturas | `simulador3d/README.md`, `PRUEBA_E2E.md` §5.3 |
| Estación web: 122 pruebas (pytest) | `web/README.md` |
| E2E en tiempo real con la estación real: `QD2_Sim_Dual` PID, MPC y MPC con falla a bordo; simulador 3D PID y MPC (todas PASA) | `PRUEBA_E2E.md` §5 |
| YOLO11n: 10 000 imágenes sintéticas, mAP50 = 0.995 en validación sintética; la estación lo carga con OpenCV-ONNX en CPU | `vision/README.md` §3 |

**Falta (hardware y laboratorio)**

- **Build QUARC** de los modelos de tierra y del dron MPC: esta laptop no tiene los targets (`quarc_win64`, `quarc_linux_qdrone2`). Solo se hizo Update Diagram y SIL.
- **Abrir los modelos en un R2022a real** (solo se exportaron).
- **OptiTrack con 2 rigid bodies, los Stream Servers reales y los drones**: nada del enlace real está probado (loopback, `Communication_Status`, respuesta de 8 doubles con la etiqueta y(8) = 2, falsos "enlace perdido").
- **MPC real**: el escape a 1.5 m/s, el vuelo apilado y el LAND ALL apilado con drones reales; medir la fuerza de downwash (en la simulación es un supuesto, `F0` = 5.9 N) y el pivot del rigid body (el autoland exige z ≤ 0.05 m).
- **Cámara real**: intrínsecos de la D435i (`camara.intrinsecos`), `rotacion_montaje` y offset del soporte.
- **YOLO con fotos reales**: 0 fotos reales en el entrenamiento; la E2E usó la cámara sintética con el detector falso, no YOLO. Hace falta capturar y etiquetar ~300–500 fotos del laboratorio (`vision/README.md` §4).
- **`quarc_run`**: los comandos de Conexiones usan `-r`, que según la documentación local de QUARC descarga y ejecuta (`-D` solo descarga y es una alternativa excluyente). La ayuda corta es `quarc_run -?`, no `-h`.
- **Abrir `QD2_Sim_Dual` en R2022a**: `qd2_e2e_web`/`qd2_correr_sim` cargan los `.slx` de R2025a. En R2022a hay que abrir `versiones/R2022a/QD2_Sim_Dual.slx` con esa carpeta como carpeta actual (`PRUEBA_E2E.md` §7).
- **Vuelo de cada dron por separado**: no hay modo de misión con un solo dron. Se usa el modo MANUAL del Equipo2 (Command Source = 1, solo dron 1); para el dron 2 hay que intercambiar rigid bodies e IPs y recompilar (checklist, etapa 7).
- **Acuerdo entre equipos**: prioridad de las alertas 9/10 frente a las del operador (hoy prevalece la del operador).
