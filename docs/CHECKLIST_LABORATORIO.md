# Checklist del día de laboratorio: 2× QDrone 2 (domingo 22 / lunes 23 nov 2026)

Fecha: ____________  Operador MATLAB/switches: ____________  Operador UI: ____________  Observador de seguridad: ____________

Lista para imprimir. Se sigue **en orden**. Cada etapa de vuelo tiene un criterio **SEGUIR / NO SEGUIR**: si sale NO SEGUIR, no pases a la siguiente etapa; aterriza, anota y corrige.
Contrato: `docs/ARQUITECTURA.md`. Detalle de cada paso: `docs/PRUEBA_E2E.md` §7, `simulink_qdrone2/pid/README.md` ("Abrir en el laboratorio"), `web/README.md`, `vision/README.md` §4.

Tiempo total estimado en el laboratorio: **6–7 h** (las fotos reales van en paralelo, con otra persona y la laptop).

---

## ABORTOS DE EMERGENCIA (tenerlo a la vista todo el día)

| Situación | Qué hacer | Qué pasa |
|---|---|---|
| Algo no se ve bien, pero los drones están estables | **Switch Takeoff/Land abajo** (área 1 del modelo de tierra) | LAND ALL ordenado (alerta 6): R aterriza primero, E se aparta. Funciona aunque la estación Python esté caída |
| Hay que regresar a casa | UI: **ABORT/RTH** o **LAND ALL** (piden confirmación) | Regresan a su home / aterrizan en su sitio |
| La estación Python se cae o se congela | Nada inmediato: la misión pasa a **PAUSED** (hover) por heartbeat. Después, **Takeoff/Land abajo** | Hover en su sitio, luego LAND ALL |
| Barra amarilla "Misión en PAUSA probablemente por heartbeat" (el puente se reconectó o hubo > 1 s sin telemetría) | Revisar el enlace estación ⇄ Simulink y dar **RESUME** (o **LAND ALL**) | Simulink pausa con > 1 s sin mensajes de la estación y **no reanuda solo**: hover en su sitio mientras nadie decida |
| Barra roja "Dron N sin enlace con tierra en vuelo" (`comm_ok` = 0 o Flight State 9; también queda en Eventos) | Dual PID: si el dron ya está en el piso, esperar la **alerta 9** (sale sola a ~1 s; ver la fila siguiente). Si sigue en el aire, o en solo explorador: **LAND ALL** (o **Takeoff/Land abajo**). No ABORT/RTH: el otro dron se quedaría en hover en su home | Ese dron no recibe consignas y a bordo entra en emergencia (FSM 7 a los 0.5 s, motores cortados a ~1 s). Con el binario PID tierra no recibe su FSM: en **dual** lo detecta cuando lo **ve en el piso** (dron caído, alerta 9); en solo explorador **no se entera** (sigue en MISSION, sin alerta) |
| Un dron pierde el rastreo de OptiTrack | Nada inmediato: a los 0.2 s (`mision.t_perdida_aterrizar_s`, alerta 2) la misión lo **aterriza sola** (takeoff = 0, nunca más, aunque vuelva el rastreo; RESUME no hace nada) y en dual el otro se aparta, regresa a su home y aterriza (ABORT → LANDING → DONE). No hay PAUSED. Mantener a la gente lejos de donde cae el perdido. Si la misión no sale de PAUSED con alerta 2 (solo pasa con `t_perdida_aterrizar_s` > 0.2 en `estacion.json`, o con el modelo de tierra sin recompilar): **no dar RESUME**, **Takeoff/Land abajo** | El dron perdido entra en emergencia a bordo a los 0.2 s (rampa de bajada sin control de posición, deriva ~0.5 m: vuelos 2-4 del 24-nov). Hay que relanzar su modelo |
| Un dron baja o aterriza solo en dual sin que la misión lo pida (batería ≤ 14.0 V a bordo, error a bordo, pérdida de enlace, tarjeta "sin comm") | Dual PID: **nada inmediato**. Cuando OptiTrack lo ve en el piso (rastreado) más de 1 s (`mision.t_caido_s`), tierra reacciona sola: **alerta 9** "Dron N en el piso sin que la misión lo pidiera (pérdida de enlace, error a bordo o batería): el otro se aparta, regresa y aterriza" (también en Eventos). Si la alerta 9 no sale en ~3 s con el dron ya en el piso (o el dron se queda bajo pero en el aire), **Takeoff/Land abajo** (o **LAND ALL**). No ABORT/RTH: deja al otro en hover en su home | Misión ABORT → LANDING → DONE: al caído se le baja takeoff y nunca vuelve a recibirlo; el otro se aparta ≥ 1.60 m del caído a su altura, regresa a su home (o aterriza donde quedó si su home está cerca del caído) y aterriza. Solo con takeoff = 1, en un estado de vuelo (no en LANDING ni DONE) y si ese dron ya había despegado (R marcando a 0.5 m no la dispara: el umbral del piso es 0.20 m para R y 0.25 m para E). Sin rastreo no aplica (eso es la alerta 2). Relanzar el binario del caído **solo después de DONE o de Stop de tierra, con Arm y Takeoff abajo**. Si fue por batería, cambiarla antes. **Si dispara sin razón en el primer vuelo**: poner `"t_caido_s": 0` en `mision` de `estacion_dual.json` (desactivada: la UI avisa en amarillo) y **recompilar `_PID_2D`**; entonces vuelve a lo de antes (la misión no se entera: LAND ALL a mano). En solo explorador no cambia nada: la misión no se entera, **Takeoff/Land abajo** |
| OptiTrack/Motive pierde a los **dos** drones a la vez | Nada útil desde tierra. Alejar a la gente; E-stop solo si van a golpear algo | Los dos pasan a 7 a bordo a los 0.2 s y bajan en su sitio sin control de posición (deriva ~0.5 m cada uno). Después: Stop de tierra y relanzar los dos binarios |
| Un dron va a chocar contra una persona o algo | **Emergency Stop** | **Corta los motores de los dos al instante: los drones CAEN** (alerta 5) |
| Último recurso, sin Simulink | `quarc_run -q …` ("Detener" en Conexiones) | Detiene el modelo del dron: **cae** |

**Nunca:**
- Bajar **Arm/Disarm** para aterrizar. En vuelo, a bordo eso es un **Error** (estado 7: rampa sin control de posición; con poco empuje el modelo se detiene y el dron puede caer). Arm abajo solo como emergencia (alerta 7).
- Detener el modelo de tierra (Stop) con un dron en el aire: el dron pierde el enlace, pasa a la rampa y **a 1 s FLIGHT SAFETY corta los motores**.
- Volar con la copia de config de la E2E (`estacion_e2e.json`) o con la barra roja "CÁMARA SINTÉTICA" visible.

---

## Sesión previa. Soporte de la cámara (con el dron en el laboratorio, 1–2 días antes) — 1 h + impresión

Sigue `impresion3d/README.md`, sección "Soporte de la cámara". La pieza A tarda varias horas en imprimirse, así que no cabe en la etapa 5.
- [ ] Imprime la probeta (~30 min). Con ella elige `TUERCA_EC_A` (fila, eje vertical) y `TUERCA_EC_B` (pared, eje horizontal) y comprueba el patrón de 45 mm con la cámara.
- [ ] Fotos del frente, de un lado y de abajo del explorador. Mide con calibrador los valores `MEDIR EN LAB` (espesor del carbono, travesaño, lengüeta de la escuadra original, enchufe USB-C, altura de la cámara sobre el piso, aros) y confirma que la escuadra original usa los 2 M3 traseros (A) o no (B).
- [ ] Holgura del cable USB ≥ 90 mm. Si no alcanza, consigue una extensión USB-C 3.1 Gen 1 corta.
- [ ] Regenera `cad/soporte_camara.py` con los valores medidos, imprime la pieza final en PETG/ASA (brim 5 mm) y móntala (pasos 2 a 7 del README).
- [ ] Mide la posición de la lente RGB respecto al pivote de Motive y actualiza `camara.offset_camara_m` (etapa 5).

## Etapa 0. Antes de salir (laptop, día anterior) — 45 min

- [ ] Copiar el proyecto a la USB **sin** lo pesado (en `cmd`):
  `robocopy "%USERPROFILE%\OneDrive\Escritorio\QDrone2_Deteccion_Tomas_Pemex" E:\QD2 /E /XD slprj saved_data runs __pycache__ datos impresion3d /XF *.slxc *.autosave`
  (~80 MB. `datos` excluye `vision/datos` y `web/datos`; **no** excluye `vision/datos_reales`. Los códigos de salida 0 a 7 de robocopy no son error.)
- [ ] En la USB existen: `E:\QD2\vision\modelos\mejor.pt`, `mejor.onnx`, `mejor.onnx.json` y `E:\QD2\simulink_qdrone2\binarios_dron\QD2_DroneStack_PID_2021a.rt-linux_qdrone2` (`certutil -hashfile … MD5` = `00d4d46e7a2e07158690fe98a9b756af`).
- [ ] Copiar también `%USERPROFILE%\Downloads\Equipo2.zip` (respaldo del binario y del modelo original).
- [ ] Laptop: `%USERPROFILE%\.venvs\qdrone2_vision\Scripts\python.exe vision\entrenar.py --help` responde (para el afinado con fotos reales). Llevar el cargador.
- [ ] Cargar **todas** las baterías de los QDrone 2 (4S). A bordo, con ≤ 14.0 V durante 0.3 s se activa el aterrizaje por batería baja, y queda activo.
- [ ] Material: antena Alfa (RTL8811AU), USB, tubería en N, 3 fuegos y 3 tomas impresos, distractores (plantas, carritos, cinta, calcomanías de flama), cinta métrica, cinta para marcar los homes, soporte de la cámara ya impreso en PETG/ASA y montado en el explorador (ver la sesión previa, arriba), tornillos de repuesto (2 M3×6 ISO 7380-2, 2 M3×8 de nylon, tuercas M3), calibrador.
- [ ] Pesar los dos drones (el explorador lleva la D435i y el soporte, ~+32 g). Anotar: dron 1 ______ kg, dron 2 ______ kg (el PID y el MPC a bordo usan 1.504 kg de trim). El soporte adelanta el centro de masa del explorador ~6 mm: en el primer hover revisar que no derive ni sostenga cabeceo.

## Etapa 1. Instalar en la PC del laboratorio — 40 min

- [ ] `robocopy E:\QD2 C:\QD2 /E`. **Ruta corta y fuera de OneDrive**: con rutas largas el build QUARC pasa de 260 caracteres y falla.
- [ ] MATLAB: `version` → anotar release: ________ (se espera R2022a). Se usarán los `.slx` de `simulink_qdrone2\versiones\<release>\`.
- [ ] QUARC instalado para ese release (menú QUARC en Simulink; `getenv('QUARC_DIR')` no vacío).
- [ ] Comprobaciones del build de [`GUIA_BUILD_QUARC_LAB.md`](GUIA_BUILD_QUARC_LAB.md) §1 (20 min): `qc_get_target_types` incluye `win64` y `linux_qdrone2`, compilador de Visual Studio C++, firewall para 18001/18002 y `ExtModeMexArgs` de tierra con comillas (§0: ya corregido en el repositorio; solo comprobarlo). **NO SEGUIR** a las etapas de vuelo si faltan los targets.
- [ ] Python ≥ 3.10: `py -3 --version`. Si no está, instalarlo (marcar "Add python.exe to PATH").
- [ ] Con internet (Wi-Fi interna): `C:\QD2\web\iniciar_web.bat --fuente mock`. La primera vez crea el venv (~5 min). En `http://127.0.0.1:8080`, START del mock llega a DONE. Cerrar.
- [ ] `cd C:\QD2\web` → `%USERPROFILE%\.venvs\qd2_web\Scripts\python -m pytest -q tests` → **0 failed** (el 26-nov eran 387 passed; si la copia no trae `vision\datos\sintetico_smoke` ni el respaldo `vision\modelos\mejor_640x640_*.onnx`, que no van a git, algunas salen *skipped*). El conteo vigente está en `web\README.md` ("Pruebas"): un número distinto no es falla, un *failed* sí.
- [ ] `cd C:\QD2` → `%USERPROFILE%\.venvs\qd2_web\Scripts\python web\preflight_lab.py`. En este punto lo normal es que falten las IPs, Motive y los intrínsecos; **no** deben faltar archivos.
- [ ] (Opcional, 15 min) SIL en esta PC: en MATLAB `cd C:\QD2\simulink_qdrone2\versiones\R2022a` **y luego** `open_system('QD2_Sim_Dual.slx')` (si abres desde otra carpeta usa la librería de R2025a y no carga). `qd2_e2e_web` **no** sirve en R2022a. Prueba a mano de `PRUEBA_E2E.md` §4 con una **copia** de la config.

## Etapa 2. Red — 15 min

- [ ] Router de los drones encendido. La **antena Alfa ("Wi-Fi 2")** se conecta a la red de los drones y la **Wi-Fi interna** a internet.
- [ ] `ipconfig`: la Alfa tiene IP de la red de los drones (Equipo2: PC 192.168.2.11, dron 192.168.2.16), **no** 169.254.x.x.
- [ ] Encender los dos drones (batería) y esperar a que arranque el Jetson (~1–2 min).
- [ ] IPs de los drones (lista DHCP del router o etiqueta): dron 1 ____________ dron 2 ____________. `ping` responde.
- [ ] UI (fuente `simulink`) → **Conexiones**: guardar las IPs. La IP de la PC que muestra es la de la Alfa.
- [ ] Internet sigue funcionando. Si se perdió al conectar la Alfa, Windows está mandando todo por la Alfa. Hay que subirle la métrica a la Alfa (PowerShell como administrador): `Set-NetIPInterface -InterfaceAlias "Wi-Fi 2" -InterfaceMetric 100`.
- [ ] `ssh nvidia@<IP_explorador> "echo ok"` responde.

**SEGUIR si:** los dos drones responden al ping desde la Alfa. **NO SEGUIR si:** alguno no responde. No continúes hasta que respondan.

## Etapa 3. OptiTrack / Motive — 30 min

**Config activa.** De la Etapa 2 a la 7-bis la estación corre sin `--config`, así que todo lo que se guarda desde la UI o a mano (IPs, `.cal`/`.motive`, rigid bodies, `bias_z`, actitud, pista, cámara, visión) va a `config\estacion.json` (solo explorador, modelo `QD2_MissionCtrl_Dual_PID`). El dual usa **otro** archivo, `config\estacion_dual.json`, con el modelo `QD2_MissionCtrl_Dual_PID_2D`: lo medido aquí se pasa en la **Etapa 8-0**.

- [ ] Abrir Motive. Calibrar con la varita y fijar el piso con la escuadra en la misma orientación que usó el Equipo2 (+x del marco RSF hacia adelante). Calidad de la calibración: ____________ (Equipo2: "Exceptional", 0.494 mm).
- [ ] Marcadores con **patrón distinto** en cada dron. Crear **dos** rigid bodies con cada dron en el piso **mirando a +x** (así yaw = 0). Anotar los IDs: dron 1 = ____, dron 2 = ____ (distintos).
- [ ] Exportar la calibración (`.cal`) y los rigid bodies (`.motive`, con **los dos** rigid bodies) a `C:\Users\user\Documents\OptiTrack\<fecha>\`.
- [ ] **Cerrar Motive.** El bloque OptiTrack Trackables usa la API de Motive y no puede haber otro proceso con las cámaras.
- [ ] UI → **Motive**: elegir el `.cal` y el `.motive` nuevos (o subirlos) y asignar los rigid bodies al dron 1 y al dron 2. Guardar.
- [ ] `preflight_lab.py`: `optitrack.archivo_cal` y `archivo_motive` "existe", "Motive cerrado", y el `.motive` interpretado: `[OK] rigid body N del dron 1 …` (y del dron 2); ninguna FALLA "NO está en …" ni "AUXILIAR" (en dual, el dron 2 necesita su rigid body **real**, no el auxiliar de `config/motive/RigidBody_dron1_aux2.motive`). Ver "Preflight antes de CADA vuelo" (Etapa 6).

## Etapa 4. Build del modelo de tierra y pista — 40 min

- [ ] En MATLAB: `cd C:\QD2\simulink_qdrone2\versiones\R2022a` y abrir `QD2_MissionCtrl_Dual_PID.slx`.
- [ ] En la consola deben salir `qd2_cargar_config: calibration_file <- …` y `rigid_body_file <- …` con las rutas del día. Si no salen, o el build falla en ese bloque, escribirlas a mano en OptiTrack Trackables (área 2) y guardar.
- [ ] **Build QUARC** (target `quarc_win64`) → **Connect** → **Start**. Anotar si compiló a la primera: ______. *Esto nunca se ha probado.* Comandos, qué debe verse y errores típicos: [`GUIA_BUILD_QUARC_LAB.md`](GUIA_BUILD_QUARC_LAB.md) §2.1 y §3.
- [ ] `C:\QD2\web\iniciar_web.bat --fuente simulink`: "Puente 50 Hz", "Modelo PID", los dos drones **rastreados**. Los drones están encendidos en el piso, sin lanzar sus binarios.
- [ ] Cada dron en el piso marca **z = 0 ± 0.02 m**: el autoland a bordo exige z ≤ 0.05 m. Si no, corregir `optitrack.bias_z_dronN` en `estacion.json` (y después en `estacion_dual.json`, Etapa 8-0).
- [ ] Mover cada dron a mano hacia +x y hacia +y: la UI lo sigue en el mismo sentido y el yaw ≈ 0 cuando mira a +x.
- [ ] **Actitud del explorador, y(61:63): llega y los signos son correctos** (2 min, sin volar, Motive cerrado). Se mira en la tarjeta del explorador (el de menor x), fila *actitud (cruda)* (φ = roll, θ = pitch): y(61:62) tal como llegan de OptiTrack, sin signos ni offset de la estación. Si dice "no disponible (y(63) = 0)" con el dron rastreado, el modelo de tierra es viejo: abrir el de `simulink_qdrone2\versiones\<release>\` de esta copia; si aun así sale viejo, anotarlo y regenerarlo **en la laptop** (`construir_modelos`), no en el laboratorio. Mientras tanto la estación geolocaliza suponiendo el dron nivelado. Con el explorador en el piso mirando a +x:
  1. Nivelado: φ ≈ 0° y θ ≈ 0° (± 1°). Anotar φ ______ θ ______. Si alguno pasa de ~1°, el rigid body se creó inclinado: escribir esos dos valores en `vision.actitud_offset_deg` (`[roll, pitch]` en grados) de `estacion.json` y reiniciar la estación.
  2. Un libro de ~3 cm bajo las patas **izquierdas** (lado izquierdo arriba): φ debe salir **positivo** (≈ +5° a +10°). Anotar ______.
  3. El libro bajo las patas **traseras** (cola arriba, nariz abajo): θ debe salir **positivo**. Anotar ______.
  4. Girar el dron 90° en sentido antihorario visto desde arriba: ψ ≈ +90°.
  - Si φ o θ sale con el signo contrario, poner −1 en esa posición de `vision.actitud_signos` (p. ej. `[1, -1]` si el pitch sale invertido) y reiniciar la estación. No hay que recompilar ni tocar código. Anotarlo en "Anotar lo que falló".
  - Después de cambiar `actitud_signos` o `actitud_offset_deg`, la fila *actitud (cruda)* **no cambia** (sigue mostrando y(61:62)). La corrección se confirma en la fila **actitud usada** que aparece debajo, en la misma tarjeta: nivelado ≈ 0°, y con el libro los signos ya positivos. Con video, la misma actitud usada sale en Anomalías → Visión → *Actitud*.
- [ ] **Tubos reales (`pista.tubos`), medidos ANTES de cualquier vuelo con anomalías.** Con la tubería ya fija, en la pestaña **Pista** elegir en *Qué se edita* **Tubo real (pista.tubos) — pieza N** (o *pieza nueva*), poner un dron (apagado) en el piso sobre cada extremo/esquina de esa pieza, en orden a lo largo del tubo → **Capturar vértice** → **Guardar tubos** (cada pieza con ≥ 2 puntos; no requiere recompilar: Simulink no lee `pista.tubos`). La estación descarta toda toma a más de `vision.dist_max_tubo_toma_m` (0.25 m) de los tubos reales (`pista.tubos`; sin esa clave, de la ruta partida en `tramos_sin_tubo`) ("lejos del tubo"): con tubos mal puestos se descartarían las tomas reales. Comprobar en el mapa 2D que la línea blanca cae sobre el tubo real (error < 0.05 m en cada extremo). Tubos medidos: ______________________.
- [ ] **Ruta de vuelo (`pista.vertices`) dentro de la zona con rastreo.** NO capturar la ruta sobre los extremos del tubo: los tubos de las orillas (y ≈ ±1.4) quedan en el borde o fuera de la envolvente (`mision.limites_y` = [−1.4, 1.2]; en dual también `mision.limites_x` = [−1.35, 1.2]); del lado y+ OptiTrack pierde al dron en vuelo en la franja x ~1.3, y ≥ 1.0 (vuelos 2-4 del 24-nov), mientras que hacia y− el rastreo llega más lejos (R vuela hasta y = −1.4 para marcar las tomas de ese borde). La ruta se edita con *Qué se edita* = **Ruta de vuelo (pista.vertices · requiere recompilar)** (máx. 16 vértices, en el orden del recorrido) y debe quedar dentro de la envolvente (punteado violeta en el mapa 2D); un vértice fuera sale en amarillo en la tabla (aviso, no bloquea). Si se cambia: **Guardar ruta** y recompilar (siguiente punto).
- [ ] **Stop** del modelo de tierra (ningún dron vuela) → **Build de nuevo**: la pista y el bias se leen **al compilar**. Connect → Start. `preflight_lab.py` debe decir `[OK] build <modelo>.rt-win64 del …; sello del …: misma config, lib/*.m y qd2_cargar_config.m`; si dice "recompila …: cambió X después del build", falta este Build.
- [ ] **Envolvente de consignas (`mision.limites_y` = [−1.4, 1.2], 25-nov; en dual también `mision.limites_x` = [−1.35, 1.2], 26-nov): RECOMPILAR** `versiones\R2022a\QD2_MissionCtrl_Dual_PID.slx` (solo explorador, `estacion.json`) y, para dual, `QD2_MissionCtrl_Dual_PID_2D.slx` (`estacion_dual.json`) con la carpeta `simulink_qdrone2\lib` del día (trae `qd2_limites_xy.m`). Al abrir/compilar la consola debe decir, en solo explorador, `qd2_cargar_config: envolvente de consignas x [-1.35 1.35], y [-1.40 1.20] m` y, en dual, `qd2_cargar_config: envolvente de consignas x [-1.35 1.20], y [-1.40 1.20] m` (`limites_x` solo está en `estacion_dual.json`: en dual ninguna consigna va a la esquina x 1.2-1.35, y > 0.95 donde OptiTrack pierde el dron; el solo explorador no cambia). Sin recompilar, R puede ir hasta y = 1.35 y x = 1.35 (franja donde OptiTrack pierde el dron). `web\preflight_lab.py` dice "envolvente de consignas … la ruta queda dentro"; en el mapa 2D y la vista 3D se ve en punteado violeta.
- [ ] **Dron caído en dual PID (`lib/qd2_mission_step.m`, 26-nov): RECOMPILAR los dos** modelos de tierra (`QD2_MissionCtrl_Dual_PID.slx` y `QD2_MissionCtrl_Dual_PID_2D.slx`) con la `lib` del día. Solo cambia el dual (`_PID_2D`): si un dron queda en el piso sin que la misión lo pidiera, alerta 9 y el otro se aparta, regresa y aterriza (tabla de emergencias); el solo explorador vuela igual, pero el preflight pide recompilarlo porque cambiaron `.m` de `lib`. `mision.t_caido_s` no va en las configs (1.0 s por defecto); `"t_caido_s": 0` la desactiva (recompilar).

**SEGUIR si:** los dos rigid bodies se rastrean sin cortes en toda la arena, z ≈ 0 en el piso, los ejes cuadran y la actitud del explorador llega con los signos correctos (o corregidos en `vision.actitud_signos`). **NO SEGUIR si:** un dron se pierde en alguna zona de la arena, o los ejes o el yaw están invertidos (revisar la orientación con que se creó el rigid body).

## Etapa 5. Cámara del explorador — 25 min (y fotos reales en paralelo)

- [ ] El explorador (el que lleva la D435i hacia abajo) se coloca **siempre en la menor x**.
- [ ] **`camara.dron`** de `estacion.json` = número del dron que lleva la D435i (por defecto 1). La estación toma el video SIEMPRE de `ip_dron{camara.dron}` y **bloquea START** (409, aviso rojo "El explorador es el dron N pero la cámara está en el dron M") si el explorador es el otro: en dual, reacomodar los drones (el de la cámara con la **menor x**) o cambiar `camara.dron` (**pero si el dron de la cámara YA está en la menor x, los rigid bodies o las IPs están cruzados: no reacomodar, hacer la prueba de correspondencia de la Etapa 6**); en solo explorador, `mision.dron_solo` debe ser igual a `camara.dron` (`preflight_lab.py` da FALLA si no). La cabecera de la cámara dice `dron M (camara.dron)` y la tarjeta de ese dron lleva **CÁMARA**.
- [ ] **Soporte de la cámara** (`impresion3d/README.md`, sección "Soporte de la cámara"). La impresión y el montaje se hacen en la **sesión previa** (al inicio de este checklist); aquí solo se verifica:
  - Variante montada: A / B. Cámara con 2 M3×6 (**nunca M3×8**): lentes abajo, rejilla hacia adelante y USB a la derecha (lado de la muesca en V) → `rotacion_montaje = 0`.
  - Con el dron en el piso, la cara de las lentes queda a ≥ 10 mm del piso: ______ mm. Las hélices giran a mano sin tocar nada y el cable USB no está tenso.
  - Mide la posición de la lente RGB (la del extremo opuesto al USB) respecto al pivote de Motive: x ______, y ______, z ______ m. Para z, si la pose está corregida con `bias_z`: altura sobre el piso + 0.004. Mídelo con el dron **nivelado** y sin restar el pivote: la estación ya sabe por `optitrack.bias_z_dronN` que la pose corregida queda 0.07 m debajo del pivote y, al inclinarse el dron, gira el offset alrededor del pivote.
  - Escríbela en `camara.offset_camara_m` (estimado `[0.212, 0.0325, 0.019]`; hoy dice `[0.20, 0.0, -0.05]`) y reinicia la estación.
- [ ] `scp web/drone/qdrone2_camera_streamer.py web/drone/correr_streamer.sh nvidia@<IP_explorador>:~/` (los dos en la misma carpeta; la pestaña **Conexiones** da los comandos con la IP).
- [ ] Números de serie: `ssh nvidia@<IP> "rs-enumerate-devices -s"`. Si no existe esa herramienta: `ssh nvidia@<IP> "python3 -c \"import pyrealsense2 as rs; [print(d.get_info(rs.camera_info.name), d.get_info(rs.camera_info.serial_number)) for d in rs.context().devices]\""`. Serie de la RealSense: ____________ (se espera **una sola**: la frontal integrada, ahora girada hacia abajo; si sale la IMU, es D435i. Si salen dos, usar la serie de la que mira hacia abajo: sin `--serie`, `--camara auto` puede abrir la otra).
- [ ] Lanzar con el relanzador: `ssh nvidia@<IP> "bash ~/correr_streamer.sh --camara auto --serie <serie>"`. Regresa de inmediato: el streamer queda en segundo plano (setsid + nohup), **sobrevive al cierre de ssh** y se relanza solo si muere. Ver que arrancó: `ssh nvidia@<IP> "bash ~/correr_streamer.sh --estado"` (últimas líneas de `~/streamer_camara.log`, con los intrínsecos y una línea `Estado:` cada 10 s: fps, reaperturas de la cámara, clientes). Log en vivo: `ssh nvidia@<IP> "tail -f ~/streamer_camara.log"`. Detener (al terminar el día o para cambiar argumentos): `ssh nvidia@<IP> "bash ~/correr_streamer.sh --detener"`. Si el log dice `terminó (código 3)` (lectura/apertura/cierre de la cámara colgado > 30 s) o `(código 4)` (sin frames > 60 s, p. ej. *device busy*), el bucle ya lo relanzó con un proceso limpio; si se repite, revisar el cable USB de la RealSense. Tras cambiar la batería sin `--detener`, el `~/.streamer_camara.pid` viejo se ignora solo (se comprueba que el PID sea el script).
- [ ] **Recuperación del video** (2 min, antes de volar): con el video en la UI, `--detener` y volver a lanzar. Eventos dice `Video caído (dron <IP>:18800) ...` y, al volver, `Video de vuelta ... tras N s`; el encabezado de la cámara pasa a rojo *SIN VIDEO hace N s · reconectando* y regresa solo (< 3 s después de que el streamer escucha) sin tocar la estación. Contadores: encabezado de la cámara y Conexiones → *Puente y video*.
- [ ] Copiar la línea `"intrinsecos": {"640x480": {...}}` que imprime a `camara.intrinsecos` de `C:\QD2\config\estacion.json` y reiniciar la estación.
- [ ] UI: "Video ~15 fps". La imagen es **la de abajo**. Con 0, arriba de la imagen = adelante del dron.
- [ ] Anomalías → Visión dice `vision/detector.py (mejor.pt)`, sin aviso rojo.
- [ ] **Geolocalización:** pon una pieza en una posición conocida (mídela con "Capturar vértice" del otro dron sobre ella) y lleva el explorador a mano a ~1.5 m encima. Error de la posición publicada: ______ m.
  - Si el error parece una rotación (sale a la izquierda en vez de adelante), ajustar `camara.rotacion_montaje` (0/90/180/270).
  - Si es un desplazamiento fijo, ajustar `camara.offset_camara_m`.
  - Estos cambios son solo de la estación: se reinicia la estación, sin recompilar Simulink.
- [ ] **Geolocalización con el dron inclinado:** con la pieza en el centro de la imagen, inclinar el explorador a mano unos 10°, primero con la nariz abajo y luego de lado. La posición (x, y) que muestra la etiqueta de la caja en la cámara debe moverse < 0.05 m. Sin la corrección se movería ~0.26 m. Si se mueve el doble (~0.5 m), un signo está invertido: repetir la prueba del libro de la etapa 4. Movimiento medido: ______ m.

**SEGUIR si:** las cajas salen sobre las piezas y el error es < 0.15 m. **NO SEGUIR con anomalías si:** no detecta tomas o el error es > 0.25 m (el radio de cluster). Se puede volar sin anomalías mientras tanto.

### 5b. Fotos reales y afinado (otra persona, laptop) — ~2 h en paralelo

Detalle: `vision/README.md` §4, "Afinado corto el mismo día de laboratorio".
- [ ] La laptop se conecta a la red de los drones. El streamer atiende hasta 3 clientes a la vez (`--max-clientes`), pero cada cliente es otra copia del video por el Wi-Fi del dron: mejor cerrar la estación de la PC del lab mientras se captura.
- [ ] Captura (~30 min, 200–300 fotos): `%PY% vision\capturar_fotos.py --dron <IP_explorador> --cada 0.7 --sesion lab_AAAAMMDD`. Alturas 0.5 / 1.0 / **1.5** / 1.8 m, giros, piezas en bordes y tapadas, 50–100 negativas.
- [ ] Sesión de validación aparte (~60–80 fotos, piezas en otras posiciones): `--sesion lab_val`.
- [ ] Etiquetar (~15–20 min): `etiquetar.py auto --entrada …lab_AAAAMMDD`, `etiquetar.py auto --entrada …lab_val --split val`, `etiquetar.py revisar` (todas).
- [ ] Afinar (~10–30 min): `%PY% vision\entrenar.py --base vision\modelos\mejor.pt --datos vision\datos\sintetico_smoke --epocas 20 --salida vision\modelos\mejor_lab.pt`.
- [ ] Comparar en la validación real: `evaluar.py --modelo vision\modelos\mejor_lab.pt --datos vision\datos_reales` contra lo mismo con `mejor.pt`. mAP50 real: mejor.pt ______ / mejor_lab.pt ______; recall: ______ / ______.
- [ ] **Usar `mejor_lab.pt` solo si mejora las dos cifras.** Llevarlo con `paquete.py exportar --sin_fotos` → `importar` en la PC del lab → `vision.modelo = "vision/modelos/mejor_lab.pt"` a mano en `estacion.json` → reiniciar la estación. Debe decir `vision/detector.py (mejor_lab.pt)`.

## Etapa 6. Prueba en seco (drones desarmados, hélices puestas, nadie en la arena) — 20 min

### Preflight antes de CADA vuelo (y después de cada `git pull`, cambio de config o Build) — 1 min

- [ ] En la PC del laboratorio, sin `--remoto`: `cd C:\QD2` → `%USERPROFILE%\.venvs\qd2_web\Scripts\python web\preflight_lab.py` (dual: `--config config\estacion_dual.json`). Termina con `Resumen: … 0 fallas`. **No volar con una FALLA.** Solo lee: no cambia nada ni lanza nada.
- Qué hacer con cada FALLA (y los AVISOS que importan):

| Línea | Qué hacer |
|---|---|
| FALLA `no hay versiones/R2022a/<modelo>.rt-win64` | Compilar ese modelo en esta PC (solo explorador: `QD2_MissionCtrl_Dual_PID`; dual: `QD2_MissionCtrl_Dual_PID_2D`): `laboratorio\INICIAR_EXPLORADOR.bat` / `INICIAR_DUAL.bat`, o MATLAB `cd …\versiones\R2022a` → abrir el modelo → HARDWARE → Monitor & Tune |
| FALLA `recompila <modelo>: cambió X después del build` | **Stop** de tierra (ningún dron en el aire) → **Build** de nuevo: la config (pista, modo, IDs, envolvente) y las `.m` de `lib` se leen **al compilar**; sin recompilar vuela con lo viejo. Se compara el CONTENIDO con el sello `versiones\R2022a\<modelo>.config_usada.jsonl` (lo escribe `qd2_cargar_config` en cada InitFcn; no se sube a git). X dice qué clave cambió: solo cuentan las que lee el modelo (`version_control`, `red.puerto_*`, `optitrack`, `mision`, `pista.vertices`, `vision.radio_cluster_m`) y las `.m` que usa tierra; `red.ip_*`, `camara`, `pista.tubos`, `web`, el resto de `vision` y `lib/qd2_quad_plant.m`, `qd2_onboard_*`, `qd2_mpc_init/position`… salen como INFO "no hace falta recompilar". Si la línea INFO dice "sin sello" o "comparo fechas" (primer build con esta versión, o build de hace > 30 min sin sello), un `git pull` o una copia sin fechas puede dar esta FALLA de más: recompilar la quita |
| AVISO `<modelo> lee config\estacion_dual.json, no …` | El modelo y la estación usarían configs distintas: correr estación y preflight con `--config` del modelo |
| AVISO `estacion_dual.json difiere de estacion.json en …` (solo dual) | Solo dice **qué claves** difieren, no cuál de los dos archivos tiene el valor bueno: revisarlas una por una. Antes del primer vuelo dual del día, lo normal es que lo medido hoy (bias, tubos, ruta, actitud, cámara, visión) quedó en `estacion.json`: copiar **esa clave** a `estacion_dual.json`. Después de volar dual, lo bueno suele estar en `estacion_dual.json` (`ajustar_tubos.py --config config\estacion_dual.json --aplicar`, o Pista/Motive/Misión guardados desde la estación dual): **no** copiar `estacion.json` encima, que revierte la corrección; el aviso no bloquea. Nunca copiar el archivo entero después de 8-0 (se pierden `red.ip_dron2` y el `.motive` del dron 2). Si el modelo lee la clave que cambió en `estacion_dual.json`, recompilar `_PID_2D`. No cuentan `mision.modo`/`dron_solo`, `red.ip_dron2`, `optitrack.archivo_motive` ni `mision.limites_x` (solo en dual; si falta lo dice la línea de abajo) |
| AVISO `modo DUAL sin mision.limites_x` | A `estacion_dual.json` le falta (o trae inválida) la envolvente en x (típico: se copió `estacion.json` encima en 8-0): poner en `mision` `"limites_x": [-1.35, 1.2]` y recompilar `_PID_2D`. Sin ella las consignas llegan a x 1.35, a la esquina donde OptiTrack pierde el dron |
| AVISO `<modelo>.slx … con fecha posterior al build` | Si se cambió el modelo, recompilar (guardar sin cambios también mueve la fecha) |
| FALLA `optitrack.archivo_motive vacío` / `NO EXISTE en esta PC` | Pestaña **Motive**: elegir el `.motive` del día (ruta de esta PC). Vacío = el `RigidBody.motive` del Equipo2 (solo ID 1): QUARC da "The rigid body ID is invalid or not found" al conectar |
| FALLA `el rigid body N del dron i NO está en <archivo>` | Exportar de Motive un `.motive` con **los dos** rigid bodies (UserData = `rigid_body_dron1/2`; el modelo pide los dos IDs también en solo explorador). En solo explorador basta el auxiliar de `config\motive\RigidBody_dron1_aux2.motive` como ID 2 |
| FALLA `modo DUAL y el rigid body 2 del dron 2 es el AUXILIAR` | **No volar dual**: el dron 2 nunca se rastrearía (WAIT_TRACKING). Crear en Motive el rigid body REAL del dron 2 (UserData = 2), **borrar el auxiliar**, exportar y apuntar `optitrack.archivo_motive` de `estacion_dual.json` a él; o volar en solo explorador |
| FALLA `el rigid body 2 del dron 2 está 2 veces en <archivo> y uno es el AUXILIAR` | Se agregó el rigid body real sin borrar el auxiliar (los dos con UserData = 2): QUARC puede tomar el auxiliar. **Borrar el auxiliar** en Motive y volver a exportar el `.motive` |
| FALLA `el dron solo N usa el rigid body … AUXILIAR` | `mision.dron_solo` o `rigid_body_dronN` apuntan al auxiliar: corregir la config |
| AVISO `misma geometría de marcadores` | Motive puede confundir los drones: mover un marcador en uno y volver a crear su rigid body |
| AVISO `no pude interpretar <archivo>.motive` | No bloquea: revisar a mano en Motive que estén los dos IDs |
| AVISO `Motive está abierto` | Cerrarlo antes de compilar y de correr tierra |
| Otras FALLAs (`camara.dron`, `ip_dron`, `quarc_run`, red) | La línea dice qué cambiar (pestañas Conexiones / Misión) |

Orden de arranque, **siempre**: tierra Start → estación → drones.
- [ ] Switches **Arm/Disarm**, **Takeoff/Land** y **Emergency Stop** abajo/en reposo. Command Source = 0 (misión).
- [ ] Abrir un `cmd` en `C:\QD2\simulink_qdrone2\binarios_dron`. Los `quarc_run` de **Conexiones** (dron 1 y dron 2) se lanzan **de uno en uno**, dentro de la prueba de correspondencia de abajo: primero **solo** el del dron 1 (si ya se lanzaron los dos, "Detener" los dos en Conexiones y relanzar solo el del dron 1). Los comandos usan `-r` (descarga y ejecuta). Ayuda: `quarc_run -?`.
- [ ] Al lanzarlo, el dron hace los pitidos de inicio. No armar antes de ~3 s.
- [ ] **Prueba de correspondencia (rigid body ⇄ IP ⇄ dron físico)**, switches abajo y drones en el piso. Si hoy el dron 2 todavía no tiene `red.ip_dron2` o usa el rigid body AUXILIAR (solo explorador), su parte no prueba nada: se hace en la Etapa 8-pre, después de 8-0. El binario del dron `i` va a `red.ip_dron{i}` y recibe de tierra (puerto 1800i) la pose del rigid body `rigid_body_dron{i}`: si el RB o la IP están cruzados, cada dron controla con la pose del **otro** y los dos se van al despegar. "comm ok" **no** sirve de prueba (la tarjeta `i` dice comm ok con el comando `i`, sea cual sea el dron físico).
  1. Lanzar **solo** el `quarc_run` de "dron 1". El dron que hace los pitidos es el de `ip_dron1`: ponerle cinta "1".
  2. Mover a mano **ese** dron: en la UI se mueven la tarjeta y el punto del **dron 1**, no los del 2. Si se mueve el 2: RB o IP cruzados → **NO volar**, corregir en Motive o en Conexiones.
  3. Lanzar ahora el `quarc_run` de "dron 2" y lo mismo con ese dron (cinta "2").
  4. El dron "1" (o el de `camara.dron`) es el que lleva la cámara.
- [ ] UI: los dos drones "comm ok", "Flight State (tierra): 0 · Reposo (Idle)", misión en ASSIGN_ROLES, y(55) = 0.
- [ ] Homes a ≥ 1.05 m: **solo se revisa con el modelo dual** (`_PID_2D`, Etapa 8-pre); el modelo de solo explorador compilado en la Etapa 4 no revisa la distancia entre homes.
- [ ] **START con los switches abajo: no despega.** En Eventos, el comando sale "recibido".
- [ ] (Al final de esta etapa) Probar el E-stop en tierra: la alerta 5 aparece. El E-stop **queda enganchado a bordo**, así que después hay que **detener y relanzar** los dos binarios de los drones ("Detener" en Conexiones) y hacer Stop/Start de tierra.

**SEGUIR si:** todo lo anterior se cumple y no hay alertas. **NO SEGUIR si:** "comm" no está ok, un dron sale en rojo, o START hace algo con los switches abajo.

---

**Antes de CADA vuelo (repetir):**
- [ ] Baterías recién cargadas en los dos.
  (En **PID** la estación no recibe la batería en vivo: el dron del Equipo2 solo devuelve el eco del timestamp. Se
  revisa **después** del vuelo con los logs a bordo y `simulink_qdrone2/bateria/qd2_bateria.m`. En MPC sale en vivo
  en Misión → Batería.)
- [ ] Arena despejada, observador listo, una mano en el switch Takeoff/Land.
- [ ] Tierra: Stop → Start (el Flight Plan se queda en 3/9 después de un vuelo; si cambió `estacion.json`, **Build**).
- [ ] Drones: "Detener" y relanzar con `quarc_run` (después de tierra).
- [ ] Explorador en la menor x, los dos mirando a +x, homes marcados con cinta. El explorador debe ser el dron de `camara.dron` (etiqueta **CÁMARA**), al menos **5 cm más atrás en x** que el otro (con |x1 − x2| < 5 cm los roles pueden invertirse por el ruido de OptiTrack): si no, START sale rechazado con aviso rojo; reacomodar los drones. **Si el dron de la cámara YA estaba en la menor x y aun así sale "El explorador es el dron N pero la cámara está en el dron M", los rigid bodies o las IPs están cruzados: no reacomodar, repetir la prueba de correspondencia (Etapa 6).**

## Etapa 7. Primer vuelo: cada dron solo (PID, modo MANUAL del Equipo2) — 45 min

No hay misión con un solo dron. Se usa el modo MANUAL original: Command Source = 1, que solo mueve el **dron 1**.
- [ ] El otro dron en el piso, apagado o sin lanzar su binario, a ≥ 1.5 m del que va a volar.
- [ ] Con los dos drones en el piso, desarmados y los switches abajo: **Command Source = 1** (si se cambia en vuelo, el cambio queda pendiente). **Command Enable = 0** ("Home Mode": hover 1 m arriba de donde armó).
- [ ] Subir **Arm/Disarm** y **Takeoff/Land**: arma y despega 1 s después.
- [ ] Hover 30 s. Anotar la deriva xy máx. ______ m y el error en z ______ m.
- [ ] **Takeoff/Land abajo** → aterriza → Flight State 3 → 9. El autoland termina (z ≤ 0.05 m) en < 10 s después de tocar el piso.
- [ ] **Dron 2 solo:**
  1. En la UI intercambiar los rigid bodies (Motive) **y** las IPs (Conexiones) de dron 1 y dron 2.
  2. **Build** de tierra.
  3. Lanzar con el comando de "dron 1" (ahora apunta al dron 2) y repetir el vuelo.
  4. **Deshacer AMBOS intercambios (Motive y Conexiones) y hacer Build otra vez.** Después, repetir la prueba de correspondencia de la Etapa 6: si solo se deshizo uno, los drones quedan cruzados.

**SEGUIR si:** cada dron hace hover estable (deriva < 0.10 m, z ± 0.05 m), aterriza solo y termina en 9 sin alertas. **NO SEGUIR si:** oscila, deriva > 0.20 m, el autoland no termina (revisar `bias_z`), o hay pérdida de enlace o de rastreo.

## Etapa 7-bis. Misión SOLO EXPLORADOR (PID, un solo QDrone) — 40 min

Un solo dron recorre la pista con la misión completa (despegue, N a `z_E`, detecciones, regreso, aterrizaje). El otro no se usa y nunca recibe arm/takeoff. No probado aún en vuelo real.
- [ ] `config/estacion.json` (la de la laptop y **la misma copiada a la PC**): `mision.modo = "solo_explorador"`, `mision.dron_solo = 1` (= `camara.dron`: el dron solo es el que lleva la D435i; si no, START queda bloqueado), la ruta del día (`pista.vertices`, dentro de la envolvente) y los tubos reales (`pista.tubos`, medidos con **Capturar vértice** en *Tubo real*) y `red.ip_pc` = IP de la PC.
- [ ] Motive: rigid body del explorador con el ID de `optitrack.rigid_body_dron1` (en el piso mirando a +x); `rigid_body_dron2` = otro ID distinto. **[NO VERIFICADO]** si el Build/Start falla porque el rigid body 2 no está en el `.motive`, crear uno auxiliar con ese ID y dejarlo fuera del volumen. El segundo QDrone **fuera de la arena** o sin marcadores visibles.
- [ ] MATLAB (PC): al abrir `QD2_MissionCtrl_Dual_PID.slx` la consola dice `qd2_cargar_config: MODO SOLO EXPLORADOR -> dron 1 (rigid body …, Stream Server :18001)`. Si dice `modo dual`, la config de la PC no es la nueva: copiarla y volver a abrir. La "Roles & Homes Preview" no aplica.
- [ ] **Command Source = 0**; Arm, Takeoff y E-Stop abajo → **Build** → Connect → Start (cambiar el modo o la pista obliga a recompilar).
- [ ] **Preflight** (`web\preflight_lab.py` en la PC, Etapa 6): 0 fallas; en particular "build … misma config" y el `.motive` con los dos IDs.
- [ ] Estación (laptop): `web\iniciar_web.bat --fuente simulink --host-puente <IP_PC>` → chip "SOLO EXPLORADOR · dron 1", dron 2 "NO USADO", estado ASSIGN_ROLES, sin avisos rojos.
- [ ] Dron 1 encendido en su home (≈ 1.25, 0.0 mirando a +x, a ≥ 0.3 m de los tubos) → `quarc_run` del dron 1 (Conexiones; `-URI_Host tcpip://<IP_PC>:18001`) → streamer.
- [ ] **Prueba en seco:** switches abajo → START: "START recibido" y el dron **no** despega. LAND ALL / ABORT → vuelve a ASSIGN_ROLES.
- [ ] **Vuelo:** Arm y Takeoff arriba → START: TAKEOFF → HOVER_READY (2 s) → MISSION (recorre todos los waypoints) → RETURN_HOME ("en home") → **LAND ALL** → DONE. En la telemetría el dron 2 siempre con arm/takeoff = 0.
- [ ] Al final: `quarc_run -q` del dron 1 con el dron en el piso → Stop de tierra si no se paró solo.
- [ ] **Después de cada vuelo: caja negra.** La pestaña Anomalías dice `Caja negra: … último: …/vuelo_HHMMSS_simulink.tel.jsonl.gz (N paquetes)` (sin ERROR; se cierra sola 10 s después de DONE). Correr `%USERPROFILE%\.venvs\qd2_web\Scripts\python web\herramientas\caja_negra.py resumen` y revisar estados, comandos con ack, pérdidas de rastreo, máximos, huecos > 0.2 s y avisos; si algo falló, verlo con `%USERPROFILE%\.venvs\qd2_web\Scripts\python -m app.server --repetir <archivo> --puerto 8081` (desde `web`, solo lectura; el `python` del sistema no tiene fastapi ni numpy). **Subir la caja negra** (`git add web/datos/vuelos`: `.tel.jsonl.gz` y `.jsonl`) aunque el vuelo haya salido bien.
- [ ] **Después de cada vuelo:** la pestaña Anomalías dice `Registro de vuelo: …/vuelo_HHMMSS_simulink.jsonl · N líneas` (sin ERROR). Correr `%USERPROFILE%\.venvs\qd2_web\Scripts\python web\herramientas\ajustar_tubos.py --ultimos 1 --svg tubos.svg` (en dual, con `--config config\estacion_dual.json`: si no, avisa que el vuelo es de otra config y `--aplicar` cambiaría `estacion.json`) y revisar las tomas "LEJOS" y el desplazamiento propuesto; si se confirma, `--aplicar` (solo cambia `pista.tubos`; la ruta no) y subir `web/datos/vuelos` (y la config si cambió) a git para la otra sesión.

**Coordenadas (x, y) de los objetos encontrados (solo en la rama `solo-explorador-coordenadas`):**
- [ ] **Antes:** en la PC, `git checkout solo-explorador-coordenadas` (o copiar `simulink_qdrone2/lib/qd2_coordenadas_objetivos.m` y `versiones/R2022a/QD2_MissionCtrl_Dual_PID.slx` de esa rama). Abrir `versiones\R2022a\QD2_MissionCtrl_Dual_PID.slx` en **R2022a**: hasta abajo del nivel superior, debajo del PYTHON BRIDGE, debe estar el área **COORDENADAS OBJETOS (x,y)**. **Guardarlo una vez en R2022a (Ctrl+S) antes del Build**: el archivo es una exportación desde la laptop (R2025a). Si al abrirlo aparecen avisos de versión de librería de QUARC o de parámetros `source` / `encoding` desconocidos en los To Host File, vienen de eso. Después, **recompilar** (Build) desde `versiones\R2022a`: el ejecutable viejo no tiene el bloque. Al cambiar de rama, `preflight_lab.py` no pide recompilar el dual (`_PID_2D`) por `lib/qd2_coordenadas_objetivos.m`, que solo usa el solo explorador. Al volver a `Quique` con el solo explorador compilado en esta rama, el AVISO `el build usó lib/qd2_coordenadas_objetivos.m, que ya no existen` es correcto: hay que recompilarlo.
- [ ] **En vivo** (Monitor & Tune): `Display coords_objetos (x,y)` muestra 32 filas `[x y]` en m (marco OptiTrack): las filas 1..n son los objetos (tomas y fuegos) en el orden en que la estación los publicó, el resto 0. `Display n objetos` dice cuántos **ids** publicó la estación. Debe coincidir con el número de filas de la tabla de la pestaña **Anomalías** de la UI con origen *estación* o *ambos*, **contando las que dicen `(= #k)`**. **No** se compara con "Objetivos conocidos": ese número es del supervisor, que une los alias (dos ids a menos de 0.25 m), mientras que el bloque los lista como dos filas casi iguales, así que `n` puede ser mayor. Si la estación se reinició sin recuperar su tabla (Eventos sin "Recuperada la tabla de objetivos…"), los objetos vuelven con ids nuevos y aparecen dos veces (ver `simulink_qdrone2/lib/README.md`).
- [ ] **Antes de parar el modelo de tierra:** anotar o capturar `Display coords_objetos (x,y)` y `Display n objetos`, junto con la tabla de Anomalías de la UI. **El `.mat` de abajo nunca se ha generado con QUARC (este es su primer vuelo)**, y si no se escribe, la tabla se pierde al siguiente Start.
- [ ] **Después del vuelo** (con el modelo de tierra parado): buscar en `versiones\R2022a\saved_data\QD2_MissionCtrl_Dual_PID\` el archivo `log_<fecha>_CoordenadasObjetos.mat`, junto a los `_MissionCtrl.mat` / `_MissionDual.mat`. Tiene la variable `coords_objetos` (66 × N): fila 1 = t, fila 2 = n, filas 3-34 = x, filas 35-66 = y. En MATLAB: `d = coords_objetos(:, end); n = d(2); xy = [d(3:2+n) d(35:34+n)]`. Subirlo a git con la caja negra. **Si no está**, anotarlo en la bitácora (con lo que se anotó del Display) y revisar si external mode seguía conectado al parar y desde qué carpeta se conectó (la ruta es relativa a la carpeta actual de MATLAB).
- [ ] La tabla solo se borra al volver a dar Start al modelo de tierra (nunca durante el vuelo). Detalles en `simulink_qdrone2/lib/README.md`, sección "Coordenadas de los objetos encontrados".

**SEGUIR si:** el recorrido completo sin pausas ni alertas, las detecciones salen "detectada" y aterriza en su sitio. **NO SEGUIR si:** la UI muestra modo dual o "pista distinta", el dron 2 recibe arm, o hay pérdida de rastreo/enlace. Emergencias iguales: Takeoff/Land abajo = LAND ALL ordenado; E-Stop corta motores.

## Etapa 8. Misión PID con los dos drones — 60 min

8-0. **Pasar a dual** (antes del primer vuelo dual y cada vez que se vuelve de solo explorador). El dual es **otro modelo** (`QD2_MissionCtrl_Dual_PID_2D.slx`) y **otra config** (`config\estacion_dual.json`); nada copia solo lo que se midió en las etapas 2-5 (quedó en `estacion.json`).
- [ ] Los dos drones en el piso y sus binarios detenidos ("Detener" en Conexiones). **Stop** de la tierra de solo explorador (`QD2_MissionCtrl_Dual_PID`) y cerrar la estación.
- [ ] Llevar a `config\estacion_dual.json` lo medido hoy (`red.*`, `optitrack.*`, `camara.*`, `vision.*`, `pista.*`): lo más simple es copiar `estacion.json` encima y dejar en `mision` `"modo": "dual"` **sin** `dron_solo` y con `"limites_x": [-1.35, 1.2]` (junto a `limites_y`; `estacion.json` no la trae; si falta, el preflight dual da AVISO `modo DUAL sin mision.limites_x` y el mensaje de la envolvente al compilar dice `x [-1.35 1.35]`) (si ya se voló dual con este `estacion_dual.json` y se le ajustaron tubos o ruta, no copiar encima: llevar solo lo que cambió hoy). Luego poner `red.ip_dron2` y `optitrack.archivo_motive` con el `.motive` que tiene el rigid body **REAL** del dron 2 (sin el auxiliar). No cambiar ningún otro valor.
- [ ] Tierra dual: `laboratorio\INICIAR_DUAL.bat`, o MATLAB `cd C:\QD2\simulink_qdrone2\versiones\R2022a` → abrir `QD2_MissionCtrl_Dual_PID_2D.slx` → **Build** → Connect → Start. La consola debe decir `modo dual (2 drones)` y `envolvente de consignas x [-1.35 1.20], y [-1.40 1.20] m` (si dice `x [-1.35 1.35]`, a `estacion_dual.json` le falta `mision.limites_x`: ver 8-0).
- [ ] Estación con la config dual, en **una sola** máquina (el Stream Server :18700 atiende a un solo cliente):
  - En la PC: `C:\QD2\web\iniciar_web.bat --fuente simulink --config C:\QD2\config\estacion_dual.json` (`INICIAR_DUAL.bat` ya la abre así).
  - En la laptop (como en 7-bis): `web\iniciar_web.bat --fuente simulink --host-puente <IP_PC> --config ..\config\estacion_dual.json` (ruta relativa a `web`: el `.bat` se cambia a esa carpeta). El `estacion_dual.json` de la laptop tiene que ser **el mismo** que el de la PC después de 8-0 (copiarlo, y otra vez después de cada cambio en cualquiera de los dos): el preflight corre en la PC y no ve el de la laptop. Con la estación en la laptop, **no** usar `INICIAR_DUAL.bat` para la estación: abre otra en la PC que le gana el :18700 (cerrar esa ventana, o arrancar tierra desde MATLAB).
  - Sin `--config`, START sale rechazado porque el modo no coincide con el del modelo.
- [ ] **Preflight dual** (`web\preflight_lab.py --config config\estacion_dual.json`): 0 fallas. Recién hecho 8-0 no debería salir el AVISO `estacion_dual.json difiere de estacion.json en …` (`mision.limites_x`, solo en dual, no lo da; ni el AVISO `modo DUAL sin mision.limites_x`, que sale si se olvidó ponerla); si sale, ver en la tabla de la Etapa 6 cuál de los dos tiene el valor bueno, copiar **solo esa clave** y, si el modelo la lee, recompilar.

8-pre. **Homes duales y prueba en seco con `_PID_2D`** (antes del primer vuelo dual).
- [ ] Homes marcados con cinta, los dos mirando a +x: **E** (el dron de `camara.dron`, etiqueta CÁMARA) en ≈ **(−1.0, 0.0)** y **R** en ≈ **(1.0, −0.3)**. Son los de las pruebas SIL con la ruta de `estacion_dual.json` (`test_envolvente_recorte`; en SIL PID el 26-nov: DONE sin alertas, 4/4 visitadas, Δh mín en el aire 1.16 m). Separación ≈ 2.0 m: la misión exige ≥ `d_min_h` + `guard_margen` = 1.05 m tras recortar los homes a la envolvente del dual (x [−1.35, 1.2], y [−1.4, 1.2]), y conviene ≥ 1.20 m para que el ruido no haga aparecer y desaparecer la alerta 1. Δx ≥ 5 cm. Los dos con y < 1.0 (lejos de la franja x ~1.3, y ≥ 1.0 sin rastreo) y a ≥ 0.3 m de los tubos del día (comprobarlo en el mapa 2D). **No** reutilizar para E el home de 7-bis (1.25, 0.0): con E ahí, R solo cabe con |y| ≥ 1.05.
- [ ] **Prueba de correspondencia (Etapa 6) SIEMPRE aquí**, aunque ya se haya hecho hoy: 8-0 cambió `red.ip_dron2` y el `.motive` (rigid body REAL del dron 2), justo lo que valida, y la de la Etapa 6 se hizo con `estacion.json` (dron 2 sin IP o con el auxiliar). Repetirla también cada vez que cambien `optitrack.archivo_motive`, `rigid_body_dronN` o `red.ip_dronN`. Lanzar **solo** el `quarc_run` del dron 1 (Conexiones) y seguir los pasos 1-4 de la Etapa 6; después el del dron 2. Luego el streamer en el dron de `camara.dron`. **Sin esta prueba, no hay START dual.**
- [ ] UI: los dos "comm ok", explorador = dron de `camara.dron` (etiqueta **CÁMARA**) y respondedor el otro, ASSIGN_ROLES, sin alerta 1 ni aviso rojo de cámara.
- [ ] Switches abajo → **START**: "START recibido" y **nadie despega**. LAND ALL o ABORT → vuelve a ASSIGN_ROLES (si queda en DONE, el Stop → Start de tierra de "Antes de CADA vuelo" lo reinicia).

8a. **Sin anomalías** (pista sin piezas). Command Source = 0.
- [ ] **Preflight dual** antes de cada vuelo (`web\preflight_lab.py --config config\estacion_dual.json` en la PC, Etapa 6): 0 fallas; build `QD2_MissionCtrl_Dual_PID_2D` con la misma config y el rigid body 2 **real** (no el auxiliar). Después de ajustar tubos o ruta en `estacion_dual.json`, el AVISO `difiere de estacion.json` es esperado: no copiar `estacion.json` encima (tabla de la Etapa 6).
- [ ] Switches Arm y Takeoff **arriba** (permiso) → START en la UI → TAKEOFF → HOVER_READY → MISSION.
- [ ] Ya en MISSION, sin aviso rojo "Sin video del explorador: no se detecta nada" (sale si no llegan frames en > 2 s con la visión activa; solo avisa: la estación reconecta sola y el streamer se relanza solo; si no se va en unos segundos, revisar `bash ~/correr_streamer.sh --estado` en el dron y `red.ip_dron{camara.dron}`, y decidir si se aborta). Al aterrizar, anotar *caídas (vuelo N)* del encabezado de la cámara: ______.
- [ ] En la UI, Δh entre E y R **siempre ≥ 0.9 m** (`d_min_h`). "GUARD ACTIVO" cuando se acercan.
- [ ] PAUSE (los dos hacen hover) → RESUME.
- [ ] RETURN_HOME → esperar **"en home"** en las dos tarjetas → LAND ALL. Los dos en el suelo a < 0.15 m de su home, DONE, y el modelo de tierra se detiene solo (Stop Allowed).

- [ ] **Después de cada vuelo (8a, 8b y 8c): caja negra y registro**, igual que en 7-bis: Anomalías dice `Caja negra: … último: …` y `Registro de vuelo: …` sin ERROR (los dos en `web\datos\vuelos\AAAA-MM-DD\`); `%USERPROFILE%\.venvs\qd2_web\Scripts\python web\herramientas\caja_negra.py resumen` (estados, acks, pérdidas de rastreo, separación mínima en el aire, huecos, avisos) y `%USERPROFILE%\.venvs\qd2_web\Scripts\python web\herramientas\ajustar_tubos.py --config config\estacion_dual.json --ultimos 1 --svg tubos.svg`. Subir `web/datos/vuelos` aunque el vuelo haya salido bien.

**SEGUIR si:** DONE sin alertas, Δh mín ≥ 0.9 m, aterrizaje en home. **NO SEGUIR si:** Δh < 0.9 m en algún momento, alerta 2/3/4, un dron no termina en 9, "sin comm" o barra roja de enlace en un dron en vuelo, o un dron baja sin que la misión lo pida (en dual PID sale la alerta 9 "Dron N en el piso sin que la misión lo pidiera" y la misión aterriza sola al otro: ver la tabla de emergencias; anotar la causa: batería, enlace o error a bordo).

8b. **Con anomalías:** primero 1 fuego y 1 toma; después 3 y 3.
- [ ] Cada pieza sale detectada → "ambos" (Simulink la confirmó) → asignada → visitada (R hace hover 3 s encima).
- [ ] Error de posición de cada objetivo < 0.15 m. Falsos positivos sobre distractores: ______.
- [ ] Piezas en las orillas (y > 1.2 o y < −1.4, o x > 1.2 en dual): en Anomalías dicen "se marca desde (x, y)" y R hace el hover en ese punto (orilla de la envolvente), no encima de la pieza. R nunca pasa de y = 1.2 ni de x = 1.2 (consignas; la posición puede pasarse unos centímetros por sobretiro): si se acerca a la franja x ~1.3, y ≥ 1.0 y pierde el rastreo, aterriza ahí y E se aparta (≥ 1.60 m), regresa y aterriza.

**SEGUIR si:** las piezas se visitan y no hay objetivos falsos que manden a R a otro lado. **NO SEGUIR si:** hay falsos positivos repetidos (subir `vision.confianza_min` o usar `mejor_lab.pt`), o no detecta tomas.

8c. **Simulacro de aborto en vuelo** (sin E-stop ni Arm):
- [ ] ABORT/RTH desde la UI en MISSION: los dos regresan a su home → LAND ALL.
- [ ] En otro vuelo, **Takeoff/Land abajo** en MISSION: LAND ALL ordenado (alerta 6), R aterriza primero.
- [ ] Guardar después de cada vuelo: pestaña **Eventos**, `web\datos\objetivos_estacion.json` (en dual, `web\datos\objetivos_estacion_dual.json`) y la carpeta `saved_data` de Simulink. Los vuelos duales también quedan en `web\datos\vuelos` (la cabecera dice la config y el modo).

## Etapa 9. Versión MPC — 90 min

- [ ] UI → Misión → versión **MPC** (`version_control`). `preflight_lab.py --version MPC`.
- [ ] Compilar el dron MPC (detalle en [`GUIA_BUILD_QUARC_LAB.md`](GUIA_BUILD_QUARC_LAB.md) §2.4–§2.5, que explica cómo compilar sin descargar al dron):
  1. En MATLAB, `cd C:\QD2\simulink_qdrone2\binarios_dron`.
  2. Abrir `..\versiones\R2022a\QD2_DroneStack_MPC.slx` y hacer el build (`quarc_linux_qdrone2`). Puede fallar al **descargar** a 192.168.2.16; da igual si el archivo `QD2_DroneStack_MPC.rt-linux_qdrone2` quedó en `binarios_dron`.
  3. **Solo la flecha ▾ de Monitor & Tune → Build for Monitoring. NO pulsar el botón grande Monitor & Tune ni Deploy en este modelo**: descargan y arrancan el modelo en 192.168.2.16. Compilar con los dos drones en el piso y sus binarios detenidos ("Detener" en Conexiones).
- [ ] Build de tierra `versiones\R2022a\QD2_MissionCtrl_Dual_MPC.slx` (**misma construcción** que el dron) → Connect → Start → estación ("Modelo MPC", sin aviso amarillo).
- [ ] En seco: "FSM a bordo: 1 · IDLE" en los dos, **sin alerta 10**, batería visible en la tarjeta: dron 1 ______ V, dron 2 ______ V (no despegar con menos de ~15.5 V).

9a. **Cada dron solo** (Command Source = 1, solo el dron 1; el dron 2 con el intercambio de la etapa 7):
- [ ] Después de Start, switch **`MPC Mode` = PID Backup**. Su `InitFcn` lo regresa a MPC en cada arranque. Hover 30 s.
- [ ] En vuelo, cambiar `MPC Mode` a **MPC**. Hover 30 s. Anotar la deriva ______ m y el error z ______ m.

**SEGUIR si:** con MPC la deriva < 0.10 m, z ± 0.05 m, sin FSM 7, y no hay "enlace perdido" falso (alerta 9 sin causa). **NO SEGUIR si:** hay alerta 9 o 10 sin causa, o el hover con MPC es peor que con PID Backup. En ese caso, volar MPC con `MPC Mode` = PID Backup y anotar.

9b. **Misión MPC sin apilarse:** home de R a ≥ 0.9 m en horizontal de cualquier tramo de la N, y sin anomalías.
- [ ] DONE sin alertas, los dos en home.

9c. **Misión MPC con anomalías (apilados):** E a 1.5 m pasa sobre R a 0.5 m.
- [ ] Δz mín ______ m (debe ser **≥ 0.8 m**). Error z del dron de abajo durante el apilado ______ m.

**SEGUIR si:** Δz ≥ 0.8 m, el de abajo no baja más de 0.10 m por el downwash y no hay FSM 7/8. **NO SEGUIR si:** el dron de abajo se hunde o se desestabiliza. En ese caso: **Takeoff/Land abajo**, anotar y no repetir apilados.

9d. (Solo si 9c salió limpio) **LAND ALL con los drones apilados:** R debe esperar a su altura hasta que E esté a ≥ 0.9 m en horizontal y después bajar.

## Etapa 10. Cierre — 15 min

- [ ] Detener los dos drones ("Detener" en Conexiones) **con los dos en el piso**. Después, Stop de tierra.
- [ ] Copiar a la USB: `C:\QD2\config\estacion.json` como `estacion_lab_<fecha>.json` (y, si hubo vuelo dual, `estacion_dual.json` como `estacion_dual_lab_<fecha>.json`), la carpeta `saved_data`, Eventos, `web\datos\objetivos_estacion.json` y, si hubo vuelo dual, `web\datos\objetivos_estacion_dual.json`. Fotos reales con `paquete.py exportar`.
- [ ] `git add web/datos/vuelos` + commit + push: los registros de vuelo del día, de solo explorador y dual (los lee `ajustar_tubos.py` en la laptop) y las **cajas negras** (`*.tel.jsonl.gz`, ≈ 3-5 MB por vuelo; se analizan con `web\herramientas\caja_negra.py resumen`).
- [ ] En la laptop **no** se sobrescriben `config\estacion.json` ni `config\estacion_dual.json` a ciegas: se comparan con los del laboratorio.
- [ ] Anotar lo que falló y en qué etapa: ____________________________________________

---

### Pendientes conocidos (no resueltos antes del laboratorio)
- El build QUARC (tierra `quarc_win64`, dron `quarc_linux_qdrone2`) y la apertura en un R2022a real **nunca se han probado**.
- `quarc_run -r` contra el QDrone 2 real todavía no se ha probado.
- Que el `InitFcn` escriba las rutas de Motive **durante el build QUARC** (sí funciona en Update Diagram).
- MPC real: falsos "enlace perdido", el escape a 1.5 m/s, el downwash real (en la simulación `F0` = 5.9 N es un supuesto), el LAND ALL apilado.
- Un solo binario MPC con un solo "Drone Weight" para los dos drones, aunque el explorador pesa más.
- YOLO sin fotos reales **cenitales** hasta la etapa 5b: solo vio fotos de Blender, negativos de COCO y capturas de webcam de frente (ya no confunde caras con tomas, pero falta validarlo desde el dron).
- El modo MANUAL en tierra MPC solo se probó como lógica (`qd2_probar_operador`), nunca en vuelo.
- La corrección de inclinación (y(61:63)) solo se probó en software. No corrige una inclinación del soporte de la cámara respecto al rigid body. Con un modelo de tierra viejo (y(63) = 0) se geolocaliza nivelado: en ese caso, explorar despacio (casi sin inclinarse).
