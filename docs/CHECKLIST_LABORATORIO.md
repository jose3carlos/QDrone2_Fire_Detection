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
| Un dron pierde el rastreo de OptiTrack | La misión pasa a PAUSED (alerta 2). Si no vuelve en unos segundos: **Takeoff/Land abajo** | El dron perdido entra en emergencia a bordo (rampa de bajada) |
| Un dron va a chocar contra una persona o algo | **Emergency Stop** | **Corta los motores de los dos al instante: los drones CAEN** (alerta 5) |
| Último recurso, sin Simulink | `quarc_run -q …` ("Detener" en Conexiones) | Detiene el modelo del dron: **cae** |

**Nunca:**
- Bajar **Arm/Disarm** para aterrizar. En vuelo, a bordo eso es un **Error** (estado 7: rampa sin control de posición; con poco empuje el modelo se detiene y el dron puede caer). Arm abajo solo como emergencia (alerta 7).
- Detener el modelo de tierra (Stop) con un dron en el aire: el dron pierde el enlace, pasa a la rampa y **a 1 s FLIGHT SAFETY corta los motores**.
- Volar con la copia de config de la E2E (`estacion_e2e.json`) o con la barra roja "CÁMARA SINTÉTICA" visible.

---

## Etapa 0. Antes de salir (laptop, día anterior) — 45 min

- [ ] Copiar el proyecto a la USB **sin** lo pesado (en `cmd`):
  `robocopy "%USERPROFILE%\OneDrive\Escritorio\QDrone2_Deteccion_Tomas_Pemex" E:\QD2 /E /XD slprj saved_data runs __pycache__ datos impresion3d /XF *.slxc *.autosave`
  (~80 MB. `datos` excluye `vision/datos` y `web/datos`; **no** excluye `vision/datos_reales`. Los códigos de salida 0 a 7 de robocopy no son error.)
- [ ] En la USB existen: `E:\QD2\vision\modelos\mejor.pt`, `mejor.onnx`, `mejor.onnx.json` y `E:\QD2\simulink_qdrone2\binarios_dron\QD2_DroneStack_PID_2021a.rt-linux_qdrone2` (`certutil -hashfile … MD5` = `00d4d46e7a2e07158690fe98a9b756af`).
- [ ] Copiar también `%USERPROFILE%\Downloads\Equipo2.zip` (respaldo del binario y del modelo original).
- [ ] Laptop: `%USERPROFILE%\.venvs\qdrone2_vision\Scripts\python.exe vision\entrenar.py --help` responde (para el afinado con fotos reales). Llevar el cargador.
- [ ] Cargar **todas** las baterías de los QDrone 2 (4S). A bordo, con ≤ 14.0 V durante 0.3 s se activa el aterrizaje por batería baja, y queda activo.
- [ ] Material: antena Alfa (RTL8811AU), USB, tubería en N, 3 fuegos y 3 tomas impresos, distractores (plantas, carritos, cinta, calcomanías de flama), cinta métrica, cinta para marcar los homes, soporte de la D435i ya montado en el explorador.
- [ ] Pesar los dos drones (el explorador lleva la D435i y el soporte). Anotar: dron 1 ______ kg, dron 2 ______ kg (el PID y el MPC a bordo usan 1.504 kg de trim).

## Etapa 1. Instalar en la PC del laboratorio — 40 min

- [ ] `robocopy E:\QD2 C:\QD2 /E`. **Ruta corta y fuera de OneDrive**: con rutas largas el build QUARC pasa de 260 caracteres y falla.
- [ ] MATLAB: `version` → anotar release: ________ (se espera R2022a). Se usarán los `.slx` de `simulink_qdrone2\versiones\<release>\`.
- [ ] QUARC instalado para ese release (menú QUARC en Simulink; `getenv('QUARC_DIR')` no vacío).
- [ ] Python ≥ 3.10: `py -3 --version`. Si no está, instalarlo (marcar "Add python.exe to PATH").
- [ ] Con internet (Wi-Fi interna): `C:\QD2\web\iniciar_web.bat --fuente mock`. La primera vez crea el venv (~5 min). En `http://127.0.0.1:8080`, START del mock llega a DONE. Cerrar.
- [ ] `cd C:\QD2\web` → `%USERPROFILE%\.venvs\qd2_web\Scripts\python -m pytest -q tests` → **122 passed**.
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

- [ ] Abrir Motive. Calibrar con la varita y fijar el piso con la escuadra en la misma orientación que usó el Equipo2 (+x del marco RSF hacia adelante). Calidad de la calibración: ____________ (Equipo2: "Exceptional", 0.494 mm).
- [ ] Marcadores con **patrón distinto** en cada dron. Crear **dos** rigid bodies con cada dron en el piso **mirando a +x** (así yaw = 0). Anotar los IDs: dron 1 = ____, dron 2 = ____ (distintos).
- [ ] Exportar la calibración (`.cal`) y los rigid bodies (`.motive`, con **los dos** rigid bodies) a `C:\Users\user\Documents\OptiTrack\<fecha>\`.
- [ ] **Cerrar Motive.** El bloque OptiTrack Trackables usa la API de Motive y no puede haber otro proceso con las cámaras.
- [ ] UI → **Motive**: elegir el `.cal` y el `.motive` nuevos (o subirlos) y asignar los rigid bodies al dron 1 y al dron 2. Guardar.
- [ ] `preflight_lab.py`: `optitrack.archivo_cal` y `archivo_motive` "existe", "Motive cerrado".

## Etapa 4. Build del modelo de tierra y pista — 40 min

- [ ] En MATLAB: `cd C:\QD2\simulink_qdrone2\versiones\R2022a` y abrir `QD2_MissionCtrl_Dual_PID.slx`.
- [ ] En la consola deben salir `qd2_cargar_config: calibration_file <- …` y `rigid_body_file <- …` con las rutas del día. Si no salen, o el build falla en ese bloque, escribirlas a mano en OptiTrack Trackables (área 2) y guardar.
- [ ] **Build QUARC** (target `quarc_win64`) → **Connect** → **Start**. Anotar si compiló a la primera: ______. *Esto nunca se ha probado.*
- [ ] `C:\QD2\web\iniciar_web.bat --fuente simulink`: "Puente 50 Hz", "Modelo PID", los dos drones **rastreados**. Los drones están encendidos en el piso, sin lanzar sus binarios.
- [ ] Cada dron en el piso marca **z = 0 ± 0.02 m**: el autoland a bordo exige z ≤ 0.05 m. Si no, corregir `optitrack.bias_z_dronN` en `estacion.json`.
- [ ] Mover cada dron a mano hacia +x y hacia +y: la UI lo sigue en el mismo sentido y el yaw ≈ 0 cuando mira a +x.
- [ ] **Pista:** con la tubería ya fija, poner un dron sobre cada extremo de tubo → **Capturar vértice** → Guardar (máx. 16 vértices, dentro de ±1.5 m).
- [ ] **Stop** del modelo de tierra (ningún dron vuela) → **Build de nuevo**: la pista y el bias se leen **al compilar**. Connect → Start.

**SEGUIR si:** los dos rigid bodies se rastrean sin cortes en toda la arena, z ≈ 0 en el piso y los ejes cuadran. **NO SEGUIR si:** un dron se pierde en alguna zona de la arena, o los ejes o el yaw están invertidos (revisar la orientación con que se creó el rigid body).

## Etapa 5. Cámara del explorador — 25 min (y fotos reales en paralelo)

- [ ] El explorador (el que lleva la D435i hacia abajo) se coloca **siempre en la menor x**.
- [ ] `scp web/drone/qdrone2_camera_streamer.py nvidia@<IP_explorador>:~/`
- [ ] Números de serie: `ssh nvidia@<IP> "rs-enumerate-devices -s"`. Si no existe esa herramienta: `ssh nvidia@<IP> "python3 -c \"import pyrealsense2 as rs; [print(d.get_info(rs.camera_info.name), d.get_info(rs.camera_info.serial_number)) for d in rs.context().devices]\""`. Serie de la D435i de abajo: ____________ (el QDrone 2 trae otra RealSense **frontal**).
- [ ] Lanzar (dejar esa terminal abierta): `ssh nvidia@<IP> "python3 ~/qdrone2_camera_streamer.py --camara auto --serie <serie>"`.
- [ ] Copiar la línea `"intrinsecos": {"640x480": {...}}` que imprime a `camara.intrinsecos` de `C:\QD2\config\estacion.json` y reiniciar la estación.
- [ ] UI: "Video ~15 fps". La imagen es **la de abajo**. Con 0, arriba de la imagen = adelante del dron.
- [ ] Anomalías → Visión dice `vision/detector.py (mejor.pt)`, sin aviso rojo.
- [ ] **Geolocalización:** pon una pieza en una posición conocida (mídela con "Capturar vértice" del otro dron sobre ella) y lleva el explorador a mano a ~1.5 m encima. Error de la posición publicada: ______ m.
  - Si el error parece una rotación (sale a la izquierda en vez de adelante), ajustar `camara.rotacion_montaje` (0/90/180/270).
  - Si es un desplazamiento fijo, ajustar `camara.offset_camara_m`.
  - Estos cambios son solo de la estación: se reinicia la estación, sin recompilar Simulink.

**SEGUIR si:** las cajas salen sobre las piezas y el error es < 0.15 m. **NO SEGUIR con anomalías si:** no detecta tomas o el error es > 0.25 m (el radio de cluster). Se puede volar sin anomalías mientras tanto.

### 5b. Fotos reales y afinado (otra persona, laptop) — ~2 h en paralelo

Detalle: `vision/README.md` §4, "Afinado corto el mismo día de laboratorio".
- [ ] La laptop se conecta a la red de los drones. **La estación de la PC del lab no debe estar conectada al streamer**: atiende un solo cliente.
- [ ] Captura (~30 min, 200–300 fotos): `%PY% vision\capturar_fotos.py --dron <IP_explorador> --cada 0.7 --sesion lab_AAAAMMDD`. Alturas 0.5 / 1.0 / **1.5** / 1.8 m, giros, piezas en bordes y tapadas, 50–100 negativas.
- [ ] Sesión de validación aparte (~60–80 fotos, piezas en otras posiciones): `--sesion lab_val`.
- [ ] Etiquetar (~15–20 min): `etiquetar.py auto --entrada …lab_AAAAMMDD`, `etiquetar.py auto --entrada …lab_val --split val`, `etiquetar.py revisar` (todas).
- [ ] Afinar (~10–30 min): `%PY% vision\entrenar.py --base vision\modelos\mejor.pt --datos vision\datos\sintetico_smoke --epocas 20 --salida vision\modelos\mejor_lab.pt`.
- [ ] Comparar en la validación real: `evaluar.py --modelo vision\modelos\mejor_lab.pt --datos vision\datos_reales` contra lo mismo con `mejor.pt`. mAP50 real: mejor.pt ______ / mejor_lab.pt ______; recall: ______ / ______.
- [ ] **Usar `mejor_lab.pt` solo si mejora las dos cifras.** Llevarlo con `paquete.py exportar --sin_fotos` → `importar` en la PC del lab → `vision.modelo = "vision/modelos/mejor_lab.pt"` a mano en `estacion.json` → reiniciar la estación. Debe decir `vision/detector.py (mejor_lab.pt)`.

## Etapa 6. Prueba en seco (drones desarmados, hélices puestas, nadie en la arena) — 20 min

Orden de arranque, **siempre**: tierra Start → estación → drones.
- [ ] Switches **Arm/Disarm**, **Takeoff/Land** y **Emergency Stop** abajo/en reposo. Command Source = 0 (misión).
- [ ] Abrir un `cmd` en `C:\QD2\simulink_qdrone2\binarios_dron` y pegar los `quarc_run` de **Conexiones** (dron 1 y dron 2). Los comandos usan `-r` (descarga y ejecuta). Ayuda: `quarc_run -?`.
- [ ] Al lanzarlo, el dron hace los pitidos de inicio. No armar antes de ~3 s.
- [ ] UI: los dos drones "comm ok", "Flight State (tierra): 0 · Reposo (Idle)", misión en ASSIGN_ROLES, y(55) = 0.
- [ ] Homes a ≥ 1.05 m (PID). Anotar: ______ m.
- [ ] **START con los switches abajo: no despega.** En Eventos, el comando sale "recibido".
- [ ] (Al final de esta etapa) Probar el E-stop en tierra: la alerta 5 aparece. El E-stop **queda enganchado a bordo**, así que después hay que **detener y relanzar** los dos binarios de los drones ("Detener" en Conexiones) y hacer Stop/Start de tierra.

**SEGUIR si:** todo lo anterior se cumple y no hay alertas. **NO SEGUIR si:** "comm" no está ok, un dron sale en rojo, o START hace algo con los switches abajo.

---

**Antes de CADA vuelo (repetir):**
- [ ] Baterías recién cargadas en los dos.
- [ ] Arena despejada, observador listo, una mano en el switch Takeoff/Land.
- [ ] Tierra: Stop → Start (el Flight Plan se queda en 3/9 después de un vuelo; si cambió `estacion.json`, **Build**).
- [ ] Drones: "Detener" y relanzar con `quarc_run` (después de tierra).
- [ ] Explorador en la menor x, los dos mirando a +x, homes marcados con cinta.

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
  4. **Deshacer el intercambio y hacer Build otra vez.**

**SEGUIR si:** cada dron hace hover estable (deriva < 0.10 m, z ± 0.05 m), aterriza solo y termina en 9 sin alertas. **NO SEGUIR si:** oscila, deriva > 0.20 m, el autoland no termina (revisar `bias_z`), o hay pérdida de enlace o de rastreo.

## Etapa 8. Misión PID con los dos drones — 60 min

8a. **Sin anomalías** (pista sin piezas). Command Source = 0.
- [ ] Switches Arm y Takeoff **arriba** (permiso) → START en la UI → TAKEOFF → HOVER_READY → MISSION.
- [ ] En la UI, Δh entre E y R **siempre ≥ 0.9 m** (`d_min_h`). "GUARD ACTIVO" cuando se acercan.
- [ ] PAUSE (los dos hacen hover) → RESUME.
- [ ] RETURN_HOME → esperar **"en home"** en las dos tarjetas → LAND ALL. Los dos en el suelo a < 0.15 m de su home, DONE, y el modelo de tierra se detiene solo (Stop Allowed).

**SEGUIR si:** DONE sin alertas, Δh mín ≥ 0.9 m, aterrizaje en home. **NO SEGUIR si:** Δh < 0.9 m en algún momento, alerta 2/3/4, o un dron no termina en 9.

8b. **Con anomalías:** primero 1 fuego y 1 toma; después 3 y 3.
- [ ] Cada pieza sale detectada → "ambos" (Simulink la confirmó) → asignada → visitada (R hace hover 3 s encima).
- [ ] Error de posición de cada objetivo < 0.15 m. Falsos positivos sobre distractores: ______.

**SEGUIR si:** las piezas se visitan y no hay objetivos falsos que manden a R a otro lado. **NO SEGUIR si:** hay falsos positivos repetidos (subir `vision.confianza_min` o usar `mejor_lab.pt`), o no detecta tomas.

8c. **Simulacro de aborto en vuelo** (sin E-stop ni Arm):
- [ ] ABORT/RTH desde la UI en MISSION: los dos regresan a su home → LAND ALL.
- [ ] En otro vuelo, **Takeoff/Land abajo** en MISSION: LAND ALL ordenado (alerta 6), R aterriza primero.
- [ ] Guardar después de cada vuelo: pestaña **Eventos**, `web\datos\objetivos_estacion.json` y la carpeta `saved_data` de Simulink.

## Etapa 9. Versión MPC — 90 min

- [ ] UI → Misión → versión **MPC** (`version_control`). `preflight_lab.py --version MPC`.
- [ ] Compilar el dron MPC:
  1. En MATLAB, `cd C:\QD2\simulink_qdrone2\binarios_dron`.
  2. Abrir `..\versiones\R2022a\QD2_DroneStack_MPC.slx` y hacer el build (`quarc_linux_qdrone2`). Puede fallar al **descargar** a 192.168.2.16; da igual si el archivo `QD2_DroneStack_MPC.rt-linux_qdrone2` quedó en `binarios_dron`.
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
- [ ] Copiar a la USB: `C:\QD2\config\estacion.json` como `estacion_lab_<fecha>.json`, la carpeta `saved_data`, Eventos y `web\datos\objetivos_estacion.json`. Fotos reales con `paquete.py exportar`.
- [ ] En la laptop **no** se sobrescribe `config\estacion.json` a ciegas: se compara con el del laboratorio.
- [ ] Anotar lo que falló y en qué etapa: ____________________________________________

---

### Pendientes conocidos (no resueltos antes del laboratorio)
- El build QUARC (tierra `quarc_win64`, dron `quarc_linux_qdrone2`) y la apertura en un R2022a real **nunca se han probado**.
- `quarc_run -r` contra el QDrone 2 real todavía no se ha probado.
- Que el `InitFcn` escriba las rutas de Motive **durante el build QUARC** (sí funciona en Update Diagram).
- MPC real: falsos "enlace perdido", el escape a 1.5 m/s, el downwash real (en la simulación `F0` = 5.9 N es un supuesto), el LAND ALL apilado.
- Un solo binario MPC con un solo "Drone Weight" para los dos drones, aunque el explorador pesa más.
- YOLO sin fotos reales hasta la etapa 5b.
- El modo MANUAL en tierra MPC solo se probó como lógica (`qd2_probar_operador`), nunca en vuelo.
