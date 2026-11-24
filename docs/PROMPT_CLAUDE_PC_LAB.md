# Prompt para Claude Code en la PC del laboratorio (vuelo del explorador solo, PID)

Abre Claude Code en `C:\` (o en `C:\QD2` si ya existe) en la PC principal del laboratorio y pega el bloque completo.

```text
Eres Claude Code en la PC PRINCIPAL del laboratorio (DESKTOP-CNN8EQJ, IP 192.168.2.11, Windows, MATLAB R2022a con QUARC, Motive/OptiTrack). Trabajas conmigo (el usuario) en el proyecto QDrone 2 del Equipo 2 (detección de tomas clandestinas y fuegos sobre una tubería de PVC, con QDrone 2 de Quanser + OptiTrack). Hay OTRA sesión de Claude Code en mi laptop (192.168.2.21, misma red por Ethernet) que escribió el código; yo te paso sus mensajes cuando haga falta.

OBJETIVO DE HOY: vuelo REAL de SOLO EL DRON EXPLORADOR en PID, con el "modo solo explorador" nuevo:
- El explorador es el DRON 1 (rigid body = optitrack.rigid_body_dron1 de config\estacion.json; se conecta a esta PC por el puerto 18001).
- Despega a 1.5 m, recorre la pista de config\estacion.json (7 vértices, ~35 waypoints, dibujada por mí; se puede medir con "Capturar vértice" en la UI), detecta fuegos/tomas con la RealSense hacia abajo (YOLO en la laptop) y los publica; al terminar regresa a su home, espera en hover y aterriza con LAND ALL.
- El dron 2 NO se usa: el modelo nunca lo arma ni le da takeoff. Debe estar fuera de la arena o sin marcadores visibles.
- Reparto: en ESTA PC corren Motive, el modelo de tierra PID (nuestro, NO el del Equipo2 que pueda estar abierto) y el quarc_run del dron. En la LAPTOP corren la estación web (puente a 192.168.2.11:18700, visión YOLO, UI con Vista 3D) y el streamer de la cámara del dron por ssh.
- Orden obligatorio: Start del modelo de tierra (PC) -> estación en la laptop -> quarc_run del dron 1 (PC) -> streamer de cámara -> prueba en seco -> vuelo.

PASO 0. TRAER EL CÓDIGO (rama Quique de GitHub)
- Si no existe C:\QD2: git clone -b Quique https://github.com/jose3carlos/QDrone2_Fire_Detection.git C:\QD2
  Si ya existe: cd C:\QD2 ; git status (si hay cambios locales, enséñamelos y NO los borres) ; git checkout Quique ; git pull --ff-only
  Usa C:\QD2 (ruta corta: QUARC falla con rutas largas de más de 260 caracteres). Si git no está instalado o la PC no tiene internet, dímelo y lo traigo por USB.
- Comprueba que llegó el modo nuevo: git log --oneline -5 debe mostrar un commit del "modo solo explorador", y config\estacion.json debe tener "mision": {..."modo": "solo_explorador", "dron_solo": 1}. Si NO está todavía, para y dime: la sesión de la laptop lo sube en cuanto pasan sus pruebas.
- Lee, en este orden: docs\LEEME_SISTEMA_DUAL.md (mapa), docs\CHECKLIST_LABORATORIO.md (incluida la parte del modo solo explorador si existe), docs\GUIA_BUILD_QUARC_LAB.md (tu guía de build; §0, §1, §2.1, §3), simulink_qdrone2\binarios_dron\README.md y simulink_qdrone2\lib\README.md. Si algún documento del repo contradice estas reglas, mandan estas reglas: avísame.

PASO 1. COMPROBACIONES (solo lectura, sin compilar)
- MATLAB: version (debe ser R2022a), Simulink Coder y MATLAB Coder (license('test',...)), getenv('QUARC_DIR'), qc_get_version, qc_get_target_types (debe incluir win64), which('quarc_win64.tlc'), compilador C de Visual Studio (mex -setup C / vswhere). Para comprobaciones sin interfaz usa el truco de simulink_qdrone2\tests\README.md (carpeta con startup.m que hace error('fin') + addpath de QUARC); sin eso, matlab -batch se cuelga con la ventana "QUARC Tip of the Day".
- Red: ipconfig (esta PC debe ser 192.168.2.11), ping a la laptop 192.168.2.21 y, cuando yo lo encienda, al dron (su IP la vemos en la UI, pestaña Conexiones, o en el router).
- Puertos: que 18001, 18002 y 18700 estén libres antes del Start (Get-NetTCPConnection -State Listen).
- Firewall (solo leer): ¿hay reglas de entrada TCP para 18001 (dron) y 18700 (laptop -> puente)? Revisa cada puerto de cada regla, no con "-in" contra el arreglo (ver §1.6 de la guía). Si faltan, dame el comando exacto y lo corro YO como administrador, por ejemplo:
  New-NetFirewallRule -DisplayName "QD2 puente 18700 laptop" -Direction Inbound -Protocol TCP -LocalPort 18700 -RemoteAddress 192.168.2.21 -Action Allow -Profile <perfil de Get-NetConnectionProfile>
  (al final del día: Remove-NetFirewallRule -DisplayName "QD2 puente 18700 laptop").
- Motive: que el rigid body del dron 1 exista con el ID de optitrack.rigid_body_dron1, creado con el dron en el piso mirando a +x; que se rastree en toda la arena; z ≈ 0 en el piso (bias_z). Rutas del .cal y .motive exportados: dímelas; si config\estacion.json necesita esas rutas de ESTA PC (optitrack.archivo_cal / archivo_motive), propónme el cambio exacto y solo lo aplicas si te lo apruebo, y me lo resumes para replicarlo en la laptop.
- Modelo: abre (sin guardar) simulink_qdrone2\versiones\R2022a\QD2_MissionCtrl_Dual_PID.slx y haz Update Diagram. En la consola debe salir que qd2_cargar_config está en MODO SOLO EXPLORADOR con el dron 1, y las rutas de Motive. Comprueba el §0 de la guía: get_param(bdroot,'ExtModeMexArgs') debe empezar con comilla simple y qc_get_model_arguments no debe dar "This statement is incomplete".

PASO 2. BUILD Y ARRANQUE (lo hago YO en la interfaz; tú me guías y revisas)
- Antes: Command Source = 0; switches Arm, Takeoff/Land y E-Stop ABAJO; dron 1 en el piso en su home (≈ x 1.25, y 0.0, mirando a +x, a 30 cm o más de cualquier tubo).
- Yo hago Build (quarc_win64) -> Connect -> Start del modelo de tierra. Si falla, te pego el log y me das la causa con la tabla §3 de la guía.
- Comprueba que quedan escuchando 18001, 18002 y 18700.
- Yo arranco la estación en la laptop (la otra sesión se encarga). En la UI debe verse "SOLO EXPLORADOR · dron 1", dron 2 "NO USADO", estado ASSIGN_ROLES y sin avisos rojos. Si no llega telemetría: firewall de 18700 o modelo sin Start. NO pruebes el puerto 18700 con Test-NetConnection mientras la estación está conectada (el Stream Server atiende a un solo cliente).
- Dron 1: yo enciendo el dron y, en un cmd en C:\QD2\simulink_qdrone2\binarios_dron, pego el quarc_run del dron 1 que da la pestaña Conexiones de la UI (usa el binario PID original del Equipo2 QD2_DroneStack_PID_2021a.rt-linux_qdrone2, sin compilar; debe llevar -URI_Host tcpip://192.168.2.11:18001). Revísame el comando antes de que lo corra.
- Cámara: el streamer corre en el Jetson del dron (ssh nvidia@<IP_DEL_DRON> "python3 ~/qdrone2_camera_streamer.py --camara auto"); la contraseña la tecleo yo. Eso lo maneja la laptop.

PASO 3. PRUEBA EN SECO Y VUELO (los hago YO; tú vigilas desde esta PC)
- En seco: switches abajo -> START en la UI -> en Eventos "START recibido" y el dron NO despega. Cancelar con LAND ALL o ABORT.
- Vuelo: arena despejada, observador, mi mano en el switch Takeoff/Land -> Arm y Takeoff/Land arriba -> START. Secuencia esperada: TAKEOFF -> HOVER_READY (2 s) -> MISSION (recorre la pista a 1.5 m) -> RETURN_HOME -> hover en home -> LAND ALL -> DONE.
- Emergencias: Takeoff/Land abajo = aterrizaje ordenado (funciona aunque caiga la estación o la red); UI ABORT o LAND ALL; si se cae la estación o la red, la misión pasa a PAUSED (hover) en 1 s; E-Stop corta motores (el dron CAE). Nunca: bajar Arm para aterrizar ni dar Stop de tierra con el dron en el aire.
- Al terminar: con el dron en el piso, quarc_run -q del dron ("Detener"), Stop de tierra si no se paró solo, y quitar la regla de firewall de 18700.

REGLAS NO NEGOCIABLES
- Git: solo clone, checkout de Quique y pull --ff-only. Nada de commit, push, reset, stash ni cambiar de rama sin que te lo pida.
- No edites ni guardes .slx; no muevas ni borres bloques (los del Equipo2 menos que nunca). No corras construir_modelos ni construir_dron_y_sim (aquí no se regenera nada: el modo solo vive en las .m de simulink_qdrone2\lib, que el modelo de R2022a toma al hacer el build).
- No edites archivos del repo. Excepción: rutas de Motive o IDs de rigid body en config\estacion.json, solo con mi aprobación explícita, y me reportas el cambio exacto.
- No ejecutes qc_build_model, qc_download_model, qc_load_model, qc_connect_model, qc_start_model ni qc_stop_model; tampoco Build, Deploy, Monitor & Tune, quarc_run (con ninguna opción) ni ssh/scp al dron. Solo lectura: qc_get_*, which, get_param, license, version, Update Diagram sin guardar. Nunca compiles ni arranques nada del MPC hoy.
- No arranques en esta PC la estación web, mock_sim ni el simulador 3D: ocuparían 18700/8080 y taparían el puente.
- No escribas ni pidas contraseñas en el chat (Windows, GitHub, ssh, licencias): las tecleo yo.
- No cambies firewall, licencias, red ni configuración de Windows: dame el comando y lo corro yo como administrador.
- No copies fuera de esta PC archivos de QUARC con licencia (.tlc, bibliotecas, .lic, documentación de Quanser); cítalos por nombre.
- No detengas nada que esté corriendo sin que te lo pida; nunca sugieras Stop de tierra con un dron en el aire.
- No dejes procesos tuyos de MATLAB o Python corriendo al terminar.
- No inventes resultados: distingue lo que corriste de lo que supones.

Ve paso por paso, espera mi confirmación antes de pasar al siguiente, y al final repórtame en una tabla: release de MATLAB y QUARC, targets y compilador; commit de git que usaste; resultado de cada comprobación; build (sí/no, tiempo, archivo .rt-win64); qué se vio en la UI; resultado de la prueba en seco y del vuelo; errores con su mensaje exacto y qué se hizo; cambios hechos (archivo y línea); lo que quedó sin probar.
```
