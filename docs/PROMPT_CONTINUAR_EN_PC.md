# Prompt para continuar TODO el trabajo desde la PC del laboratorio

Para abrir Claude Code en la PC principal (DESKTOP-CNN8EQJ) y seguir ahí directamente, sin depender de la laptop. Pega el bloque completo. Es más amplio que `PROMPT_CLAUDE_PC_LAB.md`: trae el estado del proyecto, porque la memoria de Claude de la laptop no existe en esta PC.

```text
Vas a continuar, en la PC PRINCIPAL del laboratorio, un proyecto que hasta ahora llevó otra sesión de Claude Code en mi laptop. No tienes su memoria: todo lo que necesitas está aquí y en el repo. Respóndeme en español (México), con pasos numerados cuando yo tenga que hacer algo físico.

== QUIÉN Y DÓNDE ==
- Yo soy el usuario (GitHub: Quique2). Proyecto universitario del Equipo 2: 2 QDrone 2 de Quanser + OptiTrack simulan detección de tomas clandestinas y fuegos sobre una tubería de PVC 3/4" en el piso (arena 3x3 m centrada en (0,0); marco OptiTrack: +x adelante, +y izquierda, +z arriba).
- Esta PC: DESKTOP-CNN8EQJ, 192.168.2.11, Windows, MATLAB R2022a + QUARC (licencia de targets), Motive. Red del laboratorio 192.168.2.0/24 (router 192.168.2.1). Mi laptop está en 192.168.2.21 (allí corre otra sesión de Claude; yo les paso mensajes si hace falta).
- Repo: https://github.com/jose3carlos/QDrone2_Fire_Detection (público), rama Quique. Clónalo en C:\QD2 (ruta corta; QUARC falla con rutas de más de 260 caracteres):
  si no existe: git clone -b Quique https://github.com/jose3carlos/QDrone2_Fire_Detection.git C:\QD2
  si existe: cd C:\QD2 ; git status (si hay cambios locales, enséñamelos y no los borres) ; git checkout Quique ; git pull --ff-only
- Lee primero: docs\LEEME_SISTEMA_DUAL.md (mapa del sistema), docs\ARQUITECTURA.md, docs\CHECKLIST_LABORATORIO.md, docs\GUIA_BUILD_QUARC_LAB.md (§0, §1, §2.1, §3), simulink_qdrone2\lib\README.md, simulink_qdrone2\binarios_dron\README.md, web\README.md, docs\PROMPT_CLAUDE_PC_LAB.md (pasos detallados de hoy).

== QUÉ ESTÁ HECHO (hasta el 22-nov, todo en Quique) ==
- Sistema dual: explorador (z 1.5 m, RealSense D435i mirando hacia abajo en un soporte impreso, YOLO11n en la estación, geolocalización con pose OptiTrack completa incluyendo roll/pitch) + respondedor (z 0.5 m, visita objetivos). Versiones PID y MPC. Lógica de misión en simulink_qdrone2\lib\qd2_mission_step.m (librería QD2_Mission_Lib); modelos de tierra pid\ y mpc\, y exportados en simulink_qdrone2\versiones\R2022a..R2025a. El dron PID corre el binario ORIGINAL del Equipo2 (binarios_dron\QD2_DroneStack_PID_2021a.rt-linux_qdrone2), sin compilar.
- Estación web (web\, Python 3.10+, FastAPI): puente cliente al Stream Server de Simulink en :18700, video del dron en :18800 (streamer web\drone\qdrone2_camera_streamer.py en el Jetson), visión vision\detector.py (ONNX 480x640 vía OpenCV), UI en :8080 con mapa 2D y Vista 3D (three.js desde CDN: necesita internet en el navegador).
- Pruebas en la laptop: pytest web\tests 167, test_qd2_lib 32/32, simulador 3D 18/18, E2E en tiempo real PID y MPC con actitud: todo pasa. NADA se ha compilado nunca con QUARC (la laptop no tiene licencia de targets): este es el primer build.
- Puertos: dron 1 -> 18001, dron 2 -> 18002, puente 18700, video 18800, web 8080. Telemetría: y(55) alertas, y(56:60) objetivos, y(61:63) roll/pitch/válido del explorador, y(64) dron_solo (modo nuevo).

== QUÉ SE ESTÁ HACIENDO HOY (23-nov) ==
- Meta: vuelo REAL de SOLO el dron explorador en PID ("modo solo explorador"). Lo está implementando y probando la sesión de la laptop; se sube a Quique cuando pasen sus pruebas.
- Diseño acordado: config\estacion.json -> "mision": {"modo": "solo_explorador", "dron_solo": 1}. Todo vive en las .m de simulink_qdrone2\lib; NO se regenera ningún .slx: aquí se compila el mismo versiones\R2022a\QD2_MissionCtrl_Dual_PID.slx y toma las .m nuevas. El dron 2 nunca recibe arm/takeoff. Explorador = dron 1 (rigid_body_dron1, puerto 18001). Sin respondedor: los objetivos quedan "detectados". Secuencia: ASSIGN_ROLES -> START -> TAKEOFF a 1.5 m -> HOVER_READY 2 s -> MISSION (recorre la pista) -> RETURN_HOME -> hover -> LAND ALL -> DONE.
- Pista (dibujo del usuario, arriba = +x, escalado a ±1.2 m; luego se mide con "Capturar vértice" de la UI): [[1.02,0.41],[0.99,1.20],[-0.90,1.10],[-0.27,0.29],[0.47,-0.25],[0.99,-1.01],[-1.02,-1.20]], ~35 waypoints, ~7.6 m. Home del dron ≈ (1.25, 0.0) mirando a +x, a 30 cm o más de cualquier tubo.
- Antes de cualquier paso que dependa del modo nuevo: git pull y comprueba que git log --oneline muestra el commit del "modo solo explorador" y que config\estacion.json trae "modo": "solo_explorador". Si no está, para y dímelo (lo sube la laptop). Las comprobaciones de la PC (paso 1) no dependen de eso: hazlas ya.

== DÓNDE CORRE LA ESTACIÓN: dos opciones, pregúntame cuál ==
A) En la laptop (lo planeado): la laptop corre web\iniciar_web.bat --fuente simulink --host-puente 192.168.2.11. Aquí hace falta una regla de firewall de entrada TCP 18700 desde 192.168.2.21 (la creo YO como administrador).
B) Todo en esta PC: si hay Python 3.10+ e internet, crea el venv: python -m venv %USERPROFILE%\.venvs\qd2_web ; %USERPROFILE%\.venvs\qd2_web\Scripts\pip install -r C:\QD2\web\requirements.txt ; y corre la estación con el puente en 127.0.0.1 (sin --host-puente). El modelo de visión ya viene en vision\modelos\mejor.onnx. Corre python -m pytest web\tests -q para comprobar el entorno (en un clon limpio lo esperado es 165 passed + 2 skipped, o lo que diga web\README.md después del pull). No la arranques con el modelo de tierra corriendo sin avisarme.

== TU TRABAJO, EN ORDEN (espera mi OK entre pasos) ==
1. Comprobaciones de solo lectura: version (R2022a), license Simulink Coder / MATLAB Coder, getenv('QUARC_DIR'), qc_get_version, qc_get_target_types (debe incluir win64), which('quarc_win64.tlc'), compilador C (mex -setup C / vswhere); ipconfig; puertos 18001/18002/18700/8080 libres; reglas de firewall de entrada para 18001 y 18700 (revisa cada puerto de cada regla, ver §1.6 de la guía); Python disponible (para la opción B). Para MATLAB sin interfaz usa el truco de simulink_qdrone2\tests\README.md (carpeta con startup.m que hace error('fin') y addpath de QUARC); sin eso, matlab -batch se cuelga con "QUARC Tip of the Day".
2. Motive: rigid body del dron 1 con el ID de optitrack.rigid_body_dron1, creado con el dron en el piso mirando a +x; rastreo estable en toda la arena; z ≈ 0 en el piso. Dime las rutas del .cal/.motive exportados; si config\estacion.json necesita las rutas de ESTA PC, propónme el cambio exacto y solo lo aplicas con mi aprobación. El segundo dron, fuera de la arena.
3. Modelo: abre sin guardar simulink_qdrone2\versiones\R2022a\QD2_MissionCtrl_Dual_PID.slx, Update Diagram: la consola debe decir que qd2_cargar_config está en MODO SOLO EXPLORADOR con el dron 1. Comprueba §0 de la guía (ExtModeMexArgs empieza con comilla y qc_get_model_arguments no da "This statement is incomplete").
4. Yo hago Build (quarc_win64) -> Connect -> Start (Command Source = 0; switches Arm, Takeoff/Land y E-Stop ABAJO). Si falla, te pego el log y me das la causa con la tabla §3 de la guía. Después comprueba que escuchan 18001, 18002 y 18700.
5. Estación (opción A o B). En la UI: "SOLO EXPLORADOR · dron 1", dron 2 "NO USADO", estado ASSIGN_ROLES, sin avisos rojos. No pruebes 18700 con Test-NetConnection con la estación conectada (el Stream Server atiende a un solo cliente).
6. Dron (cuando yo lo prenda): encuentra su IP (ping a 192.168.2.0/24, arp -a; el Equipo2 usó 192.168.2.16) y ponla en la pestaña Conexiones (ip_dron1). Revísame el quarc_run del dron 1 que da Conexiones antes de que yo lo corra en un cmd en C:\QD2\simulink_qdrone2\binarios_dron (binario PID del Equipo2; debe llevar -URI_Host tcpip://192.168.2.11:18001).
7. Cámara: yo copio y arranco el streamer en el Jetson (scp/ssh nvidia@<IP>; la contraseña la tecleo yo): python3 ~/qdrone2_camera_streamer.py --camara auto. Si imprime intrínsecos reales, propónme copiarlos a camara.intrinsecos. En la UI: video ~15 fps, arriba de la imagen = frente del dron (rotacion_montaje 0); pon una pieza impresa debajo y comprueba la detección.
8. Prueba en seco (switches abajo -> START -> el dron NO despega; cancelar con LAND ALL o ABORT) y luego el vuelo (Arm y Takeoff/Land arriba -> START), con observador y mi mano en el switch. Al final, con el dron en el piso: quarc_run -q del dron, Stop de tierra si no se paró solo, y quitar la regla de firewall de 18700 si se creó.

== EMERGENCIAS (recuérdamelas antes del vuelo) ==
Takeoff/Land abajo = aterrizaje ordenado (funciona aunque caiga la estación o la red). UI: ABORT o LAND ALL. Si se cae la estación o la red, la misión pasa a PAUSED (hover) en 1 s. E-Stop corta motores: el dron CAE. Nunca bajar Arm para aterrizar ni dar Stop de tierra con el dron en el aire.

== REGLAS NO NEGOCIABLES ==
- Git: solo clone, checkout de Quique y pull --ff-only. Sin commit, push, reset, stash ni otras ramas, salvo que yo te lo pida; si te pido commit, la identidad es Quique2 <Quique2@users.noreply.github.com> (nunca mi correo personal) y el mensaje termina con: Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
- No edites ni guardes .slx; no muevas ni borres bloques (los del Equipo2 jamás). No corras construir_modelos ni construir_dron_y_sim.
- No edites archivos del repo salvo rutas de Motive / IDs de rigid body / ip_dron1 / intrínsecos en config\estacion.json, y solo con mi aprobación explícita; repórtame el cambio exacto. Conserva los finales de línea de cada archivo.
- Lo que mueve hardware lo hago YO: Build, Deploy, Monitor & Tune, Connect, Start/Stop, quarc_run (cualquier opción), ssh/scp al dron, armar y volar. Tú: solo lectura (qc_get_*, which, get_param, license, version, Update Diagram sin guardar) y preparar/revisar comandos. Hoy no se compila ni se arranca nada del MPC.
- No escribas ni pidas contraseñas en el chat (Windows, GitHub, ssh, licencias).
- No cambies firewall, licencias, red ni configuración de Windows: dame el comando y lo corro yo como administrador.
- No copies fuera de esta PC archivos de QUARC con licencia (.tlc, bibliotecas, .lic, documentación de Quanser); cítalos por nombre.
- No arranques la estación, mock_sim ni el simulador 3D sin avisarme (ocupan 8080/18700). No detengas nada que corra sin que te lo pida. No dejes procesos tuyos vivos al terminar.
- No inventes resultados: distingue lo que corriste de lo que supones.

Al final de cada etapa, repórtame en una tabla corta: qué revisaste, resultado, qué falta. Al terminar el día: commit usado, builds (sí/no, tiempo, archivo .rt-win64), qué se vio en la UI, prueba en seco y vuelo, errores con su mensaje exacto, cambios hechos (archivo y línea) y lo que quedó sin probar.
```
