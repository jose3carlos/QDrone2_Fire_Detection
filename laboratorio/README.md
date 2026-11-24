# Lanzador de la PC del laboratorio

Doble clic, en este orden de uso:

| Archivo | Qué hace |
|---|---|
| `INICIAR_EXPLORADOR.bat` | Modo solo explorador (dron 1, `config/estacion.json`): revisa, arranca el modelo de tierra, abre la estación y la UI, y **pregunta** antes de lanzar el dron y la cámara |
| `INICIAR_DUAL.bat` | Igual en modo dual (`config/estacion_dual.json`, `QD2_MissionCtrl_Dual_PID_2D.slx`). **No volado**: falta el rigid body real del dron 2 y `red.ip_dron2` |
| `DETENER_DRON.bat` | `quarc_run -q` del dron explorador. **Solo con el dron en el piso** |

Lo que hace `iniciar_lab.ps1`, paso a paso:

1. **Revisiones**: QUARC Target Manager escuchando en 17000 (si no: `Restart-Service -Name "QUARC Target Manager" -Force` como administrador), Motive cerrado, otros modelos QUARC corriendo (los lista).
2. **Tierra**: si 18001/18002/18700 no escuchan, abre MATLAB R2022a con `iniciar_tierra_lab('solo'|'dual')`: `cd versiones\R2022a`, abre el modelo, `qd2_cargar_config`, `qc_build_model` y `qc_start_model(mdl, 'external')` (= HARDWARE → Monitor & Tune). Si ya hay un MATLAB abierto, ofrece escribir ese comando en su consola. Espera hasta 10 min a que escuchen los puertos.
3. **Estación**: `web\iniciar_web.bat --fuente simulink` en otra ventana (abre la UI); si ya corre, abre la UI y avisa si está en otro modo.
4. **Dron**: ping a `red.ip_droneN` y, si confirmas, `quarc_run -r …` desde `simulink_qdrone2\binarios_dron`. **Nunca** prueba el 17000 del dron con una conexión TCP: el Target Manager del dron cierra su servicio si alguien conecta y cierra sin hablar QUARC (pasó el 23-nov; se arregla con `sudo systemctl restart quarc_target_manager` en el dron).
5. **Cámara**: si confirmas, abre `ssh nvidia@<IP> "python3 ~/qdrone2_camera_streamer.py --camara auto --puerto 18800"` en otra ventana (la contraseña la tecleas tú).

`iniciar_lab.ps1 -Revisar` solo revisa y muestra lo que haría, sin lanzar nada.

Probado el 23-nov solo con `-Revisar` (modo solo y dual); la secuencia `qc_build_model` + `qc_start_model(mdl, 'external')` de `iniciar_tierra_lab.m` sale de `qc_start_model.html` y aún no se ha corrido desde el script (la que funcionó fue Monitor & Tune desde la pestaña HARDWARE).
