# Lanzador de la PC del laboratorio

Doble clic, en este orden de uso:

| Archivo | Qué hace |
|---|---|
| `INICIAR_EXPLORADOR.bat` | Modo solo explorador (dron 1, `config/estacion.json`): revisa, arranca el modelo de tierra, abre la estación y la UI, y **pregunta** antes de lanzar el dron y la cámara |
| `INICIAR_DUAL.bat` | Igual en modo dual (`config/estacion_dual.json`, `QD2_MissionCtrl_Dual_PID_2D.slx`). **No volado**: falta el rigid body real del dron 2 y `red.ip_dron2` |
| `DETENER_DRON.bat` | `quarc_run -q` del dron explorador del modo solo (`mision.dron_solo` de `config/estacion.json`). **Solo con el dron en el piso** |
| `DETENER_DUAL.bat` | `quarc_run -q` de los **dos** drones (`config/estacion_dual.json`). Un dron sin `red.ip_dronN` se salta con aviso amarillo: detenlo con **Detener** en la pestaña Conexiones. **Solo con los dos en el piso** |

Lo que hace `iniciar_lab.ps1`, paso a paso:

1. **Revisiones**: QUARC Target Manager escuchando en 17000 (si no: `Restart-Service -Name "QUARC Target Manager" -Force` como administrador), Motive cerrado, otros modelos QUARC corriendo (los lista).
2. **Tierra**: si 18001/18002/18700 no escuchan, abre MATLAB R2022a con `iniciar_tierra_lab('solo'|'dual')`: `cd versiones\R2022a`, abre el modelo, `qd2_cargar_config`, `qc_build_model` y `qc_start_model(mdl, 'external')` (= HARDWARE → Monitor & Tune). Si ya hay un MATLAB abierto, ofrece escribir ese comando en su consola. La sentencia va como **un solo** argumento entre comillas dobles (`-r "cd('…'); iniciar_tierra_lab('…')"`). Espera hasta 10 min a que escuchen los puertos; si no, imprime esa sentencia para pegarla en la consola de MATLAB. Si los puertos ya escuchan, no toca nada, pero avisa si el modelo del Target Manager no es el del modo pedido (`_PID_2D` en dual).
3. **Estación**: `web\iniciar_web.bat --fuente simulink` (en dual con `--config "<ruta de estacion_dual.json>"`) en otra ventana (abre la UI); si ya corre, abre la UI y avisa si está en otro modo.
4. **Dron**: ping a `red.ip_dronN` y, si confirmas, `quarc_run -r …` desde `simulink_qdrone2\binarios_dron`. **Nunca** prueba el 17000 del dron con una conexión TCP: el Target Manager del dron cierra su servicio si alguien conecta y cierra sin hablar QUARC (pasó el 23-nov; se arregla con `sudo systemctl restart quarc_target_manager` en el dron).
5. **Cámara**: en el dron `camara.dron` (de ahí toma el video la estación). Si confirmas, abre en otra ventana `ssh nvidia@<IP> "bash ~/correr_streamer.sh --camara auto --puerto <red.puerto_video>"` (la contraseña la tecleas tú). `correr_streamer.sh` deja el streamer en segundo plano, sobrevive al cierre de ssh y lo relanza si termina (código 3/4 cuando la cámara se cuelga); regresa de inmediato y, si ya corre, solo dice `Ya corre`. Antes, una vez: `scp web/drone/qdrone2_camera_streamer.py web/drone/correr_streamer.sh nvidia@<IP>:~/`. No pasa `--serie`: si el dron tiene más de una RealSense, lánzalo a mano con `--serie <serie>` (Etapa 5 de la checklist). Comprobar: `ssh nvidia@<IP> "bash ~/correr_streamer.sh --estado"`.

`iniciar_lab.ps1 -Revisar` solo revisa y muestra lo que haría, sin lanzar nada.

Probado el 23-nov solo con `-Revisar` (modo solo y dual). El 26-nov se corrigieron el arranque de MATLAB (la sentencia de `-r` llegaba partida en dos y `iniciar_tierra_lab` no corría) y el de la estación en dual (`cmd /k` partía el nombre del `.bat` por las comillas de `--config`); se probaron en la laptop con MATLAB R2025a y un `iniciar_web.bat` falso, en una ruta con espacios. **Pendiente: correr una vez `INICIAR_EXPLORADOR.bat` / `INICIAR_DUAL.bat` sin `-Revisar` en la PC del lab con R2022a.** La secuencia `qc_build_model` + `qc_start_model(mdl, 'external')` de `iniciar_tierra_lab.m` sale de `qc_start_model.html` y aún no se ha corrido desde el script (la que funcionó fue Monitor & Tune desde la pestaña HARDWARE).
