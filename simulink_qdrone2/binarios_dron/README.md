# Binarios de los drones (`.rt-linux_qdrone2`)

Carpeta para los ejecutables que se lanzan en los QDrone 2 con `quarc_run`. `quarc_run` busca el archivo
**en la carpeta actual**, así que los comandos de la pestaña **Conexiones** se pegan en un `cmd` abierto aquí.

| Archivo | De dónde sale | md5 |
|---|---|---|
| `QD2_DroneStack_PID_2021a.rt-linux_qdrone2` | Binario **original** del Equipo2 (versión PID), sacado tal cual de `Equipo2.zip` (compilado el 20-nov-2026 20:05 con R2022a para `quarc_linux_qdrone2`). No se recompila: es el que ya voló | `00d4d46e7a2e07158690fe98a9b756af` |
| `QD2_DroneStack_MPC.rt-linux_qdrone2` | **No existe todavía.** Se genera en la PC del laboratorio: en MATLAB `cd` a esta carpeta, abre `../versiones/R2022a/QD2_DroneStack_MPC.slx` (o el de tu versión) y haz el build QUARC. QUARC deja el ejecutable en la carpeta actual de MATLAB. Tiene que ser de la **misma construcción** que `QD2_MissionCtrl_Dual_MPC` (el dron solo acepta el índice 24 = 2) | — |

Notas:

- El build del modelo del dron intenta descargarlo al target de su configuración: el Target Manager `tcpip://192.168.2.16:17000` (el `17001` de sus argumentos es la URI **del modelo**, no la del target; con `quarc_run -t` va siempre el `:17000`). Si ese dron no está, el build debería marcar error de conexión **después** de generar el `.rt-linux_qdrone2` (no probado: en la laptop no hay targets de QUARC): revisa que el archivo exista y lánzalo con `quarc_run`.
- Para comprobar el binario PID: `certutil -hashfile QD2_DroneStack_PID_2021a.rt-linux_qdrone2 MD5`.
- Opciones de `quarc_run` (documentación local de QUARC, `%QUARC_DIR%\quarc\doc\quarc_command_line_tools.html`): `-r` descarga y ejecuta, `-D` solo descarga, `-l` solo ejecuta lo ya descargado, `-q` detiene, `-t <URI>` target. La ayuda corta es `quarc_run -?` (no `-h`).
- Los `.rt-*` no se suben a git (van en la copia USB del proyecto).
