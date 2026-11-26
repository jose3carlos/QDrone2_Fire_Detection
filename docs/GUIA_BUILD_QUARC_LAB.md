# Guía: compilar y correr con QUARC en la PC del laboratorio

Esta guía cubre lo único que **no se pudo hacer en la laptop de desarrollo**: el build QUARC de los modelos de tierra (PID y MPC, target `quarc_win64`) y del modelo a bordo MPC (target `quarc_linux_qdrone2`), y su arranque junto con la estación web. Se sigue dentro de la checklist ([`CHECKLIST_LABORATORIO.md`](CHECKLIST_LABORATORIO.md)): §1 en la etapa 1, §2.1–§2.3 en la etapa 4 y la 6, §2.4–§2.5 en la etapa 9.

**Por qué no se probó antes.** La laptop tiene QUARC Home (QUARC 26.0, 2026) sin los targets. Comprobado el 22-nov-2026 en R2025a: `qc_get_target_types` devuelve una lista vacía, `which('quarc_win64.tlc')` y `which('quarc_linux_qdrone2.tlc')` devuelven `''`, y al abrir los modelos Simulink avisa `System target file 'quarc_win64.tlc' cannot be found`. Por eso aquí solo se hizo Update Diagram y SIL (`simulink_qdrone2/pid/README.md`, `simulink_qdrone2/sim/README.md`).

**Convención.** Lo marcado **[SUPUESTO]** no se puede verificar sin la PC del laboratorio: viene de la documentación de QUARC o de cómo trabajó el Equipo2. Lo demás se leyó de los `.slx` del repositorio o se corrió en la laptop. La documentación de QUARC se cita por nombre de archivo; está en `%QUARC_DIR%\quarc\doc\` (en la laptop, `C:\Program Files\Quanser\QUARC\quarc\doc`).

## Qué se compila

Leído de la configuración activa de cada `.slx` (22-nov-2026):

| Modelo | Archivo | Target (configuración activa) | Toolchain | Produce | Etapa |
|---|---|---|---|---|---|
| Tierra PID | `versiones/<release>/QD2_MissionCtrl_Dual_PID.slx` | `quarc_win64.tlc` (`Configuration1`) | `QUARC win64 \| nmake (64-bit Windows)` | `QD2_MissionCtrl_Dual_PID.rt-win64`, corre en la PC en external mode. Solo explorador: lee `config/estacion.json` | 4 |
| Tierra PID dual | `versiones/R2022a/QD2_MissionCtrl_Dual_PID_2D.slx` | igual | igual | `QD2_MissionCtrl_Dual_PID_2D.rt-win64`. Dual (2 drones): su `InitFcn` lee `config/estacion_dual.json` | 8 |
| Tierra MPC | `versiones/<release>/QD2_MissionCtrl_Dual_MPC.slx` | `quarc_win64.tlc` (`Configuration1`) | igual | `QD2_MissionCtrl_Dual_MPC.rt-win64` | 9 |
| Dron MPC | `versiones/<release>/QD2_DroneStack_MPC.slx` | `quarc_linux_qdrone2.tlc` (`Linux64 Configuration`) | `QUARC linux_qdrone2 \| gmake (64-bit Windows)` | `QD2_DroneStack_MPC.rt-linux_qdrone2` en `binarios_dron` | 9 |
| Dron PID | — | — | — | **No se compila**: es el binario original del Equipo2 (`binarios_dron/QD2_DroneStack_PID_2021a.rt-linux_qdrone2`, md5 `00d4d46e7a2e07158690fe98a9b756af`) | 6 |

Los tres modelos usan multitasking y paso fijo de 1 ms; el dron MPC tiene además una segunda configuración `quarc_linux_x64` que **no** se usa (no la actives).

## 0. Defecto detectado al preparar esta guía: `ExtModeMexArgs` sin comillas (corregido en los `.slx` el 23-nov-2026)

**Qué pasaba.** `qd2_cargar_config.m` armaba los argumentos de external mode de tierra **sin comillas simples**:

```
-w -d %d -uri %u -URI_Host tcpip://localhost:18001 -URI_Host2 tcpip://localhost:18002
```

El `InitFcn` los escribe en el modelo en cada Update y en cada build. QUARC espera el campo como una lista de cadenas de MATLAB entre comillas (`configuration_parameters.html`, sección "MEX-file arguments"). El original del Equipo2 sí las trae: `'-w -d %d -uri %u -URI_Host tcpip://localhost:18001'`.

**Comprobado en la laptop (22-nov, R2025a, modelo cargado sin guardar):** con el modelo PID, `qc_get_model_arguments`, `qc_get_model_uri` y `qc_get_target_uri` fallaban con `Error: This statement is incomplete.` Con el original del Equipo2 esas funciones sí leen los argumentos (y luego fallan solo por `win64 is not a valid target type`, que es la falta de licencia). Con el dron MPC, que conserva las comillas, devuelven `tcpip://192.168.2.16:17001?retries=10` y `tcpip://192.168.2.16:17000?keep_alive=1`. Sin la corrección, el build o el Connect de **los dos modelos de tierra** habrían fallado con ese error **[SUPUESTO]**.

**Corrección (22-nov-2026, en el repositorio).** `qd2_cargar_config.m` ahora pone las comillas dentro de la cadena, igual que el original:
```matlab
cfg.ext_mode_args = sprintf('''-w -d %%d -uri %%u -URI_Host %s -URI_Host2 %s''', ...
    cfg.uri_dron1, cfg.uri_dron2);
```
- Comprobado en la laptop: después de `qd2_cargar_config(mdl)` (lo que hace el `InitFcn`) sobre los modelos de tierra PID y MPC, MATLAB lee el campo como **un solo** argumento y `qc_get_model_arguments` / `qc_get_target_uri` ya no dan `This statement is incomplete.` (solo `win64 is not a valid target type`, por la licencia). La prueba `test_ext_mode_args` de `simulink_qdrone2/tests/test_qd2_lib.m` lo vigila. El build no se pudo probar.
- ~~Los `.slx` guardados todavía traen el valor viejo: no importa, porque el `InitFcn` lo reescribe.~~ **Esto era falso** (PC del laboratorio, 23-nov-2026, [`HALLAZGOS_PC_LAB_23NOV.md`](HALLAZGOS_PC_LAB_23NOV.md) fila 1): **el `InitFcn` no basta.** QUARC lee el `ExtModeMexArgs` **guardado** al **abrir** el modelo y al **empezar** el build, antes del `InitFcn`; con el valor sin comillas sale `Error: This statement is incomplete.` al abrir, el Build falla igual y **no aparece la pestaña HARDWARE**. El `PostLoadFcn` llama a `qd2_cargar_config()` sin modelo y no lo corrige.
- **Corrección en los `.slx` (23-nov-2026, laptop).** En la PC del laboratorio se guardaron en R2022a, con el valor entre comillas, `versiones/R2022a/QD2_MissionCtrl_Dual_PID.slx` y `_PID_2D.slx` (compilaron y corrieron con QUARC; no se tocaron después). En la laptop se corrigieron los otros 11 modelos de tierra que aún lo traían sin comillas en la configuración activa (`Configuration1`): `pid/`, `mpc/`, `versiones/R2022a/…_MPC.slx` y `versiones/{R2022b,R2023a,R2023b,R2025a}/…_{PID,MPC}.slx`. Método: se editó **solo** ese valor dentro de `simulink/configSet1.xml` del `.slx` (es un zip), con el mismo escapado que el R2022a bueno (`&apos;-w -d %d … -URI_Host2 tcpip://localhost:18002&apos;`); las demás 83–84 partes del zip quedaron con los mismos bytes. No se regeneró nada (tierra MPC y dron MPC siguen siendo de la misma construcción) ni se abrió/guardó con Simulink (en la laptop no hay targets QUARC y guardar sin ellos podía alterar la configuración). Verificado en R2025a, modelo cargado sin guardar: `get_param(mdl, 'ExtModeMexArgs')` empieza con comilla, `qc_get_model_arguments` / `qc_get_target_uri` ya no dan `This statement is incomplete.` (solo `win64 is not a valid target type`, por la licencia), mismo número de bloques y mismos callbacks que antes.
- **Para que no vuelva a pasar.** `construir_modelos` rechaza un `QD2CFG.ext_mode_args` sin comillas y, al terminar, lee el valor guardado en cada `.slx` de tierra (`pid/`, `mpc/`, `versiones/*/`) con `qd2_ext_mode_args_slx` y falla si alguno no va entre comillas. La prueba `test_ext_mode_args_slx` de `test_qd2_lib.m` revisa lo mismo en todos los `.slx` de tierra del repositorio. El `PostLoadFcn` **no** se cambió a `qd2_cargar_config(bdroot)`: no se puede comprobar sin targets que corra antes de que QUARC lea el campo, dejaría el modelo modificado al abrirlo (escribe también las rutas de Motive del JSON) y obligaría a cambiar los dos `.slx` de R2022a ya validados.
- **Qué hacer en el laboratorio:** la comprobación de §1.7. `get_param(mdl, 'ExtModeMexArgs')` ya debe empezar con comilla simple **al abrir**, antes del Update. Si no, la copia de los `.slx` es vieja (anterior al 23-nov): trae los `.slx` actuales de la rama y anótalo (en un apuro, `qd2_cargar_config(mdl)` + `save_system(mdl)` en la copia de la PC, como se hizo el 23-nov).

**Relacionado.** La fila "Alternativa desde MATLAB (External mode)" de la pestaña **Conexiones** de la UI (`web/app/netinfo.py`) tenía el mismo problema de comillas y le faltaba la URI del modelo. Ya genera el mismo formato que el modelo del dron del Equipo2 (`'-w -d /tmp -uri %u -URI_Host …','tcpip://<IP_dron>:17001?retries=10'`, con las comillas dobladas dentro del `set_param`). Aun así, **no la uses**: los drones se lanzan con `quarc_run` (§2.3).

## 1. Requisitos y comprobaciones previas (etapa 1) — 20 min

### 1.1 Copia del proyecto
- Ruta corta, fuera de OneDrive: `C:\QD2`. Con rutas largas el build QUARC pasa de 260 caracteres y falla. `web\preflight_lab.py` avisa si la ruta es larga.
- Comprueba que venga lo necesario para compilar: `simulink_qdrone2\versiones\<release>\` (los 5 `.slx`), `simulink_qdrone2\lib\` (`qd2_*.m`), `simulink_qdrone2\equipo2_original\Motor_Mapping_7_Inch.m` (lo usa el `InitFcn` del dron MPC), `simulink_qdrone2\qd2_cargar_config.m` y `config\estacion.json`.

### 1.2 MATLAB y versión de los modelos
```matlab
version                                % anota el release
license('test', 'Real-Time_Workshop')  % Simulink Coder: debe dar 1
license('test', 'MATLAB_Coder')        % MATLAB Coder: debe dar 1
```
QUARC necesita Simulink, Simulink Coder y MATLAB Coder (`quarc_requirements.html`).

| Release de la PC | Carpeta a abrir |
|---|---|
| R2022a, R2022b, R2023a, R2023b | `simulink_qdrone2\versiones\<release>\` |
| R2024a / R2024b | `versiones\R2023b\` **[SUPUESTO]**: un release más nuevo abre archivos de uno anterior; no se probó |
| R2025a o posterior | `versiones\R2025a\` |
| Anterior a R2022a | No hay exportación. Hay que exportar desde la laptop (`construir_modelos` solo exporta a R2022a–R2023b y R2025a) |

Se espera **R2022a** **[SUPUESTO]**: el binario del Equipo2 se compiló el 20-nov-2026 con R2022a para `quarc_linux_qdrone2` (`binarios_dron/README.md`), así que esa PC ya compiló para el QDrone 2.

**Siempre** abre los tres modelos de la **misma** carpeta: el dron MPC solo acepta tramas con el índice 24 = 2 de su tierra MPC (`sim/README.md` §1).

### 1.3 QUARC y licencia de los targets
```matlab
getenv('QUARC_DIR')                   % no vacío
v = qc_get_version                    % anota major.minor (en la laptop: 26.0)
t = qc_get_target_types               % debe incluir 'win64' y 'linux_qdrone2'
which('quarc_win64.tlc')              % ruta no vacía
which('quarc_linux_qdrone2.tlc')      % ruta no vacía
```
- `win64` es el nombre del tipo de target (es el que aparece en el error de la laptop). `linux_qdrone2` sale de la extensión `.rt-linux_qdrone2` (`quarc_command_line_tools.html`: los ejecutables se llaman `.rt-{target_type}`).
- En la laptop, `t` sale vacío y los dos `which` salen `''`. Si en la PC del lab sale igual, **esa licencia no compila**: revisa la licencia con `quanser_license_viewer.exe` (en `%QUARC_DIR%`) o con "Configure Licensing" (`quarc_configuration.html`) y avisa al responsable del laboratorio. No sigas con las etapas de vuelo.
- La versión de QUARC tiene que soportar tu release de MATLAB: revisa `QUARC_Compatibility_Table.html` **de esa PC**. La de la laptop (QUARC 2026) solo lista R2025a, R2025b y R2026a, así que el QUARC del laboratorio debe ser otro, más viejo **[SUPUESTO]**.

### 1.4 Compiladores
- **`quarc_win64` (tierra):** la toolchain `nmake` necesita Microsoft Visual Studio con C++ ("Desarrollo para el escritorio con C++"). La tabla de QUARC 2026 pide Visual Studio 2019 o 2022; para el QUARC del laboratorio manda su propia `QUARC_Compatibility_Table.html`.
  ```matlab
  mex -setup C                          % debe listar "Microsoft Visual C++ 20xx"
  qc_get_compilers('win64')             % compiladores que ve QUARC para ese target
  ```
  ```powershell
  & "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe" -all -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property displayName
  ```
  El modelo de tierra original del Equipo2 ya usaba `quarc_win64` con esta toolchain, así que si ellos compilaron en esa PC, el compilador ya está **[SUPUESTO]**.
- **`quarc_linux_qdrone2` (dron):** el compilador cruzado lo instala QUARC ("Cross-compiler supplied" en la tabla de compatibilidad; `quarc_linux_qdrone2_target.html` pide el "cross-compiler suite for NVIDIA targets"). Comprueba que `qc_get_compilers('linux_qdrone2')` no salga vacío **[SUPUESTO]**: no se pudo ver la salida en la laptop.

### 1.5 Red y acceso a los drones (después de la etapa 2)
```powershell
ping <IP_dron>
Test-NetConnection <IP_dron> -Port 17000      # QUARC Target Manager del dron: TcpTestSucceeded = True
ssh nvidia@<IP_dron> "echo ok"                # la contraseña la teclea el usuario
```
- El Target Manager escucha en TCP 17000 (`quarc_configuration.html`, "Setting up Remote Access"; `quarc_default_uris.html`). Ahí descarga `quarc_run`; el modelo usa 17001.
- Opcional: en MATLAB, `qc_find_targets` lista los targets QUARC de la red con su tipo y versión. Tarda 30 s o más (`qc_find_targets.html`).

### 1.6 Firewall de la PC
Los drones se conectan **como clientes** a los Stream Servers del modelo de tierra: `tcpip://<IP_PC>:18001` (dron 1) y `:18002` (dron 2). El puente con Python (`:18700`) y la UI (`:8080`) son locales. `stream_server_block.html` ("Helpful Hints → Firewall") pide permitir la entrada en el puerto del servidor.
```powershell
# ¿Hay reglas de entrada que dejen pasar 18001/18002? (puede pedir PowerShell como administrador:
# sin elevar, Get-NetFirewallPortFilter puede responder "Access is denied")
Get-NetFirewallRule -Direction Inbound -Enabled True | Where-Object {
    $pf = $_ | Get-NetFirewallPortFilter
    ($pf.Protocol -eq 'TCP' -or $pf.Protocol -eq 'Any') -and
    (@($pf.LocalPort) | Where-Object { $_ -in '18001','18002','Any' -or $_ -match '^\d+-\d+$' })
} | Format-Table DisplayName, Action, Profile, @{n='Puertos'; e={($_ | Get-NetFirewallPortFilter).LocalPort -join ','}}
Get-NetConnectionProfile                       # perfil (Public/Private) de la red de la antena Alfa
```
- El filtro revisa **cada** puerto de la regla: una regla con varios puertos (`18001,18002`) o con un rango (`18000-18010`) también sale. Un rango que sale hay que mirarlo a ojo (que incluya 18001 y 18002). Una regla por programa ("Permitir una aplicación", `LocalPort` = `Any`) también vale si es del programa que abre los Stream Servers.
- La prueba definitiva no es esta lista sino la de §2.1–§2.3: con tierra en Start, cada dron llega a "comm ok" en la UI.
- El `:18002` es **nuevo** (el Equipo2 solo usaba un dron en `:18001`). Si no hay regla, la crea el usuario o el responsable del laboratorio, en PowerShell como administrador, limitada a la red local y al perfil que mostró `Get-NetConnectionProfile` (aquí `Private`; cámbialo si sale otro):
  ```powershell
  New-NetFirewallRule -DisplayName "QD2 Stream Servers 18001-18002" -Direction Inbound -Protocol TCP -LocalPort 18001,18002 -RemoteAddress LocalSubnet -Action Allow -Profile Private
  ```
  Si se conocen las IPs de los drones, mejor `-RemoteAddress <IP_dron1>,<IP_dron2>`. La PC es compartida: si el responsable lo pide, se borra al final del día con `Remove-NetFirewallRule -DisplayName "QD2 Stream Servers 18001-18002"`.
- **[SUPUESTO]** Que exista ya una regla para `:18001` (el Equipo2 voló con esa PC) o que el firewall de la PC del laboratorio permita la entrada.
- **Estación en la laptop (puente remoto, `--host-puente <IP_PC>`).** Si la estación web corre en otra computadora de la red (por ejemplo la laptop, con la UI 3D), se conecta como cliente al Stream Server del PYTHON BRIDGE en `tcpip://<IP_PC>:18700`. QUARC ignora el hostname de un URI que escucha (`tcpip_protocol.html`: el hostname "is ignored when establishing a listening connection"), así que `tcpip://localhost:18700` ya acepta clientes de la LAN; solo falta la regla de entrada, limitada a la IP de la laptop:
  ```powershell
  New-NetFirewallRule -DisplayName "QD2 puente 18700 laptop" -Direction Inbound -Protocol TCP -LocalPort 18700 -RemoteAddress <IP_laptop> -Action Allow -Profile <perfil de Get-NetConnectionProfile>
  # al terminar el día:
  Remove-NetFirewallRule -DisplayName "QD2 puente 18700 laptop"
  ```
  El Stream Server atiende **un solo cliente**: no probar el puerto con `Test-NetConnection` mientras la estación esté conectada. Si se cae la red, la misión pasa a PAUSED (hover) en 1 s y el switch Takeoff/Land sigue funcionando.

### 1.7 Revisar los modelos sin compilar (Motive cerrado, `estacion.json` del día)
```matlab
cd C:\QD2\simulink_qdrone2\versiones\R2022a            % la carpeta de tu release
open_system('QD2_MissionCtrl_Dual_PID.slx')
mdl = bdroot;
which('QD2_Mission_Lib')                % debe estar en ...\versiones\R2022a\
get_param(mdl, 'SystemTargetFile')      % quarc_win64.tlc
set_param(mdl, 'SimulationCommand', 'update')   % Update Diagram (Ctrl+D): corre el InitFcn
get_param(mdl, 'ExtModeMexArgs')        % debe EMPEZAR con comilla simple (§0)
qc_get_model_arguments(mdl)             % sin error; con -URI_Host ...:18001 y -URI_Host2 ...:18002
qc_get_target_uri(mdl)                  % sin error
```
- En la consola deben salir `qd2_cargar_config: calibration_file <- …` y `rigid_body_file <- …` con las rutas del día (si cambiaron respecto al modelo).
- Repite con `QD2_MissionCtrl_Dual_MPC.slx`.
- Para el dron MPC: `get_param(bdroot, 'SystemTargetFile')` debe dar `quarc_linux_qdrone2.tlc`, y `qc_get_target_uri(bdroot)` debe dar `tcpip://192.168.2.16:17000?keep_alive=1` (es lo que dio en la laptop).

## 2. Paso a paso

**Orden de arranque, siempre:** tierra (build → Connect → Start) → estación web → drones con `quarc_run`. El dron se detiene solo si a ~1.2 s de lanzarlo no recibe `isTracking = 1` de tierra. **Nunca** detengas tierra con un dron en el aire (ver la checklist, "Abortos de emergencia").

### 2.1 Modelo de tierra PID (etapa 4)
Motive cerrado. Drones encendidos en el piso, **sin** lanzar sus binarios. Switches Arm/Disarm, Takeoff/Land y Emergency Stop abajo.

**Desde la pestaña QUARC del modelo** (`quarc_tab.html`): **Monitor & Tune** hace build, descarga, Connect y Start en un solo paso. Para hacerlo por partes: Monitor & Tune ▾ → **Build for Monitoring**, luego **Deploy**, **Connect** y **Start**.

**O desde la consola de MATLAB:**
```matlab
cd C:\QD2\simulink_qdrone2\versiones\R2022a
open_system('QD2_MissionCtrl_Dual_PID.slx'); mdl = bdroot;
qc_build_model(mdl)                     % compila; el código se descarga solo (qc_build_model.html)
qc_connect_model(mdl)                   % external mode (qc_connect_model.html)
qc_start_model(mdl, 'external')         % qc_start_model.html
```

**Qué debe verse:**
- Mensajes de `qd2_cargar_config` (rutas de Motive) al principio del build.
- El log del build termina sin error y aparece `QD2_MissionCtrl_Dual_PID.rt-win64` en la carpeta actual.
- Después de Start, el tiempo de simulación avanza en la barra de estado de Simulink. Los Stream Servers escuchan:
  ```powershell
  Get-NetTCPConnection -State Listen -LocalPort 18001,18002,18700 | Format-Table LocalAddress, LocalPort, OwningProcess
  ```
  **[SUPUESTO]** Que los tres puertos aparezcan en cuanto arranca el modelo.
- La primera compilación tarda varios minutos **[SUPUESTO]**. Anota el tiempo: ______ min, y si compiló a la primera.

**Dual (etapa 8):** lo mismo con `QD2_MissionCtrl_Dual_PID_2D.slx` (o `laboratorio\INICIAR_DUAL.bat`), que lee `config/estacion_dual.json`; antes, Stop del modelo de solo explorador y copiar a `estacion_dual.json` lo medido en el día (checklist, Etapa 8-0). Debe aparecer `QD2_MissionCtrl_Dual_PID_2D.rt-win64` y la consola decir `modo dual (2 drones)`.

Después de **cualquier** cambio de la config que lea Simulink (`estacion.json`, o `estacion_dual.json` con `_PID_2D`) (Motive, rigid bodies, `bias_z`, pista, `mision.*`, puertos): Stop (`qc_stop_model(mdl)` o el botón Stop) → build de nuevo → Connect → Start. Conectar al binario anterior no relee el JSON (`pid/README.md`, "Abrir en el laboratorio").

### 2.2 Estación web
```bat
C:\QD2\web\iniciar_web.bat --fuente simulink
:: dual (_PID_2D):
C:\QD2\web\iniciar_web.bat --fuente simulink --config C:\QD2\config\estacion_dual.json
```
La estación tiene que usar la misma config que el modelo: si el modo no coincide con el del modelo, START sale rechazado.
Antes, comprueba que nada ocupe 8080 ni 18700 (por ejemplo, un simulador 3D o un mock abiertos): `Get-NetTCPConnection -State Listen -LocalPort 8080,18700`. 18700 debe salir **solo** con el proceso del modelo de tierra.
En `http://127.0.0.1:8080` deben salir "Puente 50 Hz", "Modelo PID" (igual a `version_control`) y los dos drones **rastreados**. Sigue la etapa 4 de la checklist (z ≈ 0, ejes, actitud, pista).

### 2.3 Drones PID (etapa 6)
En un `cmd` **abierto en** `C:\QD2\simulink_qdrone2\binarios_dron` (`quarc_run` busca el archivo en la carpeta actual):
```bat
certutil -hashfile QD2_DroneStack_PID_2021a.rt-linux_qdrone2 MD5
:: debe dar 00d4d46e7a2e07158690fe98a9b756af
```
Pega los comandos de **Conexiones** (dron 1 y dron 2). Tienen esta forma (generada por `web/app/netinfo.py`):
```bat
quarc_run -r -t tcpip://<IP_dron1>:17000 QD2_DroneStack_PID_2021a.rt-linux_qdrone2 -uri tcpip://<IP_dron1>:17001?retries=10 -URI_Host tcpip://<IP_PC>:18001
quarc_run -r -t tcpip://<IP_dron2>:17000 QD2_DroneStack_PID_2021a.rt-linux_qdrone2 -uri tcpip://<IP_dron2>:17001?retries=10 -URI_Host tcpip://<IP_PC>:18002
:: detener ("Detener" en Conexiones):
quarc_run -q -t tcpip://<IP_dronN>:17000 QD2_DroneStack_PID_2021a.rt-linux_qdrone2
```
- Opciones de `quarc_run` (`quarc_command_line_tools.html`):
  - `-r` descarga y ejecuta.
  - `-D` solo descarga.
  - `-l` solo ejecuta lo ya descargado.
  - `-q` detiene.
  - `-t <URI>` es el Target Manager.
  - `-r` y `-D` son alternativas excluyentes: no se combinan.
  - La ayuda corta es `quarc_run -?`.
  - No se pasa `-w` a `quarc_run`: haría que el modelo esperara a Simulink.
- **Qué debe verse:** los pitidos de inicio del dron. En la UI, "comm ok" y "Flight State (tierra): 0 · Reposo (Idle)" en los dos, sin alertas.

### 2.4 Modelo de tierra MPC (etapa 9)
Igual que §2.1, con `QD2_MissionCtrl_Dual_MPC.slx` de la **misma carpeta** que el dron MPC. Antes, en la UI → Misión → versión **MPC** (`version_control`), para que Conexiones genere los comandos del binario MPC. La estación debe decir "Modelo MPC", sin aviso amarillo.

### 2.5 Modelo a bordo MPC (`quarc_linux_qdrone2`, etapa 9)
Se compila **antes** de arrancar tierra MPC (o con tierra detenida): no hace falta ningún dron en el aire.
```matlab
cd C:\QD2\simulink_qdrone2\binarios_dron           % QUARC deja el ejecutable en la carpeta actual
open_system('C:\QD2\simulink_qdrone2\versiones\R2022a\QD2_DroneStack_MPC.slx')
get_param(bdroot, 'SystemTargetFile')              % quarc_linux_qdrone2.tlc
```
- **Antes de compilar:** los dos drones en el piso y sus binarios PID de la etapa 8 detenidos ("Detener" en Conexiones).
- **Opción A (recomendada):** pestaña QUARC → **solo la flecha ▾** de Monitor & Tune → **Build for Monitoring**. Según `quarc_tab.html`, compila y deja el ejecutable en la PC, **sin descargarlo**. **[SUPUESTO]** Que la versión de QUARC del laboratorio tenga la misma pestaña; si no, usa la opción B.
  - **NO pulses el botón grande Monitor & Tune ni Deploy en este modelo.** En tierra (§2.1, §2.4) sí se usa, pero aquí es peligroso: según `quarc_tab.html`, Monitor & Tune hace "build, download, connect, and start", y Deploy descarga y deja el modelo arrancado esperando a Simulink. Este modelo tiene `SimulationMode = external` y sus argumentos apuntan al dron `192.168.2.16` con la IP de PC del Equipo2 (`-URI_Host tcpip://192.168.2.11:18001`): un clic equivocado lanzaría el stack de vuelo en ese dron desde Simulink.
- **Opción B:** `qc_build_model(bdroot)`. Según `qc_build_model.html`, compila y **descarga** el código al target de la configuración (`tcpip://192.168.2.16:17000`); no dice que lo arranque.
  - Si ese dron no está, marca un error de conexión después de generar el archivo **[SUPUESTO]**: lo dice `binarios_dron/README.md`, que tampoco se pudo probar.
  - Si sí está, el ejecutable queda descargado en el dron. Antes del `quarc_run -r` puedes detener cualquier modelo que haya quedado cargado: `quarc_run -q -Q -t tcpip://192.168.2.16:17000 QD2_DroneStack_MPC.rt-linux_qdrone2`. Es opcional; el `-Q` evita el diálogo de error que, según `quarc_command_line_tools.html`, sale cuando no hay ningún modelo corriendo. **Solo con ese dron en el piso y desarmado.**
- **No** hagas Connect ni Start de este modelo desde Simulink: los drones se lanzan con `quarc_run` y los maneja tierra.
- **Comprueba el resultado:**
  ```bat
  cd C:\QD2\simulink_qdrone2\binarios_dron
  dir QD2_DroneStack_MPC.rt-linux_qdrone2
  certutil -hashfile QD2_DroneStack_MPC.rt-linux_qdrone2 MD5
  ```
  El archivo debe tener la fecha y hora de hoy. Anota el md5 ______ para saber después qué binario voló.
- **Lanzar:** tierra MPC (§2.4) Build → Connect → Start → estación → los `quarc_run` de **Conexiones** (ahora con `QD2_DroneStack_MPC.rt-linux_qdrone2`).
- **Qué debe verse:** en las dos tarjetas, "FSM a bordo: 1 · IDLE", **sin alerta 10** y con la batería visible.
  - Alerta 10 = protocolo incorrecto: binario PID con tierra MPC, o dron y tierra de construcciones distintas.

### 2.6 Parar al final
Con **los dos drones en el piso**: "Detener" de cada dron en Conexiones (`quarc_run -q …`) → Stop de tierra (`qc_stop_model(mdl)`) → cerrar la estación (Ctrl+C en su ventana).

## 3. Errores típicos

| Síntoma | Causa probable | Qué hacer |
|---|---|---|
| Al abrir: `System target file 'quarc_win64.tlc' cannot be found` (o `quarc_linux_qdrone2.tlc`); `qc_get_target_types` vacío; `win64 is not a valid target type` | La licencia de QUARC no trae ese target (es lo que pasa en la laptop) | §1.3: revisar la licencia con el responsable. Sin target no hay build |
| `This statement is incomplete.` al compilar, al conectar o en `qc_get_model_arguments` | `ExtModeMexArgs` de tierra sin comillas | §0 |
| `Toolchain … not found`, no encuentra `nmake`/`cl.exe` o "no supported compiler" **[SUPUESTO: texto exacto]** | Falta Visual Studio con C++ o su versión no la soporta ese QUARC | §1.4. Si en esa PC la toolchain se llama distinto, elígela en Model Settings → Code Generation → Toolchain y anótalo |
| Error de ruta demasiado larga o archivos que "no existen" a mitad del build | Ruta > 260 caracteres | Copiar a `C:\QD2` (§1.1) |
| `… was saved in a newer version …` / no abre el `.slx` | `.slx` de otro release | Abrir el de `versiones\<release>\` (§1.2) |
| Bloques de la librería sin resolver / "bad link" | MATLAB tomó `QD2_Mission_Lib.slx` de otra carpeta (la de `lib\` es de R2025a) | Reiniciar MATLAB, `cd` a `versiones\<release>` **antes** de abrir y revisar `which('QD2_Mission_Lib')` |
| No salen los mensajes `calibration_file <- …` / falla en OptiTrack Trackables | El `InitFcn` no pudo escribir las rutas de Motive durante el build (nunca probado con QUARC) | Escribir el `.cal` y el `.motive` a mano en OptiTrack Trackables (área 2), guardar y compilar otra vez. Revisar que los archivos existan y que Motive esté **cerrado** |
| `qd2_cargar_config:rb` | Rigid bodies iguales en `estacion.json` | Asignar IDs distintos en la pestaña Motive y recompilar |
| `qd2_cargar_config:sin_json` | No encuentra `config\estacion.json` | Abrir los modelos desde `C:\QD2\...` (busca hacia arriba) |
| Error de descarga a `192.168.2.16` al compilar el dron MPC | El build intenta descargar a ese target | Normal si el `.rt-linux_qdrone2` quedó en `binarios_dron` (§2.5) |
| Tierra corre, pero un dron nunca llega a "comm ok" | Firewall de la PC (sobre todo `:18002`), IP de la PC equivocada en Conexiones (debe ser la de la Alfa) u orden de arranque | §1.6; revisar `ipconfig` y el comando; tierra primero |
| El dron se detiene solo ~1 s después de lanzarlo | Tierra no estaba en Start (sin `isTracking = 1`) | Start de tierra → relanzar el dron |
| `quarc_run` no encuentra el ejecutable | El `cmd` no está en `binarios_dron` | `cd C:\QD2\simulink_qdrone2\binarios_dron` |
| `quarc_run -r` falla porque el modelo ya está cargado **[SUPUESTO]** | Quedó de un lanzamiento anterior o de la descarga del build (§2.5, opción B) | **Solo con ese dron en el piso y desarmado** (`quarc_run -q` detiene el modelo: si el dron está en el aire, **cae**; primero LAND ALL o aterrizaje, checklist "Abortos de emergencia"): `quarc_run -q -Q -t … <modelo>` y otra vez `-r`. No uses `-r` y `-D` juntos |
| Alerta 10 con MPC | Binario PID con tierra MPC, o tierra y dron de carpetas o builds distintos | Recompilar los dos desde la misma carpeta; revisar `version_control` |
| MATLAB se queda colgado en `matlab -batch` | La ventana modal "QUARC Tip of the Day" | Compilar desde la interfaz de MATLAB. Para comprobaciones sin interfaz: `matlab -sd <carpeta> -batch "…"` con un `startup.m` que solo haga `error('fin')`, y antes `addpath(fullfile(getenv('QUARC_DIR'), 'quarc'))` (`simulink_qdrone2/tests/README.md`). **[SUPUESTO]** Que un build funcione así, sin el arranque de QUARC: no se probó |
| 8080 o 18700 ocupados | Un simulador 3D, un mock o una estación anterior | Cerrarlos (`Get-NetTCPConnection -State Listen -LocalPort 8080,18700` → `OwningProcess`) |

## 4. Si usas Claude Code en la PC del laboratorio

Claude Code puede revisar requisitos, leer logs y preparar comandos. Los builds, el Start de tierra y los `quarc_run` los hace **el usuario** (§2), porque mueven hardware real. Abre Claude Code en `C:\QD2` y pega esto tal cual:

```text
Contexto: estoy en la PC del laboratorio con el proyecto QDrone 2 dual (2 QDrone 2 + OptiTrack; explorador a 1.5 m con cámara hacia abajo y YOLO, respondedor a 0.5 m; versiones PID y MPC). El proyecto está en C:\QD2 (copia de la rama Quique). Lee primero docs\GUIA_BUILD_QUARC_LAB.md (es tu guía), después docs\CHECKLIST_LABORATORIO.md (etapas 1, 4, 6 y 9) y simulink_qdrone2\binarios_dron\README.md. Nada de esto se ha compilado nunca con QUARC: en la laptop de desarrollo no había targets. Los builds, el Start de tierra y los quarc_run los hago YO en la interfaz, porque mueven hardware real; tú solo lees, revisas y preparas comandos. Si la checklist pide algo que estas reglas prohíben (por ejemplo, regenerar con construir_modelos), mandan estas reglas: avísame y no lo hagas.

Qué hay que compilar, en este orden (todos de la MISMA carpeta simulink_qdrone2\versiones\<release de esta PC>):
1. Tierra PID: QD2_MissionCtrl_Dual_PID.slx, target quarc_win64 (corre en esta PC, external mode).
2. Dron MPC: QD2_DroneStack_MPC.slx, target quarc_linux_qdrone2, con la carpeta actual de MATLAB en simulink_qdrone2\binarios_dron. Resultado esperado: binarios_dron\QD2_DroneStack_MPC.rt-linux_qdrone2.
3. Tierra MPC: QD2_MissionCtrl_Dual_MPC.slx, target quarc_win64.
El dron PID NO se compila (binario original del Equipo2, md5 00d4d46e7a2e07158690fe98a9b756af).

Tu trabajo:
A. Comprobaciones previas de la guía (§1.2 a §1.7), sin compilar: release de MATLAB, Simulink Coder y MATLAB Coder, QUARC_DIR, qc_get_version, qc_get_target_types (debe incluir win64 y linux_qdrone2), which de quarc_win64.tlc y quarc_linux_qdrone2.tlc, compilador de Visual Studio C++ (vswhere, mex -setup C), puertos 8080/18700/18001/18002 libres y reglas de firewall para 18001/18002 (solo leer), y el §0 de la guía (ExtModeMexArgs de tierra con comillas: qc_get_model_arguments sin error).
B. Para los pasos que yo haga en la interfaz de MATLAB, dame los comandos exactos y dime qué debo ver. Cuando te pegue el log de un build, revísalo y dime la causa de cualquier error usando la tabla §3 de la guía.
C. Lleva una lista de lo que salió bien, de lo que falló y de lo que se cambió.

Reglas (no negociables):
- No hagas git commit, push ni cambies de rama.
- No edites ni guardes los .slx. No muevas ni borres bloques; en especial, nunca toques los bloques originales del Equipo2 (el layout se verificó bloque por bloque). No corras construir_modelos ni construir_dron_y_sim: regeneran los modelos y aquí no se deben regenerar.
- No edites ningún archivo (tampoco config\estacion.json: lo que lee Simulink cambia lo que se compila). La única excepción es la línea del §0 de la guía en simulink_qdrone2\qd2_cargar_config.m (ya debería venir corregida), y solo si la comprobación del §0 falla y yo te lo apruebo.
- No ejecutes qc_build_model, qc_download_model, qc_load_model, qc_connect_model, qc_start_model ni qc_stop_model; tampoco Build, Deploy ni Monitor & Tune, ni quarc_run con ninguna opción, ni ssh/scp a los drones. Solo funciones de lectura: qc_get_*, which, get_param, license, version y Update Diagram sin guardar.
- No arranques la estación web, mock_sim ni el simulador 3D: ocupan 8080/18700 y taparían el puente del modelo de tierra.
- No copies fuera de esta PC (ni al repositorio) archivos de QUARC con licencia: .tlc, bibliotecas, ejecutables, licencias (.lic) ni la documentación de Quanser. Cítala por nombre de archivo.
- No escribas contraseñas ni las pidas en el chat: las de SSH, Windows o licencias las teclea el usuario.
- No cambies el firewall, la licencia ni la configuración de Windows o de red. Si hace falta, dime el comando y lo corre el usuario como administrador.
- No lances quarc_run, no hagas Start del modelo de tierra y no detengas nada que esté corriendo sin que yo te lo pida. Nunca sugieras detener tierra con un dron en el aire.
- No uses matlab -batch con el arranque normal de QUARC: la ventana "QUARC Tip of the Day" lo cuelga. Para comprobaciones sin interfaz usa el truco de simulink_qdrone2\tests\README.md (startup.m con error('fin') y addpath de QUARC). Los builds se hacen en la interfaz de MATLAB.
- No dejes procesos tuyos de MATLAB o Python corriendo al terminar.
- No inventes resultados: distingue lo que corriste de lo que supones.

Al terminar, repórtame en una tabla:
1. Release de MATLAB, versión de QUARC, targets disponibles y compilador.
2. Para cada uno de los 3 builds: ¿compiló? (sí/no), tiempo, archivo producido (.rt-win64 / .rt-linux_qdrone2) y md5 del binario MPC del dron.
3. Si fue necesaria la corrección del §0, y cualquier otro cambio hecho (archivo y línea).
4. Errores encontrados, con el mensaje exacto y qué se hizo.
5. Qué quedó sin probar.
```

## 5. Supuestos (no verificables sin la PC del laboratorio)

- La PC del laboratorio tiene **R2022a** y un QUARC con los targets `win64` y `linux_qdrone2` y licencia para compilar. Se deduce del binario del Equipo2 (R2022a, `quarc_linux_qdrone2`, 20-nov-2026), pero la licencia y la versión exacta de QUARC no se conocen.
- La toolchain `QUARC win64 | nmake (64-bit Windows)` y `QUARC linux_qdrone2 | gmake (64-bit Windows)` (nombres guardados en los `.slx` exportados) existen con ese nombre en el QUARC del laboratorio.
- Visual Studio con C++ está instalado en esa PC (el Equipo2 compiló su tierra `quarc_win64` ahí). El compilador cruzado para el QDrone 2 viene con QUARC.
- Con la corrección del §0 (en `qd2_cargar_config.m` y, desde el 23-nov, en los `.slx` guardados), el `ExtModeMexArgs` de tierra deja de bloquear el build. Comprobado con QUARC solo en `versiones/R2022a/QD2_MissionCtrl_Dual_PID.slx` (compiló y corrió en la PC del laboratorio); en los demás, la laptop solo comprobó que las funciones de QUARC ya lo leen.
- La opción **Build for Monitoring** existe en la pestaña QUARC de esa versión y compila sin descargar. Con `qc_build_model`, si el dron `192.168.2.16` está encendido, el código se descarga ahí (`qc_build_model.html`); que además quede cargado o arrancado no está documentado.
- Los Stream Servers de tierra (`:18001`, `:18002`, `:18700`) escuchan en cuanto el modelo arranca, y el firewall de la PC deja entrar a los drones (sobre todo en `:18002`, que el Equipo2 no usaba).
- Un `.slx` exportado a R2023b abre bien en R2024a/R2024b.
- El `InitFcn` puede escribir las rutas de Motive con `set_param` **durante** el build QUARC (con Update Diagram sí funciona).
- `quarc_run -r` contra el QDrone 2 real funciona con los argumentos que genera Conexiones (no se ha probado).
- Los textos exactos de los mensajes de error de la tabla §3 marcados **[SUPUESTO]** y los tiempos de compilación.

## 6. Documentación de QUARC citada

En `%QUARC_DIR%\quarc\doc\`:
- Compilar y correr: `qc_build_model.html`, `qc_connect_model.html`, `qc_start_model.html`, `qc_stop_model.html`, `qc_download_model.html`, `quarc_tab.html` (botones Monitor & Tune, Build for Monitoring, Deploy, Connect, Start).
- Targets y licencia: `qc_get_target_types.html`, `qc_get_version.html`, `qc_find_targets.html`, `quarc_win64_target.html`, `quarc_linux_qdrone2_target.html`, `qdrone2.html`, `quarc_configuration.html` (licencias y puerto 17000 del Target Manager), `QUARC_Compatibility_Table.html`, `quarc_requirements.html`.
- Compiladores: `qc_get_compilers.html`, `qc_get_default_compiler.html`.
- URIs y argumentos: `quarc_default_uris.html`, `qc_get_target_uri.html`, `qc_get_model_arguments.html`, `configuration_parameters.html` ("MEX-file arguments": `-w`, `-d`, `-uri`, comillas), `model_arguments_dialog.html`.
- Línea de comandos: `quarc_command_line_tools.html` (`quarc_run`: `-r`, `-D`, `-l`, `-q`, `-t`, `-?`, formato `.rt-{target_type}`).
- Comunicación: `stream_server_block.html` ("Helpful Hints → Firewall").
