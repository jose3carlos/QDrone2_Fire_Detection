# Hallazgos del primer build QUARC en la PC del laboratorio (23-nov-2026)

PC: DESKTOP-CNN8EQJ (192.168.2.11), MATLAB R2022a Update 5, QUARC 4.3 (build 4318, 2023) con targets `win64` y `linux_qdrone2`, Visual C++ 2022. Repo en `C:\Users\user\Documents\Minor2026\Equipo2\QD2` (ruta más larga del repo: 153 caracteres; build sin problemas de longitud).

Resultado: el modelo de tierra `versiones/R2022a/QD2_MissionCtrl_Dual_PID.slx` **compila (17–23 s), se descarga y corre** en modo solo explorador; escuchan 18001, 18002 y 18700; la estación web (`--fuente simulink`, puente en 127.0.0.1) se conecta a 50 Hz y muestra "SOLO EXPLORADOR · dron 1", dron 2 "NO USADO". Falta: rastreo del dron en la arena, `quarc_run`, cámara, prueba en seco y vuelo.

## Problemas encontrados y solución

| # | Síntoma | Causa | Solución aplicada |
|---|---|---|---|
| 1 | Al abrir el modelo: `Error: This statement is incomplete.`; Build desde la interfaz falla igual; **no aparece la pestaña HARDWARE** (sí aparece en los modelos del Equipo2) | El `.slx` guardado traía `ExtModeMexArgs` **sin comillas**. QUARC lo lee **al abrir** el modelo y al empezar el build, **antes** del `InitFcn`; el `PostLoadFcn` llama a `qd2_cargar_config()` sin modelo y no lo corrige. La pestaña HARDWARE (Run on Custom Hardware, enganchada a `quanser.connectivity.ConnectivityConfig`) no se arma por ese error. La §0 de `GUIA_BUILD_QUARC_LAB.md` suponía que el `InitFcn` bastaba: **no basta** | `qd2_cargar_config(mdl)` + `save_system(mdl)` una vez en R2022a. El `.slx` de `versiones/R2022a` ya va guardado con `'-w -d %d -uri %u -URI_Host tcpip://localhost:18001 -URI_Host2 tcpip://localhost:18002'` (config activa `Configuration1`; la inactiva `Configuration` conserva la del Equipo2). Mismo número de bloques (582) y mismos callbacks; el resto del diff es formato de R2022a (el archivo venía exportado desde R2025a). ~~Pendiente para la laptop~~ **Resuelto en la laptop (23-nov-2026):** los otros 11 `.slx` de tierra (`pid/`, `mpc/`, `versiones/R2022a/…_MPC.slx` y `versiones/{R2022b,R2023a,R2023b,R2025a}/…_{PID,MPC}.slx`) ya van guardados con el valor entre comillas (solo se editó ese valor en `simulink/configSet1.xml`; mismos bloques y callbacks; los dos de R2022a PID de esta PC quedaron sin tocar). `construir_modelos` comprueba el valor guardado en cada `.slx` de tierra (`qd2_ext_mode_args_slx`) y `test_ext_mode_args_slx` lo vigila. El `PostLoadFcn` no se cambió (ver `GUIA_BUILD_QUARC_LAB.md` §0) |
| 2 | Tras `qc_start_model`, el proceso del modelo existe pero `T=0.000`, 0 % CPU y no abre puertos | El argumento `-w` hace que el código de tiempo real **espere a que Simulink se conecte** (`configuration_parameters.html`, "Model arguments") | Usar Monitor & Tune (build → deploy → connect → start), o `qc_connect_model` antes de Start |
| 3 | Connect: `Could not execute target data map file 'QD2_MissionCtrl_Dual_PID_quarc_win64\..._targ_data_map'` | El build se hizo con la carpeta actual de MATLAB en otro lado; external mode busca la carpeta de build en la carpeta actual | Siempre `cd …\simulink_qdrone2\versiones\R2022a` antes de compilar y conectar |
| 4 | Modelo atorado en `T=0` aunque conectado | Otro modelo QUARC (`Optitrack_Visualization.exe`, de otro equipo) llevaba horas corriendo con las cámaras OptiTrack | Revisar `Get-CimInstance Win32_Process` (hijos de `quarc_target_manager`) antes de arrancar; detenerlo. Los procesos de QUARC solo se pueden terminar como **administrador** |
| 5 | `Unable to ascertain type of target tcpip://localhost:17000` | Tras terminar los modelos a la fuerza, el servicio "QUARC Target Manager" siguió "Running" pero dejó de escuchar en 17000 | `Restart-Service -Name "QUARC Target Manager" -Force` (administrador) |
| 6 | `Error occurred while executing External Mode MEX-file 'quarc_comm': The rigid body ID is invalid or not found in the rigid body definition file.` | El `.motive` del Equipo2 (`Nov 20_Team2\RigidBody.motive`) define un solo rigid body (ID 1, sin `UserData`); el modelo pide `[1 2]` (ya advertido en la checklist y en `pid/README.md`) | `config/motive/RigidBody_dron1_aux2.motive`: el rigid body 1 idéntico + uno auxiliar con `UserData` = 2 y geometría imposible en la arena (triángulo 0.60/0.75/0.90 m) para que nunca se confunda con un dron. En la PC está en `Documents\OptiTrack\Nov 20_Team2\` y `optitrack.archivo_motive` apunta ahí. **Para el modo dual hace falta un `.motive` con el rigid body REAL del dron 2** |

## Pruebas de la librería en R2022a (esta PC)

`correr_pruebas` sin el arranque de QUARC: **41 de 42**. La que falla, `test_solo_codegen`, carga `pid/QD2_MissionCtrl_Dual_PID.slx`, que está guardado en R2025a (`Simulink:Commands:LoadingNewerModel`): es la versión del archivo, no el código. El modelo equivalente de `versiones/R2022a` sí compiló y corrió con QUARC en modo solo explorador.

## Orden que funcionó

1. Motive cerrado y ningún otro modelo QUARC corriendo.
2. MATLAB: `cd …\simulink_qdrone2\versiones\R2022a`, `open_system('QD2_MissionCtrl_Dual_PID.slx')` (sin errores, con pestaña HARDWARE).
3. HARDWARE → Monitor & Tune (switches abajo, Command Source = 0). En el log: `qd2_cargar_config: MODO SOLO EXPLORADOR -> dron 1 …`.
4. `web\iniciar_web.bat --fuente simulink` en la misma PC.

## Modo dual (2 drones) en esta PC, sin tocar el de solo explorador

Archivos aparte (el modo solo explorador sigue en `config/estacion.json` y `QD2_MissionCtrl_Dual_PID.slx`):

| Archivo | Qué es |
|---|---|
| `config/estacion_dual.json` | Igual que `estacion.json` pero `"modo": "dual"` (sin `dron_solo`) |
| `versiones/R2022a/QD2_MissionCtrl_Dual_PID_2D.slx` | Copia del modelo de tierra PID (mismos 451 bloques) cuyo `PostLoadFcn` e `InitFcn` llaman a `qd2_cargar_config(..., <repo>/config/estacion_dual.json)`. Guardada en R2022a con `ExtModeMexArgs` entre comillas (pestaña HARDWARE) |

Comprobado sin hardware (MATLAB R2022a): al abrir dice `qd2_cargar_config: modo dual (2 drones)`, Update Diagram OK, `modo_mision = 0`, `rb_ids = [1 2]`, `qc_get_model_arguments` sin error (`-uri shmem://QD2_MissionCtrl_Dual_PID_2D:1 -URI_Host …:18001 -URI_Host2 …:18002`). **No se ha compilado ni volado en dual.**

Uso:
1. Motive cerrado, ningún otro modelo QUARC corriendo, solo un modelo de tierra a la vez (los dos usan 18001, 18002 y 18700).
2. `cd …\simulink_qdrone2\versiones\R2022a`, `open_system('QD2_MissionCtrl_Dual_PID_2D.slx')`, HARDWARE → Monitor & Tune.
3. Estación: `web\iniciar_web.bat --fuente simulink --config <repo>\config\estacion_dual.json` (la UI debe mostrar modo dual).
4. Drones: `quarc_run` del dron 1 (`:18001`) y del dron 2 (`:18002`) desde la pestaña Conexiones.

Falta (necesita la arena): un `.motive` con el rigid body **real** del dron 2 (ID 2, geometría de marcadores distinta a la del dron 1) y apuntar `optitrack.archivo_motive` de `estacion_dual.json` a ese archivo. Con el auxiliar actual el dron 2 nunca se rastrea y la misión se queda en WAIT_TRACKING (no despega). También falta `red.ip_dron2`.

## 24-nov: pérdida de rastreo en vuelo → aterrizar (librería de misión)

Vuelos 2, 3 y 4 (explorador solo, PID): OptiTrack perdió el dron en la franja x ~1.3, y >= 1.0 a 1.5 m; la misión pasó a PAUSED (alerta 2), pero sin pose el dron derivó ~0.5 m hacia +y y cayó fuera de la arena. Ahora, si un dron en el aire sigue sin rastreo más de `mision.t_perdida_aterrizar_s` (0.2 s, en `config/estacion*.json`: el mismo umbral con el que la FSM a bordo del Equipo2 ya pasa a 7, así que no hay PAUSED intermedio), la misión le baja takeoff (aterriza en su sitio) y nunca se lo vuelve a dar; en dual el otro se aparta, regresa a su home y aterriza. Con 0.3 s (primera versión de este cambio) una pérdida de 0.2-0.3 s, o intermitente, dejaba la misión en PAUSED (RESUME posible) con un dron que ya caía; por eso quedó en 0.2 s. Solo cambian las `.m` de `simulink_qdrone2/lib` (y `P`): para que el modelo de tierra lo use hay que **recompilar** (Build) `versiones/R2022a/QD2_MissionCtrl_Dual_PID.slx` en la PC del laboratorio; los `.slx` no cambian. El dron perdido sigue entrando a la emergencia a bordo (estado 7, rampa) a los 0.2 s: hay que relanzar su modelo.

## 25-nov: envolvente del respondedor (`mision.limites_y`)

Por la misma franja sin rastreo (x ~1.3, y ≥ 1.0) y porque hacia y− OptiTrack rastrea más lejos, `config/estacion.json` y `config/estacion_dual.json` traen `"limites_y": [-1.4, 1.2]`: ninguna consigna xy de los dos drones pasa de y = 1.2 ni baja de y = −1.4 (es el límite efectivo; el margen de 0.15 m de `limites_xy` no se le aplica). Una toma fuera (las orillas en y ≈ +1.40 y −1.48) la marca R desde el punto más cercano dentro; la pestaña Anomalías lo indica con "se marca desde (x, y)" y el mapa 2D y la vista 3D dibujan la envolvente en punteado violeta. Detalles en `simulink_qdrone2/lib/README.md` ("Envolvente por eje").

Solo cambian las `.m` de `simulink_qdrone2/lib` (nueva `qd2_limites_xy.m`) y `P`: para que el modelo de tierra la use hay que **RECOMPILAR** (Build) en la PC del laboratorio, con el `estacion*.json` ya copiado:
- `versiones/R2022a/QD2_MissionCtrl_Dual_PID.slx` (solo explorador, lee `config/estacion.json`);
- `versiones/R2022a/QD2_MissionCtrl_Dual_PID_2D.slx` (dual, lee `config/estacion_dual.json`).

Sin recompilar, el modelo sigue recortando a ±1.35 en y (R podría ir a y = 1.35, dentro de la franja sin rastreo). Al abrir/compilar, `qd2_cargar_config` debe imprimir `envolvente de consignas x [-1.35 1.35], y [-1.40 1.20] m`; si no sale, la librería es vieja (copiar la carpeta `lib` de nuevo), y si sale un warning, `limites_y` es inválido. Los `.slx` no cambian. Sin las claves todo es bit a bit como antes (probado).
