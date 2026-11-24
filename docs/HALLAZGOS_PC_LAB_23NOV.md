# Hallazgos del primer build QUARC en la PC del laboratorio (23-nov-2026)

PC: DESKTOP-CNN8EQJ (192.168.2.11), MATLAB R2022a Update 5, QUARC 4.3 (build 4318, 2023) con targets `win64` y `linux_qdrone2`, Visual C++ 2022. Repo en `C:\Users\user\Documents\Minor2026\Equipo2\QD2` (ruta más larga del repo: 153 caracteres; build sin problemas de longitud).

Resultado: el modelo de tierra `versiones/R2022a/QD2_MissionCtrl_Dual_PID.slx` **compila (17–23 s), se descarga y corre** en modo solo explorador; escuchan 18001, 18002 y 18700; la estación web (`--fuente simulink`, puente en 127.0.0.1) se conecta a 50 Hz y muestra "SOLO EXPLORADOR · dron 1", dron 2 "NO USADO". Falta: rastreo del dron en la arena, `quarc_run`, cámara, prueba en seco y vuelo.

## Problemas encontrados y solución

| # | Síntoma | Causa | Solución aplicada |
|---|---|---|---|
| 1 | Al abrir el modelo: `Error: This statement is incomplete.`; Build desde la interfaz falla igual; **no aparece la pestaña HARDWARE** (sí aparece en los modelos del Equipo2) | El `.slx` guardado traía `ExtModeMexArgs` **sin comillas**. QUARC lo lee **al abrir** el modelo y al empezar el build, **antes** del `InitFcn`; el `PostLoadFcn` llama a `qd2_cargar_config()` sin modelo y no lo corrige. La pestaña HARDWARE (Run on Custom Hardware, enganchada a `quanser.connectivity.ConnectivityConfig`) no se arma por ese error. La §0 de `GUIA_BUILD_QUARC_LAB.md` suponía que el `InitFcn` bastaba: **no basta** | `qd2_cargar_config(mdl)` + `save_system(mdl)` una vez en R2022a. El `.slx` de `versiones/R2022a` ya va guardado con `'-w -d %d -uri %u -URI_Host tcpip://localhost:18001 -URI_Host2 tcpip://localhost:18002'` (config activa `Configuration1`; la inactiva `Configuration` conserva la del Equipo2). Mismo número de bloques (582) y mismos callbacks; el resto del diff es formato de R2022a (el archivo venía exportado desde R2025a). **Pendiente para la laptop:** que `construir_modelos` guarde el valor con comillas en todos los `.slx` de tierra (PID y MPC, todas las versiones) o que el `PostLoadFcn` llame a `qd2_cargar_config(bdroot)` |
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
