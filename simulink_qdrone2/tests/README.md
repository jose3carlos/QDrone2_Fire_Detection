# Pruebas de la librería (`simulink_qdrone2/tests`)

Correr todo desde MATLAB (R2022a o posterior; probado en R2025a):

```matlab
cd simulink_qdrone2/tests
correr_pruebas
```

La primera vez tarda unos 4–6 minutos, porque Simulink compila el modelo de prueba. Cada prueba imprime lo que midió.

**Ojo con QUARC en modo `-batch`.** Al arrancar, QUARC abre una ventana modal ("QUARC Tip of the Day"). Con `matlab -batch` esa ventana bloquea MATLAB sin avisar. Hay tres formas de evitarlo:
- Cerrar la ventana a mano.
- Desactivarla en las opciones de QUARC.
- Arrancar con `matlab -sd <carpeta> -batch ...`, donde `<carpeta>` tiene un `startup.m` que solo hace `error('fin')`. Eso corta el resto de `matlabrc`, así que no se carga el arranque de QUARC. Simulink y las pruebas funcionan igual.

| Archivo | Qué hace |
|---|---|
| `test_qd2_lib.m` | Contiene 30 pruebas (`matlab.unittest`). Cubren roles (el de atrás es el explorador), densificación de la N, planta y downwash, lectura de `estacion.json`, deduplicación de objetivos, comandos y `cmd_seq_ack`, el guard unitario, MPC contra `quadprog` y su tiempo por paso, downwash con y sin feedforward/observador, error de trim, misión PID y MPC completas, 12 misiones PID aleatorias, pérdida de heartbeat → PAUSED → RESUME, ABORT → home → LAND_ALL y Simulink/codegen. Además, las pruebas de seguridad agregadas tras la revisión: ABORT/LAND_ALL antes de despegar, home demasiado cerca, overrides acotados, pérdida de rastreo/NaN, timeouts de despegue y aterrizaje, y misión PID con el rate limiter original (por eje) |
| (en `test_qd2_lib.m`) falla a bordo | `test_enlace_mpc` (validación de la respuesta de 8 doubles, alerta 10 a los 0.5 s y enlace perdido tras contacto → FSM 0 y código 2), `test_falla_antes_de_start` (FSM 7/8/9 desarmado sin alerta; con START se cancela en TAKEOFF sin takeoff y queda alerta 9; protocolo incorrecto rechaza START; compatibilidad sin los campos opcionales), `test_falla_R_apilado` (R falla debajo de E), `test_falla_E_apilado` (E falla encima de R: R sale de debajo en el paso siguiente y está a ≥ d_min_h cuando E cae a su altura; comparación con 1/3 m/s), `test_falla_landing_apilado` (LAND_ALL con los drones apilados: R espera a que E se aparte; falla de E o de R en ese momento → reacción), `test_falla_enlace_perdido` (enlace caído en los dos sentidos con E encima → alerta 9), `test_falla_protocolo` (etiqueta perdida en MISSION), `test_objetivos_y56` (y(56:60) rotando; ids fuera de 1..32 ignorados) |
| `qd2_probar_protocolo.m` | Extrae el `PROTOCOL CHECK` de `mpc/QD2_DroneStack_MPC.slx` y lo prueba con tramas sintéticas: acepta 24 = 2, rechaza 24 = 0, el flujo PID de 16 doubles y binarios/timestamp corruptos |
| `qd2_sim_lazo_cerrado.m` | Simulación SIL en MATLAB. Encadena misión, guard, rate limiter, el stack a bordo (PID o MPC), la planta y el downwash con 2 drones, más una estación Python y una visión simuladas. Puede cortar el rastreo de un dron (`trk_off`) y fijar ranuras de Python (`py_extra`). Falla a bordo: `falla_fsm` [dron t], `falla_apilado` (1 = falla E, 2 = falla R tras 1 s apilados), `proto_mal` [dron t1 t2], `enlace_off` [dron t1 t2] (enlace caído en los dos sentidos: el dron pasa a 7 a los 0.5 s y tierra retiene su última respuesta), `a_bordo` (tierra recibe la FSM a bordo, por defecto en MPC) y `rate_escape` |
| `qd2_sim_apilado.m` | Escenario de vuelo apilado: E se queda encima de R y se mide el error en z de R |
| `qd2_prueba_simulink.m` | Construye un modelo `.slx` desechable en `tempdir` con 8 bloques MATLAB Function y ejecuta `sim()`. Los bloques son: misión en versión 1 y 2, guard (con 7 y con 5 argumentos, `ctx` de 6 y de 4), referencia, 2 MPC, 2 plantas y la emulación a bordo. Así se verifica que el código compila con el generador de código de Simulink |
| `qd2_identificar_actitud.m` | Identifica el retardo de actitud y revisa el trim con los logs reales de `equipo2_original/saved_data` |

Resultados de la última corrida en esta laptop (R2025a): ver la sección "Resultados" de `../lib/README.md`.
