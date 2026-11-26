# Batería del QDrone 2 (`simulink_qdrone2/bateria`)

`qd2_bateria.m` lee los logs que guarda el dron PID del Equipo2 a bordo y resume cuánta batería se gastó en cada vuelo. Escribe archivos pequeños en `resultados/`, que sí se suben a git. Así los vuelos se pueden comparar sin los `.mat`, que pesan ~70 MB cada uno.

**Solo mide y reporta.** No cambia ningún límite de batería ni agrega ninguno, y no actúa sobre el dron, la misión ni la estación. Las dos líneas de referencia de las figuras son límites que ya existían (ver "El corte de batería a bordo").

Compatible con MATLAB R2022a (la PC del laboratorio). Usa `patch` para sombrear, no `xregion`, que es de R2023a. No necesita QUARC.

## Qué mide

Entradas: `log_<fecha>_QDrone2.mat` (variable `QDrone_data`, 56xN, 1 kHz) y, si está, su `log_<fecha>_Commander.mat` (`commander_data`, 37xN). Se emparejan por el nombre: mismo prefijo de fecha y hora con 1 s de tolerancia (el Commander suele salir 1 s antes, `19-59-40` contra `19-59-41`).

| Fila | Dato |
|---|---|
| `QDrone_data(1,:)` | t (s) |
| `QDrone_data(24,:)` | batería (V) |
| `QDrone_data(25,:)` | corriente de la electrónica: Jetson y sensores (A) |
| `QDrone_data(26,:)` | corriente de los motores (A) |
| `commander_data(19,:)` | estado FSM a bordo: 1 IDLE, 2 ARMADO, 3 DESPEGUE, 4 SUBIENDO, 5 VUELO, 6 AUTOLAND, 7/8 emergencia, 9 deshabilitado |

Por vuelo calcula:
- **Por fase FSM:** duración, V media y mínima (cruda y con media de 0.3 s), corriente media de motores y de electrónica, potencia y mAh.
- **Totales:** mAh y Wh, mAh de motores y de electrónica, y % de electrónica.
- **En el aire** (estados 3 a 6): tiempo, mAh, **mAh/min**, V media, V mínima con media móvil de 0.3 s (300 muestras a 1 kHz; la ventana se calcula con el dt real) y potencia media en VUELO (estado 5).
- **Reposo:** V en reposo en los 2 s antes de armar, y V después de aterrizar con los motores parados. Los logs del Equipo2 se cortan ~0.1 s después de aterrizar, con los motores aún girando. En ese caso `V_despues` queda vacío y se reporta `V_final_registro` (todavía bajo carga). `caida_bajo_carga_V` es la V de reposo antes de armar menos la V media en el aire.
- **Corte a bordo:** dice si en el log hubo V ≤ 14.0 V sostenido ≥ 0.3 s, la condición del corte que ya existe. Solo lo informa.

Sin Commander no hay fases. "En el aire" se aproxima entonces con la corriente de motores > 2 A (incluye el armado en el piso), y queda anotado en `criterio_aire`.

## Cómo correrlo en la PC del laboratorio

```matlab
cd C:\QD2\simulink_qdrone2\bateria
qd2_bateria('C:\ruta\a\saved_data', 'etiqueta', '24-nov con cámara')
```

- `qd2_bateria()` sin argumentos busca todos los `log_*_QDrone2.mat` bajo `simulink_qdrone2` (recursivo: `equipo2_original/saved_data`, `saved_data`, ...).
- `qd2_bateria(carpeta)` busca solo en esa carpeta (recursivo). `qd2_bateria({carpeta, archivo.mat, ...})` acepta varias carpetas o archivos.
- Opciones: `'etiqueta'`, `'reetiquetar'` (false), `'salida'` (por defecto `resultados/`), `'figuras'` (true), `'comparacion'` (true) y `'mostrar'` (true: imprime la tabla por fase).
- `'etiqueta'` es texto libre y solo se pone en los vuelos que aún no tienen `.json` o que no tienen etiqueta. Si un vuelo ya tenía otra etiqueta, se conserva y sale un aviso; para cambiarla, usa `'reetiquetar', true`. Como la etiqueta va a todos los vuelos nuevos de la llamada, si la carpeta trae vuelos de varios días o configuraciones, haz una llamada por configuración pasando sus archivos: `qd2_bateria({'...\log_24-Nov-..._QDrone2.mat', ...}, 'etiqueta', '24-nov con cámara')`.
- Si el resumen de un vuelo no cambió, su `.json` (con su `fecha_analisis`) y su `.png` no se reescriben, y `comparacion.png` solo se rehace si algún `.json` es más nuevo. Así `git status` solo marca los vuelos nuevos o los que cambiaron, más `resumen.csv` cuando cambia alguna fila.
- `resumen.csv` y `comparacion.png` van en orden cronológico (`fecha_vuelo`), no por nombre. `resumen.csv` lleva BOM UTF-8 para que Excel en Windows muestre bien los acentos.
- Tarda ~5 s por vuelo. En `matlab -batch` con QUARC instalado, usa el truco de `../tests/README.md` (`-sd` con un `startup.m` que hace `error('fin')`); si no, la ventana "QUARC Tip of the Day" bloquea MATLAB.

Pruebas: `probar_qd2_bateria` usa logs sintéticos pequeños generados en `tempdir` y, si existen, los 2 vuelos reales del Equipo2: comprueba 916.7 mAh, 886.6 mAh y 14.235 V con una tolerancia de 0.5 %. Tarda ~15 s; `probar_qd2_bateria(false)` corre solo las sintéticas.

## Qué subir a git

Solo `resultados/`:

| Archivo | Contenido |
|---|---|
| `<nombre_log>.json` | resumen del vuelo: fases, totales, aire, reposo, corte y una serie reducida a 1 s (V con media de 0.3 s y corriente de motores contra tiempo en el aire) para comparar sin los `.mat`. Pesa < 20 KB |
| `<nombre_log>.png` | voltaje (con las líneas de 14.0 V y 15.5 V), corrientes y mAh por fase. Pesa < 300 KB |
| `resumen.csv` | una fila por vuelo. Se rehace desde todos los `.json` de la carpeta, así que no duplica vuelos |
| `comparacion.png` | todos los vuelos de `resultados/` superpuestos: V contra tiempo en el aire y mAh/min |

Los `.mat` **nunca** se suben, y el script nunca los copia. `/simulink_qdrone2/equipo2_original/saved_data/` y `simulink_qdrone2/saved_data/` ya están en `.gitignore`, y `bateria/.gitignore` ignora cualquier `.mat` de esta carpeta. Antes del commit, revisa con `git status` que no haya un `.mat` en otra carpeta.

## Resultados de referencia (2 vuelos del Equipo2, 20-nov-2026, misma batería, uno tras otro)

| | Vuelo 1 (19:59:41) | Vuelo 2 (20:05:50) |
|---|---|---|
| Total | 916.7 mAh (13.78 Wh) | 886.6 mAh (12.84 Wh) |
| En el aire | 131.9 s, 895.9 mAh, 408 mAh/min | 125.8 s, 846.0 mAh, 404 mAh/min |
| Potencia en VUELO | 367 W | 352 W |
| Electrónica | 1.7 % | 1.6 % |
| V reposo antes de armar | 16.41 V | 15.54 V |
| V mín en el aire (media 0.3 s) | 14.627 V | 14.235 V |
| V ≤ 14.0 V por 0.3 s | no | no |

## Cómo interpretarlo

- **Motores ~98 %, electrónica ~1.5-2 %.** En vuelo casi todo es empuje: ~24 A de motores contra ~0.37 A de electrónica (~5.5 W). La autonomía la decide el peso, por ejemplo el soporte y la cámara.
- **Caída bajo carga ~1 V.** Al armar, el voltaje cae de golpe ~0.4-0.5 V por la resistencia interna. La V media en el aire queda ~1-1.4 V por debajo de la de reposo (`caida_bajo_carga_V`: 1.39 V y 1.06 V). La V que se ve en vuelo no es la de reposo: una batería que en el piso marca ~15.5 V vuela a ~14.5 V.
- **mAh/min** (~400 en estos vuelos) sirve para comparar configuraciones (con o sin cámara, PID o MPC). Los mAh totales dependen de cuánto duró cada vuelo.
- El vuelo 2 empezó con 15.54 V en reposo, apenas por encima de la recomendación de 15.5 V. Llegó a 14.235 V (media 0.3 s) en el aire, a ~0.24 V del corte de 14.0 V.

## El corte de batería a bordo (ya existente; este script no lo toca)

- Todos los modelos del dron (el PID del Equipo2 y nuestros MPC) tienen el bloque **"Check Battery level"**: con **V ≤ 14.0 V durante ≥ 0.3 s** (bloque Threshold de QUARC) se enclava `low_battery`. En vuelo (estados 4/5) el dron pasa a AUTOLAND (6). En IDLE, ARMADO o DESPEGUE da error y no arma. Queda enclavado.
- **No hay ningún corte a 14.8 V.** 14.8 V es el voltaje **nominal** de una LiPo 4S (3.7 V por celda), no un límite.
- **15.5 V** es la recomendación humana de `docs/CHECKLIST_LABORATORIO.md` para no despegar con menos de eso. No es automática.
- En las figuras, la línea roja es el corte de 14.0 V ("aterrizaje automático a bordo") y la naranja es la recomendación de 15.5 V ("mínimo recomendado, checklist, no automático"). No se dibuja ningún otro umbral.

## Batería en vivo en la estación

La telemetría lleva `battery_V` por dron, pero **en los vuelos PID la estación no recibe la batería en vivo**: el dron PID del Equipo2 solo devuelve el eco del timestamp, y el modelo de tierra PID pone `[NaN NaN]`. En los vuelos PID, la batería solo se ve después del vuelo, con este script y los logs a bordo. En MPC sí llega en vivo (telemetría a bordo `[ts, FSM, batería, ...]`).
