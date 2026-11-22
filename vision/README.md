# Visión: YOLO11n para fuegos y tomas clandestinas

Detector de 2 clases para la cámara cenital del dron EXPLORADOR: **0 = fuego**, **1 = toma** (válvula
abrazada al tubo). Se entrena una vez con datos sintéticos de Blender (más fotos reales cuando existan) y la
estación web sólo carga `vision/modelos/mejor.pt` (o `mejor.onnx`). **No se reentrena** salvo que se pida con `--forzar`.

| Archivo | Qué hace |
|---|---|
| `sintetico/generar_blender.py` | Genera el dataset sintético YOLO con Blender en modo consola |
| `sintetico/ver_etiquetas.py` | Mosaico de imágenes con sus cajas, para revisar etiquetas a ojo |
| `entrenar.py` | Entrena YOLO11n y guarda `modelos/mejor.pt` + `modelos/metricas.json` + `modelos/mejor.onnx` |
| `evaluar.py` | mAP por clase, P/R con la confianza de operación, velocidad (ms/imagen) de cada motor |
| `detector.py` | `Detector(model_path, conf).detect(frame_bgr) -> [(cls, conf, x1, y1, x2, y2), ...]` (lo usa `web/`) |
| `exportar_onnx.py` | `mejor.pt` → `mejor.onnx` para correr sin torch (cv2.dnn) |
| `capturar_fotos.py` | Captura fotos reales desde el stream QDF1 del dron (:18800) o una webcam |
| `etiquetar.py` | Pre-etiqueta fotos reales con el modelo y abre un editor OpenCV para corregirlas |
| `paquete.py` | Lleva modelo + fotos etiquetadas de una PC a otra en un .zip (no van a git) |
| `requirements.txt` | Versiones exactas probadas |
| `instalar_en_web.bat` | OPCIONAL: torch + ultralytics en el venv de la estación (para usar la GPU) |

Los datasets (`datos/`), los pesos (`*.pt`, `*.onnx`), las fotos reales y las corridas (`runs/`) **no van a git**
(`vision/.gitignore`). Las métricas (`modelos/metricas*.json`, `modelos/evaluacion_*.json`) sí.

## 0. Lo mínimo para el laboratorio (estación web)

1. **Copiar el modelo a la PC del laboratorio.** `mejor.pt` y `mejor.onnx` no están en git. En la laptop:
   `%PY% vision\paquete.py exportar --sin_fotos --salida E:\` (USB) y en la PC del lab:
   `python vision\paquete.py importar E:\paquete_vision_<fecha>_solo_modelo.zip` (sólo usa la biblioteca
   estándar, sirve cualquier Python 3.10+). También se puede copiar a mano `vision\modelos\mejor.pt`,
   `mejor.onnx` y `mejor.onnx.json`.
2. **Arrancar la estación normal** (`web\iniciar_web.bat`). Su venv (`qd2_web`) **no tiene torch**; en ese caso
   `detector.py` usa solo `mejor.onnx` con OpenCV (cv2.dnn, CPU, ~35–45 ms por imagen de 640×480 en esta laptop, ver §3) y detecta **fuegos y tomas**.
   En la interfaz el detector aparece como `vision/detector.py (mejor.pt)`; si en cambio dice
   `fomo (Edge Impulse) — respaldo, solo fuego`, falta `mejor.onnx`: corre el paso 1 o
   `%PY% vision\exportar_onnx.py`. Prueba rápida desde `web\`:
   ```bat
   %USERPROFILE%\.venvs\qd2_web\Scripts\python.exe -c "import json,os,sys; sys.path.insert(0,'.'); from app.vision_service import load_detector; r=os.path.abspath('..'); print(load_detector('auto', r, json.load(open(r+'/config/estacion.json',encoding='utf-8')))[:2])"
   ```
3. **Opcional, GPU en la estación:** `vision\instalar_en_web.bat` instala torch cu128 + ultralytics (~2.5 GB) en
   `%USERPROFILE%\.venvs\qd2_web`. Entonces se usa `mejor.pt` con ultralytics en la GPU (~18–26 ms).
   El motor se puede forzar en `config/estacion.json → vision.backend` (`auto`, `ultralytics`, `opencv`).

## 1. Entorno de visión (entrenar, evaluar, etiquetar)

El venv vive fuera de OneDrive:

```bat
C:\Users\ezequ\AppData\Local\Programs\Python\Python310\python.exe -m venv %USERPROFILE%\.venvs\qdrone2_vision
%USERPROFILE%\.venvs\qdrone2_vision\Scripts\python.exe -m pip install -r vision\requirements.txt
```

`requirements.txt` fija las versiones probadas: torch 2.11.0+cu128 (CUDA disponible en la RTX 4060 Laptop),
torchvision 0.26.0+cu128, ultralytics 8.4.216, opencv-python 5.1.0.95, numpy 2.2.6, onnx 1.23.2. El índice cu128
también sirve para una RTX 50xx. Abajo, `PY` = `%USERPROFILE%\.venvs\qdrone2_vision\Scripts\python.exe` y todo
se corre desde la raíz del repo.

## 2. Datos sintéticos (Blender 5.3)

```bat
"C:\Program Files\Blender Foundation\Blender 5.3\blender.exe" -b -P vision\sintetico\generar_blender.py -- --salida vision\datos\sintetico_smoke --n 600 --semilla 101
%PY% vision\sintetico\ver_etiquetas.py --dataset vision\datos\sintetico_smoke --n 16
```

Qué genera:

- **Piso:** tapete EVA gris oscuro tipo rompecabezas, con losetas de 50, 60 o 100 cm y textura generada, más el piso del laboratorio alrededor.
- **Tubería:** PVC blanco de DE 26.7 mm en N invertidas aleatorias (o polilíneas libres), con codos y coples.
- **Fuegos:** las STL `impresion3d/stl/fuego_{A,B,C}_rojo` + `_amarillo` ensambladas (rojo #C81E1E, amarillo #F5C400). La altura de la pieza amarilla sobre la roja (3.6 mm) se deduce del STL monocolor, que ya viene ensamblado. En el 15 % de los casos se usa la versión monocolor. Van junto al tubo, encima de él o sueltos.
- **Tomas:** la válvula (cuerpo + tuerca + vástago/volante + zapata) **abrazada al tubo**: el eje del tubo coincide con el de la C. El 92 % sale en rojo (la pieza real); el 8 % en negro, gris, azul o rojo oscuro para que el modelo aprenda la forma y no sólo el color (nunca amarillo, que es color de los fuegos). Se usan las mallas en posición de uso de `impresion3d/renders/_mallas_uso/`. Si no existen, se usan las STL de impresión reorientadas, y si tampoco hay, un modelo procedural. Las STL se buscan en cada ejecución.
- **Distractores:** plantas artificiales, carritos de juguete, cilindros grises, cruces de cinta, cintas de colores, papel, cables, marcadores de OptiTrack, objetos rojos y amarillos, y a veces el otro QDrone 2 volando debajo (caso MPC apilado). Las **calcomanías planas de flamas son FONDO** y no se etiquetan.
- **Cámara:** de 0.3 a 1.8 m (más densidad cerca de 1.5 m), inclinación de ±10° y yaw libre. Intrínsecos de la D435i color con VFOV 42.5° (ver §5). El 70 % de las imágenes sale a 640×480 y el 30 % a 1280×720.
- **Variación de imagen:** luces con temperatura de color aleatoria, exposición, balance de blancos, contraste, desenfoque de movimiento y de foco, viñeteo, ruido y JPEG con calidad de 72 a 95.
- **Etiquetas:** se proyectan todos los vértices de cada pieza. La oclusión se calcula con *ray casting*. Una pieza visible en menos del 25 % no se etiqueta. Si está parcialmente ocluida, la caja se ajusta a la parte visible. Las cajas se recortan al borde de la imagen.
- **Partición:** el 15 % de las **escenas** va a val; las vistas de una escena nunca se reparten entre train y val. 6 vistas por escena.
- **Reanudable:** si se corre otra vez con la misma `--salida` y `--semilla`, se saltan las escenas completas (última vista con su .jpg y su .txt). Cada etiqueta se escribe antes que su imagen y la imagen se mueve de golpe, así que matar el proceso nunca deja una imagen sin etiqueta. `meta_s*.csv` queda con una fila por imagen y `stats_s*.json` cuenta el dataset completo en disco (`ultima_corrida` = sólo la última ejecución).

Velocidad medida (Cycles + OptiX, 24 muestras con denoiser, RTX 4060 Laptop): ~110–115 imágenes/min.

## 3. Entrenar, evaluar y usar

```bat
%PY% vision\entrenar.py --datos vision\datos\sintetico_smoke --epocas 30
%PY% vision\evaluar.py --datos vision\datos\sintetico_smoke --comparar_motores
%PY% vision\detector.py --imagen alguna_foto.jpg
%PY% vision\detector.py --carpeta vision\datos_reales\images\val
%PY% vision\detector.py --camara 0
```

- `entrenar.py` no hace nada si ya existe el modelo de salida. Con `--forzar` reentrena y respalda el anterior como `mejor_<fecha>.pt` (y sus métricas).
- Las métricas van con el nombre del modelo: `mejor.pt` → `metricas.json`; `--salida modelos\otro.pt` → `metricas_otro.json` y `results_entrenamiento_otro.csv`. Al terminar exporta también `<modelo>.onnx`.
- `--datos` acepta varias carpetas o archivos `data.yaml` a la vez.
- Las fotos de `datos_reales/` se mezclan solas (sólo las **revisadas**, ver §4) y se repiten `--peso_reales` veces (3 por defecto).
- `detector.py` guarda las imágenes con cajas en `vision\runs\detector\` (nunca dentro de la carpeta de datos) e ignora los `*_det.jpg`.
- `evaluar.py` necesita `images/val` (o la estructura de `datos_reales`); nunca evalúa sobre todas las imágenes.

### Resultados del modelo actual `mejor.pt` / `mejor.onnx` (22-nov-2026, laptop RTX 4060)

- **Entrenamiento** (`modelos/metricas.json`, corrida `runs/yolo11n_20261122_0545`): `entrenar.py --forzar --datos C:\Users\ezequ\datasets\qdrone2_vision\sintetico_10k vision\datos\sintetico_smoke --epocas 80`. 80 épocas, imgsz 640, batch 16, desde `yolo11n.pt` de COCO, 111.6 min. Train: 8 610 + 534 imágenes sintéticas; val: 1 390 + 66. **0 fotos reales.**
- **Validación sintética:** **mAP50 = 0.995**, mAP50-95 = 0.937 (P = 0.992, R = 0.985); fuego AP50 = 0.995 (P 0.990 / R 0.984), toma AP50 = 0.995 (P 0.995 / R 0.985).
  **Ojo:** es validación sintética (mismo generador que el train): **no dice nada de fotos reales** (para eso, §4).
- **Comprobación con el motor de la estación** (OpenCV-ONNX en CPU, venv `qd2_web`, confianza de operación 0.5, IoU ≥ 0.5, 200 imágenes val de `sintetico_10k` tomadas cada 6.ª, 22-nov-2026): 253 aciertos, 6 falsos positivos y 2 no detectados → **P = 0.977, R = 0.992**.
- **Velocidad de `Detector.detect()` con OpenCV-ONNX en CPU** (lo que usa la estación sin torch; incluye pre y post): 100 imágenes de 640×480, media 40 ms, mediana 36 ms, p90 52 ms, medido con la CPU compartida con otras sesiones de MATLAB, así que es una cota pesimista. Las cifras de ultralytics GPU/CPU de abajo son del modelo anterior (misma arquitectura YOLO11n, mismo costo).
- `mejor.onnx.json` guarda el md5 del `mejor.pt` del que salió (`144d00b2…`); `detector.py` avisa si el `.onnx` quedó desactualizado.
- **Modelo anterior** (respaldado como `modelos/mejor_20261122_073731.pt` y `metricas_20261122_073731.json`): sólo `sintetico_smoke` (600 imágenes, 30 épocas, 3.7 min), mAP50 = 0.960 sobre 66 imágenes val. Velocidad medida entonces (`modelos/evaluacion_sintetico_smoke.json`, 640×480, GPU compartida con Blender): ultralytics GPU media 21–26 ms (17–19 ms con la GPU libre); ultralytics CPU (Ryzen 9 8945HS) media 39 ms; OpenCV-ONNX CPU media 32–43 ms.
- **Dataset grande:** 10 000 imágenes (semilla 202) en `C:\Users\ezequ\datasets\qdrone2_vision\sintetico_10k` (fuera de OneDrive), log en `...\sintetico_10k_log2.txt`. Para reanudarlo, el mismo comando:
  ```bat
  "C:\Program Files\Blender Foundation\Blender 5.3\blender.exe" -b -P vision\sintetico\generar_blender.py -- --salida C:\Users\ezequ\datasets\qdrone2_vision\sintetico_10k --n 10000 --vistas 6 --semilla 202
  ```

## 4. Fotos reales en el laboratorio (lo más importante para que funcione de verdad)

El modelo sintético es un punto de partida. El salto sintético → real se cierra con **unas 300–500 fotos reales etiquetadas**.

### Captura (~30 min)

1. Arma la pista real: la N de tubo, los 3 fuegos y las 3 tomas en posiciones variadas, más los distractores reales (plantas, carritos, cilindros, cintas, calcomanías de flama…).
2. Usa **la misma cámara y el mismo montaje del explorador**, con `qdrone2_camera_streamer.py` en el Jetson, lanzado con `--serie <serie de la D435i de abajo>` (el QDrone 2 trae otra RealSense frontal; `rs-enumerate-devices -s` da las series). Con el dron **con los motores apagados (sin modelo QUARC corriendo) y cargado a mano** (o en un palo), recorre la pista:
   ```bat
   %PY% vision\capturar_fotos.py --dron <IP_JETSON_EXPLORADOR> --cada 0.7
   ```
   - **Cierra antes la estación web:** el streamer atiende un solo cliente. Si está ocupado, el programa lo dice a los 8 s.
   - Si avisa que llegan frames de 320×240, el Jetson no tiene OpenCV y manda RAW reducido: instala `python3-opencv` en el Jetson para tener 640×480.
   - `a` activa el guardado automático y `ESPACIO` guarda una foto. También sirve `--camara 0` con una RealSense o webcam conectada a la PC. Repetir `--sesion X` continúa la misma carpeta sin sobrescribir; `--max N` cuenta sólo fotos nuevas.
3. Qué cubrir: alturas 0.5, 1.0, **1.5 (~50 %)** y 1.8 m; orientación 0/90/180/270° e inclinaciones ligeras; piezas al centro, en los bordes, cortadas y tapadas; todas las luces y la mitad. Mueve las piezas 3–4 veces.
4. Incluye **50–100 fotos negativas**: pista sin piezas, sólo distractores y calcomanías de flama.
5. Haz una **segunda sesión corta** (~80 fotos) con las piezas en otras posiciones y guárdala como **validación**.

### Etiquetado rápido (~2–4 s por foto)

```bat
REM pre-etiqueta con el modelo sintético (conf 0.25) y copia a datos_reales\images
%PY% vision\etiquetar.py auto --entrada vision\datos_reales\crudas\lab_AAAAMMDD_HHMM
REM la sesión de validación se copia SIN pre-etiquetas (se etiqueta desde cero para no sesgar la métrica)
%PY% vision\etiquetar.py auto --entrada vision\datos_reales\crudas\lab_val --split val
REM revisar y corregir TODAS (d / ESPACIO marca la foto como revisada)
%PY% vision\etiquetar.py revisar
```

**Las pre-etiquetas no cuentan como etiquetas.** `entrenar.py` y `evaluar.py` sólo usan las fotos que están en
`datos_reales\revisadas.txt`, es decir, las que alguien pasó con `revisar`; las demás se cuentan y se avisa.
(`--incluir_sin_revisar` existe, pero no se recomienda.)

Controles del editor:

- **Ratón:** arrastrar con el botón izquierdo crea una caja de la clase activa; clic izquierdo dentro de una caja la selecciona; clic derecho dentro la borra.
- **Teclado:** `1` fuego, `2` toma (también cambia la caja seleccionada), `d`/ESPACIO guardar como revisada y siguiente, `a` anterior, `x` borrar todas las cajas (negativo), `z` deshacer, `q` salir. El progreso se guarda.

Criterios (los mismos del sintético): la caja cubre **toda la pieza visible** (el fuego completo; la toma con volante y cuerpo, **sin el tubo**); piezas cortadas por el borde se etiquetan si se ve ≥ ~20 %; calcomanías y fotos de flamas planas = **sin caja**.

### Mover fotos y modelo entre la PC del lab y la laptop

```bat
%PY% vision\paquete.py exportar --salida E:\              REM modelo + datos_reales (images, labels, revisadas)
%PY% vision\paquete.py importar E:\paquete_vision_<fecha>.zip
```
Al importar se respalda el modelo local si es distinto, se conserva la etiqueta más reciente si las dos PCs la
editaron y `revisadas.txt` se une. `--con_crudas` agrega también las fotos sin procesar.

### Reentrenar con las fotos reales

```bat
REM opción A (recomendada): sintético + reales desde cero
%PY% vision\entrenar.py --forzar --datos vision\datos\sintetico_smoke C:\Users\ezequ\datasets\qdrone2_vision\sintetico_10k --epocas 80
REM opción B (rápida): afinar el modelo sintético sólo con las reales (métricas en metricas_mejor_real.json)
%PY% vision\entrenar.py --solo_reales --base vision\modelos\mejor.pt --epocas 40 --salida vision\modelos\mejor_real.pt
%PY% vision\evaluar.py --modelo vision\modelos\mejor_real.pt --datos vision\datos_reales
```
Para que la estación use `mejor_real.pt`, cambia `vision.modelo` en `config/estacion.json` (el .onnx se exporta solo).

### Afinado corto el mismo día de laboratorio (en la laptop con GPU)

La PC del laboratorio no tiene el venv `qdrone2_vision` (torch + GPU). Se captura, etiqueta y afina **en la laptop** y sólo se lleva el modelo a la PC del laboratorio.

1. **Captura desde la laptop:** conéctala a la red de los drones. Si la estación de la PC del lab está conectada al streamer, ciérrala: el streamer atiende un solo cliente. Corre `%PY% vision\capturar_fotos.py --dron <IP_explorador> --cada 0.7 --sesion lab_AAAAMMDD`. Unas 200–300 fotos en ~30 min, más una sesión de validación de ~60–80 (`--sesion lab_val`).
2. **Etiqueta** con `etiquetar.py auto` + `revisar` (arriba). A ~3 s por foto, 300 fotos son ~15–20 min.
3. **Afina partiendo del modelo sintético** (no desde cero), con una salida nueva para no pisar `mejor.pt`:
   ```bat
   REM sintético pequeño + reales (x3), ~20-30 min en la RTX 4060
   %PY% vision\entrenar.py --base vision\modelos\mejor.pt --datos vision\datos\sintetico_smoke --epocas 20 --salida vision\modelos\mejor_lab.pt
   REM o sólo reales, ~10 min
   %PY% vision\entrenar.py --solo_reales --base vision\modelos\mejor.pt --epocas 30 --salida vision\modelos\mejor_lab.pt
   %PY% vision\evaluar.py --modelo vision\modelos\mejor_lab.pt --datos vision\datos_reales
   ```
   Comprobado el 22-nov (sin fotos reales, 2 épocas sobre `sintetico_smoke`, salida en una carpeta temporal): `--base mejor.pt` arranca del modelo actual, la corrida tardó 84 s y exportó `.onnx` y `.onnx.json` junto al `.pt`. Los tiempos de arriba son estimaciones a partir de esa corrida; mídelos.
4. **Criterio para usarlo:** en la validación real (`images/val`), `mejor_lab.pt` debe tener mAP50 y recall **mayores** que `mejor.pt` evaluado igual (`evaluar.py --modelo vision\modelos\mejor.pt --datos vision\datos_reales`). Si no mejora, sigue con `mejor.pt`.
5. **Llévalo a la PC del laboratorio:** `%PY% vision\paquete.py exportar --sin_fotos --salida E:\` (lleva todos los `modelos\*.pt`/`*.onnx`) y allá `python vision\paquete.py importar E:\paquete_…zip`. La UI no edita `vision.modelo`: cámbialo a mano en `config/estacion.json` a `"vision/modelos/mejor_lab.pt"` y reinicia la estación. El detector debe decir `vision/detector.py (mejor_lab.pt)`.

Estructura de `datos_reales/`: `crudas/<sesion>/*.jpg` (sin etiquetar); `images/*.jpg` + `labels/*.txt`
(partición 80/20 fija por md5 del nombre, la misma en entrenar y evaluar); `images/val/` + `labels/val/`
(sesión reservada para validar); `revisadas.txt`.

## 5. Notas técnicas

- **Campo de visión.** La D435i color tiene un VFOV de 42.5°. El HFOV de 69.4° vale **sólo para 16:9**. En 640×480 el sensor recorta los lados y el HFOV real es ~55° (fx ≈ fy ≈ 617 px). Para geolocalizar hay que usar fx/fy: `config/estacion.json → vision.intrinsecos_nominales` (o mejor los reales de `rs2::video_stream_profile::get_intrinsics()`). `docs/ARQUITECTURA.md` §7 ya lo dice así.
- Desde 1.5 m, un fuego (~10 cm) mide ~40 px y una toma ~35–45 px en 640×480. Por eso se usa `imgsz=640` sin reducir la imagen.
- `detector.detect()` devuelve las cajas en píxeles del frame original, ordenadas por confianza.
- Al importar ultralytics, OpenCV queda en un hilo (`cv2.setNumThreads(0)`); el motor OpenCV usa todos los núcleos sólo durante su inferencia.
