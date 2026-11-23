# Visión: YOLO11n para fuegos y tomas clandestinas

Detector de 2 clases para la cámara cenital del dron EXPLORADOR: **0 = fuego**, **1 = toma** (válvula
abrazada al tubo). Se entrenó con datos sintéticos de Blender y se afinó con negativos (fotos sin piezas de COCO
y de webcam) contra falsos positivos; faltan las fotos reales del laboratorio (§4). La estación web sólo carga `vision/modelos/mejor.pt` (o `mejor.onnx`). **No se reentrena** salvo que se pida con `--forzar`.

| Archivo | Qué hace |
|---|---|
| `sintetico/generar_blender.py` | Genera el dataset sintético YOLO con Blender en modo consola |
| `sintetico/ver_etiquetas.py` | Mosaico de imágenes con sus cajas, para revisar etiquetas a ojo |
| `entrenar.py` | Entrena YOLO11n y guarda `modelos/mejor.pt` + `modelos/metricas.json` + `modelos/mejor.onnx` |
| `evaluar.py` | mAP por clase, P/R con la confianza de operación, velocidad (ms/imagen) de cada motor |
| `detector.py` | `Detector(model_path, conf).detect(frame_bgr) -> [(cls, conf, x1, y1, x2, y2), ...]` (lo usa `web/`) |
| `exportar_onnx.py` | `mejor.pt` → `mejor.onnx` (entrada 480×640) para correr sin torch (cv2.dnn) |
| `negativos_coco.py` | Arma las fotos negativas de COCO val2017 (sin piezas, etiqueta vacía) para entrenar y para medir falsos positivos |
| `capturar_fotos.py` | Captura fotos reales desde el stream QDF1 del dron (:18800) o una webcam |
| `etiquetar.py` | Pre-etiqueta fotos reales con el modelo y abre un editor OpenCV para corregirlas |
| `paquete.py` | Lleva modelo + fotos etiquetadas de una PC a otra en un .zip (no van a git) |
| `requirements.txt` | Versiones exactas probadas |
| `instalar_en_web.bat` | OPCIONAL: torch + ultralytics en el venv de la estación (para usar la GPU) |

Los datasets (`datos/`, `%USERPROFILE%\datasets\...`), los pesos (`*.pt`, `*.onnx`), las fotos reales y las corridas (`runs/`)
**no van a git** (`vision/.gitignore`), salvo el modelo vigente `modelos/mejor.pt` + `mejor.onnx` (+ `mejor.onnx.json`).
Las métricas (`modelos/metricas*.json`, `modelos/evaluacion_*.json`) sí, menos las de los candidatos (`modelos/*candidato*`).

## 0. Lo mínimo para el laboratorio (estación web)

1. **Copiar el modelo a la PC del laboratorio.** `mejor.pt`, `mejor.onnx` y `mejor.onnx.json` van en git: con el repo al
   día basta. Si no se puede hacer `git pull`, en la laptop:
   `%PY% vision\paquete.py exportar --sin_fotos --salida E:\` (USB) y en la PC del lab:
   `python vision\paquete.py importar E:\paquete_vision_<fecha>_solo_modelo.zip` (sólo usa la biblioteca
   estándar, sirve cualquier Python 3.10+). También se puede copiar a mano `vision\modelos\mejor.pt`,
   `mejor.onnx` y `mejor.onnx.json`.
2. **Arrancar la estación normal** (`web\iniciar_web.bat`). Su venv (`qd2_web`) **no tiene torch**; en ese caso
   `detector.py` usa solo `mejor.onnx` con OpenCV (cv2.dnn, CPU, mediana ~28 ms por imagen de 640×480 en esta laptop, ver §3) y detecta **fuegos y tomas**.
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

### Resultados del modelo actual `mejor.pt` / `mejor.onnx` (22-nov-2026, afinado con negativos; `.onnx` de 480×640)

- **Qué es:** el modelo sintético de 80 épocas (respaldado como `modelos/mejor_20261122_134604.pt`) **afinado 25 épocas con negativos**: 1 500 fotos de COCO sin piezas y 82 capturas de webcam sin piezas, más 16 capturas de webcam con fuego (las 98 capturas ×3), junto con todo el sintético. Corrida `runs/ft_negativos_v2_20261122`, 44.2 min en la RTX 4060. `mejor.pt` es el **`last.pt` de la época 25**, no el `best.pt`. Comando, datos y por qué: sección **Negativos** (abajo).
- **Evaluación externa** (confianza ≥ 0.5; cuenta fotos con al menos una detección). Cifras con el **motor de la estación** (OpenCV-ONNX en CPU, venv `qd2_web`, con el `mejor.onnx` de entrada 480×640); entre paréntesis, con ultralytics y el `.pt` (son las que guarda `metricas.json → evaluacion_externa`). Las del `.onnx` anterior de 640×640 están en **ONNX a 480×640** (abajo). "Anterior" = `mejor_20261122_134604`.

  | Fotos | Qué se cuenta | Anterior | Actual |
  |---|---|---|---|
  | 500 de COCO apartadas (`coco_negativos_eval`, ninguna se usó al entrenar) | con "toma" falsa | 28.8 % (27.6 %) | **0 %** (0 %) |
  | las mismas 500 | con "fuego" falso | 12.2 % (11.2 %) | **0.2 % = 1/500** (0 %) |
  | 50 capturas de webcam con la cara y sin piezas | con cualquier detección | 44/50 (45/50) | **0/50** (0/50) |
  | 56 capturas de webcam con fuego (de frente, en la mano) | con el fuego detectado (recall) | 50/56 (51/56) | **49/56** (49/56) |
  | las mismas 56 | con "toma" falsa | 41/56 | **0/56** |

- **Ojo, no es la prueba del laboratorio:** son fotos de webcam en interiores, de frente, no la cámara cenital del dron sobre el tapete; lo que decide es `evaluar.py` con fotos del laboratorio (§4). Además no son del todo independientes del entrenamiento:
  - las 50 caras son exactamente la validación del afinado (`datos_reales/images/val`): no dieron gradientes, pero se vigilaron época por época;
  - 3 de las 56 fotos de fuego también están en esa validación;
  - 22 de las 56 son de la sesión `webcam_20261122_120648`, de la que salen las 98 capturas de train (fotogramas a ~2 s entre sí, muy parecidos). Sólo con la otra sesión (`120159`, 34 fotos, ninguna en train): 31 → 30 con ultralytics y con el `.onnx` de 480×640 (con el de 640×640, 30 → 28);
  - las 500 COCO apartadas no coinciden con las de train/val ni por nombre ni por contenido (md5).
- **Fuegos que el actual no detecta** (7 de 56, los mismos con OpenCV 480×640 y con ultralytics; con el `.onnx` de 640×640 eran 9: se perdían además 2 fuegos cortados abajo): 5 son puntas o trozos cortados por el borde de la imagen (4 de ellos tampoco los detectaba el anterior) y **2 son fallos claros**: un fuego grande y bien visible con la base cortada por el borde (`webcam_20261122_120159/t0090.8`, `webcam_20261122_120648/t0074.7`), que el actual no detecta ni a 0.25. Parece que el afinado lo volvió más conservador con fuegos muy cerca y recortados: hay que vigilarlo en el laboratorio.
- **ONNX a 480×640 (hecho el 22-nov-2026).** Con la misma entrada, `.onnx` y `.pt` dan los mismos scores (diferencia ≤ 1.3e-5); lo que cambiaba era el preprocesado: ultralytics infiere una imagen de 640×480 a 640×480 sin relleno (letterbox `rect`) y `detector.py` con el `.onnx` de 640×640 la rellenaba con bandas grises de 80 px arriba y abajo. Ahora `mejor.onnx` se exporta del **mismo `mejor.pt`** con entrada fija **480×640** (alto × ancho, opset 12, sin NMS; es lo que hace `exportar_onnx.py` por defecto) y `_MotorOpenCV` hace el letterbox a la forma del `.onnx`, que lee de sus metadatos o de `mejor.onnx.json → imgsz` (sirve igual con un `.onnx` cuadrado viejo). Medido con el motor OpenCV, mismo criterio que la tabla de arriba (los scripts de la medición no están en el repo):

  | | `.onnx` 640×640 (antes) | `.onnx` 480×640 (ahora) |
  |---|---|---|
  | 56 capturas con fuego: recall a conf ≥ 0.5 | 47/56 | **49/56** (= ultralytics) |
  | 500 COCO: fotos con "fuego" / "toma" falsos | 2/500 / 0/500 | **1/500** / 0/500 |
  | 50 capturas sin piezas: con alguna detección | 0/50 | 0/50 |
  | 1 456 val sintéticas (1 390 `sintetico_10k` + 66 `sintetico_smoke`): mAP50 (conf 0.001, NMS 0.7, IoU 0.5) | 0.9944 | **0.9946** (ultralytics: 0.9946) |
  | las mismas, P / R a conf 0.5, fuego | 0.985 / 0.987 | 0.981 / 0.987 (ultralytics: igual) |
  | las mismas, P / R a conf 0.5, toma | 0.988 / 0.985 | 0.984 / 0.989 (ultralytics: 0.987 / 0.988) |
  | `Detector.detect()` en CPU, 100 capturas 640×480 ×4, mediana (p90) | 38.8 ms (51.9) | **28.3 ms (35.1)** |
  | sólo la red, 1 hilo, mediana | 138 ms | 101 ms |
  | cajas iguales a ultralytics CPU (conf ≥ 0.25, ±1 px), 106 capturas 640×480 | 3/60 (mediana 11 px) | **60/60** (0 px, misma confianza) |
  | ídem, 200 val sintéticas (640×480 y 1280×720) | 262/283 | 282/283 (máx. 2 px) |
  | las 56 capturas con fuego **giradas 90°** (480×640 vertical) | 45/56 | **41/56** |

  - En la sintética a conf 0.5 hay unos pocos falsos más (fuego 16 → 21, toma 12 → 15 en 1 456 imágenes), los mismos que da ultralytics: 10 de los 11 que sólo da el nuevo son tiras de 5–11 px de alto pegadas al borde de la imagen (piezas cortadas por el borde que el generador no etiqueta); con el relleno de 640×640 ese borde quedaba junto a la banda gris. En COCO y en las capturas sin piezas no hay más falsos.
  - **Imágenes verticales** (480 de ancho × 640 de alto, p. ej. si el dron gira la imagen con `--rotar 90`): el letterbox las reduce a 360×480 (escala 0.75) con bandas de 140 px a los lados. Funcionan, pero los objetos quedan 25 % más chicos y el recall bajó de 45/56 a 41/56 en la prueba de arriba. Por eso conviene no girar la imagen en el dron y declarar el giro en `camara.rotacion_montaje` (lo recomendado en `web/README.md`); si hace falta girarla ahí, reexporta cuadrado con `%PY% vision\exportar_onnx.py --imgsz 640` (`detector.py` toma la forma del `.onnx` solo).
  - El `.onnx` anterior (mismo `.pt`, 640×640) quedó como `modelos/mejor_640x640_20261122.onnx` (no va a git).
  - Un `detector.py` anterior a este cambio no carga bien el `.onnx` de 480×640 (forzaba 640×640): el `.onnx` y el código van juntos en git, así que el `.onnx` de 480×640 exige actualizar **todo** el repositorio (`git pull`), no solo copiar el modelo. Con un `detector.py` viejo la estación no falla en silencio: OpenCV da un error de `reshape` y cae al FOMO con aviso rojo. El aviso de `paquete.py importar` (y de `preflight_lab.py`) solo cubre el caso raro de un `paquete.py` nuevo junto a un `detector.py` viejo; un repositorio viejo corre su `paquete.py` viejo, que no avisa.
- **Validación del entrenamiento** (`modelos/metricas.json`, ultralytics, val completo de 1 609 imágenes: 1 456 sintéticas + 100 COCO + 53 webcam): mAP50 = 0.9945, mAP50-95 = 0.9375 (P = 0.991, R = 0.984); fuego AP50 = 0.9945, toma AP50 = 0.9946. Sólo las 1 456 sintéticas, con OpenCV: mAP50 0.9946 (anterior) → 0.9946 (actual, `.onnx` 480×640; 0.9944 con el de 640×640), es decir, el afinado no empeoró lo sintético. Esta validación está saturada y no mide falsos positivos: para eso está la tabla de arriba.
- **Velocidad:** misma arquitectura que el anterior. `Detector.detect()` con OpenCV-ONNX (480×640) en CPU, 640×480: mediana 28 ms con 16 hilos (~35 FPS); sólo la red con 1 hilo, 101 ms (con el `.onnx` de 640×640, 39 ms y 138 ms medidos en la misma corrida). Las cifras de ultralytics GPU/CPU son las del modelo de `sintetico_smoke` (abajo; misma arquitectura).
- `mejor.onnx.json` guarda el md5 del `mejor.pt` del que salió (`92e8dec0…`) y la forma de entrada (`"imgsz": [480, 640]`, alto × ancho); `detector.py` avisa si el `.onnx` quedó desactualizado.
- **Modelos anteriores** (los pesos sólo están en esta laptop, no van a git; las métricas sí):
  - `modelos/mejor_20261122_134604.pt` / `.onnx` + `metricas_20261122_134604.json` (el que se afinó): `entrenar.py --forzar --datos C:\Users\ezequ\datasets\qdrone2_vision\sintetico_10k vision\datos\sintetico_smoke --epocas 80`, corrida `runs/yolo11n_20261122_0545`, desde `yolo11n.pt` de COCO, 111.6 min. Train: 8 610 + 534 imágenes sintéticas; val: 1 390 + 66. **0 fotos reales.** Validación sintética: mAP50 = 0.995, mAP50-95 = 0.937 (P = 0.992, R = 0.985). Con OpenCV-ONNX (conf 0.5, IoU ≥ 0.5, 200 imágenes val de `sintetico_10k`): P = 0.977, R = 0.992. Con la webcam confundía caras y objetos con "toma" (tabla de arriba).
  - `modelos/mejor_20261122_073731.pt` + `metricas_20261122_073731.json`: sólo `sintetico_smoke` (600 imágenes, 30 épocas, 3.7 min), mAP50 = 0.960 sobre 66 imágenes val. Velocidad medida entonces (`modelos/evaluacion_sintetico_smoke.json`, 640×480, GPU compartida con Blender): ultralytics GPU media 21–26 ms (17–19 ms con la GPU libre); ultralytics CPU (Ryzen 9 8945HS) media 39 ms; OpenCV-ONNX CPU media 32–43 ms.
- **Dataset grande:** 10 000 imágenes (semilla 202) en `C:\Users\ezequ\datasets\qdrone2_vision\sintetico_10k` (fuera de OneDrive), log en `...\sintetico_10k_log2.txt`. Para reanudarlo, el mismo comando:
  ```bat
  "C:\Program Files\Blender Foundation\Blender 5.3\blender.exe" -b -P vision\sintetico\generar_blender.py -- --salida C:\Users\ezequ\datasets\qdrone2_vision\sintetico_10k --n 10000 --vistas 6 --semilla 202
  ```

## 4. Fotos reales en el laboratorio (lo más importante para que funcione de verdad)

El modelo actual (sintético + negativos de COCO y webcam) es un punto de partida. El salto sintético → real se cierra con **unas 300–500 fotos reales etiquetadas**.

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

## Falsos positivos: filtros de la estación

Con la webcam el modelo sólo-sintético confundía caras y personas con "toma" (conf 0.5–0.9). El modelo actual se
reentrenó con negativos (sección siguiente) y en las capturas de webcam ya no lo hace, pero la estación
(`web/app/vision_service.py`) sigue filtrando por física, sin tocar el modelo, como segunda barrera:

- **Tamaño real:** proyecta al piso los puntos medios de los lados de la caja y descarta lo que mida más de
  `vision.tam_max_m` (0.35 m). Una pieza mide 0.07–0.14 m de base (~40 px a 1.5 m), pero mide más en el piso en las orillas de la
  imagen por su altura (0.125–0.137 m): hasta ~0.24 m con caja holgada; una cara, varios decímetros.
- **Toma cerca del tubo:** descarta las tomas a más de `vision.dist_max_tubo_toma_m` (0.25 m) de la polilínea
  `pista.vertices`. Para fuegos está desactivado (`vision.dist_max_tubo_fuego_m = null`).
- **Inclinación del explorador:** la proyección al piso usa el roll y pitch de OptiTrack (y(61:63) de la telemetría).
  Si el dron va más inclinado que `vision.inclinacion_max_deg` (15°), la detección se descarta. Sin actitud (modelo de
  Simulink viejo) se proyecta suponiendo el dron nivelado, como antes.
- Las descartadas no llegan al cluster; en la cámara de la estación salen en gris con el motivo.

Esto **no sustituye** entrenar con negativas: sólo funciona con la cámara cenital del dron y la pose de
OptiTrack (con `detector.py --camara 0` no hay filtro). Detalle y parámetros: `web/README.md`, "Filtros físicos de la estación".

## Negativos: cómo se reentrenó contra falsos positivos

El modelo sólo-sintético nunca había visto caras, personas ni cuartos: con la webcam marcaba caras y objetos como
"toma" (conf 0.5–0.9) y a veces como "fuego". Se afinó con fotos **sin piezas y con etiqueta vacía** ("aquí no hay
nada"). Pasos para repetirlo (22-nov-2026, laptop RTX 4060; resultados en §3, "Resultados del modelo actual"):

1. **Negativos de COCO** (no van a git; viven en `%USERPROFILE%\datasets\qdrone2_vision`). Descarga de
   images.cocodataset.org `zips/val2017.zip` (~0.8 GB) y `annotations/annotations_trainval2017.zip`, y descomprímelos en
   `%USERPROFILE%\datasets\coco` (quedan `val2017\` y `annotations\instances_val2017.json`). Luego:
   ```bat
   %PY% vision\negativos_coco.py
   REM sin copiar nada: la selección debe ser igual a coco_negativos\seleccion.json
   %PY% vision\negativos_coco.py --comprobar
   ```
   Crea `coco_negativos\images\train` (1 500 = 1 000 con personas + 500 sin) y `images\val` (70 + 30), con etiquetas
   vacías, y `coco_negativos_eval\images` (300 + 200), que **no se entrena**: sirve para medir falsos positivos. La
   selección es fija (orden por md5 del nombre) y deja fuera las fotos con "fire hydrant" (parece una válvula). Sólo usa la
   biblioteca estándar; `--coco`, `--salida` y `--salida_eval` cambian las carpetas.
2. **Capturas de webcam** (no van a git; se mueven con `paquete.py`). Se tomaron con `detector.py --camara 0` (quedan en
   `vision\runs\detector\webcam_<fecha>\`): 204 fotos en dos sesiones, con la cara y el cuarto, con y sin un fuego impreso
   en la mano. Se pre-etiquetaron con el modelo anterior (conf 0.25) y se quedaron las 151 en las que coincidieron los dos
   revisores (132 sin piezas y 19 con fuego; 53 descartadas por duda):
   - `datos_reales\images\train`: sesión `webcam_20261122_120648`, 98 fotos (82 sin piezas + 16 con fuego);
   - `datos_reales\images\val`: sesión `webcam_20261122_120159`, 53 fotos (50 sin piezas + 3 con fuego).

   Todas están en `revisadas.txt`, así que `entrenar.py` las mezcla solas (×3). Detalle en
   `datos_reales\webcam_20261122_LEEME.txt`.

   > **Aviso: estas 151 capturas NO las revisó una persona.** Las revisaron dos revisores automáticos independientes
   > (Claude, mirando cada imagen) y sólo se guardaron las fotos en las que los dos coincidieron. Antes de confiar en
   > ellas, revísalas a mano con el editor de §4:
   > ```bat
   > REM las 151 (train y val)
   > %PY% vision\etiquetar.py revisar
   > REM sólo la validación
   > %PY% vision\etiquetar.py revisar --carpeta vision\datos_reales\images\val
   > ```
   > Como ya están todas marcadas, el editor abre en la primera: `d`/ESPACIO guarda y pasa a la siguiente, `a` regresa,
   > `x` deja la foto sin cajas (negativo). Una foto que no sirva se borra de `images\` y `labels\` (o se quita de
   > `revisadas.txt` para que no se use).
3. **Afinar** partiendo del modelo sintético, con una salida nueva para no pisar `mejor.pt` (~44 min):
   ```bat
   %PY% vision/entrenar.py --base vision/modelos/mejor.pt --datos C:/Users/ezequ/datasets/qdrone2_vision/sintetico_10k vision/datos/sintetico_smoke C:/Users/ezequ/datasets/qdrone2_vision/coco_negativos --epocas 25 --paciencia 100 --lr0 0.002 --warmup 1 --guardar_cada 1 --nombre ft_negativos_v2_20261122 --salida vision/modelos/candidato_negativos_v2.pt
   ```
   - Se mezcla todo el sintético para no olvidar las piezas; las capturas de `datos_reales` entran solas.
   - `--lr0 0.002` (tasa baja, de afinado) hace que `entrenar.py` use SGD: con `optimizer=auto`, ultralytics ignora `lr0`.
     `--warmup 1` = 1 época de calentamiento.
   - `--paciencia 100` = sin parada temprana; `--guardar_cada 1` deja `weights\epoch<N>.pt` para comparar épocas.
   - Ojo: con SGD, ultralytics deja `warmup_bias_lr = 0.1` (50 veces `lr0`) para los sesgos durante la época de
     calentamiento. `mejor.pt` se entrenó así; para el siguiente afinado conviene 0.0 (`entrenar.py` aún no tiene opción
     para eso). Desde ahora `metricas*.json` guarda lo que de verdad usó ultralytics en la clave `optimizador`.
   - Una primera corrida (`candidato_negativos.pt`, `runs/ft_negativos_20261122`) no se usó.
4. **Elegir el checkpoint.** `entrenar.py` copia `best.pt` a `--salida` (`candidato_negativos_v2.pt`), pero `best.pt` lo
   elige ultralytics por el mAP de la validación, que en este afinado estuvo saturado (mAP50 entre 0.9942 y 0.9947 en las
   25 épocas) y no mide falsos positivos. Se usó **`last.pt` (época 25)**. Para ponerlo como modelo vigente (a mano):
   ```bat
   REM respaldo: mejor.pt, mejor.onnx y metricas.json -> mejor_<fecha>.pt, mejor_<fecha>.onnx, metricas_<fecha>.json
   copy vision\runs\ft_negativos_v2_20261122\weights\last.pt vision\modelos\mejor.pt
   REM nuevo mejor.onnx + mejor.onnx.json con el md5 del .pt
   %PY% vision\exportar_onnx.py
   ```
   `metricas.json` se rehízo validando `mejor.pt` sobre `runs\ft_negativos_v2_20261122\data_combinada.yaml`, y se le
   agregaron a mano `checkpoint`, `optimizador`, `negativos` y `evaluacion_externa`.
5. **Evaluación externa.** Fotos con alguna detección a conf ≥ 0.5, con el motor de la estación (desde la raíz del repo;
   el 1.er argumento es la carpeta y el 2.º, opcional, el modelo):
   ```bat
   %USERPROFILE%\.venvs\qd2_web\Scripts\python.exe -c "import glob,sys,cv2; sys.path.insert(0,'.'); from vision.detector import Detector; d=Detector(sys.argv[2] if len(sys.argv)>2 else 'vision/modelos/mejor.pt',conf=0.5); fs=sorted(glob.glob(sys.argv[1]+'/*.jpg')); r=[{int(x[0]) for x in d.detect(cv2.imread(f))} for f in fs]; print(len(fs),'fotos | con fuego:',sum(0 in s for s in r),'| con toma:',sum(1 in s for s in r))" %USERPROFILE%\datasets\qdrone2_vision\coco_negativos_eval\images
   ```
   Da `500 fotos | con fuego: 2 | con toma: 0` (con `vision/modelos/mejor_20261122_134604.pt` como 2.º argumento: 61 y
   144). Con `vision\datos_reales\images\val`: 53 fotos, 3 con fuego (las 3 que tienen fuego) y 0 con toma (el anterior,
   47 con toma). Las 56 fotos de fuego son las capturas de `vision\runs\detector\webcam_20261122_*` con fuego visible
   según los dos revisores que no entraron al train (3 de val + descartadas por duda); la lista no está en el repo.

Para el próximo afinado (p. ej. con fotos del laboratorio, §4) conviene incluir `coco_negativos` en `--datos`: afinando
sólo con fotos reales se puede perder lo aprendido contra falsos positivos.

## 5. Notas técnicas

- **Campo de visión.** La D435i color tiene un VFOV de 42.5°. El HFOV de 69.4° vale **sólo para 16:9**. En 640×480 el sensor recorta los lados y el HFOV real es ~55° (fx ≈ fy ≈ 617 px). Para geolocalizar hay que usar fx/fy: `config/estacion.json → vision.intrinsecos_nominales` (o mejor los reales de `rs2::video_stream_profile::get_intrinsics()`). `docs/ARQUITECTURA.md` §7 ya lo dice así.
- Desde 1.5 m, un fuego (~10 cm) mide ~40 px y una toma ~35–45 px en 640×480. Por eso se usa `imgsz=640` sin reducir la imagen: `mejor.onnx` tiene entrada 480×640 (alto × ancho), así que un cuadro de 640×480 entra a la red tal cual, sin escalar ni rellenar. Otros tamaños se ajustan con letterbox (escala sin deformar + gris 114 centrado, como ultralytics): 1280×720 → 640×360 con bandas de 60 px; vertical 480×640 → 360×480 (ver §3).
- `detector.detect()` devuelve las cajas en píxeles del frame original, ordenadas por confianza.
- Al importar ultralytics, OpenCV queda en un hilo (`cv2.setNumThreads(0)`); el motor OpenCV usa todos los núcleos sólo durante su inferencia.
