# Piezas impresas: fuegos y tomas clandestinas

Impresora: Creality K2 Plus, PLA, boquilla 0.4 mm. Las piezas se diseñaron para imprimirse **sin soportes**.
Todas las piezas pasaron una revisión independiente: piezas watertight, sin islas flotantes, ajustes y cinemática verificados por cálculo.

## Lo más rápido: abrir las placas en Creality Print
En `placas/` hay archivos 3MF con las piezas ya acomodadas y en su orientación de impresión. Cada placa produce **3 juegos**.

| Placa | Filamento | Contenido | Ajustes |
|---|---|---|---|
| `placa1_ROJO_fuegos_base.3mf` | Rojo | Bases con lenguas de fuego A, B y C | Capa 0.2, 3 perímetros, gyroid 15 %, sin brim, tiempo mínimo de capa 8 s |
| `placa2_AMARILLO_fuegos_flama.3mf` | Amarillo | Flamas centrales A, B y C | Capa 0.2 (0.16 para puntas más finas), 3 perímetros, gyroid 15 %, **brim 5–8 mm** |
| `placa3_ROJO_valvulas_cuerpo_tuerca_zapata.3mf` | Rojo | 3 cuerpos, 3 tuercas, 3 zapatas con balero y 1 probeta de calibración | Cuerpos: capa 0.2, 4 perímetros, gyroid 25 %, brim 5 mm. Tuercas: idealmente capa 0.12–0.16 y 100 % de relleno (ajústalo por objeto) |
| `placa4_ROJO_valvulas_vastago_volante.3mf` | Rojo | 3 vástagos con volante (llevan rosca M16) | **Capa 0.12–0.16**, 4 perímetros, 30 %, **brim 5–8 mm** |

Las piezas sueltas están en `stl/`, para imprimir más o rearmar las placas. Las versiones editables están en `step/` (se abren en FreeCAD o SolidWorks), y los scripts paramétricos en `cad/` (`fuegos.py`, `valvula.py`, `soporte_camara.py`).

Si solo tienes un color para los fuegos, usa `stl/fuego_{A,B,C}_monocolor.stl`.

## Antes de las 3 válvulas: imprime la probeta
`valvula_probeta_perno` viene en la placa 3. Tiene pernos de Ø8.0, 8.1 y 8.2 mm, marcados con 1, 2 y 3 muescas.
1. Prueba en cuál entra el balero 608-2ZZ **a presión firme**. El diseño usa 8.1.
2. Si el bueno es otro, cambia `PERNO_D` en `cad/valvula.py` y vuelve a generar el vástago con la venv de CAD:

   ```powershell
   & "$env:USERPROFILE\.venvs\qdrone2_cad\Scripts\python.exe" cad\valvula.py
   ```

3. Conviene probar también **1 tuerca y 1 vástago** antes de imprimir las 3 copias, porque la holgura de la rosca depende de la calibración de la impresora.

## Armado del fuego
1. Quita el brim de la flama amarilla.
2. Alinea el plano de la "D" del perno rojo con el de la cuenca amarilla. Empuja en recto hasta que el pedestal amarillo asiente sobre la base. No gira.
3. Opcional: una gota de cianoacrilato.

## Armado de la válvula (toma clandestina)
Usa **1 balero 608-2ZZ por válvula**. También existe `valvula_zapata_sin_balero.stl` como alternativa sin balero.
1. **Tuerca:** métela de lado por la ventana de la cara +X del brazo superior, hasta que tope con el fondo hexagonal.
2. **Vástago:** entra por arriba, por el bonete. Gira en sentido horario hasta que el perno asome dentro de la C. Ahora la tuerca ya no puede salirse.
3. **Balero:** presiónalo en la zapata empujando **solo la pista exterior**, por ejemplo con una prensa de banco.
4. **Zapata:** coloca un tramo de tubo en la C y la zapata encima, con la cara cóncava hacia abajo. Aprieta suave: la rosca mete el perno en el balero.
5. **Montaje en la pista:**
   1. Afloja 6 vueltas (antihorario).
   2. Levanta el tubo unos 6 mm, sostenlo y desliza la C de lado sobre él.
   3. Aprieta horario hasta el contacto y luego **máximo ¼–½ vuelta con dos dedos**. Por arriba de ~1.3 N·m la mandíbula de PLA puede agrietarse.
6. **Para quitarla:** 6 vueltas antihorario, levanta el tubo y deslízala.

## Medidas útiles para la visión
- **Fuegos:** ~96 × 90–97 mm de base y 96 / 112 / 125 mm de alto (C / A / B). Desde arriba se ven como una estrella roja con centro amarillo.
- **Válvula:** volante de Ø66 mm (5 rayos) y 137 mm de altura total sobre el piso.

## Soporte de la cámara (RealSense hacia abajo)
Script: `cad/soporte_camara.py`. El soporte hace que la RealSense D435/D435i del QDrone 2 mire **hacia abajo** (nadir) para que el explorador use la imagen de **color**. No se taladra ni se modifica el chasis.

**Concepto.** Si la cámara se gira en su lugar, el travesaño frontal y la placa inferior le tapan la vista. Por eso el soporte la **adelanta 42 mm**, hasta el hueco entre los dos aros delanteros, justo delante del borde del travesaño. Ahí la gira 90° sobre su eje largo:
- lentes hacia abajo,
- cara de la rejilla hacia **adelante**: arriba de la imagen queda hacia adelante del dron,
- USB-C a la **derecha** del dron, como viene de fábrica.

La cámara se sujeta solo con sus **2 roscas M3 traseras**, como recomienda Intel. Se atornilla a una "cuna" plana de 4 mm. La cuna tiene una ventana en forma de **flecha que apunta hacia adelante**, que también sirve para ventilar, y una **muesca en V del lado del USB**.

Hay dos formas de sujetar la cuna al dron:

| Pieza (`stl/`) | Cuándo | Cómo se sujeta | Orientación de impresión | Masa (PETG, 40 %) |
|---|---|---|---|---|
| `soporte_camara_A_adaptador` | **Preferida.** Si la escuadra original sujeta la cámara por sus 2 M3 traseros (lo probable según las fotos de Quanser) | Se atornilla en los **mismos 2 agujeros** donde iba la cámara (45 mm entre centros), con 2 tuercas M3 cautivas en la placa de interfaz. Un brazo con 4 nervios lleva la cuna hacia adelante por debajo del travesaño, y 2 costillas sobre la cuna (delante del travesaño) la rigidizan. La placa y los nervios quedan 1.5 mm por encima de la lengüeta de la escuadra original. No hace falta medir el chasis | **De pie**, sobre la cara trasera de la placa de interfaz (el STL ya viene así). Los alojamientos de tuerca quedan abiertos hacia arriba, sin puente. **Brim de 5 mm** (mide 67 mm de alto) | ~32 g |
| `soporte_camara_B_abrazadera` | Si la escuadra original **no** usa los M3 traseros (por ejemplo, si toma la rosca 1/4-20 de abajo) | Una "C" abraza el travesaño frontal de la placa principal: la cuna se apoya debajo y 2 tornillos M3 **de nylon** con tuerca cautiva aprietan desde arriba. Las tuercas entran por una ranura desde la cara trasera de la mandíbula. Respaldo opcional: 2 ranuras para cinchos | De lado, con el lado del USB (y = −30) sobre la cama. **Brim de 5 mm** (mide 60 mm de alto) | ~22 g |
| `soporte_camara_probeta` | **Imprimir primero** (~30 min) | Patrón de 45 mm de la cuna (2 × Ø3.4, mismo espesor de 4 mm). Fila de 3 alojamientos de tuerca con **eje vertical** como en A (entrecaras 5.6 / 5.7 / 5.8). Pared con 3 alojamientos de **eje horizontal y techo puenteado** como en B (5.6 / 5.8 / 6.0). Peine de 8 ranuras de **1.9 a 2.6 mm** en pasos de 0.1. En todo, el valor menor queda del lado de la esquina achaflanada | Plana | ~15 g |

Las versiones editables están en `step/` (`soporte_camara_*.step`, más los ensambles `soporte_camara_{A,B}_ensamble_uso.step`). Los renders están en `renders/soporte_camara_*.png`.

**Impresión.** Usa **PETG o ASA** para la pieza final: la cara trasera de la D435 puede llegar a 50 °C, y el PLA se ablanda a 55–60 °C y fluye bajo la cabeza de los tornillos. El PLA sirve solo para probar el ajuste. Ajustes: capa de 0.2 mm, 4 perímetros, 5 capas arriba y abajo, gyroid al 40 %, **sin soportes**. Las comprobaciones del script dan, para las tres piezas: watertight, 1 cuerpo, 0 islas y ningún voladizo de más de 45°. A no tiene ningún puente. Los únicos puentes de B son los techos de las ranuras de tuerca (5 mm) y de las ranuras para cinchos (2.5 mm); los de la probeta, los techos de su pared de tuercas (3.3 mm).

### Tornillería
| Uso | Cantidad | Tornillo | Nota |
|---|---|---|---|
| Cámara a la cuna (A y B) | 2 | **M3×6 ISO 7380-2** (botón con brida, Ø6.9), sin arandela | Atraviesan 4 mm de cuna y entran **2.0 mm** en la cámara (máximo de Intel: 3 mm). **No uses M3×8**: entraría 4 mm y puede dañar la electrónica. Par **0.15 N·m** (dos dedos en una llave Allen corta): ~250 N de precarga y ~9 MPa bajo la brida. Los 0.4 N·m de Intel son para un soporte metálico: sobre plástico darían ~670 N y hasta ~45 MPa con cabeza DIN 912, y el plástico fluye a 50 °C |
| A: adaptador a la escuadra original | 2 + 2 tuercas M3 (DIN 934) | M3 de largo = **espesor de la escuadra + 6 a 12 mm** (con una escuadra de 3 mm: M3×10 o M3×12) | Los tornillos originales no alcanzan: miden escuadra + 3 mm como máximo |
| B: apriete sobre el travesaño | 2 + 2 tuercas M3 | **M3×8 de nylon (PA66)** | Par **≤ 0.1 N·m** (con los dedos, ~170 N). Con acero y 0.4 N·m, la punta apretaría el carbono a ~200 MPa. La cabeza queda ~1.6 mm levantada |
| B: respaldo (opcional) | 2 | Cinchos de hasta 4.8 mm de ancho | Ranuras de 5.5 × 2.5 mm con 3 mm de pared trasera. Cierran la boca de la C por detrás del travesaño, a través de las ventanas del cuello en Y |

No uses fijador de roscas en contacto con el plástico. Revisa el apriete después del primer vuelo y antes de cada sesión.

### Qué medir en el laboratorio (antes de imprimir la pieza final)
Todos estos valores son parámetros con nombre al inicio de `cad/soporte_camara.py` y están marcados `MEDIR EN LAB`. Los valores por defecto salen de las fotos y los manuales de Quanser y del datasheet de la D400. Si un valor cambia, edítalo y regenera.

| Parámetro | Valor por defecto | Cómo medir | Afecta a |
|---|---|---|---|
| `DX_TRAVESANO` | 12.0 mm | Cuánto sobresale el borde delantero del travesaño (placa superior) por delante del vidrio de la cámara de fábrica, en el centro | Cuánto se adelanta la cámara (A y B) |
| `DX_TRAVESANO_INF` | 12.0 mm (igual que el superior, caso conservador) | Lo mismo para la placa **inferior**. Si la placa inferior no tiene travesaño en el frente, mejor | Margen de los conos de profundidad |
| `W_TRAVESANO` | 19.0 mm | Ancho del travesaño en x | Profundidad de la boca de B |
| `T_CARBONO` | 2.0 mm | Espesor de la placa, **con calibrador** | Boca de B |
| `Z_EJE_ORIG` | 31.5 mm (estimado 28–35) | Altura sobre el piso de la línea media de la cámara de fábrica, con el dron apoyado | Margen al piso y z del offset |
| `X_CARA_TRASERA_ORIG` | 157.5 mm | Distancia en x del centro del dron (entre ejes de motores) a la cara trasera de la cámara | x del offset (solo cálculo y renders) |
| `LENG_ANCHO` / `LENG_LARGO` / `LENG_T` / `LENG_LABIO` | 36 / 25 / 3 / 2 mm | La lengüeta negra de la escuadra original bajo la cámara: ancho, largo, espesor y si tiene un labio que sube delante del vidrio | Holgura de la placa de interfaz y de los nervios de A |
| `R_ARO_EXT` / `R_ARO_INT` | 119 / 107 mm | Radios del aro de protección. El esquema los centra en los motores (x = ±101.6, y = ±127 mm) | Comprobaciones de visión, del enchufe y del cable |
| `BOLSILLO_FRENTE` | 20 mm | Fondo del hueco entre los aros delanteros, desde el borde del travesaño, en el eje | Cuánto sobresale la cámara por delante |
| `Y_POSTE` | ±71 mm | \|y\| de los postes en los extremos del travesaño | Distancia al enchufe |
| `L_ENCHUFE`, `H_CODO_ENCHUFE` | 20 mm, 8 mm | Cuánto sobresale el enchufe USB-C en ángulo recto más allá del extremo de la cámara, y cuánto sobresale su codo por detrás de la cara trasera | Espacio del enchufe y largo del cable |
| `R_CURVA_CABLE`, `D_CABLE` | 30 mm, 4.6 mm | Radio mínimo de curvatura y diámetro del cable USB | Ruta y largo extra del cable |
| `Z_CG_DRON` | 60 mm | Altura del centro de masa | Solo el reporte del centro de masa |

Antes de desarmar, verifica también:
1. Que la escuadra original use los 2 M3 traseros (si no, usa B).
2. Qué espesor tiene la escuadra y de qué largo son sus tornillos.
3. Que el cable USB tenga **al menos 90 mm de holgura**. La salida del cable queda 62.5 mm más adelante y gira 90° (de hacia atrás a hacia arriba). Con un radio de curvatura de 30 mm, el script calcula **80 mm** de largo extra mínimo; súmale ~10 mm de margen. Si no alcanza, usa una extensión USB-C corta y certificada para USB 3.1 Gen 1. Nunca dejes el cable tenso sobre el conector de la cámara.

Toma fotos del frente, de un lado y de abajo.

### Armado paso a paso
1. **Probeta.**
   1. Atornilla la cámara a los 2 agujeros sueltos (patrón de 45 mm) con 2 M3×6. Así compruebas los 45 mm y que el tornillo no toca fondo.
   2. Prueba una tuerca M3 en cada alojamiento de la **fila** (eje vertical) y pon en `TUERCA_EC_A` el valor del que la sujeta firme sin forzar.
   3. Haz lo mismo en la **pared** (eje horizontal, techo puenteado) y pon ese valor en `TUERCA_EC_B`. Sirve para B, cuyas tuercas tienen ese mismo eje.
   4. Mide la placa de carbono con calibrador y escribe el valor en `T_CARBONO`. Con el peine, comprueba que la placa entra sin forzar en la ranura de `T_CARBONO + 0.4` mm (la boca de B). Si solo entra en una más ancha, sube `HOLG_BOCA`.
2. Desconecta el USB-C y quita los 2 tornillos traseros de la cámara. Retírala.
3. **Variante A:**
   1. Mete las 2 tuercas M3 en los alojamientos hexagonales de la cara delantera de la placa de interfaz (entre los nervios). Un poco de cinta evita que se caigan.
   2. Apoya la placa de interfaz donde estaba la cara trasera de la cámara, con el brazo hacia adelante por debajo del travesaño. La placa y los nervios no deben tocar la lengüeta de abajo (holgura de diseño: 1.5 mm).
   3. Atornilla desde atrás de la escuadra con los M3 largos.
4. **Variante B:**
   1. Desliza las 2 tuercas M3 por las ranuras de la cara trasera de la mandíbula superior hasta los alojamientos, y enrosca un poco los M3×8 de nylon desde arriba.
   2. Desliza la C sobre el travesaño desde el frente hasta que la espina toque el borde.
   3. Aprieta los 2 tornillos con los dedos (≤ 0.1 N·m).
   4. Si quieres respaldo, pasa los cinchos por las ranuras y por las ventanas del cuello.
5. **Cámara a la cuna** con 2 M3×6 ISO 7380-2 (0.15 N·m):
   - lentes **hacia abajo**,
   - cara de la rejilla (la contraria a la rosca 1/4-20) **hacia adelante**, hacia donde apunta la flecha,
   - extremo del **USB del lado de la muesca en V** (derecha del dron).

   Si la montas al revés, la imagen sale girada 180°.
6. Reconecta el USB-C con el cable hacia arriba. Llévalo hacia atrás sobre la placa principal, sin tensión (radio ≥ 30 mm), y sujétalo con un cincho flojo. El cable no debe entrar en el disco de las hélices ni rozar bordes de carbono.
7. Con el dron apoyado en el piso, revisa:
   - holgura de la cara de las lentes al piso (≥ 10 mm; por cálculo, ~14.5 mm),
   - que las hélices giren a mano sin tocar nada,
   - que la cámara no se mueva al empujarla con un dedo.
8. En la UI, con `rotacion_montaje = 0`, arriba de la imagen debe quedar hacia adelante del dron (checklist, etapa 5).

### Regenerar
```powershell
& "$env:USERPROFILE\.venvs\qdrone2_cad\Scripts\python.exe" cad\soporte_camara.py
```
El script escribe `stl/soporte_camara_*.stl` (en orientación de impresión), `step/soporte_camara_*.step` y las mallas de ensamble en `renders/_mallas_uso/`. También imprime las comprobaciones: orientación (`rotacion_montaje`), conos de visión, piso, enchufe y cable, interferencias, cobertura de la carcasa, largo extra del cable, masa y centro de masa, modos de vibración, tornillería e imprimibilidad capa a capa.

Los renders se hacen con `tools/render_stl.py` (Blender headless), con las mallas `renders/_mallas_uso/soporte_camara_*.stl`. Esas mallas están en el marco del cuerpo del dron (mm, origen en el centro y z = 0 en el piso).

### Resultados (con los valores por defecto)
- **Conos de visión** (+1.5° de tolerancia por lado, desde la cara de las lentes hacia abajo). El esquema del dron tiene los aros centrados en los motores:
  - el soporte queda por completo arriba del plano de las lentes;
  - **RGB** (69.4° × 42.5°): libre, con +5.1 mm de margen a la placa inferior en A (+4.8 en B) y +20 mm o más a los postes;
  - **Profundidad** (87° × 58°): la IR izquierda queda libre por +1.0 mm, pero la IR derecha roza ~1 mm el borde supuesto del travesaño de la placa inferior. Lo mismo el **proyector IR** (90° × 63°, ~1.2 mm). No afecta al color.
- **Vuelo por delante:** la cara delantera de la cámara queda en x = 224.6 mm. Sobresale **+3.3 mm** del largo total del dron (442.58 / 2) y **+10 mm** si el bolsillo entre los aros mide 20 mm, como sugieren las fotos. En un choque frontal la cámara pega primero. Con `HOLG_TRAV = 3` sobresaldría 1.3 mm del largo total; para no sobresalir haría falta `HOLG_TRAV ≤ 1.7`, y entonces los conos de profundidad quedarían tapados.
- **Piso:** la cara de las lentes queda a **14.5 mm** del piso en A y a 15.0 mm en B. Con `Z_EJE_ORIG` entre 28 y 35 mm queda entre 10.9 y 17.9 mm. No aterrices sobre el tubo de PVC (26.7 mm de alto).
- **Enchufe y cable USB-C:** sin interferencias con el soporte ni con el esquema del chasis. El enchufe queda a 4.2 mm del aro de la placa principal y a 7 mm del poste. El cable (radio 30 mm) pasa a 5.5 mm de la placa principal. El borde del soporte queda a 15 mm del extremo del USB.
- **Disipación:** el soporte toca el 57 % de la cara trasera y el 13 % de toda la carcasa. Las caras con rejilla (adelante y atrás) quedan libres, y la ventana deja pasar el aire de las hélices.
- **Masa y centro de masa:** A ~32 g (PETG al 40 %; 41 g maciza), B ~22 g. La masa de vuelo sube de 1504 g a ~1536 g (+2.2 %). La cámara (75 g) se adelanta 42 mm, y el centro de masa del dron se mueve unos **+5.9 mm en x** con A (+4.9 con B). En hover eso pide ~6 % más de empuje en los motores delanteros y ~6 % menos en los traseros. Actualiza el trim de masa con el peso medido (checklist, etapa 0) y revisa en el primer hover que el dron no derive ni sostenga un cabeceo.
- **Rigidez y vibración de A:** el brazo es un voladizo de 48.5 mm con E = 2000 MPa: rigidez vertical de 780 a 970 N/mm y flecha de ~1 µm a 1 g. El centro de masa de la cámara queda 17 mm por debajo del brazo, así que el primer modo es de **cabeceo acoplado a flexión: 340 Hz** si la cuna es flexible y **445 Hz** si la cámara atornillada la rigidiza. Los rotores giran a ~190 Hz en hover y hasta ~360 Hz a tensión máxima (1P sin carga, una cota superior). El modo queda 1.8 a 2.3 veces por encima del hover, pero no fuera de toda la banda. Además, el cálculo no incluye la flexibilidad de la placa de interfaz ni de la escuadra original, que bajan la frecuencia. Antes de usar la cámara para detección, valida en vuelo: graba un video en hover y busca el efecto "jello" (la RGB tiene obturador rolling). Si es D435i, registra también el espectro de su IMU. Si aparece, la opción es aislar con elastómero.

### Calibración: `rotacion_montaje` y `offset_camara_m`
- **`camara.rotacion_montaje = 0`.** El script lo **deriva de la cámara modelada**: x_cam apunta al extremo del USB, z_cam es la normal de la cara de lentes y y_cam = z_cam × x_cam. Luego lo compara con `R_BODY_CAM0` de `web/app/geo.py`. Si se monta con el USB a la izquierda, el valor es 180 (y la `y` del offset cambia de signo). `config/estacion.json` ya tiene 0, así que no hay que cambiarlo.
- **`camara.offset_camara_m` estimado = [0.212, 0.0325, 0.019] m.** Es la posición del centro óptico RGB:
  - x: 212 mm por delante del centro del dron;
  - y: +32.5 mm a la izquierda, porque la RGB está a 32.5 mm del centro de la cámara, en el extremo opuesto al USB;
  - z: 19 mm sobre el piso.

  Hoy la configuración tiene `[0.20, 0.0, -0.05]`. **No se cambió** porque depende de valores sin confirmar:
  - el montaje real (`X_CARA_TRASERA_ORIG`, `Z_EJE_ORIG`);
  - dónde está el pivote del rigid body en Motive;
  - si la z de la pose que recibe la estación es la corregida con `bias_z` (z ≈ 0 con el dron en el piso, como supone `simulador3d`).

  Si es la corregida, la z del offset debe ser **positiva**: la altura de la lente sobre el piso más ~4 mm. El −0.05 actual pondría la cámara 7 cm más abajo. A 1.5 m eso es un error de escala de ~5 %.
- **Cómo actualizarlo:**
  1. Mide en x e y, con cinta o calibrador, la distancia del pivote de Motive (o del centro entre motores) al centro de la lente RGB. Es la lente pequeña del extremo opuesto al USB.
  2. Mide la altura de esa lente sobre el piso con el dron apoyado y súmale 0.004 m. Si la pose no está corregida con `bias_z`, réstale además la altura del pivote sobre el piso.
  3. Escribe `[x, y, z]` en metros en `config/estacion.json → camara.offset_camara_m` y reinicia la estación (no hay que recompilar Simulink).
  4. Valida con la prueba de geolocalización de la etapa 5 del checklist.
