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

Las piezas sueltas están en `stl/`, para imprimir más o rearmar las placas. Las versiones editables están en `step/` (se abren en FreeCAD o SolidWorks), y los scripts paramétricos en `cad/` (`fuegos.py`, `valvula.py`).

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
