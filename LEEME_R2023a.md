# Trayectory_Model para MATLAB R2023a

En la rama `main`, `Trayectory_Model.slx` está guardado con R2023b, y R2023a no lo puede
abrir. En esta rama (`Quique`) el modelo ya viene **reconstruido y guardado en R2023a**, así
que abre en R2023a y en R2023b. Además, `rebuild_trayectory_model.m` lo vuelve a construir
desde código en tu versión de MATLAB. Conserva los mismos bloques, posiciones, parámetros,
conexiones y el código exacto de los 6 bloques MATLAB Function, que están en
`simulink/trayectory_blocks/`.

El `Trayectory_Model.slx` de esta rama se generó con
`rebuild_trayectory_model('ConnectTrigger',true)` y con la *Integrated Webcam* de la PC
donde se probó. En otra computadora, elige tu cámara con doble clic en
*Video 3D Capture → From Video Device*.

## Requisitos en R2023a (verificados en esta PC)

1. **Compilador MinGW-w64.** Instala el add-on *MATLAB Support for MinGW-w64 C/C++/Fortran
   Compiler* y ejecuta `mex -setup C++` y `mex -setup C` eligiendo **MinGW64**. Con Visual
   C++ el SDK de Edge Impulse no compila: usa `INCBIN`, `#warning` y `__attribute__` de GCC.
2. **CMake 3.20 o más nuevo.** R2023a trae CMake 3.19.7, que no sirve. `instalar` lo
   resuelve solo: busca uno en el PATH, en cualquier Visual Studio y en `C:\Program Files\CMake`,
   y si no encuentra ninguno lo descarga. Guarda la ruta en `tools\cmake_path.txt`, que es
   local y no se sube a git, y `config.m` la lee de ahí.
3. **Ruta corta y sin espacios**, por ejemplo `C:\QDrone2_Fire_Detection`. Con rutas largas
   la compilación de TensorFlow Lite pasa el límite de 260 caracteres de Windows.
4. Solo para la webcam: el support package *Image Acquisition Toolbox Support Package for
   OS Generic Video Interface*.

Si falta el punto 1 o el 2, `rebuild_trayectory_model` guarda el modelo y se detiene con
un mensaje que explica qué corregir, sin dejar la compilación a medias.

## Uso rápido (MATLAB Command Window, en la carpeta del proyecto)

```matlab
instalar
open_system('Trayectory_Model')
```

`instalar` hace todo en orden:
1. Revisa la carpeta.
2. Selecciona MinGW64.
3. Busca un CMake 3.20 o más nuevo. Si no hay, descarga el 3.27.9 oficial (44 MB),
   verifica su SHA-256 y lo deja en `tools\cmake`.
4. Compila el detector y lo prueba.
5. Enlaza `Trayectory_Model` y prepara la visualización. Si tu MATLAB es más viejo que el
   del modelo, lo reconstruye.

Se puede correr otra vez sin problema. Se probó en R2023a simulando una PC nueva (Visual C++
seleccionado, sin `build`, modelo guardado en R2023b), con CMake de Visual Studio y con CMake
descargado.

A mano, sin `instalar`:

```matlab
setup_custom_model('Trayectory_Model')   % compila el detector y lo enlaza en ESTA PC
setup_fomo_visualization                 % para el bloque de figuras FOMO
open_system('Trayectory_Model')
```

Para volver a generarlo desde código:

```matlab
rebuild_trayectory_model('ConnectTrigger',true)
open_system('Trayectory_Model')
```

`rebuild_trayectory_model` hace tres cosas:
1. Mueve el `Trayectory_Model.slx` existente a `backups/original_models/`. No se borra nada.
2. Construye el modelo y lo guarda en tu versión.
3. Corre `setup_custom_model('Trayectory_Model')`, que compila el detector y lo enlaza al modelo.

Si vas a usar el bloque de figuras (`MATLAB Function`), corre además `setup_fomo_visualization`.

### Opciones

| Opción | Efecto |
|---|---|
| `'Camera','synthetic'` | Pone una imagen negra de 1280x720 a 30 fps en lugar de la webcam. Sirve para probar sin cámara. |
| `'ConnectTrigger',true` | Conecta `startCamera` a `Trigger` mediante un Unit Delay de 0.05 s (ver nota). |
| `'Visualization',false` | Quita el bloque de figuras FOMO y sus Terminators. |
| `'SetupNative',false` | Solo construye el modelo, sin compilar ni enlazar el DLL. |
| `'Name','OtroNombre'` | Guarda con otro nombre, sin tocar `Trayectory_Model.slx`. |

## Qué se probó (MATLAB R2023a Update, MinGW64, CMake 3.27)

- `setup_project`: el DLL compiló en ~1 minuto, las 2 pruebas nativas pasaron, y la imagen
  negra dio `status=2` (sin detección).
- Copia fiel: se guardó como R2023a con los mismos bloques, puertos, conexiones y código, y
  con 0.05 s en *Trayectory SM*. `setup_custom_model` la enlazó y compiló sin errores.
- `'Camera','synthetic','ConnectTrigger',true`, con el dron puesto en el waypoint 1 y
  `Manual Switch` en 1: `startCamera` subió en t = 3.05 s, el lote de 25 fotos se completó en
  t = 6.10 s (Status 2), la inferencia FOMO corrió sobre las 25 fotos y la máquina de
  estados avanzó al waypoint 2.
- No se probó con webcam ni con fotos reales de fuego.

QUARC agrega solo los bloques de logo *Powered by QUARC* y *Quanser* a cada modelo nuevo.
Son inofensivos.

## Notas sobre el modelo original

- **La captura nunca se dispara en el original.** `Trigger` de *Image Batch Capture* está
  conectado a una constante `false`, y `startCamera` de *Trayectory SM* no está conectado.
  La copia fiel conserva eso. Usa `'ConnectTrigger',true` para que la máquina de estados
  dispare las 25 fotos en cada waypoint.
- **La posición es una constante `[0;0;1;0]`**, así que la misión no avanza del primer
  waypoint hasta que conectes la pose real del QDrone2 (ver `GUIA_QDRONE2.md`).
- **Webcam:** para el bloque *From Video Device* instala el support package
  *Image Acquisition Toolbox Support Package for OS Generic Video Interface*
  (Home → Add-Ons). Sin él, el bloque queda con `(none)`.
