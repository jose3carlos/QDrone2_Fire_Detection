# QDrone2 Fire Detection — MATLAB R2023b

> **Rama `Quique`:** agrega compatibilidad con **MATLAB R2023a** y una **versión web sin
> MATLAB**.
> - `Trayectory_Model.slx` está guardado en R2023a (abre en R2023a y R2023b). Se regenera con
>   `rebuild_trayectory_model.m`. Requisitos y pruebas en [LEEME_R2023a.md](LEEME_R2023a.md).
> - [web/](web/) es la versión en Python con el SDK de Quanser y una página web: detector FOMO
>   con LiteRT, la misma máquina de estados de waypoints y la captura por lotes. Guía paso a
>   paso para el QDrone2 en [web/GUIA_QDRONE2.md](web/GUIA_QDRONE2.md).

Fire detection for the QDrone2 project using an Edge Impulse FOMO detector,
MATLAB and Simulink on **Windows x64**. The selected export is
`models/fire_hazard-cpp-mcu-v7-impulse-#1.zip`: **180 × 180 grayscale**, one
foreground class (`Fire`). RGB images are converted automatically.

Native libraries
and MEX files are built on each computer; they are not shipped in Git.

## 1. Get the project

Clone the repository into a writable folder with a short path and **no spaces**:

```powershell
git clone https://github.com/jose3carlos/QDrone2_Fire_Detection.git C:\QDrone2_Fire_Detection
```

Alternatively, use **Code → Download ZIP** on GitHub and extract it there. Open
MATLAB **R2023b** and set Current Folder to the folder containing this README
and `setup_project.m`.

## 2. Install the dependencies

- MATLAB R2023b; Simulink for the models and full validation suite.
- **MinGW-w64 GCC 8.1**, the compiler used for this project's validation. Install
  MATLAB Support for MinGW-w64 C/C++ Compiler through MATLAB Add-Ons. For a
  manual or portable installation, follow [Installation](docs/INSTALLATION.md).
- CMake 3.20 or newer. Setup also looks for the CMake bundled with MATLAB.
- For the custom model's camera/viewer blocks, install the products and device
  support listed in [Installation](docs/INSTALLATION.md#custom-model-and-hardware-dependencies).
  Quanser software, licenses and drivers must be installed separately when used.

The compiler download is excluded from Git. A fresh clone does **not** contain
`tools/mingw64/`; see [local build tools](tools/README.md) for the portable option.

## 3. Build and check MATLAB inference

Run these commands in the **MATLAB Command Window**, from the project folder:

```matlab
setup_project                 % session paths, compiler, native build and checks
info = load_ei_model();
disp(info.labels)             % Fire
result = run_ei_inference(zeros(180,180,'uint8'))
close_ei_model
```

The black image is an execution check; no detection (`status == 2`) is normal.
Try your own image with `result = run_ei_inference('C:\images\fire.jpg')`.
Use `result.detected` for presence, `result.confidence` for confidence in percent,
and `result.status` for execution status. `objectClass` remains false for legacy
port compatibility. See [image inputs](docs/IMAGE_INPUT.md).

The first build takes longer. Later calls reuse the local build. Keep model
ZIPs intact; setup extracts and patches only disposable copies under `build/`.

## 4. Choose a Simulink workflow

**Existing QDrone model:** keep `Trayectory_Model.slx` in the project root. Save
any pending edits, then run:

```matlab
setup_custom_model('Trayectory_Model')
open_system('Trayectory_Model')
```

Setup backs up the saved model and refreshes its native-library bindings for
this computer, preserving its blocks, solver and sample times. Configure the
camera and external dependencies before running it. See
[custom-model integration](docs/CUSTOM_MODEL_INTEGRATION.md).

**Standalone example:** run `setup_on_this_computer`, then
`open_system('EdgeImpulse_YOLO_Simulink')`. Setup backs up and regenerates only
that example with local paths and a black-image source. It does not regenerate
`Trayectory_Model`. See [Simulink usage](docs/SIMULINK_USAGE.md) to load
photographs or live inputs. Use Normal simulation with Fast Restart off.

## 5. Validate the installation

```matlab
results = run_validation();
```

This runs the MATLAB/Simulink suite; a fresh native build also runs native tests.
Tests regenerate the standard example, so save your own work in a separate
model. For the custom model, also run `validate_custom_model_integration` after
setup. It uses a disposable copy with synthetic camera input. See
[validation results and limits](docs/VALIDATION.md).

## Project map

| Location | Purpose |
|---|---|
| Root `.m` files | Configuration, setup, build and validation entry points |
| `Trayectory_Model.slx` | Existing custom capture/inference model |
| `matlab/` | Single-image API and model lifecycle |
| `native/` | C/C++ DLL, MEX gateway, image adapter and detection selection |
| `simulink/` | Event/live/batch helpers and standard example |
| `models/` | Original detector ZIPs; [versions and checksums](models/README.md) |
| `visualization/` | Optional visualization MEX source |
| `scripts/` | Build, inspection and validation helpers |
| `tests/` | Native, MATLAB and Simulink tests |
| `docs/` | Detailed guides and validation notes |
| `tools/` | Compiler installation notes; compiler binaries stay local |
| `build/`, `backups/`, `slprj/` | Generated local files, excluded from Git |

## Further guides

- [Installation](docs/INSTALLATION.md) · [Troubleshooting](docs/TROUBLESHOOTING.md)
- [Transfer to another computer](docs/TRANSFER.md)
- [Model switching](docs/MODEL_SWITCHING.md) · [Model inspection](docs/MODEL_INSPECTION.md)
- [API reference](docs/API_REFERENCE.md)
- [Batch capture](docs/CAPTURE_BATCH.md) · [Optional visualization](docs/FOMO_VISUALIZATION.md)
- [Contribution and repository conventions](CONTRIBUTING.md)

Each explicit MATLAB call or rising `newImage` edge performs one synchronous
inference. Outputs latch until the next result. Recognition accuracy and QUARC
target deployment need separate validation with representative data and hardware.
Upstream SDK/model license notices remain inside the original export archives.
