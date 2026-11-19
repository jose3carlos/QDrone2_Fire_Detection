# Installation and MATLAB integration, step by step

Current input handling supports single-class RGB **and grayscale** models, with
automatic conversion from common image files and arrays. Follow
[IMAGE_INPUT.md](IMAGE_INPUT.md) for the current Fire model (180 x 180 grayscale);
this guide also accepts RGB images through automatic conversion.

## 1. Open the project in MATLAB R2023b

```matlab
cd('C:\QDrone2_Fire_Detection') % use your actual checkout folder
setup_project(false)
ver
```

MATLAB is sufficient for standalone image inference. Simulink is required for
the models and the full validation suite. `mex` builds a handwritten C++ gateway
using base MATLAB; no MATLAB Coder license is used. The project targets
normal-mode desktop simulation, not deployment to a real-time or embedded target.

### Custom model and hardware dependencies

`Trayectory_Model.slx` also contains camera and video-viewer blocks. Install
Image Acquisition Toolbox and the appropriate camera support package/driver for
its From Video Device block, and Computer Vision Toolbox for Video Viewer.
Select the destination computer's camera/device format in the block settings.
These products are additional to the standalone detector's requirements.
Install Quanser QUARC/QLabs, licenses and drivers separately if your workflow
uses those devices or blocks. Hardware target deployment has separate
requirements and is not established by passing the host simulation tests.

## 2. Configure a compatible compiler

The compiler is **not included in Git**. Install it before the first build.
The original working folder used portable MinGW 8.1 under `tools/mingw64`;
you may supply the same folder using the download in [tools/README.md](../tools/README.md).
When present, `setup_project` detects it, sets `MW_MINGW64_LOC` and prepends its
`bin` folder to the **current MATLAB process** PATH. It selects that compiler
for C/C++ if the selection is missing or points elsewhere. Otherwise, it uses
your selected compiler or invokes MEX setup. Preserve the compiler's upstream
notices when installing it.

For a fresh copy without that portable folder, install **MATLAB Support for MinGW-w64 C/C++ Compiler** through Home → Add-Ons, or obtain MinGW 8.1 and configure it as follows:

```matlab
setenv('MW_MINGW64_LOC','C:\mingw64') % folder containing bin, no spaces
setenv('PATH',['C:\mingw64\bin' pathsep getenv('PATH')])
mex -setup C
mex -setup C++
```

MathWorks lists MinGW 8.1 and MSVC 2022 as compatible with R2023b in its [release-specific compiler table](https://www.mathworks.com/content/dam/mathworks/mathworks-dot-com/support/sysreq/files/system-requirements-release-2023b-supported-compilers.pdf). See also the [MathWorks MinGW setup instructions](https://www.mathworks.com/matlabcentral/answers/313298-i-already-have-mingw-on-my-computer-how-do-i-configure-it-to-work-with-matlab).

MSVC option: install Visual Studio 2022 Build Tools, select **Desktop development with C++**, the MSVC x64 toolset and a Windows SDK, restart MATLAB, then run both `mex -setup` commands and select Microsoft Visual C++. The CMake configuration provides an MSVC C++20 route and a GNU-annotation compatibility header. This route has not been executed in this workspace; the supplied SDK contains GNU extensions, so use the validated MinGW route if an MSVC export fails. Do not mix MinGW import archives with an MSVC simulation target.

CMake 3.20 or newer is required. The build script first checks PATH, then the CMake distributed in MATLAB R2023b. You can set `cfg.cmake` in `config.m` to a separate CMake executable if needed.

## 3. Select the model ZIP

The Fire archive is in `models/`. The only model-selection variable is at the top of `config.m`:

```matlab
MODEL_PATH = "models/fire_hazard-cpp-mcu-v7-impulse-#1.zip";
```

An extracted export directory also works. Do not replace the archive with a `.tflite` file: the SDK, generated model parameters and operator resolver must accompany it.

## 4. Build and verify native loading

```matlab
setup_project
```

This validates the export, extracts a build copy, applies a guarded TFLM lifetime adaptation, builds `ei_inference_<fingerprint>.dll` and its import library, runs native tests, compiles the MATLAB gateway, reads actual compiled metadata and checks that MATLAB can load the DLL. Each fingerprint has a distinct DLL name, preventing Windows from resolving a switched model to an already loaded binary. No rebuild occurs per image request.

From PowerShell you can also run:

```powershell
.\scripts\build_windows.ps1 -Matlab 'C:\Program Files\MATLAB\R2023b\bin\matlab.exe'
```

## 5. Run one image

```matlab
info = load_ei_model();
disp(info.labels)          % exported order
disp(info.booleanLabels)  % legacy field: the sole foreground label
image = imread('C:\images\photo.jpg');
result = run_ei_inference(image)
```

Image input may be a filename or grayscale/RGB/RGBA array. Supported array
types are uint8, uint16, logical and finite single/double in [0,1]. File paths
also preserve indexed colormaps and alpha during decoding. Conversion produces
the selected model's uint8 grayscale or RGB layout; see [IMAGE_INPUT.md](IMAGE_INPUT.md).

`prepare_image` follows the export's resize geometry (squash for Fire,
fit-shortest crop or fit-longest padding for other exports) using nearest-neighbor
pixel-center sampling. Resize-none requires exact-size input. Exact 180 x 180
input avoids resizing for Fire. The SDK performs tensor scaling, DSP extraction,
quantization and postprocessing. Interpolation can differ from Studio's.

Inspect `label`, `classIndex`, `confidence`, `detected`, `objectClass`, `status`, `boundingBox`, `elapsedMs`, `resultID` and `errorMessage`. A no-detection result has status 2, empty label, index -1, confidence 0 and detected false. No detection is a completed inference.

## 6. Classify another image without restarting

```matlab
nextImage = imread('C:\images\next_photo.jpg');
nextResult = run_ei_inference(nextImage);
lastResult = get_ei_result();
reset_ei_inference(); % clear result and request counter, keep initialized model
close_ei_model();     % release interpreter and arena
```

An explicit call is a request. No 40-second pause is required. The native interpreter and tensor arena persist until close/termination. Input image pointers are used synchronously and never retained after the call returns.

## 7. Generate and run Simulink

```matlab
close_ei_model
model = create_simulink_model();
sequence = prepare_image_sequence({image},0);
load_image_sequence(model,sequence);
open_system(model)
out = sim(model);
```

The generator writes `simulink/EdgeImpulse_YOLO_Simulink.slx`, configures simulation-target custom code, declares image and Boolean input types and adds six outputs. See [Simulink usage](SIMULINK_USAGE.md) for intermittent and live submissions. Starting each simulation resets the block's latch, edge state and native result counter. The default example intentionally submits a black test image at time zero; replace it with your RGB image as above.

## 8. Run all validation

```matlab
results = run_validation();
```

Inspect `build/validation_results.mat` and `build/matlab_simulink_validation.log`. Accuracy and a second distinct model require additional real data; neither can be established using the supplied model archive alone.
