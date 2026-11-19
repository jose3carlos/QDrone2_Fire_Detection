# Copying the project to another computer

The source project is transferable; generated DLL/MEX/CMake paths are local to
each computer. Clone/download the repository, or copy the whole project folder,
to a writable location with a short path without spaces such as `C:\QDrone2_Fire_Detection`.
Keep `models/` and, for the bundled compiler, **`tools/mingw64/`**. The compiler
folder is excluded from Git, so a source checkout alone does not include it.
You may omit `build/`, `slprj/`, `*.slxc` and compiled `*.mexw64` files.
Also save and copy any separate/custom `.slx` files containing your pasted
capture/inference blocks; a model stored outside this folder is not included
when you copy only the project folder.

Requirements:

- Windows x64. The current native implementation is not a macOS/Linux build.
- MATLAB R2023b (tested release). A different release must use a compiler
  supported by that release and pass local validation.
- A compatible C/C++ compiler. The bundled MinGW 8.1 is the validated option
  for R2023b; setup selects it when present. Its path must not contain spaces.
- CMake >=3.20 on PATH, or the copy bundled with MATLAB. No absolute MATLAB
  installation path needs editing: discovery uses `matlabroot`.
- Simulink for Simulink models. The standalone MATLAB interface does not need
  Simulink. MATLAB/Simulink Coder and image/deep-learning toolboxes are not
  required by the ordinary host-simulation inference implementation.
- Quanser QUARC/QLabs, their licenses and device drivers when using those
  blocks/devices. These are installed separately, not copied with the project.
  QUARC target deployment has its own compiler/product requirements; a Windows
  host DLL cannot be used unchanged on another target platform.

For a fresh checkout, install the compiler first using
[Installation](INSTALLATION.md). For the existing custom model, follow the
`setup_custom_model` workflow below. For the standalone example, open MATLAB,
change Current Folder to the project folder and run:

```matlab
setup_on_this_computer
result=run_ei_inference('C:\images\photo.jpg');
```

The setup detects paths locally, configures the compiler, builds or selects a
local native library/MEX gateway, runs native checks, verifies MATLAB loading
and regenerates the standard Simulink example with the new library paths.
The previous example is backed up in `simulink/transfer_backups/`. Custom
models with other names are not modified. Regenerating the example resets its
image source to the standard black-image test; reload your sequence afterwards.

Build identity now includes the project location, MATLAB installation/version,
architecture and host, alongside model/source/compiler fingerprints. Cached
manifests are also checked against the current root, so copying old `build/`
files does not silently use the original computer's DLL paths. No cache deletion
is required. Keep `MODEL_PATH` relative to the project, as in the default config.

For the existing custom model, run `setup_custom_model('Trayectory_Model')`
in the MATLAB Command Window instead of regenerating the example. This backs
up the saved model, builds/selects the local backend and attaches its simulation
bindings without changing blocks, solver settings or sample times. The model
must be saved inside the project (subfolders are supported). Managed callbacks
find the project from the model's new location and refresh native paths when
the model opens and before compilation. After replacing the detector ZIP, run
`setup_custom_model('Trayectory_Model')` again before simulation; the image
dimensions/channels must remain compatible with your unchanged blocks.
See [custom-model integration](CUSTOM_MODEL_INTEGRATION.md). The older
`configure_simulink_model` helper remains intended for the generated example.
Machine-specific camera selections, external files and third-party block
settings must be configured for the new computer.

Finally run `run_validation`. A new physical machine still needs local checks
with its own camera and representative images; matching software does not
establish image-recognition accuracy or real-time deadlines.

## Relocation check

Validated on 2026-11-10 by copying the project and portable compiler into a
different folder while keeping the original folder available. A stale original
build manifest was deliberately copied too. Setup ignored its old absolute
paths, compiled a new local DLL/MEX, passed native tests and MATLAB inference,
and regenerated/simulated the example using the copied folder's library.
Evidence: `build/portability_setup_validation.log` and
`build/relocation_validation.log`. This was a folder-relocation test on the
same Windows computer, not validation on a second physical computer.
