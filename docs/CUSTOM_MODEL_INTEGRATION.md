# Existing custom models

From the project folder, run in the **MATLAB Command Window**:

```matlab
setup_custom_model('Trayectory_Model')
open_system('Trayectory_Model')
```

Save any pending model edits first. Setup backs up the existing `.slx` under
`backups/custom_models/<unique-folder>/`, selects/builds the native artifact for
this project location and computer, and saves only the native integration
settings and metadata. It never regenerates the model. All blocks, Stateflow
scripts, solver settings, sample times, target language and existing callback
bodies remain unchanged. Managed code is delimited by `EICUSTOM_BINDING` markers;
repeated setup replaces those sections instead of duplicating them. Do not use
the example generator or `configure_simulink_model` on your custom model.
Older comments inside pasted inference blocks may still name that example
helper; use `setup_custom_model` for this model instead. The block calculations
do not need editing.

Simulation Target include directories and libraries are merged with existing
entries. The header is `native/ei_wrapper.h`; the import library and DLL come
from the active build, not a hard-coded hash. The custom initialization starts
the engine once; custom termination releases it. Before initialization, the
MATLAB standalone gateway releases its ownership. Native guards still reject
other simultaneous owners. Use one inference block/engine at a time.

The added PostLoadFcn and InitFcn sections locate the project by walking upward
from the model's own filename. They preserve and then execute existing callback
bodies. Opening the model selects/builds local artifacts if necessary; InitFcn
validates the active export and refreshes bindings before target compilation.
Callbacks do not save the model or change its simulation timing. Normal-mode
Windows x64 simulation and Fast Restart off are required. Unsupported settings
produce an explicit error instead of being overwritten. Model-file workspaces
are supported; external workspace files are deliberately not modified.

Copy the entire project, including your model and `tools/mingw64` if using the
bundled compiler, to the other Windows computer. Keep the model within the
project folder or its subfolders. Run the setup command there before first use.
An already attached model also refreshes on opening at its new location.
`setup_on_this_computer` continues to regenerate only the standard example.

After changing `MODEL_PATH` or replacing its ZIP, stop simulation and run the
custom setup again. If the active artifact was already rebuilt by `select_model`,
InitFcn picks up its new library and build identity automatically. Replacements
must preserve the fixed image contract. For the current InferFOMOBatch this is
`uint8 [180 180 N]` grayscale; setup does not alter resizing, batching, confidence
threshold calculations or the majority decision. A changed shape/channel count
is rejected when rebinding a previously attached model.

The DLL uses a C-compatible interface, so a custom model's existing C target
language is valid even though the DLL itself is compiled as C++. Matching the
selected compiler family is necessary. Camera configuration, device drivers,
QUARC deployment and real-time deadlines remain separate from host simulation.

## Verification

```matlab
setup_project(false)
validate_custom_model_integration
```

This validates a disposable copy of Trayectory_Model with a synthetic RGB camera
and no viewer. The actual capture, resizing and inference scripts are retained.
It observes native result IDs and outputs during execution, checks a complete
20-image batch, compares the result with standalone inference, repeats the run,
tests stale bindings and checks that the original saved model is unchanged.
It also exercises the classic C/MEX compilation path with custom-code importing
temporarily disabled in the disposable copy, and verifies preservation of
unrelated custom headers, libraries and callback bodies.
The test settings and camera substitutions are never saved to the original.
Results are written to `build/custom_model_validation/results.mat`. These are
integration checks, not a measurement of detector accuracy or hardware timing.

Validated on 2026-11-10 with MATLAB R2023b Update 10 and MinGW 8.1: the actual
180 x 180 grayscale Fire model processed 20 images per run and completed the
batch at simulation time 4.6333 seconds, before the 5.2-second test ended.
Repeated runs and standalone comparison passed. Synthetic black frames returned
no detection and average confidence 0, as expected for this smoke test. A copied
project, seeded with the original absolute-path manifest and copied compiler,
automatically selected a new local build and passed live batch inference and
binding-preservation checks. This relocation test used the same physical PC.
Evidence: `build/custom_model_runtime_validation.log` and
`build/custom_model_relocation_validation.log`.
