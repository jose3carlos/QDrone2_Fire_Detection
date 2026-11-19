# Optional batch visualization

This add-on adds files alongside the existing project. It does not modify the
native DLL sources, detector ZIP, averaging block, configuration scripts or
Trayectory_Model. The Simulink block code is supplied in the conversation only;
it is deliberately not saved to an `.m` or `.slx` file by this add-on.

Run in the MATLAB Command Window while simulation is stopped:

```matlab
setup_fomo_visualization
```

The setup reuses the existing native artifact and compiles a separate MEX
gateway under `build/visualization/<fingerprint>`. The gateway dynamically calls
the existing `ei_sim_infer` and `ei_get_result` exports from the same DLL used
by the model. It never initializes or terminates the engine. Existing model
callbacks continue to own engine startup/cleanup. No detector SDK rebuild is
needed when the existing native artifact is valid.

Paste the provided VisualizeFOMOBatch code into an additional MATLAB Function
block. Connect the same `Images` and `ImgCaptured` signals as the averaging
block. Images must be fixed-size real `uint8 [180 180 N]` grayscale, currently
N=20; readiness must be a logical scalar. Inherit the same discrete sample time
as the batch pipeline, such as 1/30 second. Keep Windows normal simulation and
Fast Restart off. For a separate model, attach its existing native lifecycle
first with `setup_custom_model('YourModelName')`. Do not regenerate your model.

One rising readiness edge snapshots the batch. Each subsequent block execution
processes one image, including images without detections, and paints its figure
immediately. Holding readiness high never repeats the batch. A new batch while
busy raises an error; lower readiness and wait for BatchComplete before sending
another completed batch. Native/graphics failures stop simulation with an
explicit error rather than reporting a false no-detection result.

The add-on returns:

| Port | Meaning |
|---|---|
| ImageOut | Exact original uint8 180-by-180 grayscale frame; overlays are separate graphics. |
| BoundingBox | uint32 [x y width height] in native zero-based coordinates; all zeros when absent. |
| Detected | Logical detection flag for this image. |
| Confidence | Single confidence fraction in [0,1] for this image. |
| FrameIndex | uint32 image index 1..N, initially 0. |
| FrameUpdated | One-execution pulse when a frame is processed. |
| BatchComplete | Latched true after N frames, reset when a new batch starts. |

All per-image results latch until the next processed image. Display windows use
the exact grayscale CData and fixed intensity range 0..255. Native pixel zero
has MATLAB center 1, so rectangle edges are plotted at x+0.5 and y+0.5 with the
original width and height. FOMO boxes mark detected grid regions, not the full
physical object outline. The currently selected highest-confidence box is
shown; collecting every detected instance would require a larger native API.
The gateway copies the result immediately and verifies its result ID against
the inference call so a box cannot silently belong to a different frame.

The default is one reusable figure per image index: 20 figures for a 20-image
batch, updated for subsequent batches. Closing a window is supported; it is
recreated when that index is next processed. Figure UserData stores the native
result and batch/image indices. Images and results are not automatically saved
to disk. Use `close_fomo_visualization` to close only these project figures.

When the averaging block and viewer both run, each makes its own N inference
calls. They use one simulation-owned engine sequentially, so there are 40 calls
per 20-image batch. Native result IDs can have gaps from each block's viewpoint;
they remain increasing within each block. The existing average/majority logic
is not changed. This host-only diagnostic uses `coder.extrinsic` and base MATLAB
graphics; it needs no additional image/vision toolbox. Twenty figure updates
add wall-clock overhead and are not a hard real-time QUARC deployment feature.

## Transfer or detector replacement

Copy the whole project, including these new root helpers and `visualization/`,
as well as the normal model/compiler dependencies. On the destination computer:

```matlab
setup_custom_model('Trayectory_Model')
setup_fomo_visualization
```

Use the same commands after changing/replacing the detector ZIP. Its image
contract must remain 180-by-180 grayscale. The add-on fingerprint includes the
native build key, source/header hashes, project location, MATLAB environment,
host and compiler. Copied stale visualization manifests are rejected; binaries
are compiled locally instead of reusing the original computer's absolute paths.

## Validation

`tests/simulink_tests/validate_fomo_visualization.m` accepts the block script as
an in-memory argument and never saves it. Validation covers 20 live figure
updates with actual native inference, exact grayscale pixels, the selected-box
overlay coordinates, independent operation alongside the original averaging
block, figure reuse/recreation and explicit rejection of an uninitialized
engine. The saved custom model is hashed before/after to ensure it is unchanged.
Evidence is in `build/visualization_runtime_validation.log` and
`build/visualization/validation_results.mat`. Synthetic detections test rectangle
placement; these checks do not establish detector accuracy on camera images.

Executed on MATLAB R2023b Update 10 / Windows x64 with the actual grayscale Fire
detector. A copied project with a deliberately stale original visualization
manifest correctly rejected the stale binding, compiled a local add-on gateway
and passed the same live 20-figure simulation alongside the averaging block.
The relocation check used another folder on this PC; evidence is in
`build/visualization_relocation_validation.log`. Hardware QUARC deployment was
not tested and the MATLAB graphics viewer is intended for host simulation.
