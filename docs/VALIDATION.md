# Executed validation

The results below describe earlier local runs. Their `build/` logs and MAT files
are generated evidence and are intentionally excluded from Git. To produce
evidence on a new checkout, run `run_validation` in MATLAB R2023b; custom-model
checks are described in [CUSTOM_MODEL_INTEGRATION.md](CUSTOM_MODEL_INTEGRATION.md).
See [GitHub preparation validation](GITHUB_PREPARATION.md) for the source-copy
checks performed for this publication.

Current grayscale implementation: the actual supplied Fire export was built
and loaded in MATLAB R2023b. Compiled metadata confirms **180 x 180, 1 grayscale
channel, 1 class (`Fire`)**, FOMO/TensorFlow Lite Micro, squash resize mode.
The exported detection floor is 0.30; the application threshold remains 0.50.

**Native algorithm and real-model tests passed. All 18 MATLAB/Simulink tests
passed (0 failures, 0 incomplete), in 23.7058 seconds.** Coverage includes RGB
and grayscale conversion, normalized floats, uint16, logical, RGBA/transparency,
indexed PNG, 16-bit PNG and JPEG decoding, resizing, invalid input rejection,
real grayscale/RGB-equivalent inference, event retention, no-detection/error
clearing and live transfer during simulation. Simulink uses an H x W grayscale
port. Test-only injected failures use a separate fixture DLL; native inference
and the ordinary event/live tests use the real Fire detector.

Three black-image native runs returned no detection and took **20.752 ms,
20.656 ms and 21.658 ms**. These are local timings, not latency guarantees.
Labeled Fire/non-Fire photographs and Studio references have not been supplied,
so recognition accuracy remains unverified. MSVC remains untested.

Evidence: `build/grayscale_validation.log`,
`build/matlab_simulink_validation.log`, `build/validation_results.mat`, and
`build/active_model.mat`. The generated `.slx` was compiled and simulated.

Additional checks passed: all six conversion tests were rerun, the public
`select_model` workflow regenerated the grayscale model, and a 1200 x 1600 RGB
JPEG was decoded, resized and classified through `run_ei_inference(filename)`
(status 2/no detection, 20.8883 ms native time). Evidence:
`build/grayscale_workflow_validation.log`. Six event/live tests were also rerun
with explicit assertions that native inference status was nonnegative for
every processed real-model event; all passed in 16.6266 seconds. Evidence:
`build/grayscale_event_status_validation.log`.

The remainder of this report records the historical two-class version.

Executed locally on **2026-11-06** with Windows x64, MATLAB **R2023b Update 10** (23.2.0.2859533), Simulink and MinGW-w64 **GCC 8.1.0**. Installed products contain neither MATLAB Coder nor Simulink Coder nor image/deep-learning toolboxes. The baseline tests used only MATLAB and Simulink.

## Passed

- Built the supplied SDK, model arrays and native C wrapper into a Windows x64 DLL with a matching import archive.
- Executed native algorithm tests: each class alone, both classes, neither, repeated instances, below-threshold detections, exact ties, invalid/nonfinite scores and asymmetric MATLAB RGB layout conversion.
- Executed native real-model tests: compiled metadata, initialization, invalid buffer length, reset, three independent real TensorFlow Lite Micro runs, output ranges, request IDs, termination and reinitialization.
- Compiled the handwritten MEX gateway, loaded the real DLL from MATLAB and cross-checked compiled labels/dimensions against ZIP inspection.
- **11 MATLAB/Simulink tests passed; 0 failed; 0 incomplete**, in 19.7927 seconds in the final complete validation run, after the final native API guards were added.

The 11 tests comprise four MATLAB interface tests, five event/retention tests, one live-source test and one deterministic failure-transition test. The failure-transition test uses a separately named, test-only backend DLL; real-model tests use the supplied TFLM detector. The actual `.slx` was generated, compiled and simulated in R2023b.

Event tests confirmed exactly four completed requests at 0, 40, 85 and 130 simulation seconds, four requests at fractional irregular times 0, 0.3, 1.7 and 4.2 seconds, only one request for a held-high trigger, no request without a trigger, held outputs between events, reset across simulation starts and rejection of ambiguous/off-grid source events. Repeated identical pixel arrays were accepted as separate image events.

The deterministic transition test confirmed status `[1,2,1,-3]`, validity `[true,false,true,false]` and confidence `[91,0,91,0]` for valid → no detection → valid → inference failure. Thus a new no-detection or failed request clears the old detection.

The live test queued a complete image before simulation, rejected a second request while the slot was occupied, submitted another image from a supported execution listener **while simulation was active**, and observed exactly two native result updates. This verifies the polling source actually transfers live data; it does not rely on From Workspace rereading a changed variable.

## Measured timing

The final native real-model smoke test reported **5.001 ms, 4.590 ms and 4.811 ms** for three black 96×96 RGB images. All returned status 2, confidence 0 and no detection. Earlier runs were also about 4–5 ms. These measurements include SDK preprocessing, inference and selection, and exclude image decoding/resizing and initialization. They are observations on this machine, not latency guarantees.

## Evidence

- `build/full_validation.log`: build, real-model timings, MEX loading, complete test totals.
- `build/matlab_simulink_validation.log`: MATLAB/Simulink test report.
- `build/validation_results.mat`: executable MATLAB test results.
- `build/active_model.mat`: selected artifact manifest and metadata.
- `build/workflow_validation.log`: public single-image script and current-model selection/regeneration workflow; no second distinct export was available.
- `simulink/EdgeImpulse_YOLO_Simulink.slx`: generated example with actual native-link settings.

Run `run_validation` to reproduce. Native tests are executed on a fresh build; cached builds reuse previously verified binaries and rerun the MATLAB/Simulink suite.

## Unverified aspects

- **Recognition accuracy and Studio parity:** no labeled Audrey/Lazara photographs or Studio reference detections were provided. Black-image smoke tests establish execution, not person-recognition performance. Resizing uses documented nearest-neighbor sampling; exact-size input avoids interpolation differences from Studio.
- **A second distinct real model:** switching workflow, dynamic class mapping, per-model compilation and dimension regeneration are implemented. Different trained weights, YOLO variants, SDK revisions, operators and resolutions still require a second compatible export for integration validation.
- **MSVC:** CMake provides an MSVC route, but no compatible MSVC installation was present; only the supported MinGW 8.1 route was executed.
- **Interactive pause/submit/resume UI sequence:** the live polling mechanism was verified through a simulation execution listener; the manual toolbar sequence is documented for interactive use.
- Accelerator, Fast Restart, deployment/code generation and asynchronous/real-time execution are outside the baseline.
