# APIs and ownership

Current version requires one foreground class. `class_index` is 0 when detected
and -1 otherwise; `object_class` is always 0 for port compatibility. Use
`detected` for presence/absence. The reserved `false_class_index` option must be
0; `labels[1]` is unused. Earlier two-class mapping descriptions below apply
only to the original version. Event and bounding-box behavior are unchanged.

## C ABI

Include `native/ei_wrapper.h`. Link the matching x64 import library and make `ei_inference_<fingerprint>.dll` discoverable. MinGW uses `libei_inference_<fingerprint>.dll.a`; MSVC uses `ei_inference_<fingerprint>.lib`. The ABI contains fixed-size integers, bounded UTF-8 labels and caller-owned result/metadata structs. No STL, C++ exception or SDK-internal type crosses it.

| Function | Behavior |
|---|---|
| `ei_initialize(options)` | validate options, allocate persistent model/interpreter/arena; no Invoke |
| `ei_get_model_info(info, bytes)` | compiled dimensions, labels, last layer and SDK threshold; allowed before initialization |
| `ei_infer_image(rgb,bytes,h,w,result,resultBytes)` | one synchronous request; exact model dimensions, column-major uint8 H x W grayscale or H x W x 3 RGB, according to metadata.channels |
| `ei_get_result(result,bytes)` | read last retained result without inference |
| `ei_reset()` | clear retained result and ID; preserve initialized model |
| `ei_terminate()` | release interpreter, arena and SDK state |
| `ei_get_last_error(text,capacity)` | copy caller-thread error text, NUL terminated |
| `ei_sim_start(...)`, `ei_sim_stop()` | claim/release singleton engine for simulation |
| `ei_sim_infer(...)` | scalar-output bridge used by `coder.ceval` |

All input/output memory belongs to the caller. The DLL does not retain the image pointer. The caller must keep that image unchanged for the duration of inference. The native mutex uses try-lock: concurrent calls are rejected with -6. Simulation ownership rejects standalone submissions even between native calls. Close the standalone session before simulation starts. One engine owner is supported per model-specific DLL.

`ei_options.min_confidence` is a fraction, and selection is independent for each request. Invalid/nonfinite/nonpositive or above-one scores are ignored. Candidates below the threshold are ignored. Highest confidence wins. Exactly equal confidence uses configured Boolean class order (false first), then original SDK detection index. Bounds and class labels are validated. The configured box limit applies to the complete SDK array including zero-confidence padding; overflow returns an explicit error instead of truncating and potentially selecting the wrong object.

Returned `ei_result.confidence` is percentage. `class_index` is zero-based exported order, or -1 for no valid result. `object_class` is the Boolean mapping. Bounding boxes use zero-based model-image coordinates `[x y width height]`. With FOMO these are grid-derived detection regions, not full object extents. `elapsed_ms` measures adapter conversion callback, DSP, Invoke, postprocessing and selection; image decoding/resizing and one-time initialization are outside that interval. IDs are per initialized session and increment for accepted inference attempts, including invalid-image attempts; lock-busy requests are not accepted.

## Status codes

| Value | Meaning |
|---:|---|
| 0 | initialized/ready, no request yet |
| 1 | inference completed with a selected detection |
| 2 | inference completed with no qualifying detection |
| -1 | invalid image dimensions/length |
| -2 | model not initialized |
| -3 | SDK inference, tensor or detection failure |
| -4 | native exception/library failure |
| -5 | unsupported model |
| -6 | busy, owned, or pending live queue full |
| -7 | invalid options or insufficient output capacity |
| -8 | SDK array exceeds configured detection bound |
| -9 | measured synchronous inference exceeded deadline |

MATLAB argument type errors and DLL loading failures raise `EI:*` exceptions. Runtime inference failures return a result struct with a negative status and invalid detection. The simulation initializer raises `EI:SimInit` if it cannot claim or initialize the engine. A native engine unavailable during an event returns -2 and clears the block latch.

The ABI and exceptions assume valid caller-owned pointers and capacities; it cannot make arbitrary corrupt process memory safe.

## MATLAB functions

`load_ei_model` builds only if needed and initializes the session. `get_ei_model_info` reads model/Boolean labels. `run_ei_inference(image)` validates/resizes a single RGB array and invokes the same DLL as Simulink. `get_ei_result` reads the latest result. `reset_ei_inference` clears it. `close_ei_model` releases resources and unloads the gateway's DLL reference. `prepare_image(image,info)` is available for sources.

`build_model` returns an artifact manifest, including model hash, DLL path, matching import library, MEX gateway name, dimensions and Boolean mapping. `build_model(true)` requests a rebuild. `clean_build` releases MEX files and deletes only the project's verified `build` folder; first save and close the generated Simulink model yourself. It does not close other models.
