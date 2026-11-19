# Troubleshooting

- **No C++ compiler:** run `setup_project(false)`, then `mex -setup C` and `mex -setup C++`. Confirm `mex.getCompilerConfigurations('C++','Selected')` is nonempty. The included MinGW folder is detected per MATLAB session. Ensure no spaces in an external MinGW root.
- **CMake missing:** set `cfg.cmake` to CMake >=3.20. R2023b includes a usable CMake in its installation on this machine.
- **Wrong model path or incomplete ZIP:** select the complete Edge Impulse C++ Library export. Inspection reports the missing filename. A `.tflite` file alone is insufficient.
- **Unsupported SDK setup:** a different SDK revision may have changed TFLM initialization. The guarded lifetime adaptation rejects it. Keep the old working model and adapt the setup patch after inspecting the new SDK; do not blindly force string replacements.
- **MSVC syntax errors:** the provided archive uses GNU-specific annotations and initializers. MinGW 8.1 is the locally validated route. Use the matching MEX compiler and native import library.
- **Windows DLL error 126:** a DLL dependency is missing. Use `setup_project` so the native `bin` directory is on PATH. With MinGW, `libwinpthread-1.dll` is copied next to the inference DLL. Keep the matching compiler runtime. Error 193 usually means an x86/x64 mismatch.
- **DLL locked during rebuild:** stop simulations, close model windows and run `close_ei_model`, then `clear mex`. Run `clean_build` only after saving any open models. Never change models during simulation or Fast Restart.
- **Invalid RGB:** check `class(image)`, `size(image)` and `isreal(image)`. `imread` may return grayscale/indexed data. Supply explicit RGB `uint8`, not normalized doubles.
- **No detections:** status 2 is valid. The SDK's own 0.5 detection floor applies before the application selector. Lowering the application threshold cannot restore boxes already removed by the SDK. Check framing, crop, lighting and Studio reference results.
- **Class seems reversed:** model indices and Boolean mapping are separate. For this model, Audrey is model index 0 but Boolean true. Inspect `info.labels` and `info.booleanLabels`.
- **Repeated inference:** ensure the block is the provided rising-edge version. One low sample is needed before another event. Observe `resultID`, not confidence, to count requests.
- **No second event:** a continuously high Boolean has no second rising edge. Lower it for a full sample, then raise it again with a complete new frame.
- **Workspace edits do nothing live:** From Workspace reads a preloaded timeseries. Use `enable_live_source` and `submit_live_image` for live operation, or stop/reload a sequence.
- **Live accepted but not processed:** accepted means queued. Resume the model and allow source sample hits; inference appears at a later hit. Use pacing or pause/submit/resume so MATLAB callbacks can execute. A full queue returns -6.
- **Stale dimensions after switching:** run `select_model` to regenerate the source and block dimensions. Old `.slx` files intentionally fail their build-key check.
- **Custom code cannot resolve `ei_sim_infer`:** run `configure_simulink_model(model)`, check `SimUserIncludeDirs` and `SimUserLibraries`, ensure compiler families match, and disable Fast Restart.
- **Native timeout -9:** inference completed but exceeded the configured deadline. This baseline does not forcefully interrupt native code. Increase/disable the deadline or use a smaller supported model.

Record `build/full_validation.log`, `build/matlab_simulink_validation.log`, model hash, MATLAB release and selected compiler when diagnosing failures.
