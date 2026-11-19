# Switching models between simulation runs

The project now requires **exactly one foreground class** (background is not a
second class). Run the `.m` commands below in the **MATLAB Command Window**.
The original bundled Lazara/Audrey ZIP has two classes and is now rejected;
select a single-class RGB or grayscale FOMO C++/TensorFlow Lite Micro ZIP.
Channels and resolution are inferred automatically. See [image input](IMAGE_INPUT.md).

1. Stop all simulations and call `close_ei_model`.
2. Place a second, complete C++ Library export in `models/`.
3. Change **only** `MODEL_PATH` at the top of `config.m`:

   ```matlab
   MODEL_PATH = "models/Alternative_Model.zip";
   ```

4. Run:

   ```matlab
   setup_project(false)
   artifact=select_model();
   info=load_ei_model();
   disp(info.labels)
   disp(info.booleanLabels)
   close_ei_model
   open_system('EdgeImpulse_YOLO_Simulink')
   ```

`select_model` rebuilds or selects the appropriate cached DLL, reloads metadata and regenerates the example's image port dimensions and source data. The generic `EI_YOLO_Function.m` is unchanged. Replace the regenerated black-image sample with the new RGB source before classification.

The sole label comes from the selected model. A detection returns classIndex 0;
no detection returns -1 and an empty label. Use `detected` for presence/absence.
`objectClass` stays false for compatibility with the existing Simulink ports;
the legacy `info.booleanLabels` field contains only the sole class label.
Confidence, selected bounding-box coordinates, event triggering and output
retention retain their existing behavior. No Boolean class mapping is needed.

Each model is compiled with its own SDK sources, generated DSP settings, operator resolver and model arrays. ZIPs are not interchangeable runtime binaries. The cache fingerprint covers the original archive/extracted source tree, generic native source, builder/SDK patch and compiler identity. Switching the path may select an existing compatible build instead of recompiling.

The supported baseline is a single-class RGB or grayscale camera, single DSP image block and single learning block, non-EON TensorFlow Lite Micro export with a nonzero arena and no tracking. Model-specific postprocessing is supplied by that export's SDK. A changed SDK setup signature, unsupported resize mode, full TensorFlow Lite-only model, EON export, custom hardware dependency, missing source or unsupported operator produces an explicit inspection/build error. Supporting every historical/future Edge Impulse SDK is not claimed.

After selecting a different model, verify its label, operators, SDK compatibility
and image dimensions locally:

```matlab
test_model_switch  % run once before, then again after changing MODEL_PATH
run_validation
```

The switching test records the prior model fingerprint and confirms the generic block file did not change. Native and MATLAB smoke tests derive expected dimensions and Boolean mapping from the selected model/configuration. Validate the new model's recognition accuracy using its own labeled images and Studio references.
