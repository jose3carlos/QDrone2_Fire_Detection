# Image input and grayscale models

The selected export determines its input resolution and whether its image DSP
uses **Grayscale (1 channel)** or **RGB (3 channels)**. No manual channel setting
is necessary. The Fire export uses **180 x 180 grayscale**, one label `Fire`.

Run in the **MATLAB Command Window**, from the project folder:

```matlab
setup_project(false)
select_model                  % builds DLL and regenerates Simulink dimensions
info = load_ei_model();        % info.channels is 1 for this model
result = run_ei_inference('C:\images\photo.jpg')
% Arrays also work:
result = run_ei_inference(imread('C:\images\photo.jpg'))
close_ei_model
```

`run_ei_inference`, `prepare_image`, preloaded image sequences and live
submissions accept:

- A filename readable by base MATLAB `imread`, including common JPEG/PNG files.
  Indexed files are expanded using their embedded colormap. For multi-page or
  animated files, only the first image is decoded.
- Grayscale H x W, RGB H x W x 3, or RGBA H x W x 4 arrays.
- `uint8`, full-range `uint16`, logical, or finite single/double values in [0,1].

Floating-point values outside [0,1], complex/sparse/empty arrays and unsupported
channel counts are rejected. CMYK/four-channel file data must be converted to
RGB explicitly; arbitrary four-channel file data is never assumed to be RGBA.
For array input, four channels explicitly mean RGBA. Transparency is composited
against black. Indexed arrays without a filename/colormap are ambiguous: pass
their filename or expand the colormap explicitly first.

RGB becomes grayscale using 0.299 R + 0.587 G + 0.114 B, rounded to uint8.
Grayscale is replicated across RGB channels when the selected model requires
RGB. The configured/exported resize geometry is preserved; interpolation is
still nearest-neighbor. Orientation should match the images used for training.
Exact Studio parity can depend on rounding and interpolation.

The native API receives model-ready column-major uint8 arrays: **H x W for
grayscale**, **H x W x 3 for RGB**. Native packing replicates a grayscale pixel
into the SDK's packed RGB signal representation; the exported SDK continues to
perform its own DSP and tensor quantization. No model weights or original ZIP
are modified. No Image Processing Toolbox is required.

Simulink image ports/sources are regenerated with the selected model's channel
count. Files and differently sized/color images must pass through
`prepare_image_sequence` or `submit_live_image`; directly connected Simulink
signals must already have the fixed model-ready size and type. The MATLAB
Function block's rising-edge trigger, output retention and result IDs are
unchanged.

```matlab
model = create_simulink_model();
sequence = prepare_image_sequence({'C:\images\one.jpg','C:\images\two.png'},[0 38]);
load_image_sequence(model,sequence);
out = sim(model);
```

To change models, stop simulation, change only `MODEL_PATH` in `config.m`, and
run `select_model` again. A successful build reads compiled channel metadata
back from the DLL and checks it against the export. Use `run_validation` to
test native inference, conversion, Simulink events and live image transfer.
