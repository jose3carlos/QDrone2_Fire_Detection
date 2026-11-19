# Event-triggered Simulink usage

## Signals and execution

The principal MATLAB Function block has two inputs: model-ready uint8 `image`
(H x W for grayscale, H x W x 3 for RGB) and scalar logical `newImage`. Fire uses
180 x 180 grayscale. A sampled rising edge invokes the native DLL once. Holding
`newImage=true` for 100 steps invokes it once. A low sample re-arms the input.
Identical pixels may be submitted as a new event. Source helpers automatically
convert filenames and differently sized/color arrays; direct signals must have
the model-ready layout. See [IMAGE_INPUT.md](IMAGE_INPUT.md).

The six outputs are:

| Output | Type | Meaning |
|---|---|---|
| confidence | single | selected confidence, 0–100 |
| objectClass | logical | always false; retained for port compatibility |
| detected | logical | valid selected detection available |
| status | int32 | 0 ready, 1 selected detection, 2 no detection, negative error |
| resultID | uint64 | completed attempt sequence, resets each simulation |
| resultUpdated | logical | one source sample pulse on completion |

Confidence, class, validity, status and ID remain latched between events. Status describes the held result, so a successful result retains status 1 until the next request. A new no-detection result clears the old detection. An inference error clears confidence and validity and retains an explicit negative status. Use `resultUpdated && detected && status==1` to act once on a valid result.

The model uses a fixed discrete grid, default 0.1 simulation seconds. This is the source polling resolution, **not an inference period**. Event times can be irregular, fractional and shorter or longer than 40 seconds, subject to this configured grid. Change `cfg.image.sampleTime` and regenerate the model for finer resolution. The sequence helper rejects off-grid times and events closer than two samples, ensuring a low sample between rising edges. To accept consecutive high samples as distinct events, a request-ID input would be needed; this two-input baseline deliberately requires the low interval.

Native inference is synchronous: simulation time does not advance until it returns. It can take longer in wall time than the nominal interval. The `timeoutMs` option detects a deadline overrun after return; it cannot interrupt TensorFlow Lite safely. There are no real-time scheduling guarantees.

## Mode A: static image, one event

```matlab
setup_project
image=imread('C:\images\photo.jpg');
model=create_simulink_model();
sequence=prepare_image_sequence({image},0);
load_image_sequence(model,sequence);
out=sim(model);
out.yout.getElement('resultID').Values % ends at 1
```

The source updates the complete image and raises the event at time zero. It lowers the event one source sample later. The event at zero is intentional; model initialization by itself performs no neural inference.

## Mode B: pre-recorded irregular sequence

```matlab
images={imread('C:\images\one.jpg'),imread('C:\images\two.jpg'), ...
        imread('C:\images\three.jpg'),imread('C:\images\four.jpg')};
sequence=prepare_image_sequence(images,[0 38 83 119]);
load_image_sequence(model,sequence);
out=sim(model);
ids=out.yout.getElement('resultID').Values;
updates=out.yout.getElement('resultUpdated').Values;
assert(ids.Data(end)==uint64(4))
assert(nnz(updates.Data)==4)
```

Both From Workspace blocks read paired, zero-order-held timeseries stored in the **model workspace before simulation**. All frames are intentionally preloaded for this test mode. Changing an unrelated MATLAB variable while the model runs does not update either source. Stop and reload the sequence to replace it.

If connecting your own source to the block, set its image port to exact model dimensions and `uint8`, trigger to logical, and ensure the completed image and trigger share the same discrete sample. Rate Transition or a synchronous subsystem is needed if your acquisition source runs at a different rate. Never update pixels incrementally while the trigger is high.

## Live MATLAB submissions during normal-mode simulation

Use the provided Level-2 MATLAB S-function source. Its supported `Update` callback reads a MATLAB mailbox once per sample, transfers a complete image into discrete state, and emits a one-sample event at the following output hit. `Outputs` never dequeues, which avoids duplicate consumption when Simulink evaluates a source more than once.

```matlab
model=create_simulink_model();
enable_live_source(model);    % changes the example sources; resets pending mailbox
open_system(model)
```

For interactive use, enable Simulation Pacing in the Simulink toolbar, then click Run. Pause the simulation from the toolbar, submit from the MATLAB Command Window, and resume:

```matlab
receipt=submit_live_image(imread('C:\images\photo.jpg'));
assert(receipt.accepted)
set_param(model,'SimulationCommand','continue') % resume a paused simulation
```

Pause again and call `submit_live_image` with the next RGB image whenever it is ready, then resume. This pause/submit/resume workflow ensures MATLAB is available to execute the submission and the image cannot be partly updated. A simulation execution listener or acquisition callback that runs on the MATLAB thread can also submit while the simulation runs; the automated live-transfer test exercises that path.

An accepted receipt means the frame was copied into the pending slot. It does not mean inference completed. If the slot is full, `accepted=false` and status -6 reject the request; retry after it is consumed. The source holds only its current frame plus one pending frame. A forced low sample separates requests. Receipt request IDs identify accepted mailbox entries; block result IDs identify processed requests. Reset the mailbox before a fresh live run:

```matlab
set_param(model,'SimulationCommand','stop')
ei_live_mailbox('reset')
set_param(model,'SimulationCommand','start')
```

The native engine also rejects concurrent/reentrant access with busy status. During synchronous inference, MATLAB callbacks on the same thread wait until inference returns; they never modify the buffer being read. A separate native caller must own its immutable input until return.

Fast Restart, accelerator modes, code generation and external real-time targets are outside the validated baseline. Regenerate the model to restore preloaded sources after trying live mode. Do not run multiple detector models/blocks against the singleton DLL concurrently.

The integration follows MathWorks' [MATLAB Function custom C code mechanism](https://www.mathworks.com/help/simulink/ug/incorporate-c-code-using-a-matlab-function-block.html): `coder.ceval`, simulation-target include paths and a native import library. `loadlibrary` is never called from generated block code.
