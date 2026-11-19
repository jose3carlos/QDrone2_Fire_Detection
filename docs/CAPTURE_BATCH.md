# Triggered QUARC camera batch capture

Paste `simulink/CaptureFOMOBatch.m` into a MATLAB Function block. It captures
prepared images only; it does not call the inference library or modify the
existing inference block. The current Fire export requires 180 x 180 grayscale
and squash/nearest-neighbor resizing. The conversion matches `prepare_image`.

## Connections

| Input | Connect | Type/size |
|---|---|---|
| rgb | Video3D Capture `image`, format MATLAB RGB | uint8 [480 640 3] |
| Trigger | Manual Switch between Boolean Constants false/true | logical scalar |
| newFrame | Video3D Capture `new` | logical scalar |
| frameNumber | Video3D Capture `#` | uint64 scalar |
| t | Digital Clock in seconds | double scalar |

Use Video3D Initialize as required by Quanser. Set Capture to a positive
discrete sample time, for example 1/30 second when compatible with your model's
base step. Run the MATLAB Function block and Digital Clock at that same rate.
The block must execute throughout acquisition, not only on the trigger edge.
Keep image, new flag and frame number synchronized. Use the actual frame number
and new flag, not dummy constants, with the real camera.

Quanser documents the MATLAB RGB H x W x 3 layout, `new` flag and uint64 frame
number at https://docs.quanser.com/quarc/documentation/video3d_capture_block.html.

## Configuration and timing

Edit `N = 20` and `CaptureSeconds = 4.0` at the top of the function before
simulation/build. N must be a positive integer. Changing it changes the fixed
output dimensions; leave the batch output dimensions inherited in Simulink.
These are compile-time settings, not signals that change during acquisition.

A sampled rising Trigger edge starts one attempt. For N=20 and 4 seconds,
nominal captures occur at offsets 0, 4/19, 8/19, ..., 4 seconds. Each slot uses
the first fresh camera frame at or after its target time. Camera sample timing
quantizes those times: this code permits at most 50 ms lateness, capped at half
the sample interval. Thus the nominal four-second burst can complete up to
50 ms late. The 30-fps simulated test completed at exactly 4.0 seconds.
Timing uses the supplied model clock, not MATLAB wall-clock `tic/toc`.

At most one image is captured per function execution. A missing/late frame or
repeated/backward frame number cannot generate fabricated catch-up images: if
the next deadline is missed, the attempt fails with Status=-2 and ImgCaptured
remains false. Camera frame rate and execution rate must be sufficient for the
requested N/duration. A backward or nonfinite clock invalidates the attempt.

## Outputs and handshake

- `Images`: uint8 [180 180 N], with the kth prepared image at `Images(:,:,k)`.
  N=20 occupies 648,000 bytes (about 0.62 MiB). Raw RGB frames are not retained.
- `ImgCaptured`: false initially and at each accepted start; true only after
  all N frames are present. It remains true until the next accepted start.
- `Status`: int32, 0 idle, 1 capturing, 2 complete, -1 invalid/backward clock,
  -2 missing/late frame. Terminate this port if not needed, but retain it while
  commissioning the camera to distinguish failure from ongoing capture.

Holding Trigger high never starts another batch. Lower then raise it after
completion to capture again. Dropping Trigger while busy does not cancel the
attempt; rising edges while busy are ignored and not queued. A failed attempt
also requires a new low-to-high trigger to retry. Restarting the simulation
resets all persistent state.

The batch is progressively filled while ImgCaptured=false. Do not consume it
until ImgCaptured=true and Status=2. The completed batch remains unchanged
until a new attempt is accepted. Do not request another batch until the future
consumer has finished using this one. The future consumer should respond to
the rising edge of ImgCaptured once, then handle individual Images(:,:,k)
matrices. The existing detector expects one 180 x 180 matrix, not the entire
three-dimensional batch.

## Verification

Five MATLAB R2023b/Simulink tests passed, including a compiled MATLAB Function
block, a 30-fps four-second schedule, held trigger, busy retrigger, missing and
duplicate frames, invalid clock, retained completed output, and pixel equality
with project preprocessing. See `build/capture_batch_validation.log`.
The actual QUARC device and target build were not available for validation.
