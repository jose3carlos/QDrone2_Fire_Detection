# Inspection of the supplied archive

Original: `../person_recognition-cpp-mcu-v2-impulse-#2.zip`.

SHA-256, confirmed identical for `models/Lazara_Audrey.zip`:

```text
b2cb8ce388d56812f22d9c35ec23c9df0c015d6e2400dd406467eb04f77e8d7a
```

| Item | Actual export |
|---|---|
| Project / deployment | Person_Recognition / 1158666 / version 2 |
| SDK generation | Studio 1.97.4 |
| Image | 96×96, RGB, fit shortest resize |
| Class indices | 0 Audrey; 1 Lazara |
| Detector | FOMO (`EI_CLASSIFIER_LAST_LAYER_FOMO`, all YOLO flags zero) |
| Runtime | TensorFlow Lite Micro; EON/compiled flag 0 |
| Input/output tensor type | int8 / int8 |
| DSP | generated RGB image feature extractor; 9,216 packed pixel samples → 27,648 features |
| TFLM arena | 182,579 bytes |
| Exported detection threshold | 0.5 |
| Detection array | use `ei_impulse_result_t.bounding_boxes_count`, not the minimum count macro 10 |
| Operator resolver | Add, Conv2D, DepthwiseConv2D, Softmax |
| Tracking | disabled |

Relevant files inspected before adapter development:

- `model-parameters/model_metadata.h`: dimensions, detector, runtime and feature macros.
- `model-parameters/model_variables.h`: actual labels, generated RGB DSP and learning/postprocessing configurations, `ei_default_impulse`.
- `tflite-model/tflite_learn_1158666_5.{h,cpp,tflite}`: compiled model byte array and original model.
- `tflite-model/tflite-resolver.h`: required TensorFlow Lite Micro operators.
- `edge-impulse-sdk/classifier/ei_run_classifier.h`: actual `run_classifier_init`, `run_classifier`, `run_classifier_deinit` functions.
- `edge-impulse-sdk/classifier/ei_classifier_types.h`: bounding boxes, confidence and timing structs.
- `edge-impulse-sdk/classifier/inferencing_engines/tflite_micro.h`: setup, Invoke and tensor extraction.
- `edge-impulse-sdk/tensorflow/`, `dsp/`, `third_party/`: bundled implementation dependencies.

The exported top-level CMake file is aimed at an `app` target and Zephyr/embedded sources. The project CMake selects portable TensorFlow Lite Micro and DSP sources and provides desktop timers/allocation/logging. ARM CMSIS/ESP acceleration is disabled. Compilation used MinGW 8.1 in GNU C++17 mode; this SDK uses designated-initializer extensions. The optional MSVC route requires C++20 and remains separately unverified.

## Persistent inference resources

The original SDK creates/deletes a MicroInterpreter for each inference. To satisfy this project's persistent-initialization requirement, `scripts/ei_patch_sdk.m` replaces **only the setup/lifetime implementation in a disposable extracted copy** with `native/ei_tflm_cache.inc`. It guards the original setup signature and fails if the expected SDK shape is absent. It does not replace DSP, tensor quantization, Invoke, output extraction or object-detection postprocessing.

`ei_initialize` allocates the interpreter and aligned arena once, without a neural Invoke. Each inference resets interpreter state using the actual exported `MicroInterpreter::Reset()` API so independent images cannot inherit variable state. Termination releases the interpreter before the arena. The supplied TFLM implementation, resolver and trained model remain authoritative.

Upstream model headers and the exported SDK include license terms. Their original notices are preserved in the model ZIP and extracted copies.
