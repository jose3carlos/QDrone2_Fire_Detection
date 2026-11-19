# GitHub preparation validation — 2026-11-19

This publication preserves the working MATLAB R2023b source layout. Changes
are limited to documentation and Git metadata: the README setup walkthrough,
installation/transfer corrections, contribution notes, archive checksums,
ignore rules and binary/line-ending attributes. Existing `.m`, C/C++ and build
scripts, both `.slx` files and all three detector export ZIPs are unchanged.

## Checks performed for this publication

- Prepared a clean source copy without existing build artifacts, MEX files,
  Simulink caches, backups or the portable compiler folder.
- Built the active v7 Fire detector and native bridge from that copy using
  MinGW-w64 GCC 8.1 and MATLAB R2023b's bundled CMake 3.25.0. The compiler was
  supplied separately from the original local tool installation. Applied the
  existing guarded SDK adaptation only to a disposable extracted export.
- Ran the native algorithm test executable: passed confidence/tie/invalid-score
  cases and RGB/grayscale memory-layout checks.
- Ran the real-model integration test executable: passed initialization,
  inference, reset and lifecycle checks. Compiled metadata was 180 × 180,
  one class (`Fire`). Three synthetic black-image requests returned status 2
  (no detection) with successive request IDs. These are execution checks,
  not evidence of Fire recognition accuracy.
- Verified hashes of all 75 existing code, model and export files in the
  publication manifest against the working source after validation.
- Checked relative Markdown file links and exclusion of generated files.

Native logs are local under `build/github-publication/native-check/` and are
excluded from Git. The bundled CMake installation has no `ctest.exe`, so the
two test executables were invoked directly, as in `build_model.m`'s fallback.

## MATLAB/Simulink validation limit

Attempted a MATLAB R2023b batch run of the clean copy's
`setup_on_this_computer`, `run_validation` and custom-model validation workflow.
MATLAB remained in startup without executing the script; its local MathWorks
ServiceHost log reported an identity-token validation failure. The publication
attempt was stopped. Consequently, no new MATLAB, MEX, Simulink or camera test
results are claimed for this publication. Earlier results remain explicitly
identified as historical in [VALIDATION.md](VALIDATION.md).

After completing MATLAB sign-in/license setup on the destination computer,
follow [Installation](INSTALLATION.md) and run:

```matlab
setup_on_this_computer
results = run_validation();
setup_custom_model('Trayectory_Model')
report = validate_custom_model_integration();
```

The custom-model check uses a disposable copy with synthetic input. Camera
operation, recognition accuracy and Quanser target deployment still require
their own data, dependencies and hardware validation.
