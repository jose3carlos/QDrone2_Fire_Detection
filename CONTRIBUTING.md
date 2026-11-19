# Working on this project

Use Windows x64 and MATLAB R2023b. Follow [Installation](docs/INSTALLATION.md)
before changing code. Keep the existing folder layout and public entry points;
existing Simulink models depend on them.

- Change only `MODEL_PATH` in `config.m` when selecting another detector. Keep
  export ZIPs intact and preserve their upstream notices. The current adapter
  requires exactly one foreground class.
- Keep `Trayectory_Model.slx` in the repository root. Use `setup_custom_model`
  to refresh local bindings. Keep generated examples separate from custom models.
- Commit source, documentation and intentional model edits. Do not commit
  `build/`, compiler downloads, backups, MEX files or Simulink caches.
- After native, preprocessing or event-handling changes, run `run_validation`
  in MATLAB. For custom-model changes, also run
  `validate_custom_model_integration`. Report MATLAB/compiler versions and
  any unavailable hardware checks with your change.
- Review binary `.slx` changes explicitly; tests and setup can save refreshed
  paths into the standard example. Avoid committing incidental regeneration.

The repository does not assign a new license to third-party SDKs, detector
exports, MATLAB or Quanser software. Their original terms continue to apply.
