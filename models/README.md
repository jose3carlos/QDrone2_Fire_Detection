# Detector exports

These are the original C++ Library export archives. Keep their filenames and
contents intact. The build extracts disposable SDK copies under `build/`.

| Archive | Role |
|---|---|
| `fire_hazard-cpp-mcu-v7-impulse-#1.zip` | Active default in `config.m`; Fire, 180 × 180 grayscale, one class |
| `fire_hazard-cpp-mcu-v6-impulse-#1.zip` | Previous Fire export, retained for reproducibility |
| `Lazara_Audrey.zip` | Historical two-class export; incompatible with the current single-class adapter |

The archives contain SDK/model sources and upstream license notices, and are
tracked directly in Git. No Edge Impulse account or API key is needed to build
these local exports. See [model switching](../docs/MODEL_SWITCHING.md) before
changing the active export.

`SHA256SUMS` records each archive's SHA-256 checksum. To inspect local hashes
from PowerShell, run `Get-FileHash .\models\*.zip -Algorithm SHA256`.
