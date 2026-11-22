# Fotos reales

Aquí van las fotos reales del laboratorio. No se suben a git (se mueven con `vision/paquete.py`); ver
`vision/README.md`, sección 4.

- `crudas/<sesion>/`: salida de `capturar_fotos.py`.
- `images/` + `labels/`: fotos etiquetadas (partición 80/20 fija por md5 del nombre).
- `images/val/` + `labels/val/`: sesión reservada para validación (se copia sin pre-etiquetas).
- `revisadas.txt`: fotos que una persona ya revisó con `etiquetar.py revisar`. **Sólo éstas** se usan para
  entrenar y evaluar; las pre-etiquetas automáticas sin revisar se ignoran.
