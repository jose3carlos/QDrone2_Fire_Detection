# -*- coding: utf-8 -*-
"""
Arma las fotos NEGATIVAS (sin fuegos ni tomas) de COCO val2017 para reentrenar contra falsos positivos,
y un conjunto aparte, nunca usado al entrenar, para medir la tasa de falsos positivos.

Con la webcam el modelo sólo-sintético confundía caras, personas y objetos con "toma" (y a veces "fuego").
Estas fotos van con etiqueta vacía (= "aquí no hay nada"). Ver vision/README.md, "Negativos".

Fuente (no va a git; val2017 ocupa ~0.8 GB): en %USERPROFILE%\\datasets\\coco
  val2017\\*.jpg                              <- http://images.cocodataset.org/zips/val2017.zip
  annotations\\instances_val2017.json         <- http://images.cocodataset.org/annotations/annotations_trainval2017.zip

Salida (fuera de OneDrive y de git):
  %USERPROFILE%\\datasets\\qdrone2_vision\\coco_negativos\\images\\{train,val} + labels\\{train,val} (vacías)
      train: 1000 con personas + 500 sin personas; val: 70 + 30
  %USERPROFILE%\\datasets\\qdrone2_vision\\coco_negativos_eval\\images + labels (300 con personas + 200 sin personas)
      sólo para medir falsos positivos; NO se pasa a entrenar.py
  coco_negativos\\seleccion.json (nombres elegidos) y coco_negativos\\LEEME.txt

La selección es determinista (orden por md5 del nombre de archivo): correrlo otra vez da las mismas fotos.
Se excluyen las fotos con "fire hydrant" (se parece demasiado a una válvula; no se enseña como "nada").

  python vision/negativos_coco.py                 (copia las fotos; sólo biblioteca estándar)
  python vision/negativos_coco.py --comprobar     (no copia: compara la selección con seleccion.json)
"""
import argparse
import hashlib
import json
import os
import shutil
import sys
from collections import defaultdict

U = os.path.expanduser("~")
COCO = os.path.join(U, "datasets", "coco")
DST = os.path.join(U, "datasets", "qdrone2_vision", "coco_negativos")
EVAL = os.path.join(U, "datasets", "qdrone2_vision", "coco_negativos_eval")
EXCLUIR = {"fire hydrant"}


def seleccionar(coco):
    ruta_ann = os.path.join(coco, "annotations", "instances_val2017.json")
    with open(ruta_ann, encoding="utf-8") as f:
        ann = json.load(f)
    cat = {c["id"]: c["name"] for c in ann["categories"]}
    cats_img = defaultdict(set)
    for a in ann["annotations"]:
        cats_img[a["image_id"]].add(cat[a["category_id"]])
    imgs = sorted(ann["images"], key=lambda im: hashlib.md5(im["file_name"].encode()).hexdigest())
    ok = [im for im in imgs if not (cats_img[im["id"]] & EXCLUIR)]
    persona = [im for im in ok if "person" in cats_img[im["id"]]]
    otras = [im for im in ok if "person" not in cats_img[im["id"]]]
    print("COCO val2017:", len(imgs), "| sin excluidas:", len(ok), "| con persona:", len(persona),
          "| sin persona:", len(otras))
    return {
        "train": persona[:1000] + otras[:500],
        "val": persona[1000:1070] + otras[500:530],
        "eval": persona[1070:1370] + otras[530:730],
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--coco", default=COCO, help="carpeta con val2017/ y annotations/ (por defecto %(default)s)")
    ap.add_argument("--salida", default=DST, help="negativos para entrenar (por defecto %(default)s)")
    ap.add_argument("--salida_eval", default=EVAL, help="negativos apartados para medir (por defecto %(default)s)")
    ap.add_argument("--comprobar", action="store_true",
                    help="no copia nada: compara la selección con <salida>/seleccion.json")
    a = ap.parse_args()

    if not os.path.isfile(os.path.join(a.coco, "annotations", "instances_val2017.json")):
        sys.exit("falta %s\\annotations\\instances_val2017.json (y val2017\\): descárgalos de "
                 "images.cocodataset.org (ver la ayuda: -h)" % a.coco)
    sel = seleccionar(a.coco)
    nombres = {k: [im["file_name"] for im in v] for k, v in sel.items()}

    if a.comprobar:
        ruta = os.path.join(a.salida, "seleccion.json")
        with open(ruta, encoding="utf-8") as f:
            previa = json.load(f)
        iguales = all(previa.get(k) == v for k, v in nombres.items())
        for k, v in nombres.items():
            print("%-5s %4d fotos, igual a seleccion.json: %s" % (k, len(v), previa.get(k) == v))
        sys.exit(0 if iguales else 1)

    for split, lst in sel.items():
        base = a.salida_eval if split == "eval" else a.salida
        sub = "" if split == "eval" else split
        di = os.path.join(base, "images", sub)
        dl = os.path.join(base, "labels", sub)
        os.makedirs(di, exist_ok=True)
        os.makedirs(dl, exist_ok=True)
        for im in lst:
            n = "coco_" + im["file_name"]
            shutil.copy2(os.path.join(a.coco, "val2017", im["file_name"]), os.path.join(di, n))
            open(os.path.join(dl, os.path.splitext(n)[0] + ".txt"), "w").close()   # vacío = sin objetos
        print(split, len(lst), "->", di)
    with open(os.path.join(a.salida, "seleccion.json"), "w") as f:
        json.dump(nombres, f, indent=0)
    with open(os.path.join(a.salida, "LEEME.txt"), "w", encoding="utf-8") as f:
        f.write(
            "Negativos (fotos sin fuegos ni tomas) tomados de COCO val2017 (images.cocodataset.org, licencia CC BY 4.0 de las anotaciones;\n"
            "las imágenes conservan la licencia de Flickr de cada autor). Se excluyeron las fotos con 'fire hydrant'.\n"
            "train: 1000 con personas + 500 sin personas; val: 70 + 30. coco_negativos_eval (300 + 200) NO se usa para entrenar:\n"
            "sirve para medir la tasa de falsos positivos. Generado por vision/negativos_coco.py.\n")


if __name__ == "__main__":
    main()
