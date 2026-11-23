"""Arma placas 3MF (orientacion de impresion) para Creality Print, 350x350 mm.

Uso:  python armar_placas.py <dir_stl> <dir_salida> [placa ...]
Sin nombres arma todas; con nombres arma solo esas (para no sobrescribir placas editadas a mano).
"""
import sys, trimesh, numpy as np
from pathlib import Path
STL = Path(sys.argv[1]); OUT = Path(sys.argv[2]); OUT.mkdir(exist_ok=True); SOLO = set(sys.argv[3:])
GAP = 18.0
PLACAS = {
    "placa1_ROJO_fuegos_base": ["fuego_A_rojo", "fuego_B_rojo", "fuego_C_rojo"],
    "placa2_AMARILLO_fuegos_flama": ["fuego_A_amarillo", "fuego_B_amarillo", "fuego_C_amarillo"],
    "placa3_ROJO_valvulas_cuerpo_tuerca_zapata": ["valvula_cuerpo"]*3 + ["valvula_tuerca"]*3 + ["valvula_zapata_balero"]*3 + ["valvula_probeta_perno"],
    "placa4_ROJO_valvulas_vastago_volante": ["valvula_vastago_volante"]*3,
    "placa5_PETG_soporte_camara_A_B": ["soporte_camara_A_adaptador", "soporte_camara_B_abrazadera"],
}
faltan = SOLO - set(PLACAS)
if faltan: sys.exit(f"placas desconocidas: {sorted(faltan)}")
for nombre, piezas in PLACAS.items():
    if SOLO and nombre not in SOLO: continue
    mallas = [trimesh.load(STL / f"{p}.stl", force="mesh") for p in piezas]
    # acomodo en filas (shelf packing) dentro de 330 mm
    x = y = fila_h = 0.0; esc = trimesh.Scene()
    for p, m in zip(piezas, mallas):
        m = m.copy(); m.apply_translation(-m.bounds[0])
        w, d = m.extents[0], m.extents[1]
        if x + w > 330: x, y, fila_h = 0.0, y + fila_h + GAP, 0.0
        m.apply_translation([x, y, 0]); x += w + GAP; fila_h = max(fila_h, d)
        esc.add_geometry(m, node_name=f"{p}_{len(esc.geometry)}")
    b = esc.bounds; tam = b[1] - b[0]
    esc.apply_translation([175 - (b[0][0] + tam[0] / 2), 175 - (b[0][1] + tam[1] / 2), 0])
    esc.export(OUT / f"{nombre}.3mf")
    print(f"{nombre}: {len(piezas)} piezas, ocupa {tam[0]:.0f} x {tam[1]:.0f} mm, ok={tam[0] <= 340 and tam[1] <= 340}")
