# -*- coding: utf-8 -*-
"""
Válvula de compuerta cosmética tipo "toma clandestina" que se abraza a un tubo
de PVC 3/4" SCH40 (DE 26.67 mm) tendido en el piso.

Piezas (todas en PLA rojo):
  1. valvula_cuerpo            -> abrazadera en "C" + niple octagonal + cuerpo de válvula
  2. valvula_tuerca            -> tuerca hexagonal con rosca interna M16x2 (inserto)
  3. valvula_vastago_volante   -> volante + vástago con rosca externa M16x2 + perno 8 mm
  4. valvula_zapata_balero     -> zapata con alojamiento para balero 608-2ZZ
  5. valvula_zapata_sin_balero -> zapata alternativa sin balero (agujero 8.4 + 3 costillas a Ø7.8)
  6. valvula_probeta_perno     -> (opcional) probeta con pernos Ø8.0/8.1/8.2 para calibrar el ajuste
Revisión 2: tuerca CAUTIVA en ranura lateral (entra por la cara +X del brazo), filete lomo-mandíbula R8,
labio 2.0, asiento del balero con alivio cónico, perno 8.1, costillas de retención imprimibles.

Sistema de coordenadas de USO:
  eje del tubo = Y, eje del tornillo = Z (arriba), X = dirección de la boca de la C
  (la C abre hacia +X). z = 0 es la cara superior de la mandíbula inferior, donde
  descansa el tubo. El centro del tubo está en (0, y, R_TUBO).
Todas las cotas en mm. La posición "apretada" (zapata tocando el tubo) es la
referencia para el vástago: desplazamiento s = 0; s > 0 = vástago levantado.

Ejecutar:  python valvula.py
"""
import math
import os
import sys

import numpy as np
import trimesh
import manifold3d
from build123d import (Align, Axis, Box, Cone, Cylinder, Part, Plane, Polygon,
                       Pos, RegularPolygon, Rot, Solid, Compound, Location,
                       export_step, export_stl, extrude)
from bd_warehouse.thread import IsoThread

# ============================================================================
# PARÁMETROS (todas las cotas son editables)
# ============================================================================
# --- Tubo ---
DE_TUBO = 26.67                 # diámetro exterior PVC 3/4" SCH40
R_TUBO = DE_TUBO / 2
Z_TUBO_SUP = DE_TUBO            # parte superior del tubo (descansa en z=0)

# --- Cuerpo en "C" (perfil en XZ, extruido en Y) ---
D = 30.0                        # profundidad del cuerpo a lo largo del tubo (Y)
HOLGURA_LOMO = 1.5              # holgura entre lomo y tubo
T_LOMO = 7.0                    # espesor del lomo (-X)
T_MANDIBULA = 4.0               # espesor de la mandíbula inferior
X_LABIO_INT = 9.0               # cara interior del labio (tope del tubo)
H_LABIO = 2.0                   # altura del labio sobre la mandíbula (rev.: 2.5 -> 2.0, levantar tubo 6 mm)
W_LABIO_SUP = 1.2               # ancho de la cresta del labio
ANG_RAMPA = 35.0                # rampa exterior del labio (grados sobre horizontal)
H_PUNTA = 1.0                   # altura de la punta de la rampa sobre el piso
X_BRAZO_FIN = 17.0              # extremo del brazo superior en +X
R_FILETE_INT_INF = 8.0          # filete interior lomo-mandíbula (rev.: 4 -> 8, acorta el brazo de palanca)
R_FILETE_INT_SUP = 2.5          # filete interior lomo-brazo
R_FILETE_EXT = 3.0              # filetes exteriores

# --- Tuerca (inserto) y su alojamiento ---
ROSCA_D = 16.0                  # M16
ROSCA_P = 2.0                   # paso 2 mm
ROSCA_D_INT = 16.4              # diámetro mayor de la rosca interna (holgura impresión)
ROSCA_D_EXT = 15.7              # diámetro mayor de la rosca externa (holgura impresión)
TUERCA_EC = 22.0                # entre caras del hexágono
TUERCA_H = 11.0                 # altura de la tuerca
TUERCA_CHAFLAN = 0.6            # chaflán de aristas exteriores
TUERCA_AVELLANADO = 0.6         # avellanado de entrada de la rosca (profundidad)
HOLG_ALOJ = 0.4                 # holgura entre caras de la ranura/alojamiento hexagonal
HOLG_ALOJ_Z = 0.3               # holgura en altura de la ranura (la tuerca entra deslizando)
T_PISO_TUERCA = 3.0             # piso bajo la tuerca: la deja CAUTIVA (rev.: antes abierta hacia abajo)
T_HOMBRO = 4.0                  # material sobre la tuerca (hombro de apoyo)
# La tuerca entra de LADO por una ranura que abre en la cara +X del brazo superior,
# con caras planas perpendiculares a Y (no puede girar) y piso + hombro (no puede subir ni bajar).
D_BARRENO = 17.0                # barreno libre del vástago (gota hacia +Y)

# --- Niple, cuerpo de válvula y bonete ---
H_NIPLE = 6.0                   # niple octagonal (entre caras = D)
VB_X = 46.0                     # cuerpo de válvula, largo en X
VB_H = 28.0                     # cuerpo de válvula, alto
VB_CHAFLAN = 2.5
PUERTO_X_FIN = 33.0             # extremo de los puertos (sin brida)
PUERTO_ALTO = 20.0              # alto del puerto (octágono alargado Y30 x Z20)
PUERTO_CHAFLAN = 5.0
BRIDA_T = 4.0                   # espesor de la brida (octágono regular EC = D)
BOCA_PUERTO_D = 14.0            # boca decorativa en la brida (gota)
BOCA_PUERTO_PROF = 5.0
BONETE_X = 26.0
BONETE_H = 10.0
BONETE_CHAFLAN = 6.0

# --- Zapata ---
ZAP_X = 27.0                    # cabe entre lomo (holgura 1.3) y con holgura en la boca
ZAP_Y = 28.0
R_ZAPATA = 13.5                 # radio de la cara cóncava
ANG_ABRAZO = 45.0               # abrazo ± grados
BAL_DE = 22.0                   # balero 608-2ZZ
BAL_DI = 8.0
BAL_W = 7.0
ALOJ_BAL_D = 22.15              # alojamiento a presión
ALOJ_BAL_PROF = 7.0
ASIENTO_R_INT = 9.6             # el asiento plano sólo va de r=9.6 a r=11.075 (cara de la pista exterior)
ALIVIO_PROF = 2.0               # alivio cónico a 45° hacia adentro (r 9.6 -> 7.6) y puente de Ø15.2
PISO_ZAPATA = 4.0               # material entre el ápice cóncavo y el asiento del balero (rev.: 3.5 -> 4.0)
ZAP_CHAFLAN_V = 3.0
ZAP_CHAFLAN_SUP = 0.8
AGUJ_SIN_BAL_D = 8.4            # zapata sin balero
AGUJ_SIN_BAL_PROF = 7.5
COSTILLA_D = 7.8                # diámetro efectivo de las 3 costillas (0.30 mm salientes; 0.15 mm de apriete
                                # por lado contra el perno de 8.1)

# --- Vástago y volante ---
PERNO_D = 8.1                   # perno para la pista interior (a presión; rev.: 8.0 -> 8.1)
PROBETA_D = (8.0, 8.1, 8.2)     # probeta de ajuste del perno (imprimir primero)
PERNO_L = 7.0
CUELLO_D = 11.0                 # hombro que empuja sólo la pista interior
CUELLO_L = 2.0
ROSCA_L = 33.0                  # longitud roscada (rev.: 30 -> 33 por la tuerca 3 mm más alta)
VOLANTE_D = 66.0
VOLANTE_ARO_ANCHO = 7.0
VOLANTE_ARO_T = 8.0
VOLANTE_RAYOS = 5
VOLANTE_RAYO_ANCHO = 7.0
VOLANTE_RAYO_T = 6.0
VOLANTE_MUESCAS = 12            # muescas de agarre en el aro
VOLANTE_MUESCA_R = 3.0
CUBO_D = 24.0
CUBO_H = 13.0
CUBO_CHAFLAN = 3.0
HOLG_CUBO = 6.0                 # holgura cubo-bonete en posición apretada

# --- Cinemática ---
CARRERA_ABIERTA = 12.0          # elevación de la zapata para pasar el tubo (6 vueltas)
HOLG_TOPE = 2.0                 # zapata - brazo en posición abierta (tope físico)

# ============================================================================
# COTAS DERIVADAS (posición apretada, coordenadas de uso)
# ============================================================================
X_LOMO_INT = -(R_TUBO + HOLGURA_LOMO)
X_LOMO_EXT = X_LOMO_INT - T_LOMO
Z_MAND_INF = -T_MANDIBULA
X_PUNTA = X_LABIO_INT + W_LABIO_SUP + (H_LABIO - (Z_MAND_INF + H_PUNTA)) / math.tan(math.radians(ANG_RAMPA))

ZC_ZAPATA = Z_TUBO_SUP - R_ZAPATA                         # centro de la cara cóncava
Z_ZAP_BORDE = ZC_ZAPATA + R_ZAPATA * math.cos(math.radians(ANG_ABRAZO))  # punto más bajo
Z_ZAP_SUP = Z_TUBO_SUP + PISO_ZAPATA + ALOJ_BAL_PROF       # cara superior de la zapata
Z_HOMBRO_VAST = Z_ZAP_SUP                                  # hombro del vástago sobre la pista interior
Z_PERNO_INF = Z_HOMBRO_VAST - PERNO_L
Z_ROSCA_0 = Z_HOMBRO_VAST + CUELLO_L
Z_ROSCA_1 = Z_ROSCA_0 + ROSCA_L

Z_BRAZO_INF = Z_ZAP_SUP + CARRERA_ABIERTA + HOLG_TOPE      # cara inferior del brazo superior
Z_RANURA_INF = Z_BRAZO_INF + T_PISO_TUERCA                 # piso de la ranura de la tuerca
Z_ALOJ_SUP = Z_RANURA_INF + TUERCA_H + HOLG_ALOJ_Z         # hombro donde apoya la tuerca
Z_TUERCA_SUP = Z_ALOJ_SUP                                  # apretado: la tuerca sube contra el hombro
Z_TUERCA_INF = Z_TUERCA_SUP - TUERCA_H
Z_BRAZO_SUP = Z_ALOJ_SUP + T_HOMBRO
Z_NIPLE_SUP = Z_BRAZO_SUP + H_NIPLE
Z_VB_INF = Z_NIPLE_SUP
Z_VB_SUP = Z_VB_INF + VB_H
Z_BONETE_SUP = Z_VB_SUP + BONETE_H
Z_CUBO_INF = Z_BONETE_SUP + HOLG_CUBO
Z_VOLANTE_SUP = Z_CUBO_INF + CUBO_H

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)
DIR_STL = os.path.join(RAIZ, "stl")
DIR_STEP = os.path.join(RAIZ, "step")
DIR_REND = os.path.join(RAIZ, "renders")
DIR_USO = os.path.join(DIR_REND, "_mallas_uso")
for d in (DIR_STL, DIR_STEP, DIR_REND, DIR_USO):
    os.makedirs(d, exist_ok=True)


# ============================================================================
# UTILIDADES
# ============================================================================
def octagono(w, h, c):
    """Puntos de un rectángulo w x h centrado con esquinas achaflanadas c."""
    a, b = w / 2, h / 2
    return [(a - c, -b), (a, -b + c), (a, b - c), (a - c, b),
            (-a + c, b), (-a, b - c), (-a, -b + c), (-a + c, -b)]


def gota(r, punta="v", n=48):
    """Perfil de gota (círculo + punta a 45°) con la punta hacia +v local (o +u)."""
    pts = []
    for i in range(n + 1):
        ang = math.radians(135 + 270 * i / n)   # de 135° a 405° (pasando por abajo)
        pts.append((r * math.cos(ang), r * math.sin(ang)))
    pts.append((0.0, r * math.sqrt(2)))
    if punta == "u":
        pts = [(v, -u) for (u, v) in pts]
    return pts


def aristas_en(parte, puntos, eje, tol=0.05):
    """Aristas paralelas a 'eje' cuyo centro coincide (en el plano normal) con algún punto."""
    sel = []
    for e in parte.edges():
        try:
            dirv = e.tangent_at(0.5)
        except Exception:
            continue
        comp = {"X": abs(dirv.X), "Y": abs(dirv.Y), "Z": abs(dirv.Z)}[eje]
        if comp < 0.999:
            continue
        c = e.center()
        for (p, q) in puntos:
            if eje == "Y" and abs(c.X - p) < tol and abs(c.Z - q) < tol:
                sel.append(e)
            elif eje == "Z" and abs(c.X - p) < tol and abs(c.Y - q) < tol:
                sel.append(e)
    return sel


def sol(x):
    """Convierte a Solid único (verifica que sea un solo cuerpo válido)."""
    s = x.solids()
    assert len(s) == 1, f"se esperaban 1 sólido, hay {len(s)}"
    assert s[0].is_valid, "sólido inválido"
    return s[0]


# ============================================================================
# 1. CUERPO
# ============================================================================
def hacer_cuerpo():
    # Perfil en C en el plano XZ (Plane.XZ: x local = X, y local = Z)
    pts = [
        (X_LOMO_EXT, Z_MAND_INF),
        (X_PUNTA, Z_MAND_INF),
        (X_PUNTA, Z_MAND_INF + H_PUNTA),
        (X_LABIO_INT + W_LABIO_SUP, H_LABIO),
        (X_LABIO_INT, H_LABIO),
        (X_LABIO_INT, 0.0),
        (X_LOMO_INT, 0.0),
        (X_LOMO_INT, Z_BRAZO_INF),
        (X_BRAZO_FIN, Z_BRAZO_INF),
        (X_BRAZO_FIN, Z_BRAZO_SUP),
        (X_LOMO_EXT, Z_BRAZO_SUP),
    ]
    perfil = Plane.XZ * Polygon(*pts, align=None)
    c = sol(extrude(perfil, amount=D / 2, both=True))
    # filetes del perfil (aristas paralelas a Y): todos se imprimen como paredes verticales
    c = c.fillet(R_FILETE_INT_INF, aristas_en(c, [(X_LOMO_INT, 0.0)], "Y"))
    c = c.fillet(R_FILETE_INT_SUP, aristas_en(c, [(X_LOMO_INT, Z_BRAZO_INF)], "Y"))
    c = c.fillet(R_FILETE_EXT, aristas_en(c, [(X_LOMO_EXT, Z_MAND_INF), (X_LOMO_EXT, Z_BRAZO_SUP)], "Y"))
    c = c.fillet(2.0, aristas_en(c, [(X_BRAZO_FIN, Z_BRAZO_SUP), (X_BRAZO_FIN, Z_BRAZO_INF)], "Y"))
    c = c.fillet(0.5, aristas_en(c, [(X_PUNTA, Z_MAND_INF + H_PUNTA), (X_LABIO_INT + W_LABIO_SUP, H_LABIO),
                                     (X_LABIO_INT, H_LABIO)], "Y"))

    # Niple octagonal (EC = D: sus caras planas coinciden con y = ±D/2)
    niple = Pos(0, 0, Z_BRAZO_SUP - 0.5) * extrude(
        RegularPolygon(D / 2, 8, major_radius=False, rotation=22.5), amount=H_NIPLE + 1.0)

    # Cuerpo de la válvula
    vb = Pos(0, 0, Z_VB_INF + VB_H / 2) * Box(VB_X, D, VB_H)
    vb = sol(vb).chamfer(VB_CHAFLAN, None, sol(vb).edges())
    zc_p = Z_VB_INF + VB_H / 2
    # Puertos de entrada/salida (octágono alargado Y=D x Z=PUERTO_ALTO) y bridas (octágono regular)
    c_reg = D * (2 - math.sqrt(2)) / 2
    piezas = [c, niple, vb]
    for sgn in (1, -1):
        x0 = sgn * (VB_X / 2 - 1.0)
        L = PUERTO_X_FIN - VB_X / 2 + 1.0
        pl = Plane(origin=(x0, 0, zc_p), x_dir=(0, 1, 0), z_dir=(sgn, 0, 0))
        puerto = extrude(pl * Polygon(*octagono(D, PUERTO_ALTO, PUERTO_CHAFLAN), align=None), amount=L)
        plb = Plane(origin=(sgn * (PUERTO_X_FIN - 0.5), 0, zc_p), x_dir=(0, 1, 0), z_dir=(sgn, 0, 0))
        brida = extrude(plb * Polygon(*octagono(D, D, c_reg), align=None), amount=BRIDA_T + 0.5)
        piezas += [puerto, brida]
    # Bonete (octágono alargado en XY)
    bon = Pos(0, 0, Z_VB_SUP - 0.5) * extrude(Polygon(*octagono(BONETE_X, D, BONETE_CHAFLAN), align=None),
                                               amount=BONETE_H + 0.5)
    bon = sol(bon)
    bon = bon.chamfer(1.5, None, [e for e in bon.edges() if abs(e.center().Z - Z_BONETE_SUP) < 0.01])
    piezas.append(bon)

    cuerpo = piezas[0]
    for p in piezas[1:]:
        cuerpo = cuerpo.fuse(p)
    cuerpo = cuerpo.clean()

    # --- Cortes ---
    # Ranura de la tuerca CAUTIVA: hexágono (caras planas perpendiculares a Y, vértice hacia -X) en el eje
    # + canal recto de ancho EC hacia +X que sale por la cara extrema del brazo. La tuerca entra de lado,
    # el piso (T_PISO_TUERCA) y el hombro (T_HOMBRO) la sujetan en Z, las paredes y = ±EC/2 impiden su giro
    # y el vástago que la atraviesa impide que salga en X. Al imprimir de lado el techo de la ranura
    # (y = +EC/2) es un puente de sólo TUERCA_H + holgura (~11.3 mm) y el extremo -X son caras a 30° de la vertical.
    h_ran = TUERCA_H + HOLG_ALOJ_Z
    aloj = Pos(0, 0, Z_RANURA_INF) * extrude(
        RegularPolygon((TUERCA_EC + HOLG_ALOJ) / 2, 6, major_radius=False), amount=h_ran)
    canal = Pos((X_BRAZO_FIN + 1.0) / 2, 0, Z_RANURA_INF + h_ran / 2) * Box(
        X_BRAZO_FIN + 1.0, TUERCA_EC + HOLG_ALOJ, h_ran)
    # Barreno del vástago en gota (punta hacia +Y: al imprimir de lado no requiere soportes); atraviesa
    # el piso, la ranura, el hombro, el niple, el cuerpo y el bonete.
    z0 = Z_BRAZO_INF - 1.0
    barreno = Pos(0, 0, z0) * extrude(Polygon(*gota(D_BARRENO / 2), align=None), amount=Z_BONETE_SUP - z0 + 1)
    # chaflán de entrada del barreno en la cara inferior del brazo
    entrada = Pos(0, 0, Z_BRAZO_INF - 0.01) * extrude(Polygon(*gota(D_BARRENO / 2 + 0.8), align=None),
                                                       amount=0.8, taper=45)
    # Bocas decorativas en las bridas (gota con punta hacia +Y)
    bocas = []
    for sgn in (1, -1):
        xb = sgn * (PUERTO_X_FIN + BRIDA_T + 0.01)
        pl = Plane(origin=(xb, 0, zc_p), x_dir=(0, 1, 0), z_dir=(-sgn, 0, 0))
        bocas.append(extrude(pl * Polygon(*gota(BOCA_PUERTO_D / 2, punta="u"), align=None), amount=BOCA_PUERTO_PROF))
    for h in [aloj, canal, barreno, entrada] + bocas:
        cuerpo = cuerpo.cut(h)
    return sol(cuerpo.clean())


# ============================================================================
# 2. TUERCA
# ============================================================================
def hacer_tuerca():
    amin = (Align.CENTER, Align.CENTER, Align.MIN)
    H = TUERCA_H
    rosca = IsoThread(major_diameter=ROSCA_D_INT, pitch=ROSCA_P, length=H, external=False,
                      end_finishes=("fade", "fade"))
    # tubo del mismo largo que la rosca + rosca (unión robusta), luego se recorta con el hexágono
    tubo = Cylinder(TUERCA_EC, H, align=amin) - Cylinder(ROSCA_D_INT / 2, H, align=amin)
    roscado = tubo + rosca
    hexa = sol(extrude(RegularPolygon(TUERCA_EC / 2, 6, major_radius=False), amount=H))
    hexa = hexa.chamfer(TUERCA_CHAFLAN, None,
                        [e for e in hexa.edges() if abs(e.center().Z) < 1e-3 or abs(e.center().Z - H) < 1e-3])
    t = roscado & hexa
    # avellanados de entrada a 45° en ambas caras
    r0 = ROSCA_D_INT / 2
    for z, rot in ((0.0, 0), (H, 180)):
        cono = Pos(0, 0, z + (-0.001 if rot == 0 else 0.001)) * Rot(rot, 0, 0) * Cone(
            r0 + TUERCA_AVELLANADO, r0 - 0.5, TUERCA_AVELLANADO + 0.5, align=amin)
        t = t - cono
    return sol(t)


# ============================================================================
# 3. VÁSTAGO + VOLANTE (coordenadas de uso, posición apretada, ángulo 0)
# ============================================================================
def hacer_vastago():
    amin = (Align.CENTER, Align.CENTER, Align.MIN)
    perno = Pos(0, 0, Z_PERNO_INF) * Cylinder(PERNO_D / 2, PERNO_L + 0.5, align=amin)
    perno = sol(perno)
    perno = perno.chamfer(0.6, None, [e for e in perno.edges() if abs(e.center().Z - Z_PERNO_INF) < 1e-3])
    cuello = Pos(0, 0, Z_HOMBRO_VAST) * Cylinder(CUELLO_D / 2, CUELLO_L + 0.5, align=amin)
    rosca = IsoThread(major_diameter=ROSCA_D_EXT, pitch=ROSCA_P, length=ROSCA_L, external=True,
                      end_finishes=("fade", "fade"))
    r_nucleo = rosca.min_radius
    # núcleo roscado (se fusiona primero, mismo largo que la rosca: unión robusta)
    roscado = Cylinder(r_nucleo, ROSCA_L, align=amin) + rosca
    roscado = Pos(0, 0, Z_ROSCA_0) * sol(roscado)
    # eje liso (mismo diámetro que la raíz: pasa libre por la tuerca)
    eje = Pos(0, 0, Z_ROSCA_1 - 0.5) * Cylinder(r_nucleo, Z_CUBO_INF - Z_ROSCA_1 + 1.0, align=amin)
    # cubo con chaflán cónico hacia el eje
    cubo = sol(Pos(0, 0, Z_CUBO_INF) * Cylinder(CUBO_D / 2, CUBO_H, align=amin))
    cubo = cubo.chamfer(CUBO_CHAFLAN, None, [e for e in cubo.edges() if abs(e.center().Z - Z_CUBO_INF) < 1e-3])
    # aro del volante
    r_ext = VOLANTE_D / 2
    r_int = r_ext - VOLANTE_ARO_ANCHO
    aro = Pos(0, 0, Z_VOLANTE_SUP - VOLANTE_ARO_T) * (Cylinder(r_ext, VOLANTE_ARO_T, align=amin) -
                                                     Cylinder(r_int, VOLANTE_ARO_T, align=amin))
    aro = sol(aro)
    zb = Z_VOLANTE_SUP - VOLANTE_ARO_T
    aro = aro.fillet(2.0, [e for e in aro.edges() if abs(e.center().Z - zb) < 1e-3])
    for i in range(VOLANTE_MUESCAS):
        a = 2 * math.pi * (i + 0.5) / VOLANTE_MUESCAS
        m = Pos(r_ext * math.cos(a) * 1.02, r_ext * math.sin(a) * 1.02, zb - 1) * Cylinder(
            VOLANTE_MUESCA_R, VOLANTE_ARO_T + 2, align=amin)
        aro = aro.cut(m)
    # rayos
    L_rayo = r_int - CUBO_D / 2 + 3.0
    rayos = []
    for i in range(VOLANTE_RAYOS):
        ang = 360.0 * i / VOLANTE_RAYOS + 90.0
        rc = CUBO_D / 2 - 1.5 + L_rayo / 2
        r = Rot(0, 0, ang) * Pos(rc, 0, Z_VOLANTE_SUP - VOLANTE_RAYO_T / 2) * Box(L_rayo, VOLANTE_RAYO_ANCHO,
                                                                                 VOLANTE_RAYO_T)
        rayos.append(r)
    v = roscado
    for p in [perno, cuello, eje, cubo, aro] + rayos:
        v = v + p
    return sol(v.clean())


# ============================================================================
# 4/5. ZAPATAS
# ============================================================================
def bloque_zapata():
    h = Z_ZAP_SUP - Z_ZAP_BORDE
    b = sol(Pos(0, 0, Z_ZAP_BORDE + h / 2) * Box(ZAP_X, ZAP_Y, h))
    b = b.chamfer(ZAP_CHAFLAN_V, None, [e for e in b.edges()
                                         if abs(e.tangent_at(0.5).Z) > 0.999])
    b = b.chamfer(ZAP_CHAFLAN_SUP, None, [e for e in b.edges()
                                           if abs(e.center().Z - Z_ZAP_SUP) < 1e-3])
    concavo = Pos(0, 0, ZC_ZAPATA) * Rot(90, 0, 0) * Cylinder(R_ZAPATA, ZAP_Y + 4)
    return sol(b.cut(concavo))


def hacer_zapata_balero():
    z = bloque_zapata()
    aloj = Pos(0, 0, Z_ZAP_SUP - ALOJ_BAL_PROF) * Cylinder(ALOJ_BAL_D / 2, ALOJ_BAL_PROF + 1,
                                                          align=(Align.CENTER, Align.CENTER, Align.MIN))
    # Alivio cónico a 45°: el asiento plano queda sólo bajo la pista exterior (r 9.6..11.075, voladizo de
    # ~1.5 mm al imprimir boca abajo); hacia adentro el cono es autoportante y termina en un puente de Ø15.2
    # que queda 2 mm lejos de la pista interior (si el puente cuelga un poco, no roza el balero).
    r_puente = ASIENTO_R_INT - ALIVIO_PROF
    alivio = Pos(0, 0, Z_ZAP_SUP - ALOJ_BAL_PROF - ALIVIO_PROF) * Cone(
        r_puente, ASIENTO_R_INT + 0.01, ALIVIO_PROF + 0.01, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # chaflán de entrada 0.5 mm (también compensa la "pata de elefante" al imprimir boca abajo)
    ch = Pos(0, 0, Z_ZAP_SUP + 0.001) * Rot(180, 0, 0) * Cone(ALOJ_BAL_D / 2 + 0.5, ALOJ_BAL_D / 2 - 0.5, 1.0,
                                                               align=(Align.CENTER, Align.CENTER, Align.MIN))
    z = z.cut(aloj).cut(alivio).cut(ch)
    return sol(z.clean())


def hacer_zapata_sin_balero():
    z = bloque_zapata()
    ag = Pos(0, 0, Z_ZAP_SUP - AGUJ_SIN_BAL_PROF) * Cylinder(AGUJ_SIN_BAL_D / 2, AGUJ_SIN_BAL_PROF + 1,
                                                             align=(Align.CENTER, Align.CENTER, Align.MIN))
    ch = Pos(0, 0, Z_ZAP_SUP + 0.001) * Rot(180, 0, 0) * Cone(AGUJ_SIN_BAL_D / 2 + 0.6, AGUJ_SIN_BAL_D / 2 - 0.4,
                                                               1.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    z = z.cut(ag).cut(ch)
    # 3 costillas verticales de retención (se aplastan contra el perno de 8 mm)
    anch = (AGUJ_SIN_BAL_D - COSTILLA_D) / 2
    for i in range(3):
        a = 120.0 * i + 90.0
        cost = Rot(0, 0, a) * Pos(COSTILLA_D / 2 + anch / 2 + 0.1, 0,
                                  Z_ZAP_SUP - AGUJ_SIN_BAL_PROF + (AGUJ_SIN_BAL_PROF - 1.2) / 2) * Box(
            anch + 0.2, 1.2, AGUJ_SIN_BAL_PROF - 1.2)
        z = z.fuse(cost)
    return sol(z.clean())


def hacer_probeta():
    """Probeta de ajuste: placa con 3 pernos (Ø8.0 / 8.1 / 8.2, 1-2-3 muescas) iguales al perno del vástago.
    Se imprime igual que el vástago (perno vertical) para elegir PERNO_D según la impresora."""
    amin = (Align.CENTER, Align.CENTER, Align.MIN)
    paso = 14.0
    n = len(PROBETA_D)
    L = paso * n + 2.0
    p = sol(Box(L, 16.0, 3.0, align=amin))
    for i, d in enumerate(PROBETA_D):
        x = -L / 2 + 1.0 + paso * (i + 0.5)
        cuello = Pos(x, 0, 2.99) * Cylinder(CUELLO_D / 2, CUELLO_L + 0.01, align=amin)
        perno = sol(Pos(x, 0, 3.0 + CUELLO_L) * Cylinder(d / 2, PERNO_L, align=amin))
        perno = perno.chamfer(0.6, None, [e for e in perno.edges()
                                          if abs(e.center().Z - (3.0 + CUELLO_L + PERNO_L)) < 1e-3])
        p = p + cuello + perno
        for k in range(i + 1):   # muescas de identificación en el borde -Y
            xm = x + (k - i / 2) * 2.2
            p = p - Pos(xm, -8.0, -0.01) * Cylinder(0.7, 3.2, align=amin)
    return sol(p.clean())


def hacer_balero():
    amin = (Align.CENTER, Align.CENTER, Align.MIN)
    b = Pos(0, 0, Z_ZAP_SUP - BAL_W) * (Cylinder(BAL_DE / 2, BAL_W, align=amin) - Cylinder(BAL_DI / 2, BAL_W, align=amin))
    return sol(b)


# ============================================================================
# MALLAS E INTERFERENCIAS
# ============================================================================
def a_trimesh(shape, ruta):
    export_stl(shape, ruta, tolerance=0.01, angular_tolerance=0.1)
    return trimesh.load(ruta, force="mesh")


def a_manifold(tm):
    return manifold3d.Manifold(manifold3d.Mesh(vert_properties=np.asarray(tm.vertices, np.float32),
                                               tri_verts=np.asarray(tm.faces, np.uint32)))


def vol_inter(a, b):
    return (a ^ b).volume()


def tubo_tm(largo=200.0):
    t = trimesh.creation.cylinder(radius=R_TUBO, height=largo, sections=360)
    t.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
    t.apply_translation([0, 0, R_TUBO])
    return t


def mover(tm, dz=0.0, ang=0.0, dx=0.0):
    t = tm.copy()
    if ang:
        t.apply_transform(trimesh.transformations.rotation_matrix(math.radians(ang), [0, 0, 1]))
    t.apply_translation([dx, 0, dz])
    return t


def acople(s):
    """Longitud de rosca acoplada en la tuerca para un desplazamiento s del vástago."""
    ext0 = Z_ROSCA_0 + s + ROSCA_P / 4      # fade de 1/4 de paso en cada extremo
    ext1 = Z_ROSCA_1 + s - ROSCA_P / 4
    nut0 = Z_TUERCA_INF + TUERCA_AVELLANADO
    nut1 = Z_TUERCA_SUP - TUERCA_AVELLANADO
    return max(0.0, min(ext1, nut1) - max(ext0, nut0))


# ============================================================================
# PRINCIPAL
# ============================================================================
def main():
    print("Construyendo piezas ...")
    cuerpo = hacer_cuerpo()
    tuerca = hacer_tuerca()                      # z 0..H (se coloca en Z_TUERCA_INF)
    vastago = hacer_vastago()
    zap_b = hacer_zapata_balero()
    zap_sb = hacer_zapata_sin_balero()
    balero = hacer_balero()
    tuerca_uso = Pos(0, 0, Z_TUERCA_INF) * tuerca

    # ---------------- Cinemática ----------------
    print("\n===== CINEMÁTICA (s = desplazamiento del vástago; s=0 zapata tocando el tubo) =====")
    tope = Z_BRAZO_INF - Z_ZAP_SUP                      # zapata topa contra el brazo
    sobre = HOLG_CUBO                                   # cubo topa contra el bonete
    print(f"Paso de rosca: {ROSCA_P} mm/vuelta")
    print(f"Holgura zapata(punto más bajo)-tubo, abierta (s=+{CARRERA_ABIERTA}): "
          f"{Z_ZAP_BORDE + CARRERA_ABIERTA - Z_TUBO_SUP:.2f} mm  (>= 6 requerido)")
    print(f"  idem con el tubo levantado sobre el labio ({H_LABIO} mm): "
          f"{Z_ZAP_BORDE + CARRERA_ABIERTA - (Z_TUBO_SUP + H_LABIO):.2f} mm")
    print(f"Vueltas de abierta a contacto: {CARRERA_ABIERTA / ROSCA_P:.1f}")
    print(f"Tope superior físico (zapata contra brazo) en s=+{tope:.2f}")
    print(f"Sobrecarrera tras el contacto antes de que el cubo toque el bonete: {sobre:.2f} mm")
    # recorrido de rosca disponible tras el contacto con acople >= 8
    s = 0.0
    while acople(s - 0.01) >= 8.0 and s > -40:
        s -= 0.01
    print(f"Recorrido de rosca restante tras el contacto (acople >= 8 mm): {-s:.2f} mm  (>= 4 requerido)")
    for nombre, sv in (("cubo toca bonete", -sobre), ("contacto", 0.0), ("abierta", CARRERA_ABIERTA),
                       ("tope superior", tope)):
        print(f"  acople de rosca en '{nombre}' (s={sv:+.1f}): {acople(sv):.2f} mm")
    print(f"Holgura cubo-bonete en contacto: {Z_CUBO_INF - Z_BONETE_SUP:.2f} mm; mínima en todo el recorrido "
          f"(s>=-{sobre:.0f}): 0 (tope)")
    print(f"Zapata {ZAP_X}x{ZAP_Y}x{Z_ZAP_SUP - Z_ZAP_BORDE:.2f}; holgura zapata-lomo: "
          f"{-X_LOMO_INT - ZAP_X / 2:.2f} mm; boca vertical libre "
          f"(labio->brazo) = {Z_BRAZO_INF - H_LABIO:.1f} mm > alto zapata {Z_ZAP_SUP - Z_ZAP_BORDE:.1f}")
    print(f"Labio: holgura mínima al tubo = "
          f"{math.hypot(X_LABIO_INT, R_TUBO - H_LABIO) - R_TUBO:.2f} mm; "
          f"levantar el tubo para meterlo = mandíbula {T_MANDIBULA} + labio {H_LABIO} = {T_MANDIBULA + H_LABIO:.1f} mm")
    # filete lomo-mandíbula: holgura mínima al tubo (centrado y recargado contra el lomo)
    xcf, zcf = X_LOMO_INT + R_FILETE_INT_INF, R_FILETE_INT_INF
    angs = np.radians(np.linspace(180, 270, 721))
    for dx in (0.0, -HOLGURA_LOMO):
        g = np.min(np.hypot(xcf + R_FILETE_INT_INF * np.cos(angs) - dx,
                            zcf + R_FILETE_INT_INF * np.sin(angs) - R_TUBO)) - R_TUBO
        print(f"Filete R{R_FILETE_INT_INF:.0f} lomo-mandíbula: holgura mínima al tubo (tubo en x={dx:+.1f}) = {g:.2f} mm")

    # --- Tuerca cautiva: estado abierto SIN carga ---
    s_ab = CARRERA_ABIERTA
    print(f"Tuerca CAUTIVA en ranura lateral: juego axial {HOLG_ALOJ_Z:.2f} mm, piso {T_PISO_TUERCA} mm, "
          f"hombro {T_HOMBRO} mm")
    print(f"  Abierta y colgando (s=+{s_ab:.0f}, tuerca apoyada en el piso, todo baja {HOLG_ALOJ_Z} mm): "
          f"holgura zapata-tubo = {Z_ZAP_BORDE + s_ab - HOLG_ALOJ_Z - Z_TUBO_SUP:.2f} mm (>= 6); "
          f"acople = {acople(s_ab - HOLG_ALOJ_Z):.2f} mm")
    print(f"  Aflojar con la zapata apoyada en el tubo: la tuerca baja {HOLG_ALOJ_Z} mm al piso y luego el "
          f"vástago SUBE {ROSCA_P} mm por vuelta (ya no puede salirse la tuerca)")

    # --- Resistencia de la mandíbula inferior (voladizo desde el apoyo del tubo en x=0) ---
    xs = np.linspace(X_LOMO_INT + 0.05, -0.5, 600)
    h = np.where(xs < xcf, T_MANDIBULA + zcf - np.sqrt(np.clip(R_FILETE_INT_INF ** 2 - (xs - xcf) ** 2, 0, None)),
                 T_MANDIBULA)
    sig_por_N = 6.0 * np.abs(xs) / (D * h ** 2)          # MPa por N de fuerza de apriete
    i = int(np.argmax(sig_por_N))
    SIG_PLA = 45.0                                          # MPa, PLA impreso (orientación favorable)
    F_fl = SIG_PLA / sig_por_N[i]
    # par en el volante -> fuerza axial (rosca M16x2, mu=0.3, balero sin fricción en la zapata)
    mu = 0.3
    r_p = (ROSCA_D_EXT - 0.65 * ROSCA_P) / 2
    lam = math.atan(ROSCA_P / (2 * math.pi * r_p))
    mu_p = mu / math.cos(math.radians(30))
    k_par = r_p * 1e-3 * (math.tan(lam) + mu_p) / (1 - mu_p * math.tan(lam))   # N·m por N
    print(f"Mandíbula: sección crítica en x={xs[i]:.1f} (h={h[i]:.2f} mm), {sig_por_N[i]:.3f} MPa/N -> "
          f"fluencia (~{SIG_PLA:.0f} MPa) a ~{F_fl:.0f} N = ~{F_fl * k_par:.2f} N·m en el volante "
          f"({1 / k_par:.0f} N por N·m). Apretar con dos dedos (~0.3 N·m -> ~{0.3 / k_par:.0f} N).")

    # ---------------- Mallas en uso ----------------
    print("\nGenerando mallas de uso e interferencias ...")
    p = lambda n: os.path.join(DIR_USO, n)
    m_cuerpo = a_trimesh(cuerpo, p("cuerpo.stl"))
    m_tuerca = a_trimesh(tuerca_uso, p("tuerca.stl"))
    m_vast = a_trimesh(vastago, p("vastago_ang0.stl"))
    m_zap = a_trimesh(zap_b, p("zapata_balero.stl"))
    m_zapsb = a_trimesh(zap_sb, p("zapata_sin_balero.stl"))
    m_bal = a_trimesh(balero, p("balero608.stl"))
    m_tubo = tubo_tm()
    for nom, m in (("cuerpo", m_cuerpo), ("tuerca", m_tuerca), ("vastago", m_vast), ("zapata", m_zap),
                   ("zapata_sb", m_zapsb)):
        assert m.is_watertight, nom

    # Fase de la rosca: girar el vástago hasta que los filetes encajen con la tuerca
    M_tuerca = a_manifold(m_tuerca)
    mejor = None
    for ang in range(0, 360, 10):
        v = vol_inter(a_manifold(mover(m_vast, ang=ang)), M_tuerca)
        if mejor is None or v < mejor[1]:
            mejor = (ang, v)
    ang0 = mejor[0]
    for ang in np.arange(ang0 - 9, ang0 + 9.5, 1.0):
        v = vol_inter(a_manifold(mover(m_vast, ang=ang)), M_tuerca)
        if v < mejor[1]:
            mejor = (float(ang), v)
    ANG = mejor[0]
    print(f"Fase de rosca: vástago girado {ANG:.0f}° -> interferencia vástago-tuerca {mejor[1]:.3f} mm3")
    # al subir/bajar s, el giro correspondiente es -s/p*360 (rosca derecha)

    def vast_en(s):
        return mover(m_vast, dz=s, ang=ANG + s / ROSCA_P * 360.0)

    piezas = {"cuerpo": m_cuerpo, "tuerca": m_tuerca, "zapata": m_zap, "balero": m_bal, "tubo": m_tubo}
    print("\n===== INTERFERENCIAS en posición APRETADA (mm3; sólo zapata-tubo debe ser contacto) =====")
    res = {}
    M = {k: a_manifold(v) for k, v in piezas.items()}
    M["vastago"] = a_manifold(vast_en(0.0))
    nombres = list(M.keys())
    for i in range(len(nombres)):
        for j in range(i + 1, len(nombres)):
            a, b = nombres[i], nombres[j]
            v = vol_inter(M[a], M[b])
            res[(a, b)] = v
            print(f"  {a:8s} x {b:8s}: {v:8.3f}")
    # Distancias de contacto
    print("  (balero-vastago y balero-zapata son ajustes a presión nominales: perno " + f"{PERNO_D}" + " / alojamiento " + f"{ALOJ_BAL_D}" + ")")

    # Tuerca cautiva: apoyada en el piso (sin carga) y trayectoria de inserción lateral por la ranura (+X)
    print("\n===== TUERCA CAUTIVA =====")
    v_piso = vol_inter(a_manifold(mover(m_tuerca, dz=-HOLG_ALOJ_Z)), M["cuerpo"])
    print(f"  tuerca apoyada en el piso de la ranura x cuerpo: {v_piso:.3f} mm3")
    peor = 0.0
    for dx in np.arange(0.0, X_BRAZO_FIN + 15.0, 0.5):
        v = vol_inter(a_manifold(mover(m_tuerca, dz=-HOLG_ALOJ_Z / 2, dx=float(dx))), M["cuerpo"])
        peor = max(peor, v)
    print(f"  inserción lateral (x de 0 a {X_BRAZO_FIN + 15:.0f} mm, paso 0.5): interferencia máxima {peor:.3f} mm3")
    # la tuerca no puede subir ni bajar: se prueba desplazarla 1 mm en Z
    for dz in (-HOLG_ALOJ_Z - 1.0, 1.0):
        v = vol_inter(a_manifold(mover(m_tuerca, dz=dz)), M["cuerpo"])
        print(f"  tuerca desplazada dz={dz:+.1f} mm -> choca con el cuerpo: {v:.1f} mm3 (>0 = retenida)")

    print("\n===== INTERFERENCIAS en posición ABIERTA y en TOPE =====")
    for etiqueta, s in (("abierta", CARRERA_ABIERTA), ("tope", tope - 0.05), ("cubo en bonete", -sobre + 0.05)):
        mv = a_manifold(vast_en(s))
        mz = a_manifold(mover(m_zap, dz=s))
        mb = a_manifold(mover(m_bal, dz=s))
        tubo_ok = s >= 0
        txt = []
        for nom, mm in (("vastago", mv), ("zapata", mz), ("balero", mb)):
            for nom2 in ("cuerpo", "tuerca") + (("tubo",) if tubo_ok else ()):
                if nom == "vastago" and nom2 == "tuerca":
                    continue
                txt.append(f"{nom}x{nom2}={vol_inter(mm, M[nom2]):.3f}")
        txt.append(f"vastagoxtuerca={vol_inter(mv, M['tuerca']):.3f}")
        print(f"  s={s:+6.2f} ({etiqueta}): " + ", ".join(txt))
    zmin_zap_ab = mover(m_zap, dz=CARRERA_ABIERTA).bounds[0][2]
    print(f"  Verificación malla: z mínima zapata abierta = {zmin_zap_ab:.2f}; tubo arriba = {Z_TUBO_SUP:.2f} "
          f"-> holgura {zmin_zap_ab - Z_TUBO_SUP:.2f} mm")
    zv = vast_en(0.0)
    sel = zv.vertices[:, 2] < Z_CUBO_INF + 0.5
    print(f"  Volante: z mínima del cubo (contacto) = {Z_CUBO_INF:.2f} vs bonete {Z_BONETE_SUP:.2f}")

    # ---------------- Exportación ----------------
    print("\nExportando STL (orientación de impresión) y STEP ...")
    vast_rot = Rot(0, 0, ANG) * vastago
    imp = {
        # cuerpo: acostado sobre su cara y = -D/2 (Y de uso -> Z de impresión)
        "valvula_cuerpo": Pos(0, 0, D / 2) * Rot(90, 0, 0) * cuerpo,
        "valvula_tuerca": tuerca,
        # volante sobre la cama, vástago hacia arriba
        "valvula_vastago_volante": Pos(0, 0, Z_VOLANTE_SUP) * Rot(180, 0, 0) * vastago,
        # zapatas boca abajo (cara superior sobre la cama), cara cóncava hacia arriba
        "valvula_zapata_balero": Pos(0, 0, Z_ZAP_SUP) * Rot(180, 0, 0) * zap_b,
        "valvula_zapata_sin_balero": Pos(0, 0, Z_ZAP_SUP) * Rot(180, 0, 0) * zap_sb,
        # probeta de ajuste del perno (opcional, imprimir primero)
        "valvula_probeta_perno": hacer_probeta(),
    }
    for nombre, shp in imp.items():
        bb = shp.bounding_box()
        shp = Pos(-(bb.min.X + bb.max.X) / 2, -(bb.min.Y + bb.max.Y) / 2, -bb.min.Z) * shp
        export_stl(shp, os.path.join(DIR_STL, nombre + ".stl"), tolerance=0.01, angular_tolerance=0.1)
        export_step(shp, os.path.join(DIR_STEP, nombre + ".step"))
        tm = trimesh.load(os.path.join(DIR_STL, nombre + ".stl"), force="mesh")
        n = tm.face_normals
        on_bed = tm.triangles[:, :, 2].max(axis=1) <= tm.bounds[0][2] + 0.25
        real = float(tm.area_faces[(n[:, 2] < -math.sin(math.radians(50))) & ~on_bed].sum())
        print(f"  {nombre:28s} vol={tm.volume / 1000:6.1f} cm3  masa sólida PLA={tm.volume * 1.24e-3:5.1f} g  "
              f"bbox={np.round(tm.extents, 1)}  voladizo real >50° de la vertical={real:.0f} mm2")
    # Ensamble STEP en posición de uso apretada (referencia)
    ens = Compound([cuerpo, tuerca_uso, vast_rot, zap_b, balero])
    export_step(ens, os.path.join(DIR_STEP, "valvula_ensamble_uso.step"))

    # Mallas de uso para renders
    vast_en(0.0).export(p("vastago.stl"))
    vast_en(CARRERA_ABIERTA).export(p("vastago_abierto.stl"))
    mover(m_zap, dz=CARRERA_ABIERTA).export(p("zapata_abierta.stl"))
    mover(m_bal, dz=CARRERA_ABIERTA).export(p("balero_abierto.stl"))
    # Cortes por y=0 (se conserva y>0) para la vista frontal tipo sección
    caja = manifold3d.Manifold.cube([400, 200, 400]).translate([-200, 0, -100])
    # posición ABIERTA sin carga: la tuerca cautiva descansa en el piso y todo lo móvil baja HOLG_ALOJ_Z
    dzc = -HOLG_ALOJ_Z
    for nom, tm in (("cuerpo", m_cuerpo), ("tuerca", m_tuerca), ("vastago", vast_en(0.0)), ("zapata", m_zap),
                    ("balero", m_bal), ("tubo", tubo_tm(120)),
                    ("tuerca_abierta", mover(m_tuerca, dz=dzc)),
                    ("vastago_abierto", mover(vast_en(CARRERA_ABIERTA), dz=dzc)),
                    ("zapata_abierta", mover(m_zap, dz=CARRERA_ABIERTA + dzc)),
                    ("balero_abierto", mover(m_bal, dz=CARRERA_ABIERTA + dzc))):
        c = a_manifold(tm) ^ caja
        mm = c.to_mesh()
        trimesh.Trimesh(np.asarray(mm.vert_properties)[:, :3], np.asarray(mm.tri_verts)).export(p(f"corte_{nom}.stl"))
    print("Listo.")


if __name__ == "__main__":
    main()
