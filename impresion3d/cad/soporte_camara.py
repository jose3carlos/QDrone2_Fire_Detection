# -*- coding: utf-8 -*-
"""
Soporte impreso para que la Intel RealSense D435/D435i integrada al frente del QDrone 2 mire
HACIA ABAJO (nadir) y el dron explorador use su cámara de COLOR (RGB) para el detector YOLO.
No se taladra ni se modifica el chasis de fibra de carbono.

CONCEPTO
--------
De fábrica la cámara mira hacia adelante, colgada bajo la placa principal (cuello en "Y"), entre
los dos aros delanteros y detrás del travesaño frontal. Si se gira en su lugar, el travesaño y la
placa inferior le tapan la vista. Por eso la cámara se ADELANTA hasta quedar delante del borde
del travesaño (en el hueco entre los aros delanteros) y se gira 90° sobre su eje largo:
  - cara de las lentes hacia abajo (eje óptico = -z del cuerpo),
  - cara superior (rejilla) hacia ADELANTE  ->  arriba de la imagen = adelante del dron,
  - conector USB-C del mismo lado que de fábrica (derecha del dron, -y); RGB a la izquierda (+y).
Con esto `camara.rotacion_montaje = 0` (convención de web/app/geo.py). El script lo deriva de la
cámara modelada: x_cam = hacia el extremo del USB, z_cam = normal de la cara de lentes,
y_cam = z_cam x x_cam, y lo compara con geo.R_BODY_CAM0.
La cámara se atornilla por sus 2 roscas M3 traseras (45 mm entre centros, 3 mm de inserción
máxima), que es lo que Intel recomienda. Intel da 0.4 N·m para un soporte METÁLICO; sobre la cuna
impresa se aprieta a PAR_CAM_NM (0.15 N·m) con cabeza de botón con brida ISO 7380-2 para no
aplastar el plástico. La cara trasera queda hacia ARRIBA, contra una "cuna" plana de 4 mm con una
ventana en forma de flecha que apunta hacia adelante (ventilación y marca de orientación) y una
muesca en V del lado del USB.

La MISMA cuna se usa en dos variantes de sujeción al chasis:
  A. ADAPTADOR (preferida). Se desatornilla la cámara de su escuadra original y el adaptador se
     atornilla en los MISMOS 2 agujeros donde iba la cámara (patrón M3 de 45 mm, con tuercas M3
     cautivas en la placa de interfaz). Un brazo con 4 nervios lleva la cuna hacia adelante,
     pasando por el espacio que ocupaba la cámara y por debajo del travesaño (zona que era el
     campo de visión de fábrica, así que está libre). Dos costillas SOBRE la cuna, delante del
     travesaño, suben la rigidez al cabeceo (vibración de los rotores). La placa de interfaz y los
     nervios quedan HOLG_LENG por encima de la lengüeta de la escuadra original.
  B. ABRAZADERA (si la escuadra original NO usa los M3 traseros, p. ej. si toma la rosca 1/4-20).
     Una "C" abraza el travesaño frontal de la placa principal (espesor y ancho paramétricos);
     la cuna se apoya bajo el travesaño y 2 tornillos M3 de NYLON con tuercas cautivas (en la
     mandíbula superior) aprietan desde arriba. Las tuercas entran por una ranura desde la cara
     trasera de la mandíbula. Respaldo: 2 ranuras para cinchos que cierran la boca de la C por
     detrás del travesaño (a través de las ventanas del cuello), con 3 mm de pared trasera.

PIEZAS (impresas sin soportes)
  soporte_camara_A_adaptador   -> imprimir DE PIE sobre la cara trasera de la placa de interfaz
                                  (la que apoya en la escuadra original); brazo y cuna hacia
                                  arriba. Alojamientos de tuerca abiertos hacia arriba, sin puente.
                                  Brim de 5 mm.
  soporte_camara_B_abrazadera  -> imprimir DE LADO (cara y = -30, lado del USB, sobre la cama).
                                  Brim de 5 mm.
  soporte_camara_probeta       -> plana. Patrón de 45 mm de la cuna (2 x Ø3.4, mismo espesor),
                                  3 alojamientos de tuerca con eje VERTICAL como en A (EC 5.6/5.7/
                                  5.8), una pared con 3 alojamientos de eje HORIZONTAL y techo
                                  puenteado como en B (EC 5.6/5.8/6.0) y un peine de ranuras de
                                  1.9 a 2.6 mm (pasos de 0.1) para la boca de B.
Material: PETG o ASA para la pieza final (la cara trasera de la D435 llega a 50 °C; PLA tiene Tg
55-60 °C y fluye bajo la cabeza del tornillo). PLA sólo para pruebas.

SISTEMAS DE COORDENADAS (mm)
  Local (en el que se modela): origen en el centro de la cara TRASERA de la cámara en su posición
  de fábrica (= cara de apoyo de la escuadra original), +x adelante, +y izquierda, +z arriba.
  z = 0 es la línea media de la cámara de fábrica (donde están sus M3).
  Cuerpo (renders y comprobaciones): origen en el centro geométrico del dron en planta (centro
  entre ejes de motor), z = 0 en el piso con el dron apoyado.
  cuerpo = local + (X_CARA_TRASERA_ORIG, 0, Z_EJE_ORIG).

Todo valor no confirmado lleva el comentario "MEDIR EN LAB" con su fuente y justificación.

Ejecutar (venv de CAD):
  & "$env:USERPROFILE\\.venvs\\qdrone2_cad\\Scripts\\python.exe" cad\\soporte_camara.py
Escribe STL (orientación de impresión) en ../stl, STEP en ../step y mallas de ensamble (marco del
cuerpo) en ../renders/_mallas_uso, e imprime las comprobaciones numéricas.
"""
import math
import os
import sys

import numpy as np
import trimesh
import manifold3d
from scipy.spatial import cKDTree
from build123d import (Align, Box, Compound, Cylinder, Ellipse, Plane, Polygon, Pos, Rot,
                       export_step, export_stl, extrude)

# ============================================================================
# PARÁMETROS
# ============================================================================
# --- RealSense D435 / D435i (datasheet D400 rev. 337029-017, fig. 10-9 y tablas 3-49/3-52/4-17)
CAM_L = 90.0                    # largo (eje largo), confirmado
CAM_H = 25.0                    # alto, confirmado
CAM_P = 25.05                   # fondo (cara de vidrio a cara trasera), confirmado (plano: 25.05)
CAM_R_EXT = 12.5                # radio de los extremos del contorno "estadio" (probable: mitad del alto)
M3_SEP = 45.0                   # separación de las 2 roscas M3 traseras, confirmado
M3_PROF_MAX = 3.0               # profundidad máxima de inserción del tornillo, confirmado
M3_PAR_INTEL_NM = 0.4           # par de Intel para soporte METÁLICO (nota fig. 10-9), confirmado
MASA_CAM = 75.0                 # g nominal (±10 %), confirmado
LENTE_PROF = 4.3                # centro óptico detrás de la cara frontal (4.2 + 0.1), confirmado (IR) / probable (RGB)
# Lentes sobre la línea media, a lo largo del eje largo desde el centro, + = izquierda de la cámara
# (= +y del dron). Relativos entre sí confirmados; referencia al centro probable (±0.7 mm).
# (nombre, posición, FOV horizontal, FOV vertical, semiancho de la ventana en y, en x)
LENTES = [
    ("RGB", +32.5, 69.4, 42.5, 3.5, 3.5),
    ("IR izquierda (profundidad)", +17.5, 87.0, 58.0, 5.5, 5.5),
    ("IR derecha (profundidad)", -32.5, 87.0, 58.0, 5.5, 5.5),
    ("Proyector IR", -11.5, 90.0, 63.0, 10.5, 6.5),
]
TOL_FOV = 1.5                   # grados extra por lado (tolerancia ±3° entre unidades, tabla 3-49)

# --- Montaje de fábrica en el QDrone 2 (referencia del adaptador A y de los renders)
X_CARA_TRASERA_ORIG = 157.5     # MEDIR EN LAB. x de la cara trasera original desde el centro del dron
                                # (fotos Quanser escaladas: centro de la cámara ~+170, vidrio ~+180; probable)
Z_EJE_ORIG = 31.5               # MEDIR EN LAB. Altura de la línea media de la cámara de fábrica sobre el piso
                                # con el dron apoyado (estimado 28-35 mm por perspectiva; supuesto)
DX_TRAVESANO = 12.0             # MEDIR EN LAB. Cuánto sobresale el borde delantero del travesaño de la placa
                                # principal por delante del vidrio, en el eje (vista superior hero-drone2.1,
                                # escala por los 45 mm; probable)
W_TRAVESANO = 19.0              # MEDIR EN LAB. Ancho (en x) del travesaño frontal (misma foto; probable)
T_CARBONO = 2.0                 # MEDIR EN LAB. Espesor de las placas de carbono (estimado de fotos; supuesto).
                                # Medir con calibrador (el peine de la probeta sólo comprueba la boca de B)
DX_TRAVESANO_INF = DX_TRAVESANO  # MEDIR EN LAB. Borde delantero de la placa INFERIOR respecto al vidrio
                                # (no se ve en las fotos; se supone igual al superior, caso conservador)
# Lengüeta negra de la escuadra original bajo la cámara (foto frontal QD2_cameras fig. 1 / cam-000)
LENG_ANCHO = 36.0               # MEDIR EN LAB. Ancho en y (foto frontal escalada con los 90 mm; probable)
LENG_LARGO = 25.0               # MEDIR EN LAB. Largo en x desde la cara trasera (supuesto: todo el fondo)
LENG_T = 3.0                    # MEDIR EN LAB. Espesor (supuesto)
LENG_LABIO = 2.0                # MEDIR EN LAB. Labio que sube delante del vidrio (supuesto; puede no existir)

# --- Esquema del frente del dron (sólo comprobaciones y renders)
DRON_L, DRON_W = 442.58, 490.8  # manual Quanser, tabla 10 (confirmado)
X_MOTOR, Y_MOTOR = 101.6, 127.0  # ejes de motor: qdrone2_thrust_parametrization tabla 1, pitch axis 0.2032 m y
                                # roll axis 0.254 m (valores confirmados; leerlos como distancia entre motores: probable)
R_ARO_EXT = 119.0               # MEDIR EN LAB. Radio exterior del aro, concéntrico con el motor (supuesto:
                                # DRON_L/2 - X_MOTOR = 119.7 y DRON_W/2 - Y_MOTOR = 118.4)
R_ARO_INT = 107.0               # MEDIR EN LAB. Radio interior del aro (supuesto: aro de 12 mm)
BOLSILLO_FRENTE = 20.0          # MEDIR EN LAB. Fondo del "bolsillo" entre los aros delanteros, delante del
                                # borde del travesaño, en el eje (fig. 6 hw-049 escalada; probable). El esquema
                                # da ~26 mm: la diferencia es la incertidumbre de X_CARA_TRASERA_ORIG + DX_TRAVESANO
Y_POSTE = 71.0                  # MEDIR EN LAB. |y| de los postes en los extremos del travesaño (foto superior
                                # escalada con el ancho de 90 mm de la cámara; probable)
R_POSTE = 2.5                   # radio de los postes (supuesto)
L_ENCHUFE = 20.0                # MEDIR EN LAB. Cuánto sobresale el enchufe USB-C en ángulo recto más allá del
                                # extremo de la cámara (práctica común; supuesto)
H_CODO_ENCHUFE = 8.0            # MEDIR EN LAB. Cuánto sobresale el codo del enchufe por detrás de la cara trasera
                                # de la cámara (de fábrica: hacia atrás; girada: hacia arriba). Supuesto
R_CURVA_CABLE = 30.0            # MEDIR EN LAB. Radio mínimo de curvatura del cable USB (5-10 x diámetro; supuesto)
D_CABLE = 4.6                   # MEDIR EN LAB. Diámetro exterior del cable USB (supuesto)
MASA_DRON = 1504.0              # g, masa de despegue con batería (qdrone2_thrust_parametrization; trim del PID/MPC)
Z_CG_DRON = 60.0                # MEDIR EN LAB. Altura del centro de masa del dron (supuesto: ~mitad de la altura)
KV_MOTOR = 1300.0               # RPM/V (manual, tabla 5)
V_BAT_MAX = 16.8                # V, 4S cargada
EMPUJE_POLI = (0.0362, 0.117, -0.0121)   # T(V) = a V^2 + b V + c [N] (qdrone2_thrust_parametrization, ec. 3)

# --- Diseño del soporte
T_CUNA = 4.0                    # espesor de la cuna (bajo la cabeza del tornillo de la cámara)
ANCHO_CUNA = 60.0               # ancho de la cuna y del brazo (y); cubre los M3 a ±22.5 con 7.5 mm de borde
CHAFLAN_CUNA = 4.0              # chaflán a 45° de las esquinas delanteras
HOLG_TECHO = 0.5                # A: holgura entre la cara superior del brazo y la cara inferior de la placa
                                # principal (supuesta = cara superior de la cámara de fábrica)
HOLG_TRAV = 5.0                 # holgura en x entre la cámara girada y el borde del travesaño. Con 3.0 el
                                # cono RGB sigue libre pero los de profundidad rozan el borde (supuesto) de la
                                # placa inferior; con 5.0 quedan libres a cambio de 2 mm más de vuelo
HOLG_LENG = 1.5                 # A: holgura entre la placa de interfaz / raíz de los nervios y la lengüeta
T_INTERFAZ = 6.0                # A: espesor de la placa de interfaz (3.4 mm detrás de la tuerca)
NERVIOS_Y = (-28.0, -10.0, 10.0, 28.0)   # A: centros de los 4 nervios bajo el brazo (rectangulares: ocupan el
                                # hueco que dejó la cámara, arriba de la lengüeta y fuera de los conos)
NERVIO_T = 4.0                  # A: espesor de los nervios
COSTILLAS_Y = (-16.5, 16.5)     # A: costillas SOBRE la cuna, entre la ventana (|y| <= 12) y las cabezas de
                                # los tornillos de la cámara (|y| >= 19.05 con brida Ø6.9)
COSTILLA_T = 3.0                # A: espesor de las costillas
COSTILLA_H = 12.0               # A: altura de las costillas sobre la cuna (arrancan a 45° delante del travesaño)
HOLG_COSTILLA = 1.0             # A: x entre el borde del travesaño y el arranque de las costillas
D_PASO_M3 = 3.4                 # agujero de paso M3 (gota a 45° si queda horizontal al imprimir)
TUERCA_EC_A = 5.7               # A: alojamiento con eje VERTICAL al imprimir (entrecaras en el plano XY).
                                # Elegir con la probeta (fila de 5.6/5.7/5.8)
TUERCA_EC_B = 5.8               # B: alojamiento con eje HORIZONTAL al imprimir (caras planas formadas por
                                # capas, techo puenteado). Elegir con la pared de la probeta (5.6/5.8/6.0)
TUERCA_H = 2.6                  # profundidad del alojamiento (tuerca DIN 934 de 2.4 mm)
L_TORN_CAM = 6.0                # tornillo de la cámara: M3x6 -> 6 - T_CUNA = 2.0 mm dentro de la cámara
PAR_CAM_NM = 0.15               # par de los M3x6 de la cámara sobre la cuna impresa
D_BRIDA_TORN = 6.9              # ISO 7380-2 M3 (botón con brida): diámetro de apoyo
# Ventana en flecha (apunta a +x) en la cuna, relativa al centro de la cámara. Lados a 45°.
VENT_PUNTA, VENT_SEMI = 8.0, 12.0
# B: abrazadera
HOLG_BOCA = 0.4                 # holgura vertical de la boca de la C sobre el espesor de la placa
T_MANDIBULA = 6.0               # espesor de la mandíbula superior
T_ESPINA = 4.5                  # espesor de la espina de la C (delante del borde del travesaño)
HOLG_ESPINA = 0.5               # holgura entre el borde del travesaño y la espina
X_TORN_B, Y_TORN_B = 27.5, 15.0  # tornillos de apriete de B (sobre el travesaño; evita agujeros en y=0, ±37)
PAR_B_NM = 0.1                  # par máximo de los tornillos de NYLON de B sobre el carbono
Y_CINCHO = (20.0, 25.5)         # ranuras para cinchos (|y|): 5.5 mm, cincho de hasta 4.8 mm x 1.5 mm
X_RANURA_CINCHO = 3.0           # pared entre la ranura y el extremo trasero de B (>= 6 líneas de 0.4)
ANCHO_RANURA_CINCHO = 2.5
# Probeta
PROBETA_EC_A = (5.6, 5.7, 5.8)          # alojamientos de eje vertical (como A)
PROBETA_EC_B = (5.6, 5.8, 6.0)          # alojamientos de eje horizontal con techo puenteado (como B)
PROBETA_RANURAS = (1.9, 2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6)   # peine para la boca de B

# --- Materiales y cálculo
RHO_PLA, RHO_PETG = 1.24e-3, 1.27e-3     # g/mm3
E_IMPRESO = 2000.0              # MPa, módulo conservador de PLA/PETG impreso
PERIMETROS_MM = 1.2             # espesor de pared+tapas que se imprime macizo (3 perímetros 0.4 / 5 capas 0.2)
RELLENO = 0.40                  # relleno recomendado (gyroid 40 %)
CAPA = 0.2
K_PAR = 0.2                     # factor de par (tornillo seco): F = T / (K d)

# ============================================================================
# COTAS DERIVADAS (marco local)
# ============================================================================
Z_BASE = -CAM_H / 2                         # cara inferior de la cámara de fábrica (= cara superior de la lengüeta)
Z_RAIZ = Z_BASE + HOLG_LENG                 # A: borde inferior de la placa de interfaz
Z_NERV = Z_BASE + max(LENG_LABIO, 0.0) + HOLG_LENG   # A: borde inferior de los nervios (sobre el labio)
Z_TECHO = CAM_H / 2                         # cara inferior de la placa principal (supuesto)
X_TRAV_DEL = CAM_P + DX_TRAVESANO           # borde delantero del travesaño
X_TRAV_TRAS = X_TRAV_DEL - W_TRAVESANO      # borde trasero del travesaño
XC = X_TRAV_DEL + HOLG_TRAV + CAM_H / 2     # x del centro de la cámara girada (y de sus lentes)
Z_SUP_A = Z_TECHO - HOLG_TECHO              # cara superior del brazo/cuna A
Z_SUP_B = Z_TECHO                           # cara superior de la cuna B (apoyada bajo el travesaño)
X_NERV_FIN = XC - CAM_H / 2 - 1.0           # los nervios terminan 1 mm antes de la cámara
X_COST0 = X_TRAV_DEL + HOLG_COSTILLA        # arranque de las costillas de A
X_COST1 = XC + CAM_H / 2 - 1.0              # fin de las costillas de A
X_B0 = X_TRAV_TRAS - 6.0                    # extremo trasero de la abrazadera B
TX, TZ = X_CARA_TRASERA_ORIG, Z_EJE_ORIG    # local -> cuerpo

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)
DIR_STL = os.path.join(RAIZ, "stl")
DIR_STEP = os.path.join(RAIZ, "step")
DIR_USO = os.path.join(RAIZ, "renders", "_mallas_uso")
for _d in (DIR_STL, DIR_STEP, DIR_USO):
    os.makedirs(_d, exist_ok=True)
PREF = "soporte_camara_"

# geo.R_BODY_CAM0 (web/app/geo.py): cuerpo <- cámara con rotacion_montaje = 0 (columnas = ejes de cámara)
R_BODY_CAM0 = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])


# ============================================================================
# UTILIDADES
# ============================================================================
def sol(x):
    """Sólido único y válido."""
    s = x.solids()
    assert len(s) == 1, f"se esperaba 1 sólido, hay {len(s)}"
    assert s[0].is_valid, "sólido inválido"
    return s[0]


def gota(r, punta="+v", n=48):
    """Círculo con punta a 45° (agujero horizontal imprimible sin soporte). punta: +v, -v o +u."""
    pts = [(r * math.cos(math.radians(135 + 270 * i / n)), r * math.sin(math.radians(135 + 270 * i / n)))
           for i in range(n + 1)]
    pts.append((0.0, r * math.sqrt(2)))
    if punta == "-v":
        pts = [(u, -v) for (u, v) in pts]
    elif punta == "+u":
        pts = [(v, u) for (u, v) in pts]
    return pts


def circulo(r, n=48):
    return [(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n)) for i in range(n)]


def hexa(ec):
    """Hexágono de entrecaras ec con las caras planas en v = ±ec/2 (techo plano = puente corto)."""
    r, a = ec / math.sqrt(3), ec / 2
    return [(r, 0), (r / 2, a), (-r / 2, a), (-r, 0), (-r / 2, -a), (r / 2, -a)]


def mov(pts, du, dv):
    return [(u + du, v + dv) for (u, v) in pts]


def ccw(pts):
    """Ordena el polígono en sentido antihorario (si no, extrude() saldría hacia el lado contrario)."""
    a = sum(u0 * v1 - u1 * v0 for (u0, v0), (u1, v1) in zip(pts, pts[1:] + pts[:1]))
    return list(pts) if a > 0 else list(reversed(pts))


def prisma_xy(pts, z0, z1):
    return Pos(0, 0, z0) * extrude(Polygon(*ccw(pts), align=None), amount=z1 - z0)


def prisma_xz(pts, y0, y1):
    """Perfil (x, z) extruido entre y0 e y1."""
    return Pos(0, (y0 + y1) / 2, 0) * extrude(Plane.XZ * Polygon(*ccw(pts), align=None), amount=(y1 - y0) / 2,
                                               both=True)


def prisma_yz(pts, x0, x1):
    """Perfil (y, z) extruido entre x0 y x1."""
    return Pos(x0, 0, 0) * extrude(Plane.YZ * Polygon(*ccw(pts), align=None), amount=x1 - x0)


def caja(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def a_trimesh(shape, ruta):
    export_stl(shape, ruta, tolerance=0.01, angular_tolerance=0.1)
    return trimesh.load(ruta, force="mesh")


def a_manifold(tm):
    return manifold3d.Manifold(manifold3d.Mesh(vert_properties=np.asarray(tm.vertices, np.float32),
                                               tri_verts=np.asarray(tm.faces, np.uint32)))


def de_manifold(m):
    mm = m.to_mesh()
    return trimesh.Trimesh(np.asarray(mm.vert_properties)[:, :3], np.asarray(mm.tri_verts), process=True)


def m_caja(x0, x1, y0, y1, z0, z1):
    return manifold3d.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def m_cil(x, y, z0, z1, r, seg=48):
    return manifold3d.Manifold.cylinder(z1 - z0, r, r, seg).translate([x, y, z0])


def m_tubo(pts, r):
    """Tubo (unión de cilindros y esferas) por una polilínea."""
    m = manifold3d.Manifold.sphere(r, 16).translate(list(pts[0]))
    for p0, p1 in zip(pts, pts[1:]):
        if np.linalg.norm(np.subtract(p1, p0)) < 1e-6:
            continue
        c = trimesh.creation.cylinder(radius=r, segment=[p0, p1], sections=16)
        m = m + a_manifold(c) + manifold3d.Manifold.sphere(r, 16).translate(list(p1))
    return m


def a_cuerpo(tm):
    t = tm.copy()
    t.apply_translation([TX, 0, TZ])
    return t


def varilla(p0, p1, r=0.6):
    return trimesh.creation.cylinder(radius=r, segment=[p0, p1], sections=12)


# ============================================================================
# CUNA COMPARTIDA (cámara girada, cara trasera hacia arriba contra la cuna)
# ============================================================================
def poligono_ventana():
    t, s = VENT_PUNTA, VENT_SEMI
    return [(t - 1.5 * s, -s / 2), (t - 1.5 * s, s / 2), (t - s, s), (t, 0.0), (t - s, -s)]


def hacer_cuna(z_sup, punta):
    """punta: dirección de la punta de la gota de los agujeros M3 = 'arriba' al imprimir (+u = +x, +v = +y)."""
    x0, x1, W, c = XC - CAM_H / 2, XC + CAM_H / 2, ANCHO_CUNA / 2, CHAFLAN_CUNA
    zi = z_sup - T_CUNA
    cuna = prisma_xy([(x0, -W), (x1 - c, -W), (x1, -W + c), (x1, W - c), (x1 - c, W), (x0, W)], zi, z_sup)
    for s in (1, -1):   # agujeros de paso para los M3 traseros de la cámara
        cuna = cuna - prisma_xy(mov(gota(D_PASO_M3 / 2, punta), XC, s * M3_SEP / 2), zi - 1, z_sup + 1)
    cuna = cuna - prisma_xy(mov(poligono_ventana(), XC, 0), zi - 1, z_sup + 1)
    # muesca en V del lado del USB (-y), lados a 45°
    cuna = cuna - prisma_xy([(XC - 3.5, -W - 0.5), (XC + 3.5, -W - 0.5), (XC, -W + 3.0)], zi - 1, z_sup + 1)
    return sol(cuna)


# ============================================================================
# VARIANTE A: ADAPTADOR A LOS M3 DE LA ESCUADRA ORIGINAL
# ============================================================================
def altura_costilla(x):
    return float(np.clip(x - X_COST0, 0.0, COSTILLA_H)) if x <= X_COST1 else 0.0


def hacer_A():
    zs, zi, W, c = Z_SUP_A, Z_SUP_A - T_CUNA, ANCHO_CUNA / 2, 3.0
    placa = prisma_yz([(-W, Z_RAIZ + c), (-W + c, Z_RAIZ), (W - c, Z_RAIZ), (W, Z_RAIZ + c), (W, zs), (-W, zs)],
                      0.0, T_INTERFAZ)
    brazo = prisma_xy([(T_INTERFAZ - 0.5, -W), (XC - CAM_H / 2 + 0.5, -W), (XC - CAM_H / 2 + 0.5, W),
                       (T_INTERFAZ - 0.5, W)], zi, zs)
    a = placa + brazo + hacer_cuna(zs, "+u")      # A se imprime con +x hacia arriba
    for yc in NERVIOS_Y:
        a = a + prisma_xz([(0.0, Z_NERV), (X_NERV_FIN, Z_NERV), (X_NERV_FIN, zi + 0.5), (0.0, zi + 0.5)],
                          yc - NERVIO_T / 2, yc + NERVIO_T / 2)
    for yc in COSTILLAS_Y:   # costilla sobre la cuna: arranque a 45° (voladizo a 45° al imprimir de pie)
        a = a + prisma_xz([(X_COST0, zs - 0.5), (X_COST0 + COSTILLA_H + 0.5, zs + COSTILLA_H),
                           (X_COST1, zs + COSTILLA_H), (X_COST1, zs - 0.5)],
                          yc - COSTILLA_T / 2, yc + COSTILLA_T / 2)
    for s in (1, -1):
        y = s * M3_SEP / 2
        # agujero de paso (vertical al imprimir: círculo)
        a = a - prisma_yz(mov(circulo(D_PASO_M3 / 2), y, 0.0), -1.0, T_INTERFAZ + 1.0)
        # tuerca M3 cautiva desde la cara delantera de la placa (abierta hacia arriba al imprimir)
        a = a - prisma_yz(mov(hexa(TUERCA_EC_A), y, 0.0), T_INTERFAZ - TUERCA_H, T_INTERFAZ + 0.01)
    return sol(a.clean())


# ============================================================================
# VARIANTE B: ABRAZADERA AL TRAVESAÑO DE LA PLACA PRINCIPAL
# ============================================================================
def hacer_B():
    zt, zi, W = Z_SUP_B, Z_SUP_B - T_CUNA, ANCHO_CUNA / 2
    zb = zt + T_CARBONO + HOLG_BOCA
    zj = zb + T_MANDIBULA
    xs0 = X_TRAV_DEL + HOLG_ESPINA
    xs1 = xs0 + T_ESPINA
    x0 = X_B0
    perfil = [(x0, zi), (xs1, zi), (xs1, zj), (x0 + 1, zj), (x0, zj - 1), (x0, zb + 1), (x0 + 1, zb),
              (xs0, zb), (xs0, zt), (x0 + 1, zt), (x0, zt - 1)]
    b = prisma_xz(perfil, -W, W) + hacer_cuna(zt, "+v")    # B se imprime con +y hacia arriba
    for s in (1, -1):
        y = s * Y_TORN_B
        b = b - Pos(X_TORN_B, y, zb - 0.5) * extrude(Polygon(*ccw(gota(D_PASO_M3 / 2)), align=None),
                                                   amount=zj - zb + 1.0)
        # tuerca cautiva abierta hacia la boca; entra deslizando por una ranura desde la cara trasera
        b = b - Pos(X_TORN_B, y, zb - 0.01) * extrude(Polygon(*ccw(hexa(TUERCA_EC_B)), align=None),
                                                    amount=TUERCA_H + 0.01)
        b = b - caja(x0 - 1.0, X_TORN_B, y - TUERCA_EC_B / 2, y + TUERCA_EC_B / 2, zb - 0.01, zb + TUERCA_H)
        y0, y1 = sorted((s * Y_CINCHO[0], s * Y_CINCHO[1]))
        xr = x0 + X_RANURA_CINCHO
        b = b - prisma_xy([(xr, y0), (xr + ANCHO_RANURA_CINCHO, y0), (xr + ANCHO_RANURA_CINCHO, y1), (xr, y1)],
                          zi - 1, zj + 1)
    return sol(b.clean())


# ============================================================================
# PROBETA DE AJUSTE
# ============================================================================
def hacer_probeta():
    amin = (Align.CENTER, Align.CENTER, Align.MIN)
    L, Wp = 70.0, 42.0
    p = Box(L, Wp, T_CUNA, align=amin)
    # pared para los alojamientos de eje horizontal (como B): y de -17 a -11, alto T_CUNA + 11
    yw0, yw1, zc = -Wp / 2 + 4.0, -Wp / 2 + 10.0, T_CUNA + 5.5
    p = p + Pos(0, (yw0 + yw1) / 2, T_CUNA - 0.01) * Box(44.0, yw1 - yw0, 11.01, align=amin)
    for i, ec in enumerate(PROBETA_EC_B):
        x = -14.0 + 14.0 * i
        p = p - prisma_xz(mov(hexa(ec), x, zc), yw1 - TUERCA_H, yw1 + 0.01)
        p = p - prisma_xz(mov(gota(D_PASO_M3 / 2), x, zc), yw0 - 1, yw1 + 1)
    # fila de alojamientos de eje vertical (como A), patrón de 22.5 mm
    for i, ec in enumerate(PROBETA_EC_A):
        x = -22.5 + 22.5 * i
        p = p - Pos(x, 0.0, -1) * Cylinder(D_PASO_M3 / 2, T_CUNA + 2, align=amin)
        p = p - Pos(x, 0.0, T_CUNA - TUERCA_H) * extrude(Polygon(*ccw(hexa(ec)), align=None), amount=TUERCA_H + 1)
    # patrón de 45 mm de la cuna
    for s in (-1, 1):
        p = p - Pos(s * M3_SEP / 2, Wp / 2 - 6.0, -1) * Cylinder(D_PASO_M3 / 2, T_CUNA + 2, align=amin)
    # peine (de la esquina achaflanada hacia el otro lado, de menor a mayor)
    for i, w in enumerate(PROBETA_RANURAS):
        x = -15.75 + 4.5 * i
        p = p - Pos(x, Wp / 2 - 3.5 + 0.01, -1) * Box(w, 7.0, T_CUNA + 2, align=amin)
    # esquina achaflanada = lado de los valores menores (ranura de 1.9, EC 5.6)
    p = p - prisma_xy([(-L / 2 - 1, Wp / 2 - 4), (-L / 2 + 4, Wp / 2 + 1), (-L / 2 - 1, Wp / 2 + 1)], -1, T_CUNA + 1)
    return sol(p.clean())


# ============================================================================
# CÁMARA SIMPLIFICADA (girada, marco local) Y ESQUEMA DEL DRON (marco del cuerpo)
# ============================================================================
def hacer_camara(z_tras):
    """Caja D435 girada: cara trasera en z = z_tras, lentes hacia abajo. Devuelve dict de piezas."""
    zf = z_tras - CAM_P
    amin = (Align.CENTER, Align.CENTER, Align.MIN)
    cuerpo = Pos(XC, 0, zf) * Box(CAM_H, CAM_L - 2 * CAM_R_EXT, CAM_P, align=amin)
    for s in (1, -1):
        cuerpo = cuerpo + Pos(XC, s * (CAM_L / 2 - CAM_R_EXT), zf) * Cylinder(CAM_R_EXT, CAM_P, align=amin)
    lentes, rgb = None, None
    for nombre, yl, _, _, ry, rx in LENTES:
        if nombre.startswith("Proyector"):
            d = Pos(XC, yl, zf - 0.4) * extrude(Ellipse(rx, ry), amount=0.41)
        else:
            d = Pos(XC, yl, zf - 0.4) * Cylinder(min(rx, ry), 0.41, align=amin)
        if nombre == "RGB":
            rgb = d
        else:
            lentes = d if lentes is None else lentes + d
    # enchufe USB-C en ángulo recto (extremo -y, mitad trasera = mitad de arriba), codo hacia arriba
    ye = -CAM_L / 2
    z_e = z_tras - 6.5                          # centro del enchufe a 6.5 mm de la cara trasera
    ench = caja(XC - 5, XC + 5, ye - L_ENCHUFE, ye + 1.0, z_e - 4, z_e + 4)
    ench = ench + caja(XC - 4, XC + 4, ye - L_ENCHUFE, ye - L_ENCHUFE + 9, z_e, z_tras + H_CODO_ENCHUFE)
    salida = (XC, ye - L_ENCHUFE + 4.5, z_tras + H_CODO_ENCHUFE)
    return {"camara": sol(cuerpo), "lentes": lentes, "rgb": rgb, "usb": ench}, salida


def hacer_dron():
    """Esquema del frente del dron (marco del cuerpo, manifold3d). SÓLO para comprobar y renderizar."""
    xa, ya = X_MOTOR, Y_MOTOR
    x_borde = TX + X_TRAV_DEL
    x_borde_inf = TX + CAM_P + DX_TRAVESANO_INF
    z_pp = TZ + Z_TECHO                              # cara inferior de la placa principal
    niveles = {"inferior": (0.0, T_CARBONO), "principal": (z_pp, z_pp + T_CARBONO)}
    partes = {}
    for nom, (z0, z1) in niveles.items():
        m = None
        for sy in (1, -1):
            aro = m_cil(xa, sy * ya, z0, z1, R_ARO_EXT, 160) - m_cil(xa, sy * ya, z0 - 1, z1 + 1, R_ARO_INT, 160)
            m = aro if m is None else m + aro
        xb = x_borde if nom == "principal" else x_borde_inf
        m = m + m_caja(xb - W_TRAVESANO, xb, -Y_POSTE + 6, Y_POSTE - 6, z0, z1)
        if nom == "principal":
            m = m + m_caja(100.0, xb - W_TRAVESANO + 1, -6, 6, z0, z1)        # barra central del cuello en Y
        partes[nom] = m
    postes = None
    r_mid = (R_ARO_INT + R_ARO_EXT) / 2
    pos = [(xa + math.sqrt(r_mid ** 2 - (ya - Y_POSTE) ** 2), sy * Y_POSTE) for sy in (1, -1)]
    for sy in (1, -1):
        for k in range(10):
            ang = math.radians(18 + 36 * k)
            pos.append((xa + r_mid * math.cos(ang), sy * ya + r_mid * math.sin(ang)))
    for (x, y) in pos:
        if x > 60:
            p = m_cil(x, y, T_CARBONO, z_pp, R_POSTE, 16)
            postes = p if postes is None else postes + p
    partes["postes"] = postes
    # escuadra original (forma desconocida: bloque detrás de la cámara + lengüeta bajo la cámara)
    zb = TZ + Z_BASE
    esc = m_caja(TX - 8, TX, -30, 30, zb - LENG_T, z_pp)
    esc = esc + m_caja(TX - 0.1, TX + LENG_LARGO, -LENG_ANCHO / 2, LENG_ANCHO / 2, zb - LENG_T, zb)
    if LENG_LABIO > 0:
        esc = esc + m_caja(TX + CAM_P, TX + CAM_P + 2.0, -LENG_ANCHO / 2, LENG_ANCHO / 2, zb - LENG_T,
                           zb + LENG_LABIO)
    partes["escuadra_original"] = esc
    return partes, pos[:2]


def ruta_cable(salida, n_arco=16):
    """Polilínea del cable (marco del cuerpo): sube del codo, curva con R_CURVA_CABLE hacia atrás y corre
    sobre la placa principal hasta detrás del travesaño."""
    x_e, y_e, z_e = salida
    R = R_CURVA_CABLE
    pts = [np.array([x_e, y_e, z_e])]
    for k in range(1, n_arco + 1):
        f = math.pi / 2 * k / n_arco
        pts.append(np.array([x_e - R + R * math.cos(f), y_e, z_e + R * math.sin(f)]))
    pts.append(np.array([TX + X_TRAV_TRAS - 15.0, y_e, z_e + R]))
    return pts


# ============================================================================
# COMPROBACIONES
# ============================================================================
def puntos(tm, n=60000):
    p, _ = trimesh.sample.sample_surface(tm, n, seed=1)
    return np.vstack([p, tm.vertices])


def conos(z_cara_cuerpo):
    """Ápice (centro óptico) y semiángulos de cada lente en el marco del cuerpo."""
    out = []
    for nombre, yl, fh, fv, ry, rx in LENTES:
        apex = np.array([TX + XC, yl, z_cara_cuerpo + LENTE_PROF])
        ty = math.tan(math.radians(fh / 2 + TOL_FOV))      # horizontal de la imagen = eje y del cuerpo
        tx = math.tan(math.radians(fv / 2 + TOL_FOV))      # vertical de la imagen = eje x del cuerpo
        out.append((nombre, apex, tx, ty, rx, ry))
    return out


def margen_cono(P, cono):
    """Margen (mm) de cada punto fuera de la pirámide de visión; < 0 = dentro."""
    _, apex, tx, ty, rx, ry = cono
    d = apex[2] - P[:, 2]
    m = d > 1e-6
    if not np.any(m):
        return np.inf, 0
    Q, d = P[m], d[m]
    mx = np.abs(Q[:, 0] - apex[0]) - (d * tx + rx)
    my = np.abs(Q[:, 1] - apex[1]) - (d * ty + ry)
    mg = np.maximum(mx, my)
    return float(mg.min()), int((mg < 0).sum())


def dist_min(Pa, Pb):
    return float(cKDTree(Pb).query(Pa)[0].min())


def imprimibilidad(tm, nombre):
    """Watertight, cuerpos, voladizos > 45° y análisis capa a capa (islas y área sin apoyo)."""
    n = tm.face_normals
    zmin = tm.bounds[0][2]
    en_cama = tm.triangles[:, :, 2].max(axis=1) <= zmin + 0.25
    volado = (n[:, 2] < -math.sin(math.radians(46))) & ~en_cama
    puente = (n[:, 2] < -0.999) & ~en_cama
    nz = np.clip(-n[:, 2], -1, 1)
    no_plano = (n[:, 2] < -0.01) & (n[:, 2] > -0.999) & ~en_cama
    peor_ang = float(np.degrees(np.arcsin(nz[no_plano])).max()) if np.any(no_plano) else 0.0
    M = a_manifold(tm)
    alto = tm.bounds[1][2] - zmin
    prev, islas, sin_apoyo, peor = None, 0, 0.0, (0.0, 0.0)
    for k in range(int(alto / CAPA)):
        z = zmin + (k + 0.5) * CAPA
        cs = M.slice(z)
        if prev is not None and not cs.is_empty():
            apoyo = prev.offset(CAPA * math.tan(math.radians(45)) + 0.02, manifold3d.JoinType.Round)
            libre = (cs - apoyo).area()
            sin_apoyo += libre
            if libre > peor[0]:
                peor = (libre, z)
            for comp in cs.decompose():
                if (comp ^ prev).area() < 1e-6:
                    islas += 1
        prev = cs
    return dict(nombre=nombre, watertight=bool(tm.is_watertight), cuerpos=len(tm.split(only_watertight=False)),
                vol=float(tm.volume), area=float(tm.area), bbox=np.round(tm.extents, 1),
                volado=float(tm.area_faces[volado].sum()), puente=float(tm.area_faces[puente].sum()),
                peor_ang=peor_ang, islas=islas, sin_apoyo=sin_apoyo, peor=peor)


def masa(r, rho):
    """(masa maciza, masa impresa estimada) en g: casco macizo de PERIMETROS_MM + relleno RELLENO."""
    v = r["vol"]
    v_casco = min(v, r["area"] * PERIMETROS_MM)
    return v * rho, (v_casco + RELLENO * (v - v_casco)) * rho


def seccion_A(x):
    """Rectángulos (ancho, z0, z1) de la sección del brazo A en x (marco local)."""
    zs, zi = Z_SUP_A, Z_SUP_A - T_CUNA
    u = x - XC
    t, s = VENT_PUNTA, VENT_SEMI
    if t - 1.5 * s <= u <= t - s:
        hueco = 2 * (s / 2 + (u - (t - 1.5 * s)))
    elif t - s < u <= t:
        hueco = 2 * (t - u)
    else:
        hueco = 0.0
    rect = [(ANCHO_CUNA - hueco, zi, zs)]
    if x <= X_NERV_FIN:
        rect.append((len(NERVIOS_Y) * NERVIO_T, Z_NERV, zi))
    h = altura_costilla(x)
    if h > 0:
        rect.append((len(COSTILLAS_Y) * COSTILLA_T, zs, zs + h))
    A = sum(b * (z1 - z0) for b, z0, z1 in rect)
    zc = sum(b * (z1 - z0) * (z0 + z1) / 2 for b, z0, z1 in rect) / A
    I = sum(b * (z1 - z0) ** 3 / 12 + b * (z1 - z0) * ((z0 + z1) / 2 - zc) ** 2 for b, z0, z1 in rect)
    return I, zc


def rigidez_A():
    """Voladizo del brazo A (Euler-Bernoulli, EI(x)) empotrado en la placa de interfaz, con la cámara como
    cuerpo rígido (masa + inercia de cabeceo) en x = XC. Modos de 2 GDL (flecha y giro de la punta).
    Casos: 'cuna flexible' (todo el voladizo flexible hasta XC) y 'cuna rígida' (la cámara atornillada
    rigidiza la cuna desde su borde trasero). No incluye la flexibilidad de la placa de interfaz, de la
    escuadra original ni de las uniones: la frecuencia real es menor."""
    xs = np.linspace(T_INTERFAZ, XC, 3000)
    dx = xs[1] - xs[0]
    sec = [seccion_A(x) for x in xs]
    EI = np.array([E_IMPRESO * I for I, _ in sec])            # N·mm2
    zc_tip = sec[-1][1]
    m = (MASA_CAM + RHO_PETG * (CAM_H * ANCHO_CUNA * T_CUNA + len(COSTILLAS_Y) * COSTILLA_T * COSTILLA_H *
                                (X_COST1 - X_COST0))) / 1000.0           # kg (cámara + cuna y costillas)
    e = (zc_tip - (Z_SUP_A - T_CUNA - CAM_P / 2)) / 1000.0          # m, CM de la cámara bajo el eje neutro
    Icg = MASA_CAM / 1000.0 * ((CAM_H / 1000) ** 2 + (CAM_P / 1000) ** 2) / 12.0
    J = Icg + MASA_CAM / 1000.0 * e ** 2
    out = {}
    for caso, xmax in (("cuna flexible", XC), ("cuna rígida", XC - CAM_H / 2)):
        k_ = xs <= xmax
        L = XC
        f11 = np.sum(((L - xs) ** 2 / EI)[k_]) * dx
        f12 = np.sum(((L - xs) / EI)[k_]) * dx
        f22 = np.sum((1 / EI)[k_]) * dx
        K = np.linalg.inv(np.array([[f11, f12], [f12, f22]]))       # N/mm, N, N·mm/rad
        Ksi = np.array([[K[0, 0] * 1e3, K[0, 1]], [K[1, 0], K[1, 1] * 1e-3]])
        w2 = np.linalg.eigvals(np.linalg.solve(np.diag([m, J]), Ksi))
        fr = sorted(float(np.sqrt(abs(w))) / (2 * math.pi) for w in np.real(w2))
        out[caso] = dict(k=1.0 / f11, f_tras=math.sqrt(1e3 / f11 / m) / (2 * math.pi), modos=fr,
                         flecha=f11 * m * 9.81)
    return out, e * 1000, J, min(I for I, _ in sec)


def banda_rotores(masa_kg):
    """Frecuencia de giro 1P (Hz, sin carga = cota superior) en hover y a la tensión máxima."""
    a, b, c = EMPUJE_POLI
    T = masa_kg * 9.81 / 4
    V = (-b + math.sqrt(b * b - 4 * a * (c - T))) / (2 * a)
    return V, KV_MOTOR * V / 60.0, KV_MOTOR * V_BAT_MAX / 60.0


def eje_dominante(v):
    k = int(np.argmax(np.abs(v)))
    e = np.zeros(3)
    e[k] = math.copysign(1.0, v[k])
    return e


def rotacion_montaje(cam_tm):
    """Deriva R (cuerpo <- cámara) de la cámara MODELADA (marco del cuerpo) y devuelve el ángulo de geo.py.
    x_cam (derecha de la imagen) = hacia el extremo del USB-C (la IR derecha está en ese extremo);
    z_cam (eje óptico) = normal saliente de la cara de lentes; y_cam = z_cam x x_cam (hacia la rosca 1/4-20)."""
    c = cam_tm["camara"].centroid
    x_cam = eje_dominante(cam_tm["usb"].centroid - c)
    lente = cam_tm["rgb"]                       # disco de la lente: su cara grande saliente da la normal
    d = lente.centroid - c
    ejes = {}
    for n, a in zip(lente.face_normals, lente.area_faces):
        if np.dot(n, d) > 0:
            k = tuple(eje_dominante(n))
            ejes[k] = ejes.get(k, 0.0) + a
    z_cam = np.array(max(ejes, key=ejes.get))
    y_cam = np.cross(z_cam, x_cam)
    R = np.column_stack([x_cam, y_cam, z_cam])
    Mz = R_BODY_CAM0.T @ R
    assert abs(Mz[2, 2] - 1) < 1e-9, "la cámara no mira hacia abajo"
    return round(math.degrees(math.atan2(Mz[1, 0], Mz[0, 0]))) % 360, R


# ============================================================================
# PRINCIPAL
# ============================================================================
def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    print("Construyendo piezas ...")
    A = hacer_A()
    B = hacer_B()
    probeta = hacer_probeta()
    camA, salA = hacer_camara(Z_SUP_A - T_CUNA)
    camB, salB = hacer_camara(Z_SUP_B - T_CUNA)
    dron, postes_trav = hacer_dron()

    # ---------------- Exportación (orientación de impresión) ----------------
    imp = {
        PREF + "A_adaptador": Rot(0, -90, 0) * A,         # de pie: cara trasera de la interfaz (x=0) sobre la cama
        PREF + "B_abrazadera": Rot(90, 0, 0) * B,         # de lado: y=-30 sobre la cama
        PREF + "probeta": probeta,
    }
    res = {}
    for nombre, shp in imp.items():
        bb = shp.bounding_box()
        shp = Pos(-(bb.min.X + bb.max.X) / 2, -(bb.min.Y + bb.max.Y) / 2, -bb.min.Z) * shp
        export_step(shp, os.path.join(DIR_STEP, nombre + ".step"))
        tm = a_trimesh(shp, os.path.join(DIR_STL, nombre + ".stl"))
        res[nombre] = imprimibilidad(tm, nombre)

    # ---------------- Mallas de uso (marco del cuerpo) ----------------
    p = lambda n: os.path.join(DIR_USO, PREF + n + ".stl")
    tmp = os.path.join(DIR_USO, "_tmp.stl")
    mA = a_cuerpo(a_trimesh(A, tmp))
    mB = a_cuerpo(a_trimesh(B, tmp))
    mallas = {"A": mA, "B": mB}
    cams = {}
    for var, cam in (("A", camA), ("B", camB)):
        cams[var] = {k: a_cuerpo(a_trimesh(v, tmp)) for k, v in cam.items()}
    os.remove(tmp)
    salidas = {v: np.array(s) + np.array([TX, 0, TZ]) for v, s in (("A", salA), ("B", salB))}
    mA.export(p("A_uso"))
    mB.export(p("B_uso"))
    for var in ("A", "B"):
        for k, v in cams[var].items():
            v.export(p(f"{var}_{k}"))
    dron_tm = {k: de_manifold(v) for k, v in dron.items()}
    recorte = m_caja(140, 240, -95, 95, -5, 80)      # recorte del esquema para los renders
    for k, v in dron.items():
        de_manifold(v ^ recorte).export(p(f"dron_{k}"))
    # piso, eje óptico y pirámides de visión (alambre) para los renders
    piso = trimesh.creation.box(extents=[150, 190, 1.0])
    piso.apply_translation([190, 0, -0.5])
    piso.export(p("piso"))
    cables = {}
    for var, z_sup in (("A", Z_SUP_A), ("B", Z_SUP_B)):
        z_cara = TZ + z_sup - T_CUNA - CAM_P
        _, apex, tx, ty, _, _ = conos(z_cara)[0]
        prof = 70.0
        esq = [apex + np.array([sx * prof * tx, sy * prof * ty, -prof]) for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
        hilos = [varilla(apex, e) for e in esq] + [varilla(esq[i], esq[(i + 1) % 4]) for i in range(4)]
        hilos.append(varilla(apex, apex + np.array([0, 0, -prof]), 0.9))
        trimesh.util.concatenate(hilos).export(p(f"{var}_cono_rgb"))
        # cable USB: sube desde el codo, curva con R_CURVA_CABLE y vuelve sobre la placa principal
        ruta = ruta_cable(salidas[var])
        cables[var] = (ruta, m_tubo(ruta, D_CABLE / 2))
        de_manifold(cables[var][1]).export(p(f"{var}_cable"))
    export_step(Compound([Pos(TX, 0, TZ) * A, Pos(TX, 0, TZ) * camA["camara"]]),
                os.path.join(DIR_STEP, PREF + "A_ensamble_uso.step"))
    export_step(Compound([Pos(TX, 0, TZ) * B, Pos(TX, 0, TZ) * camB["camara"]]),
                os.path.join(DIR_STEP, PREF + "B_ensamble_uso.step"))

    # ======================= RESUMEN =======================
    print("\n================ SOPORTE DE CÁMARA: COMPROBACIONES ================")
    rot, R = rotacion_montaje(cams["A"])
    rotB, _ = rotacion_montaje(cams["B"])
    print(f"Orientación derivada de la cámara modelada: x_cam={R[:, 0]}, y_cam={R[:, 1]}, z_cam={R[:, 2]} (cuerpo)")
    print(f"  -> camara.rotacion_montaje = {rot} (A) / {rotB} (B)  (montada al revés, con el USB en +y: 180)")
    print(f"Centro de la cámara girada: x_local = {XC:.2f} mm (borde del travesaño en {X_TRAV_DEL:.2f}, "
          f"holgura {HOLG_TRAV:.1f}); adelanto respecto al centro de fábrica: {XC - CAM_P / 2:.1f} mm")
    x_frente = TX + XC + CAM_H / 2
    x_aros = TX + X_TRAV_DEL + BOLSILLO_FRENTE
    print(f"Cara delantera de la cámara girada en x = {x_frente:.1f} mm. Vuelo por delante de los aros: "
          f"{x_frente - DRON_L / 2:+.1f} mm respecto al largo total (x = {DRON_L / 2:.1f}); "
          f"{x_frente - x_aros:+.1f} mm con el bolsillo de {BOLSILLO_FRENTE:.0f} mm (x = {x_aros:.1f}). "
          f"Para no sobresalir del largo total: HOLG_TRAV <= {HOLG_TRAV - (x_frente - DRON_L / 2):.1f}")

    PD = {k: puntos(v) for k, v in dron_tm.items()}
    for var, z_sup in (("A", Z_SUP_A), ("B", Z_SUP_B)):
        z_tras = TZ + z_sup - T_CUNA
        z_cara = z_tras - CAM_P
        rgb = np.array([TX + XC, LENTES[0][1], z_cara + LENTE_PROF])
        print(f"\n---------------- VARIANTE {var} ----------------")
        print(f"Cara de lentes a {z_cara:.2f} mm del piso (MARGEN AL PISO con el dron apoyado; Z_EJE_ORIG="
              f"{Z_EJE_ORIG} supuesto). Con Z_EJE_ORIG 28..35 mm: {z_cara - (Z_EJE_ORIG - 28):.1f}.."
              f"{z_cara + (35 - Z_EJE_ORIG):.1f} mm")
        print(f"Centro óptico RGB (cuerpo, desde el centro geométrico; z desde el piso): "
              f"({rgb[0]:.1f}, {rgb[1]:+.1f}, {rgb[2]:.1f}) mm")
        print(f"  -> offset_camara_m estimado = [{rgb[0] / 1000:.3f}, {rgb[1] / 1000:.4f}, {rgb[2] / 1000:.3f}] "
              f"(si el pivote de Motive está en el centro geométrico en planta y la z de la pose es la "
              f"corregida con bias_z, z = 0 en el piso)")
        sop_pts = puntos(mallas[var], 40000)
        print("Conos de visión (+%.1f° por lado; cara frontal y abajo):" % TOL_FOV)
        for cono in conos(z_cara):
            txt = []
            for nom, P in [("soporte", sop_pts)] + [(k, v) for k, v in PD.items()]:
                mg, n_in = margen_cono(P, cono)
                txt.append(f"{nom}={'—' if mg == np.inf else f'{mg:+.1f}'}" + (f"[{n_in} DENTRO]" if n_in else ""))
            print(f"  {cono[0]:28s} margen mínimo (mm, >0 = fuera): " + ", ".join(txt))
        # interferencias
        Ms = a_manifold(mallas[var])
        Mc = a_manifold(cams[var]["camara"])
        Mu = a_manifold(cams[var]["usb"])
        Mk = cables[var][1]
        inter = [("soporte x cámara", (Ms ^ Mc).volume())]
        for k, v in dron.items():
            inter.append((f"soporte x {k}", (Ms ^ v).volume()))
            inter.append((f"cámara x {k}", (Mc ^ v).volume()))
            inter.append((f"enchufe x {k}", (Mu ^ v).volume()))
            inter.append((f"cable x {k}", (Mk ^ v).volume()))
        inter.append(("enchufe x soporte", (Mu ^ Ms).volume()))
        inter.append(("cable x soporte", (Mk ^ Ms).volume()))
        print("Interferencias (mm3, contacto plano = 0): " + ", ".join(f"{n}={v:.2f}" for n, v in inter))
        # enchufe y cable: distancias mínimas
        Pu = puntos(cams[var]["usb"], 8000)
        Pk = puntos(de_manifold(Mk), 8000)
        du = {k: dist_min(Pu, v) for k, v in PD.items()}
        dk = {k: dist_min(Pk, v) for k, v in PD.items()}
        print(f"Enchufe USB-C ({L_ENCHUFE:.0f} mm, supuesto): distancia mínima al esquema " +
              ", ".join(f"{k}={v:.1f}" for k, v in du.items()) + f"; al soporte {dist_min(Pu, sop_pts):.1f} mm")
        print(f"Cable (Ø{D_CABLE}, R {R_CURVA_CABLE:.0f}): distancia mínima al esquema " +
              ", ".join(f"{k}={v - D_CABLE / 2:.1f}" for k, v in dk.items()) + " mm")
        print(f"  borde del soporte a {CAM_L / 2 - ANCHO_CUNA / 2:.1f} mm del extremo del USB")
        # contacto térmico
        zc = z_tras + 0.05
        sec = Ms.slice(zc)
        huella = a_manifold(cams[var]["camara"]).slice(z_tras - 0.05)
        cont = (sec ^ huella).area()
        cara = huella.area()
        area_cam = cams[var]["camara"].area
        print(f"Cara trasera de la cámara cubierta por el soporte: {cont:.0f} de {cara:.0f} mm2 "
              f"({100 * cont / cara:.0f} %); de toda la carcasa: {100 * cont / area_cam:.0f} %. "
              f"Caras con rejilla (adelante/atrás) libres.")

    # cable: largo adicional respecto a la ruta de fábrica
    print("\n---------------- CABLE USB ----------------")
    x_sal_fab = -H_CODO_ENCHUFE                     # de fábrica el codo sale hacia atrás de la cara trasera
    for var, s in (("A", salA), ("B", salB)):
        dx_sal = s[0] - x_sal_fab
        extra = dx_sal + (math.pi / 2 - 1) * R_CURVA_CABLE
        print(f"{var}: la salida del cable pasa de x_local = {x_sal_fab:.1f} (hacia -x) a x_local = {s[0]:.1f} "
              f"(hacia +z): {dx_sal:.1f} mm más adelante y gira 90°. Largo extra mínimo = {dx_sal:.1f} + "
              f"(π/2 - 1)·{R_CURVA_CABLE:.0f} = {extra:.0f} mm (centro del enchufe: "
              f"{XC - 6.5:.1f} mm más adelante). Recomendado: holgura >= {math.ceil((extra + 10) / 5) * 5:.0f} mm "
              f"o extensión USB-C 3.1 Gen 1")

    print("\n---------------- PIEZAS (orientación de impresión) ----------------")
    for nombre, r in res.items():
        ms, mi = masa(r, RHO_PETG)
        print(f"{nombre:28s} watertight={r['watertight']} cuerpos={r['cuerpos']} vol={r['vol'] / 1000:.2f} cm3 "
              f"bbox={r['bbox']} | PETG macizo {ms:.1f} g, impreso ~{mi:.1f} g ({RELLENO * 100:.0f} % + "
              f"{PERIMETROS_MM} mm de casco) | voladizo >45° {r['volado']:.1f} mm2 (planos {r['puente']:.1f} mm2; "
              f"voladizo no plano más inclinado {r['peor_ang']:.1f}°) | islas {r['islas']} | área sin apoyo capa "
              f"a capa {r['sin_apoyo']:.1f} mm2 (máx {r['peor'][0]:.1f} mm2 en z={r['peor'][1]:.1f})")
        assert r["watertight"] and r["cuerpos"] == 1 and r["islas"] == 0, nombre

    # masa y centro de masa
    print("\n---------------- MASA Y CENTRO DE MASA ----------------")
    for var, z_sup in (("A", Z_SUP_A), ("B", Z_SUP_B)):
        r = res[PREF + ("A_adaptador" if var == "A" else "B_abrazadera")]
        _, m_s = masa(r, RHO_PETG)
        c_s = mallas[var].center_mass
        c_cam_ant = np.array([TX + CAM_P / 2, 0.0, TZ])
        c_cam_new = np.array([TX + XC, 0.0, TZ + z_sup - T_CUNA - CAM_P / 2])
        cg = np.array([0.0, 0.0, Z_CG_DRON])
        dcg = (MASA_CAM * (c_cam_new - c_cam_ant) + m_s * (c_s - cg)) / (MASA_DRON + m_s)
        dT = dcg[0] / X_MOTOR
        print(f"{var}: soporte ~{m_s:.1f} g (PETG) -> masa de vuelo {MASA_DRON + m_s:.0f} g "
              f"(+{100 * m_s / MASA_DRON:.1f} % sobre {MASA_DRON:.0f}); cámara movida {c_cam_new[0] - c_cam_ant[0]:+.1f} "
              f"mm en x y {c_cam_new[2] - c_cam_ant[2]:+.1f} mm en z; CM del dron ≈ ({dcg[0]:+.1f}, {dcg[1]:+.1f}, "
              f"{dcg[2]:+.1f}) mm -> en hover los motores delanteros dan ~{100 * dT:+.1f} % y los traseros "
              f"~{-100 * dT:+.1f} % de empuje (brazo {X_MOTOR} mm)")

    print("\n---------------- RIGIDEZ, VIBRACIÓN Y TORNILLERÍA ----------------")
    rig, e_mm, J, Imin = rigidez_A()
    m_vuelo = (MASA_DRON + 30.0) / 1000.0
    V_h, f_h, f_max = banda_rotores(m_vuelo)
    for caso, d in rig.items():
        print(f"A ({caso}): voladizo {XC - T_INTERFAZ:.1f} mm, E={E_IMPRESO:.0f} MPa: k vertical = {d['k']:.0f} N/mm, "
              f"flecha a 1 g = {d['flecha'] * 1000:.1f} µm; sólo traslación {d['f_tras']:.0f} Hz; modos acoplados "
              f"flexión+cabeceo: {d['modos'][0]:.0f} Hz y {d['modos'][1]:.0f} Hz")
    print(f"   (CM de la cámara {e_mm:.1f} mm bajo el eje neutro, J = {J:.2e} kg·m2, I mínima {Imin:.0f} mm4; "
          f"sin la flexibilidad de la placa de interfaz ni de la escuadra original)")
    print(f"Rotores (1P, cota sin carga): hover ~{V_h:.1f} V -> ~{f_h:.0f} Hz; máximo {V_BAT_MAX} V -> ~{f_max:.0f} Hz. "
          f"Paso de pala (2P) hasta ~{2 * f_max:.0f} Hz. Modo más bajo de A / 1P hover = "
          f"{rig['cuna flexible']['modos'][0] / f_h:.2f}..{rig['cuna rígida']['modos'][0] / f_h:.2f}")
    print(f"Espesores mínimos: cuna {T_CUNA} mm, interfaz {T_INTERFAZ} mm ({T_INTERFAZ - TUERCA_H:.1f} detrás de la tuerca), "
          f"nervios {NERVIO_T} mm, costillas {COSTILLA_T} mm, espina B {T_ESPINA} mm, mandíbula B {T_MANDIBULA} mm "
          f"({T_MANDIBULA - TUERCA_H:.1f} sobre la tuerca), pared trasera de las ranuras de cincho {X_RANURA_CINCHO} mm")
    rosca = L_TORN_CAM - T_CUNA
    F = PAR_CAM_NM / (K_PAR * 0.003)
    a_brida = math.pi / 4 * (D_BRIDA_TORN ** 2 - D_PASO_M3 ** 2)
    a_912 = math.pi / 4 * (5.5 ** 2 - D_PASO_M3 ** 2)
    F_intel = M3_PAR_INTEL_NM / (K_PAR * 0.003)
    print(f"Cámara: 2 x M3x{L_TORN_CAM:.0f} ISO 7380-2 (brida Ø{D_BRIDA_TORN}) sobre {T_CUNA} mm de cuna -> {rosca:.1f} "
          f"mm dentro de la cámara (máx {M3_PROF_MAX}; {'OK' if 1.5 <= rosca <= M3_PROF_MAX else 'REVISAR'}). "
          f"Par {PAR_CAM_NM} N·m -> precarga ~{F:.0f} N, presión bajo la brida {F / a_brida:.0f} MPa "
          f"(con {M3_PAR_INTEL_NM} N·m y DIN 912 serían {F_intel:.0f} N y {F_intel / a_912:.0f} MPa: demasiado para "
          f"plástico a 50 °C). NO usar M3x8 ({8 - T_CUNA:.0f} mm > {M3_PROF_MAX}).")
    print(f"A: 2 x M3 de largo >= espesor de la escuadra original + {T_INTERFAZ:.0f} mm + 2 tuercas M3 en la placa "
          f"de interfaz (la punta puede sobresalir hasta ~30 mm por delante, bajo el brazo: no toca nada).")
    hueco_b = HOLG_BOCA + T_MANDIBULA
    F_b = PAR_B_NM / (K_PAR * 0.003)
    print(f"B: 2 x M3x8 de NYLON + 2 tuercas M3 (entran por la ranura trasera). De la cara superior de la mandíbula "
          f"a la placa hay {hueco_b:.1f} mm: la cabeza queda {8 - hueco_b:.1f} mm levantada. Par <= {PAR_B_NM} N·m "
          f"(~{F_b:.0f} N; con dedos).")

    print("\n---------------- PARÁMETROS 'MEDIR EN LAB' ----------------")
    for n, v in (("X_CARA_TRASERA_ORIG", X_CARA_TRASERA_ORIG), ("Z_EJE_ORIG", Z_EJE_ORIG),
                 ("DX_TRAVESANO", DX_TRAVESANO), ("W_TRAVESANO", W_TRAVESANO), ("T_CARBONO", T_CARBONO),
                 ("DX_TRAVESANO_INF", DX_TRAVESANO_INF), ("LENG_ANCHO", LENG_ANCHO), ("LENG_LARGO", LENG_LARGO),
                 ("LENG_T", LENG_T), ("LENG_LABIO", LENG_LABIO), ("R_ARO_EXT", R_ARO_EXT), ("R_ARO_INT", R_ARO_INT),
                 ("BOLSILLO_FRENTE", BOLSILLO_FRENTE), ("Y_POSTE", Y_POSTE), ("L_ENCHUFE", L_ENCHUFE),
                 ("H_CODO_ENCHUFE", H_CODO_ENCHUFE), ("R_CURVA_CABLE", R_CURVA_CABLE), ("D_CABLE", D_CABLE),
                 ("Z_CG_DRON", Z_CG_DRON)):
        print(f"  {n:20s} = {v}")
    print("Listo.")


if __name__ == "__main__":
    main()
