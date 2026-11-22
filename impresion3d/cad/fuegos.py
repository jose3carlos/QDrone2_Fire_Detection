"""
Fueguitos bicolor (base ROJA + flama central AMARILLA) para el proyecto QDrone 2.
Tres variantes (A, B, C) para dar diversidad al dataset de deteccion.

Pieza ROJA    : placa base lobulada con chaflan + perno central en "D" (anti-giro)
                + anillo de N lenguas de fuego que suben, se abren hacia afuera y giran.
Pieza AMARILLA: flama central (1, 3 o 2 lenguas) que nace de un pedestal redondo
                ("brasa") de base plana. Por debajo tiene una cuenca en "D" que
                encaja sobre el perno de la roja; el pedestal se apoya plano sobre
                la placa roja (sin pegamento).
Monocolor     : base + flama fusionadas en una sola pieza (para quien tenga un solo color).

Revision 2 (tras revision de diseno):
  - Union invertida: el perno en "D" ahora esta en la ROJA y la cuenca en la AMARILLA.
    Asi la amarilla se imprime sobre una base plana de ~29 mm de diametro (antes un
    perno de 14 mm), mucho mas estable para piezas de 92-121 mm de alto.
  - Boca de la cuenca con chaflan de 0.5 mm a 45 grados (absorbe la "pata de elefante"
    de la primera capa) y punta del perno con chaflan de entrada.
  - Puntas de las lenguas un poco mas gruesas (elipse final 1.6 x 1.4 mm).
  - Todas las constantes de forma movidas aqui arriba.

Orientacion de impresion (sin soportes):
  - roja y monocolor: placa sobre la cama.
  - amarilla: pedestal sobre la cama, punta hacia arriba (brim recomendado 5-8 mm).
Todas las superficies colgantes quedan <= ~45 grados respecto a la vertical.

Uso:  python fuegos.py [A B C]   (escribe STL y STEP en ../stl y ../step)
"""
import math
import os
import numpy as np
from scipy.interpolate import PchipInterpolator
from build123d import (Solid, Wire, Edge, Vertex, Plane, Vector, export_stl, export_step)
from OCP.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCP.TopoDS import TopoDS

# ----------------------------------------------------------------------------
# Rutas
# ----------------------------------------------------------------------------
AQUI = os.path.dirname(os.path.abspath(__file__))
DIR_STL = os.path.normpath(os.path.join(AQUI, "..", "stl"))
DIR_STEP = os.path.normpath(os.path.join(AQUI, "..", "step"))

# ----------------------------------------------------------------------------
# Placa roja (mm)
# ----------------------------------------------------------------------------
ESPESOR_PLACA = 3.6        # espesor de la placa base
CHAFLAN_SUP = 1.2          # chaflan del borde superior de la placa
CHAFLAN_INF = 0.4          # chaflan inferior (anti "pata de elefante", 45 grados)
PUNTOS_POR_LOBULO = 12     # puntos del spline del contorno por cada lobulo
EMPOTRADO = 1.0            # cuanto se hunden las lenguas dentro de la placa

# ----------------------------------------------------------------------------
# Union roja/amarilla: perno en "D" (roja) dentro de cuenca en "D" (amarilla)
# ----------------------------------------------------------------------------
R_PERNO = 7.0              # radio del perno en "D" de la roja
PLANO_D = 5.2              # distancia del centro al plano de la "D" (anti-giro)
ALTO_PERNO = 8.0           # altura del perno sobre la cara superior de la placa
CHAFLAN_PERNO = 0.6        # chaflan de entrada en la punta del perno (45 grados)
RAIZ_PERNO = 0.5           # el perno nace 0.5 mm dentro de la placa (fusion limpia)
HOLGURA = 0.22             # holgura radial perno/cuenca (en el arco y en el plano)
HOLGURA_TECHO = 0.4        # luz entre la punta del perno y el techo de la cuenca
CHAFLAN_BOCA = 0.5         # chaflan 45 grados en la boca de la cuenca (pata de elefante)
ANG_TECHO = 40.0           # techo conico de la cuenca, grados respecto a la vertical
                           # (imprimible sin soporte), truncado arriba
R_TECHO_MIN = 0.4          # radio de la tapita plana del techo (un apice exacto da
                           # triangulos degenerados en el STL)

# ----------------------------------------------------------------------------
# Pedestal ("brasa") de la amarilla, en coordenadas locales (z=0 = cama)
# ----------------------------------------------------------------------------
R_PEDESTAL = 14.5          # radio de la base plana de la amarilla
CHAFLAN_PEDESTAL = 0.4     # chaflan inferior del pedestal (anti pata de elefante)
ALTO_PEDESTAL_RECTO = 3.0  # tramo cilindrico del pedestal
R_CUELLO = 10.0            # el pedestal se cierra a 45 grados hasta este radio
Z_FLAMA_INI = 6.0          # z local donde arranca el loft de la flama (dentro del pedestal)
R_FLAMA_INI = 10.5         # semieje inicial de la flama (simple)
DESFASE_TRENZA = 1.0       # centro inicial de cada lengua trenzada respecto al eje;
                           # su semieje inicial se reduce lo mismo (R_FLAMA_INI - 1.0)

# ----------------------------------------------------------------------------
# Lofts de lenguas (rojas y amarillas)
# ----------------------------------------------------------------------------
N_SECC = 14                # secciones por loft de cada lengua
PUNTA_A, PUNTA_B = 0.8, 0.7     # semiejes de la elipse de la punta (1.6 x 1.4 mm)
SEMIEJE_MIN = 0.8          # semieje minimo de las secciones intermedias (rojas)
# perfil de inclinacion radial de las lenguas rojas: (u, fraccion de 'inclinacion')
# vertical al nacer, se abre, y la punta se recoge un poco
PERFIL_INCL_RADIAL = [(0, 0.0), (0.2, 0.55), (0.5, 1.0), (0.8, 1.0), (1.0, 0.5)]
# perfil de inclinacion tangencial (remolino): (u, fraccion de 'remolino')
PERFIL_INCL_TANG = [(0, 0.0), (0.3, 0.5), (0.7, 1.0), (1.0, 1.2)]
FORMA_EXP = 0.8            # forma(u) = (1-u)^FORMA_EXP * (1 + FORMA_PANZA*sin(pi u))
FORMA_PANZA = 0.45
N_INTEGRACION = 60         # muestras para integrar la inclinacion
# perfiles de semiejes de la flama amarilla: (u, fraccion de a_max/b_max);
# el primer punto (u=0) es el radio inicial y el ultimo (u=1) la punta
PERFIL_SIMPLE = [(0.14, 0.85), (0.32, 1.0), (0.55, 0.78), (0.8, 0.38), (0.93, 0.16)]
PERFIL_TRENZA = [(0.25, 1.0), (0.5, 0.8), (0.8, 0.33), (0.93, 0.16)]
# curva en "S" del eje de la flama simple: x = AMP_X sin(FREC_X pi u) u ; y = AMP_Y sin(pi u) u
S_AMP_X, S_FREC_X, S_AMP_Y = 3.0, 1.6, 1.5
U_ESPIRAL_MAX = 0.7        # fraccion de altura en la que la trenza alcanza r_espiral

# ----------------------------------------------------------------------------
# Variantes
#   n           : numero de lenguas rojas
#   r_placa     : radio medio de la placa; lobulo = amplitud de los lobulos
#   r_anillo    : radio donde nacen las lenguas rojas
#   a0, b0      : semiejes (tangencial, radial) de la base de cada lengua roja
#   alturas     : alturas de las lenguas rojas sobre la placa (se repite el patron)
#   inclinacion : inclinacion maxima hacia afuera (grados)
#   remolino    : inclinacion tangencial maxima (grados) -> efecto de giro
#   torsion     : giro de la elipse de la base a la punta (grados)
#   amarilla    : flama central. tipo "simple" (una lengua) o "trenzada" (n lenguas
#                 en espiral). z_punta = altura total del fuego ensamblado.
#                 a_max/b_max = semiejes maximos de cada lengua amarilla,
#                 r_espiral = radio de la espiral, giro = vueltas en grados,
#                 factores = altura relativa de cada lengua trenzada.
# ----------------------------------------------------------------------------
VARIANTES = {
    "A": dict(n=6, r_placa=38.0, lobulo=4.0, r_anillo=24.0, a0=10.5, b0=5.5,
              alturas=[52.0, 45.0], inclinacion=34.0, remolino=8.0, torsion=25.0,
              amarilla=dict(tipo="simple", z_punta=112.0, torsion=150.0,
                            a_max=18.0, b_max=14.0)),
    "B": dict(n=5, r_placa=37.0, lobulo=5.0, r_anillo=24.0, a0=12.0, b0=6.0,
              alturas=[62.0, 54.0, 58.0, 52.0, 60.0], inclinacion=26.0, remolino=24.0,
              torsion=50.0,
              amarilla=dict(tipo="trenzada", n=3, z_punta=125.0, giro=220.0,
                            a_max=11.0, b_max=9.0, r_espiral=7.5,
                            factores=[1.0, 0.84, 0.70])),
    "C": dict(n=7, r_placa=40.0, lobulo=3.0, r_anillo=25.0, a0=9.5, b0=5.0,
              alturas=[44.0, 38.0, 41.0], inclinacion=38.0, remolino=-6.0, torsion=-15.0,
              amarilla=dict(tipo="trenzada", n=2, z_punta=96.0, giro=-160.0,
                            a_max=12.5, b_max=10.0, r_espiral=8.0,
                            factores=[1.0, 0.78])),
}

# la amarilla se apoya sobre la cara superior de la placa roja
DZ_ENSAMBLE = ESPESOR_PLACA


# ----------------------------------------------------------------------------
# Utilidades
# ----------------------------------------------------------------------------
def interp(ctrl, u):
    """Interpolacion suave y monotona (PCHIP, sin sobrepasos) entre puntos de
    control [(u, valor), ...]; evita ondulaciones en las superficies del loft."""
    us = np.array([c[0] for c in ctrl], dtype=float)
    vs = np.array([c[1] for c in ctrl], dtype=float)
    return float(PchipInterpolator(us, vs, extrapolate=True)(np.clip(u, us[0], us[-1])))


def elipse(cx, cy, z, a, b, ang):
    """Wire eliptico horizontal centrado en (cx, cy, z), eje a girado 'ang' rad."""
    pl = Plane(origin=(cx, cy, z), x_dir=(math.cos(ang), math.sin(ang), 0), z_dir=(0, 0, 1))
    return Wire([Edge.make_ellipse(a, b, pl)])


def lengua(z_ini, z_punta, centro, semi_a, semi_b, angulo, z_empotre=None):
    """Loft de elipses horizontales que termina en una punta casi afilada.
    centro(u)->(x,y); semi_a(u), semi_b(u); angulo(u) en rad; u en [0,1]."""
    secciones = []
    if z_empotre is not None:  # tramo vertical hundido en la pieza de abajo
        x, y = centro(0.0)
        secciones.append(elipse(x, y, z_empotre, semi_a(0.0), semi_b(0.0), angulo(0.0)))
    for u in np.linspace(0.0, 1.0, N_SECC)[:-1]:
        x, y = centro(u)
        z = z_ini + (z_punta - z_ini) * u
        secciones.append(elipse(x, y, z, semi_a(u), semi_b(u), angulo(u)))
    # punta: elipse pequena (PUNTA_A x PUNTA_B) con tapa plana; un vertice exacto
    # genera triangulos degenerados en el STL y una punta demasiado fragil
    x, y = centro(1.0)
    secciones.append(elipse(x, y, z_punta, PUNTA_A, PUNTA_B, angulo(1.0)))
    return loft_suave(secciones)


def loft_suave(secciones):
    """ThruSections sin 'CheckCompatibility' (las elipses ya vienen alineadas por
    su eje x; la verificacion de OCCT falla con torsiones grandes)."""
    lb = BRepOffsetAPI_ThruSections(True, False, 1e-6)
    lb.CheckCompatibility(False)
    for s in secciones:
        if isinstance(s, Vertex):
            lb.AddVertex(s.wrapped)
        else:
            lb.AddWire(s.wrapped)
    lb.Build()
    return Solid(TopoDS.Solid_s(lb.Shape()))


def desplazamiento(perfil_grados, altura, u):
    """Integra tan(inclinacion(u)) para obtener el corrimiento horizontal en u."""
    us = np.linspace(0.0, u, N_INTEGRACION)
    pend = [math.tan(math.radians(interp(perfil_grados, s))) for s in us]
    return float(np.trapezoid(pend, us)) * altura if u > 0 else 0.0


def unico(r):
    """Las operaciones booleanas pueden devolver ShapeList; se toma el solido."""
    if isinstance(r, (list, tuple)) or type(r).__name__ == "ShapeList":
        r = r[0] if len(r) == 1 else r[0].fuse(*r[1:])
    return r


def fusionar(solidos):
    res = solidos[0].fuse(*solidos[1:])
    return unico(res).clean()


def perfil_d(r, plano, z):
    """Wire horizontal en forma de "D": arco de radio r cortado por la recta x = plano."""
    h = math.sqrt(r * r - plano * plano)
    p1, p2 = Vector(plano, -h, z), Vector(plano, h, z)
    recta = Edge.make_line(p1, p2)
    arco = Edge.make_three_point_arc(p2, Vector(-r, 0, z), p1)
    return Wire([recta, arco])


def prisma_d(secciones):
    """Prisma en "D" reglado entre secciones [(z, r, plano), ...] (permite chaflanes)."""
    return Solid.make_loft([perfil_d(r, p, z) for z, r, p in secciones], ruled=True)


# ----------------------------------------------------------------------------
# Pieza ROJA
# ----------------------------------------------------------------------------
def contorno_placa(v, z, recorte):
    """Contorno lobulado (un lobulo bajo cada lengua) reducido 'recorte' mm."""
    n = v["n"]
    pts = []
    for t in np.linspace(0, 2 * math.pi, PUNTOS_POR_LOBULO * n, endpoint=False):
        r = v["r_placa"] + v["lobulo"] * math.cos(n * t) - recorte
        pts.append((r * math.cos(t), r * math.sin(t), z))
    return Wire([Edge.make_spline(pts, periodic=True)])


def placa(v):
    t = ESPESOR_PLACA
    secc = [contorno_placa(v, 0.0, CHAFLAN_INF),
            contorno_placa(v, CHAFLAN_INF, 0.0),
            contorno_placa(v, t - CHAFLAN_SUP, 0.0),
            contorno_placa(v, t, CHAFLAN_SUP)]
    return Solid.make_loft(secc, ruled=True)


def perno_rojo():
    """Perno en "D" con chaflan de entrada en la punta."""
    z0 = ESPESOR_PLACA - RAIZ_PERNO
    z2 = ESPESOR_PLACA + ALTO_PERNO
    z1 = z2 - CHAFLAN_PERNO
    c = CHAFLAN_PERNO
    return prisma_d([(z0, R_PERNO, PLANO_D), (z1, R_PERNO, PLANO_D),
                     (z2, R_PERNO - c, PLANO_D - c)])


def lenguas_rojas(v):
    n = v["n"]
    sol = []
    for k in range(n):
        th = 2 * math.pi * k / n
        H = v["alturas"][k % len(v["alturas"])]
        er = np.array([math.cos(th), math.sin(th)])
        et = np.array([-math.sin(th), math.cos(th)])
        L, S = v["inclinacion"], v["remolino"]
        p_rad = [(u, f * L) for u, f in PERFIL_INCL_RADIAL]
        p_tan = [(u, f * S) for u, f in PERFIL_INCL_TANG]
        tabla = {}
        for u in np.linspace(0, 1, N_SECC):
            tabla[round(float(u), 6)] = (desplazamiento(p_rad, H, u), desplazamiento(p_tan, H, u))

        def centro(u, er=er, et=et, tabla=tabla):
            dr, dt = tabla[round(float(u), 6)]
            p = er * (v["r_anillo"] + dr) + et * dt
            return float(p[0]), float(p[1])

        def forma(u):  # ensancha un poco abajo y afila hacia la punta
            return (1 - u) ** FORMA_EXP * (1 + FORMA_PANZA * math.sin(math.pi * u))

        a = lambda u, a0=v["a0"]: max(a0 * forma(u), SEMIEJE_MIN)
        b = lambda u, b0=v["b0"]: max(b0 * forma(u), SEMIEJE_MIN)
        ang = lambda u, th=th: th + math.pi / 2 + math.radians(v["torsion"]) * u
        sol.append(lengua(ESPESOR_PLACA, ESPESOR_PLACA + H, centro, a, b, ang,
                          z_empotre=ESPESOR_PLACA - EMPOTRADO))
    return sol


def pieza_roja(v, con_perno=True):
    partes = [placa(v)] + lenguas_rojas(v)
    if con_perno:
        partes.append(perno_rojo())
    return fusionar(partes)


# ----------------------------------------------------------------------------
# Pieza AMARILLA (coordenadas locales: z = 0 es la base del pedestal / la cama)
# ----------------------------------------------------------------------------
def pedestal(con_chaflan=True):
    """Brasa redonda: base plana con chaflan, tramo recto y cierre a 45 grados.
    En la monocolor el pedestal no lleva chaflan y nace 0.5 mm dentro de la placa."""
    z_cuello = ALTO_PEDESTAL_RECTO + (R_PEDESTAL - R_CUELLO)
    cierre = Solid.make_cone(R_PEDESTAL, R_CUELLO, z_cuello - ALTO_PEDESTAL_RECTO,
                             Plane.XY.offset(ALTO_PEDESTAL_RECTO))
    if con_chaflan:
        c = CHAFLAN_PEDESTAL
        base = Solid.make_cone(R_PEDESTAL - c, R_PEDESTAL, c)
        recto = Solid.make_cylinder(R_PEDESTAL, ALTO_PEDESTAL_RECTO - c, Plane.XY.offset(c))
        return fusionar([base, recto, cierre])
    recto = Solid.make_cylinder(R_PEDESTAL, ALTO_PEDESTAL_RECTO + 0.5, Plane.XY.offset(-0.5))
    return fusionar([recto, cierre])


def cuenca():
    """Hueco en "D" para el perno rojo: boca con chaflan a 45 grados y techo conico."""
    rh, ph = R_PERNO + HOLGURA, PLANO_D + HOLGURA
    c = CHAFLAN_BOCA
    z_techo = ALTO_PERNO + HOLGURA_TECHO
    d = prisma_d([(-1.0, rh + c + 1.0, ph + c + 1.0), (c, rh, ph), (z_techo, rh, ph)])
    alto_techo = (rh - R_TECHO_MIN) / math.tan(math.radians(ANG_TECHO))
    techo = Solid.make_cone(rh, R_TECHO_MIN, alto_techo, Plane.XY.offset(z_techo))
    return unico(d.fuse(techo))


def flama_simple(cfg, z_punta):
    tor = math.radians(cfg["torsion"])
    A, B = cfg["a_max"], cfg["b_max"]
    # perfil de gota: crece suave, maximo a ~1/3, afila a la punta
    pa = [(0, R_FLAMA_INI)] + [(u, f * A) for u, f in PERFIL_SIMPLE] + [(1, PUNTA_A)]
    pb = [(0, R_FLAMA_INI)] + [(u, f * B) for u, f in PERFIL_SIMPLE] + [(1, PUNTA_B)]

    def centro(u):  # ligera "S"
        return (S_AMP_X * math.sin(S_FREC_X * math.pi * u) * u,
                S_AMP_Y * math.sin(math.pi * u) * u)

    return [lengua(Z_FLAMA_INI, z_punta, centro, lambda u: interp(pa, u),
                   lambda u: interp(pb, u), lambda u: tor * u)]


def flama_trenzada(cfg, z_punta):
    n = cfg["n"]
    giro = math.radians(cfg["giro"])
    A, B = cfg["a_max"], cfg["b_max"]
    rc_max = cfg["r_espiral"]
    # cada lengua nace con su centro a DESFASE_TRENZA del eje; su semieje inicial se
    # reduce lo mismo para que el conjunto quede dentro del pedestal
    r_ini = R_FLAMA_INI - DESFASE_TRENZA
    pa = [(0, r_ini)] + [(u, f * A) for u, f in PERFIL_TRENZA] + [(1, PUNTA_A)]
    pb = [(0, r_ini)] + [(u, f * B) for u, f in PERFIL_TRENZA] + [(1, PUNTA_B)]
    sol = []
    for k, f in enumerate(cfg["factores"]):
        zt = Z_FLAMA_INI + (z_punta - Z_FLAMA_INI) * f
        f0 = 2 * math.pi * k / n

        def centro(u, f0=f0):
            rc = DESFASE_TRENZA + (rc_max - DESFASE_TRENZA) * math.sin(
                0.5 * math.pi * min(u / U_ESPIRAL_MAX, 1.0))
            phi = f0 + giro * u
            return rc * math.cos(phi), rc * math.sin(phi)

        sol.append(lengua(Z_FLAMA_INI, zt, centro, lambda u: interp(pa, u),
                          lambda u: interp(pb, u), lambda u, f0=f0: f0 + giro * u))
    return sol


def cuerpo_amarillo(v, con_chaflan=True):
    """Pedestal + flama, macizo (sin cuenca), en coordenadas locales."""
    cfg = v["amarilla"]
    z_punta = cfg["z_punta"] - DZ_ENSAMBLE   # altura local de la punta
    flama = (flama_simple(cfg, z_punta) if cfg["tipo"] == "simple"
             else flama_trenzada(cfg, z_punta))
    return fusionar([pedestal(con_chaflan)] + flama)


def pieza_amarilla(v):
    return unico(cuerpo_amarillo(v).cut(cuenca())).clean()


# ----------------------------------------------------------------------------
# Exportacion
# ----------------------------------------------------------------------------
def exportar(forma, nombre):
    stl = os.path.join(DIR_STL, nombre + ".stl")
    stp = os.path.join(DIR_STEP, nombre + ".step")
    export_stl(forma, stl, tolerance=0.02, angular_tolerance=0.08)
    export_step(forma, stp)
    print("exportado", nombre, "valido=", forma.is_valid, "vol=", round(forma.volume, 1))


def main(variantes=None):
    os.makedirs(DIR_STL, exist_ok=True)
    os.makedirs(DIR_STEP, exist_ok=True)
    for clave, v in VARIANTES.items():
        if variantes and clave not in variantes:
            continue
        roja = pieza_roja(v)
        amarilla = pieza_amarilla(v)   # ya en orientacion de impresion (base en z=0)
        mono = fusionar([pieza_roja(v, con_perno=False),
                         cuerpo_amarillo(v, con_chaflan=False).moved(
                             Plane.XY.offset(DZ_ENSAMBLE).location)])
        exportar(roja, f"fuego_{clave}_rojo")
        exportar(amarilla, f"fuego_{clave}_amarillo")
        exportar(mono, f"fuego_{clave}_monocolor")
    print(f"Ensamble: la amarilla se coloca en dz = {DZ_ENSAMBLE} mm sobre la roja")


if __name__ == "__main__":
    import sys
    main(sys.argv[1:] or None)
