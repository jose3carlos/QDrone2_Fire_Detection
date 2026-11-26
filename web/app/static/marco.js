// Marco de la arena: el piso y la retícula sobre los que se dibujan los tubos (vista 3D y mapa 2D).
// Misma regla que simulador3d/sim3d_marco.m:
//   caja que contiene los tubos REALES (s.tubos = pista.tubos), la ruta de vuelo (pista.vertices),
//   los homes de los drones (si se conocen) y el rectángulo mision.limites_xy; se le suma
//   MARCO_MARGEN por lado y se redondea HACIA AFUERA a múltiplos de MARCO_PASO (el origen
//   OptiTrack cae siempre en una línea de la retícula). Sin datos: el marco ±1.5 de siempre.
// Retícula: líneas finas cada 0.25 m, marcadas cada 0.5 m y más marcadas cada 1 m.
// limites_xy se dibuja aparte como "límite de vuelo" (punteado ámbar), distinto del borde del piso.
// La envolvente de las consignas (mision.limites_x / limites_y, s.envolvente de la estación = qd2_limites_xy)
// se dibuja en punteado violeta; no entra al marco (siempre queda dentro de limites_xy).
export const MARCO_MARGEN = 0.25;    // m
export const MARCO_PASO = 0.5;       // m
export const RETICULA_FINA = 0.25;   // m
const MARCO_DEF = { x0: -1.5, x1: 1.5, y0: -1.5, y1: 1.5 };
const COORD_MAX = 10;                // m; un punto más lejos es basura (telemetría sin inicializar)

const ok = (v) => typeof v === 'number' && Number.isFinite(v) && Math.abs(v) <= COORD_MAX;

// limites_xy de la misión ([lo, hi] para x e y) o null si no es válido
export function limitesXY(s) {
  const l = s && s.mision && s.mision.limites_xy;
  return Array.isArray(l) && l.length === 2 && ok(l[0]) && ok(l[1]) && l[0] < l[1] ? [l[0], l[1]] : null;
}

// envolvente EFECTIVA de las consignas xy ({ x0, x1, y0, y1 } en m) o null sin envolvente (sin las claves)
export function envolvente(s) {
  const e = s && s.envolvente;
  const par = (l) => Array.isArray(l) && l.length === 2 && ok(l[0]) && ok(l[1]) && l[0] < l[1];
  return e && par(e.x) && par(e.y) ? { x0: e.x[0], x1: e.x[1], y0: e.y[0], y1: e.y[1] } : null;
}

// límite EFECTIVO de las consignas xy ({ x0, x1, y0, y1 }, = qd2_limites_xy): la envolvente si existe; si no,
// mision.limites_xy -/+ XY_MARGEN (±1.35 con ±1.5). La ruta de vuelo debe quedar dentro (zona con rastreo).
export const XY_MARGEN = 0.15;       // m, = p.xy_margen
export function limiteConsignas(s) {
  const e = envolvente(s);
  if (e) return e;
  const l = limitesXY(s) || [-1.5, 1.5];
  return { x0: l[0] + XY_MARGEN, x1: l[1] - XY_MARGEN, y0: l[0] + XY_MARGEN, y1: l[1] - XY_MARGEN };
}

// punto [x, y] recortado a la envolvente (desde donde R marca un objetivo; = recortar_xy de qd2_mission_step)
export function recortarEnvolvente(env, x, y) {
  return [Math.min(Math.max(x, env.x0), env.x1), Math.min(Math.max(y, env.y0), env.y1)];
}

// homes conocidos de la telemetría (t.home_x/home_y), lista de [x, y]
export function homesTel(s) {
  const out = [];
  const tel = s && s.tel;
  if (tel && tel.drones) for (const id of [1, 2]) {
    const d = tel.drones[id];
    if (d && ok(d.home_x) && ok(d.home_y)) out.push([d.home_x, d.home_y]);
  }
  return out;
}

// { x0, x1, y0, y1 } en m, múltiplos de MARCO_PASO
export function marcoArena({ tubos = [], ruta = [], homes = [], limites = null } = {}) {
  const pts = [];
  for (const P of tubos || []) for (const p of P || []) pts.push(p);
  for (const p of ruta || []) pts.push(p);
  for (const p of homes || []) pts.push(p);
  if (limites) pts.push([limites[0], limites[0]], [limites[1], limites[1]]);
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  for (const p of pts) {
    if (!Array.isArray(p) || !ok(p[0]) || !ok(p[1])) continue;
    x0 = Math.min(x0, p[0]); x1 = Math.max(x1, p[0]); y0 = Math.min(y0, p[1]); y1 = Math.max(y1, p[1]);
  }
  if (!(x0 <= x1 && y0 <= y1)) return { ...MARCO_DEF };
  // 1e-9: que un borde que ya cae en un múltiplo (p. ej. -1.75 - 0.25) no salte un paso por redondeo
  const abajo = (v) => Math.floor((v - MARCO_MARGEN) / MARCO_PASO + 1e-9) * MARCO_PASO;
  const arriba = (v) => Math.ceil((v + MARCO_MARGEN) / MARCO_PASO - 1e-9) * MARCO_PASO;
  return { x0: abajo(x0), x1: arriba(x1), y0: abajo(y0), y1: arriba(y1) };
}

// el marco a partir del estado de la estación (s de /ws o /api/estado)
export const marcoDeEstado = (s) => marcoArena({
  tubos: (s && s.tubos) || [], ruta: (s && s.pista) || [], homes: homesTel(s), limites: limitesXY(s),
});

// valores k*paso dentro de [a, b] (enteros de paso, sin error acumulado)
export function multiplos(a, b, paso) {
  const out = [];
  for (let i = Math.ceil(a / paso - 1e-9); i * paso <= b + 1e-9; i++) out.push(+(i * paso).toFixed(6));
  return out;
}

// 0 = línea fina (0.25), 1 = marcada (0.5), 2 = más marcada (1 m)
export function nivelLinea(v) {
  const r = (q) => Math.abs(v / q - Math.round(v / q)) < 1e-6;
  return r(1) ? 2 : r(0.5) ? 1 : 0;
}
