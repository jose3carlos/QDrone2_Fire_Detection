// Mapa 2D cenital (canvas). x hacia la derecha, y hacia arriba (marco OptiTrack RSF).
import { marcoDeEstado, limitesXY, envolvente, multiplos, nivelLinea, RETICULA_FINA } from './marco.js';

const COL = { e: '#22d3ee', r: '#fb923c', none: '#8b97a6', detectada: '#facc15', asignada: '#60a5fa', visitada: '#4ade80' };
const RETICULA = ['#161c24', '#222a34', '#2e3744'];   // líneas de 0.25 / 0.5 / 1 m (nivelLinea)
const LIMITE = 'rgba(210,153,34,.85)';                 // límite de vuelo (mision.limites_xy), punteado ámbar
const ENVOLVENTE = 'rgba(167,139,250,.9)';             // envolvente de consignas (mision.limites_x/_y), punteado violeta

export class Mapa2D {
  constructor(canvas) {
    this.c = canvas;
    this.g = canvas.getContext('2d');
    this.marco = { x0: -1.5, x1: 1.5, y0: -1.5, y1: 1.5 };   // marco de la arena (marco.js)
    this.lim = null;             // mision.limites_xy
    this.env = null;             // envolvente de consignas { x0, x1, y0, y1 } (marco.js) o null
    this._encuadre();
    this.trails = { 1: [], 2: [] };
    this.draft = null;           // puntos en edición (pestaña Pista): ruta de vuelo o una pieza de tubo
    this.draftTipo = 'ruta';     // 'ruta' (pista.vertices) | 'tubo' (una pieza de pista.tubos)
    this.draftCtx = null;        // { ruta, tubos } que no se editan, dibujados tenues como referencia
    this.hover = null;
    this.onClick = null;
    this.state = null;
    canvas.addEventListener('mousemove', (ev) => { this.hover = this.toWorld(ev); this.draw(); });
    canvas.addEventListener('mouseleave', () => { this.hover = null; this.draw(); });
    canvas.addEventListener('click', (ev) => { if (this.onClick) this.onClick(this.toWorld(ev)); });
    new ResizeObserver(() => this._resize()).observe(canvas);
    this._resize();
  }

  _resize() {
    const r = this.c.getBoundingClientRect();
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const size = Math.max(200, Math.round(r.width * dpr));
    if (this.c.width !== size) { this.c.width = size; this.c.height = size; this.draw(); }
  }

  // canvas cuadrado centrado en el marco: half = medio lado mayor + ~12 % (o 0.15 m) para las etiquetas de metros
  _encuadre() {
    const m = this.marco, L = Math.max(m.x1 - m.x0, m.y1 - m.y0) / 2;
    this.cx = (m.x0 + m.x1) / 2; this.cy = (m.y0 + m.y1) / 2;
    this.half = Math.max(L * 1.12, L + 0.15);    // medio ancho visible (m)
  }

  get s() { return this.c.width / (2 * this.half); }
  px(x, y) { return [this.c.width / 2 + (x - this.cx) * this.s, this.c.height / 2 - (y - this.cy) * this.s]; }
  toWorld(ev) {
    const r = this.c.getBoundingClientRect();
    const k = this.c.width / r.width;
    const u = (ev.clientX - r.left) * k, v = (ev.clientY - r.top) * k;
    return [this.cx + (u - this.c.width / 2) / this.s, this.cy - (v - this.c.height / 2) / this.s];
  }

  clearTrails() { this.trails = { 1: [], 2: [] }; }

  update(s) {
    this.state = s;
    this.marco = marcoDeEstado(s); this.lim = limitesXY(s); this.env = envolvente(s); this._encuadre();
    const tel = s.tel;
    if (tel) for (const id of [1, 2]) {
      const d = tel.drones[id];
      if (d.x === null) continue;
      const tr = this.trails[id], last = tr[tr.length - 1];
      if (!last || Math.hypot(last[0] - d.x, last[1] - d.y) > 0.015) { tr.push([d.x, d.y]); if (tr.length > 900) tr.shift(); }
    }
    this.draw();
  }

  draw() {
    const g = this.g, W = this.c.width, s = this.s, S = this.state;
    const lw = W / 560;
    g.clearRect(0, 0, W, W);
    g.fillStyle = '#0a0d12'; g.fillRect(0, 0, W, W);
    this._arena(lw);
    // ejes en el origen
    this._arrow(0, 0, 0.3, 0, '#ef4444', lw); this._arrow(0, 0, 0, 0.3, '#22c55e', lw);
    if (!S) return;
    // pista
    const verts = this.draft || S.pista || [];
    const linea = (P) => { g.beginPath(); P.forEach(([x, y], i) => { const [u, v] = this.px(x, y); i ? g.lineTo(u, v) : g.moveTo(u, v); }); g.stroke(); };
    if (this.draft) {
      // contexto tenue: lo que no se está editando (tubos reales en blanco, ruta de vuelo punteada celeste)
      const ctx = this.draftCtx || {};
      g.globalAlpha = 0.35; g.strokeStyle = '#f1f3f5'; g.lineWidth = Math.max(2, 0.0267 * s); g.setLineDash([]);
      for (const P of ctx.tubos || []) if (P.length >= 2) linea(P);
      if (ctx.ruta && ctx.ruta.length >= 2) {
        g.strokeStyle = '#38bdf8'; g.lineWidth = 1.5 * lw; g.setLineDash([5 * lw, 5 * lw]); linea(ctx.ruta);
      }
      g.globalAlpha = 1; g.setLineDash([]);
    }
    if (verts.length) {
      if (this.draft) {
        // en edición, amarillo: la ruta punteada (como la ruta de vuelo), una pieza de tubo continua (como un tubo)
        g.strokeStyle = '#facc15'; g.lineWidth = Math.max(2, 0.0267 * s);
        g.setLineDash(this.draftTipo === 'tubo' ? [] : [8 * lw, 6 * lw]);
        linea(verts);
      } else {
        // tubos REALES (S.tubos = pista.tubos o la ruta sin los tramos_sin_tubo): línea gruesa blanca
        g.strokeStyle = '#f1f3f5'; g.lineWidth = Math.max(2, 0.0267 * s); g.setLineDash([]);
        for (const P of S.tubos || []) linea(P);
        // ruta de VUELO (pista.vertices, la que compila Simulink): línea fina punteada celeste
        g.strokeStyle = '#38bdf8'; g.globalAlpha = 0.7; g.lineWidth = 1.5 * lw; g.setLineDash([5 * lw, 5 * lw]);
        linea(verts);
      }
      g.setLineDash([]); g.globalAlpha = 1;
      g.font = `bold ${11 * lw}px Segoe UI, sans-serif`;
      verts.forEach(([x, y], i) => {
        const [u, v] = this.px(x, y);
        g.fillStyle = '#0a0d12'; g.beginPath(); g.arc(u, v, 8 * lw, 0, 7); g.fill();
        g.strokeStyle = '#f1f3f5'; g.lineWidth = lw; g.stroke();
        g.fillStyle = '#f1f3f5'; g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillText(String(i + 1), u, v + 0.5);
      });
      g.textAlign = 'start'; g.textBaseline = 'alphabetic';
    }
    // verdad del mock
    for (const a of S.verdad_mock || []) {
      const [u, v] = this.px(a.x, a.y);
      g.strokeStyle = 'rgba(156,163,175,.6)'; g.setLineDash([3 * lw, 3 * lw]); g.lineWidth = lw;
      g.beginPath(); g.arc(u, v, 0.06 * s, 0, 7); g.stroke(); g.setLineDash([]);
    }
    const tel = S.tel;
    // huella de la cámara
    if (S.huella && S.huella.length === 4) {
      g.fillStyle = 'rgba(34,211,238,.10)'; g.strokeStyle = 'rgba(34,211,238,.7)'; g.lineWidth = lw;
      g.beginPath(); S.huella.forEach(([x, y], i) => { const [u, v] = this.px(x, y); i ? g.lineTo(u, v) : g.moveTo(u, v); });
      g.closePath(); g.fill(); g.stroke();
    }
    const role = (id) => tel ? (tel.drones[id].role === 1 ? 'e' : tel.drones[id].role === 2 ? 'r' : 'none') : 'none';
    // modo solo explorador (y(64)): sin círculo d_min_h ni línea E–R; el dron no usado en gris
    const ds = S.modo && S.modo.efectivo === 'solo_explorador' && [1, 2].includes(S.modo.dron_solo) ? S.modo.dron_solo : 0;
    const noUsado = (id) => ds && id !== ds;
    // estelas
    for (const id of [1, 2]) {
      const tr = this.trails[id]; if (tr.length < 2 || noUsado(id)) continue;
      g.strokeStyle = COL[role(id)]; g.globalAlpha = 0.55; g.lineWidth = 1.5 * lw;
      g.beginPath(); tr.forEach(([x, y], i) => { const [u, v] = this.px(x, y); i ? g.lineTo(u, v) : g.moveTo(u, v); }); g.stroke();
      g.globalAlpha = 1;
    }
    // candidatos (aún sin confirmar)
    for (const c of S.candidatos || []) {
      const [u, v] = this.px(c.x, c.y);
      g.strokeStyle = 'rgba(250,204,21,.55)'; g.lineWidth = lw; g.beginPath(); g.arc(u, v, 4 * lw, 0, 7); g.stroke();
    }
    // anomalías
    for (const t of S.objetivos || []) this._target(t, lw);
    // separación y distancia
    if (!ds && tel && tel.explorer_id && tel.responder_id) {
      const e = tel.drones[tel.explorer_id], r = tel.drones[tel.responder_id];
      if (e.x !== null && r.x !== null) {
        const guard = tel.guard_active === 1;
        if (tel.version !== 2) {
          const [u, v] = this.px(e.x, e.y);
          g.strokeStyle = guard ? '#ef4444' : 'rgba(139,151,166,.7)'; g.setLineDash([6 * lw, 5 * lw]); g.lineWidth = 1.2 * lw;
          g.beginPath(); g.arc(u, v, ((S.mision && S.mision.d_min_h) || 0.9) * s, 0, 7); g.stroke(); g.setLineDash([]);
        }
        const [a, b] = this.px(e.x, e.y), [c, d] = this.px(r.x, r.y);
        g.strokeStyle = guard ? '#ef4444' : 'rgba(255,255,255,.45)'; g.lineWidth = lw; g.setLineDash([4 * lw, 4 * lw]);
        g.beginPath(); g.moveTo(a, b); g.lineTo(c, d); g.stroke(); g.setLineDash([]);
        g.fillStyle = guard ? '#ef4444' : '#cbd5e1'; g.font = `${11 * lw}px ui-monospace, Consolas, monospace`;
        g.fillText(`${Math.hypot(e.x - r.x, e.y - r.y).toFixed(2)} m`, (a + c) / 2 + 6 * lw, (b + d) / 2 - 6 * lw);
      }
    }
    // drones, homes y consignas
    if (tel) for (const id of [1, 2]) {
      const d = tel.drones[id]; if (d.x === null || d.y === null) continue;
      if (noUsado(id)) {
        if (!d.tracked) continue;
        g.globalAlpha = 0.45; this._drone(d, COL.none, id, lw, 'none', ' NO USADO'); g.globalAlpha = 1;
        continue;
      }
      const col = COL[role(id)];
      if (d.home_x !== null) {
        const [hu, hv] = this.px(d.home_x, d.home_y);
        g.strokeStyle = col; g.lineWidth = lw; g.strokeRect(hu - 6 * lw, hv - 6 * lw, 12 * lw, 12 * lw);
        g.fillStyle = col; g.font = `${10 * lw}px Segoe UI`; g.fillText('H', hu - 3 * lw, hv + 3.5 * lw);
      }
      if (d.xd !== null) {
        const [u, v] = this.px(d.xd, d.yd), k = 6 * lw;
        g.strokeStyle = col; g.lineWidth = 2 * lw;
        g.beginPath(); g.moveTo(u - k, v - k); g.lineTo(u + k, v + k); g.moveTo(u - k, v + k); g.lineTo(u + k, v - k); g.stroke();
      }
      this._drone(d, col, id, lw, role(id));
    }
    // cursor (arriba a la derecha: abajo y a la izquierda van las etiquetas de metros)
    if (this.hover) {
      g.fillStyle = '#cbd5e1'; g.font = `${11 * lw}px ui-monospace, Consolas, monospace`;
      g.textAlign = 'right';
      g.fillText(`(${this.hover[0].toFixed(2)}, ${this.hover[1].toFixed(2)})`, W - 8 * lw, 15 * lw);
      g.textAlign = 'start';
    }
  }

  // piso del marco, retícula (0.25 / 0.5 / 1 m) alineada a los múltiplos, borde, límite de vuelo y metros
  _arena(lw) {
    const g = this.g, m = this.marco;
    const [a0, b0] = this.px(m.x0, m.y1), [a1, b1] = this.px(m.x1, m.y0);
    g.fillStyle = '#0d1117'; g.fillRect(a0, b0, a1 - a0, b1 - b0);
    const xs = multiplos(m.x0, m.x1, RETICULA_FINA), ys = multiplos(m.y0, m.y1, RETICULA_FINA);
    const borde = (v, a, b) => Math.abs(v - a) < 1e-6 || Math.abs(v - b) < 1e-6;
    g.lineWidth = lw;
    for (const nivel of [0, 1, 2]) {           // de la más tenue a la más marcada (las marcadas quedan encima)
      g.strokeStyle = RETICULA[nivel]; g.beginPath();
      for (const x of xs) if (!borde(x, m.x0, m.x1) && nivelLinea(x) === nivel) {
        const [u] = this.px(x, 0); g.moveTo(u, b0); g.lineTo(u, b1);
      }
      for (const y of ys) if (!borde(y, m.y0, m.y1) && nivelLinea(y) === nivel) {
        const [, v] = this.px(0, y); g.moveTo(a0, v); g.lineTo(a1, v);
      }
      g.stroke();
    }
    g.strokeStyle = '#8b97a6'; g.lineWidth = 1.5 * lw;
    g.strokeRect(a0, b0, a1 - a0, b1 - b0);
    if (this.lim) {
      const [l0, l1] = this.lim, [u0, v0] = this.px(l0, l1), [u1, v1] = this.px(l1, l0);
      g.strokeStyle = LIMITE; g.lineWidth = 1.3 * lw; g.setLineDash([7 * lw, 5 * lw]);
      g.strokeRect(u0, v0, u1 - u0, v1 - v0); g.setLineDash([]);
    }
    if (this.env) {               // envolvente: guiones cortos violeta (distinta del límite de vuelo y del piso)
      const e = this.env, [u0, v0] = this.px(e.x0, e.y1), [u1, v1] = this.px(e.x1, e.y0);
      g.strokeStyle = ENVOLVENTE; g.lineWidth = 1.3 * lw; g.setLineDash([3 * lw, 3 * lw]);
      g.strokeRect(u0, v0, u1 - u0, v1 - v0); g.setLineDash([]);
      g.fillStyle = ENVOLVENTE; g.font = `${10 * lw}px Segoe UI, sans-serif`;
      g.textAlign = 'right'; g.textBaseline = 'bottom';
      g.fillText('envolvente', u1 - 3 * lw, v0 - 2 * lw);
      g.textAlign = 'start'; g.textBaseline = 'alphabetic';
    }
    // metros: x abajo, y a la izquierda (cada 0.5 m; cada 1 m si el marco pasa de 5 m)
    const paso = Math.max(m.x1 - m.x0, m.y1 - m.y0) > 5 ? 1 : 0.5;
    const txt = (v) => (Math.abs(v) < 1e-9 ? '0' : String(+v.toFixed(2)));
    g.fillStyle = '#8b97a6'; g.font = `${10 * lw}px Segoe UI, sans-serif`;
    g.textAlign = 'center'; g.textBaseline = 'top';
    for (const x of multiplos(m.x0, m.x1, paso)) g.fillText(txt(x), this.px(x, 0)[0], b1 + 5 * lw);
    g.textAlign = 'right'; g.textBaseline = 'middle';
    for (const y of multiplos(m.y0, m.y1, paso)) g.fillText(txt(y), a0 - 5 * lw, this.px(0, y)[1]);
    g.font = `bold ${11 * lw}px Segoe UI, sans-serif`;
    g.textAlign = 'right'; g.textBaseline = 'top'; g.fillText('x (m)', a1, b1 + 17 * lw);
    g.textAlign = 'center'; g.textBaseline = 'bottom'; g.fillText('y (m)', a0 - 14 * lw, b0 - 6 * lw);
    g.textAlign = 'start'; g.textBaseline = 'alphabetic';
  }

  _arrow(x, y, dx, dy, col, lw) {
    const g = this.g, [a, b] = this.px(x, y), [c, d] = this.px(x + dx, y + dy);
    g.strokeStyle = col; g.lineWidth = 2 * lw; g.beginPath(); g.moveTo(a, b); g.lineTo(c, d); g.stroke();
  }

  _drone(d, col, id, lw, role, extra = '') {
    const g = this.g, s = this.s, [u, v] = this.px(d.x, d.y);
    const yaw = d.yaw || 0;
    g.save(); g.translate(u, v); g.rotate(-yaw);
    g.fillStyle = 'rgba(0,0,0,.35)'; g.beginPath(); g.arc(0, 0, 0.25 * s, 0, 7); g.fill();
    g.strokeStyle = col; g.lineWidth = 1.6 * lw;
    for (const [mx, my] of [[0.126, 0.15], [0.126, -0.15], [-0.126, 0.15], [-0.126, -0.15]]) {
      g.beginPath(); g.moveTo(0, 0); g.lineTo(mx * s, -my * s); g.stroke();
      g.beginPath(); g.arc(mx * s, -my * s, 0.093 * s, 0, 7); g.stroke();
    }
    g.fillStyle = col; g.fillRect(-0.1 * s, -0.06 * s, 0.2 * s, 0.12 * s);
    g.beginPath(); g.moveTo(0.18 * s, 0); g.lineTo(0.1 * s, -0.05 * s); g.lineTo(0.1 * s, 0.05 * s); g.fill();
    g.restore();
    g.fillStyle = '#e6edf3'; g.font = `bold ${11 * lw}px Segoe UI, sans-serif`;
    g.fillText(`D${id}${role === 'e' ? ' E' : role === 'r' ? ' R' : ''}${extra}  z ${d.z === null ? '—' : d.z.toFixed(2)}`, u + 0.27 * s, v - 0.12 * s);
  }

  _target(t, lw) {
    const g = this.g, [u, v] = this.px(t.x, t.y), col = COL[t.estado] || COL.detectada;
    // anillo punteado = objetivo que solo reporta Simulink (y(56:60)), no lo publicó esta estación
    g.strokeStyle = col; g.lineWidth = 2.5 * lw; g.setLineDash(t.origen === 'simulink' ? [4 * lw, 3 * lw] : []);
    g.beginPath(); g.arc(u, v, 12 * lw, 0, 7); g.stroke(); g.setLineDash([]);
    if (t.cls === 0) {         // fuego: llama
      g.fillStyle = '#ff4d1a';
      g.beginPath(); g.moveTo(u, v - 9 * lw); g.quadraticCurveTo(u + 8 * lw, v, u, v + 7 * lw); g.quadraticCurveTo(u - 8 * lw, v, u, v - 9 * lw); g.fill();
      g.fillStyle = '#ffd23f'; g.beginPath(); g.arc(u, v + 2 * lw, 3 * lw, 0, 7); g.fill();
    } else {                   // toma: válvula
      g.fillStyle = '#3b82f6'; g.fillRect(u - 6 * lw, v - 4 * lw, 12 * lw, 8 * lw);
      g.strokeStyle = '#ef4444'; g.lineWidth = 2 * lw; g.beginPath(); g.arc(u, v - 6 * lw, 4 * lw, 0, 7); g.stroke();
    }
    g.fillStyle = '#e6edf3'; g.font = `bold ${11 * lw}px Segoe UI, sans-serif`;
    g.fillText(`#${t.id}`, u + 14 * lw, v - 6 * lw);
    g.fillStyle = '#9aa5b4'; g.font = `${10 * lw}px ui-monospace, Consolas, monospace`;
    g.fillText(`${t.x.toFixed(2)}, ${t.y.toFixed(2)}`, u + 14 * lw, v + 7 * lw);
  }
}
