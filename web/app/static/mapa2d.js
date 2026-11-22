// Mapa 2D cenital (canvas). x hacia la derecha, y hacia arriba (marco OptiTrack RSF).
const COL = { e: '#22d3ee', r: '#fb923c', none: '#8b97a6', detectada: '#facc15', asignada: '#60a5fa', visitada: '#4ade80' };

export class Mapa2D {
  constructor(canvas) {
    this.c = canvas;
    this.g = canvas.getContext('2d');
    this.half = 1.65;            // medio ancho visible (m)
    this.trails = { 1: [], 2: [] };
    this.draft = null;           // vértices en edición (pestaña Pista)
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

  get s() { return this.c.width / (2 * this.half); }
  px(x, y) { return [this.c.width / 2 + x * this.s, this.c.height / 2 - y * this.s]; }
  toWorld(ev) {
    const r = this.c.getBoundingClientRect();
    const k = this.c.width / r.width;
    const u = (ev.clientX - r.left) * k, v = (ev.clientY - r.top) * k;
    return [(u - this.c.width / 2) / this.s, -(v - this.c.height / 2) / this.s];
  }

  clearTrails() { this.trails = { 1: [], 2: [] }; }

  update(s) {
    this.state = s;
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
    // arena y retícula
    g.strokeStyle = '#222a34'; g.lineWidth = lw;
    for (let k = -1.5; k <= 1.5001; k += 0.5) {
      let [a, b] = this.px(k, -1.5), [c, d] = this.px(k, 1.5);
      g.beginPath(); g.moveTo(a, b); g.lineTo(c, d); g.stroke();
      [a, b] = this.px(-1.5, k); [c, d] = this.px(1.5, k);
      g.beginPath(); g.moveTo(a, b); g.lineTo(c, d); g.stroke();
    }
    g.strokeStyle = '#8b97a6'; g.lineWidth = 1.5 * lw;
    const [x0, y0] = this.px(-1.5, 1.5);
    g.strokeRect(x0, y0, 3 * s, 3 * s);
    g.fillStyle = '#8b97a6'; g.font = `${11 * lw}px Segoe UI, sans-serif`;
    g.fillText('x', ...this.px(1.52, 0.02)); g.fillText('y', ...this.px(0.02, 1.53));
    for (const k of [-1, 0, 1]) { const [u, v] = this.px(k, -1.5); g.fillText(k.toFixed(0), u - 3 * lw, v + 13 * lw); }
    // ejes en el origen
    this._arrow(0, 0, 0.3, 0, '#ef4444', lw); this._arrow(0, 0, 0, 0.3, '#22c55e', lw);
    if (!S) return;
    // pista
    const verts = this.draft || S.pista || [];
    if (verts.length) {
      g.strokeStyle = this.draft ? '#facc15' : '#f1f3f5'; g.lineWidth = Math.max(2, 0.0267 * s);
      g.setLineDash(this.draft ? [8 * lw, 6 * lw] : []);
      g.beginPath(); verts.forEach(([x, y], i) => { const [u, v] = this.px(x, y); i ? g.lineTo(u, v) : g.moveTo(u, v); }); g.stroke();
      g.setLineDash([]);
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
    // estelas
    for (const id of [1, 2]) {
      const tr = this.trails[id]; if (tr.length < 2) continue;
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
    if (tel && tel.explorer_id && tel.responder_id) {
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
      const d = tel.drones[id]; if (d.x === null) continue;
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
    // cursor
    if (this.hover) {
      g.fillStyle = '#cbd5e1'; g.font = `${11 * lw}px ui-monospace, Consolas, monospace`;
      g.fillText(`(${this.hover[0].toFixed(2)}, ${this.hover[1].toFixed(2)})`, 8 * lw, W - 8 * lw);
    }
  }

  _arrow(x, y, dx, dy, col, lw) {
    const g = this.g, [a, b] = this.px(x, y), [c, d] = this.px(x + dx, y + dy);
    g.strokeStyle = col; g.lineWidth = 2 * lw; g.beginPath(); g.moveTo(a, b); g.lineTo(c, d); g.stroke();
  }

  _drone(d, col, id, lw, role) {
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
    g.fillText(`D${id}${role === 'e' ? ' E' : role === 'r' ? ' R' : ''}  z ${d.z.toFixed(2)}`, u + 0.27 * s, v - 0.12 * s);
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
