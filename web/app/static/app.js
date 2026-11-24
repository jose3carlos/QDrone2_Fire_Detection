// Estación QDrone 2 — lógica de la interfaz (sin paso de compilación).
import { Mapa2D } from './mapa2d.js';

const $ = (id) => document.getElementById(id);
// El texto del estado del dron (o+2) lo arma el servidor (protocol.fsm_info) según la versión y(3):
// PID = Flight State de tierra del Equipo2 (0 Idle, 1 Armed, 2 Takeoff, 3 Land, 9 MS Stop);
// MPC = FSM a bordo (1..9; 7, 8 y 9 son fallas).
let manualSimMode = false;   // clic en el mapa = objetivo "de Simulink" en el mock
const f2 = (v, d = 2) => (v === null || v === undefined || Number.isNaN(v)) ? '—' : Number(v).toFixed(d);
let S = null;            // último estado recibido
let cfg = null;          // config/estacion.json
let scene3d = null;
let manualMode = false;
let draftVerts = null;   // vértices en edición

const map = new Mapa2D($('map'));

// ------------------------------------------------------------------ utilidades
function toast(msg, err = false) {
  const t = $('toast'); t.textContent = msg; t.className = 'toast show' + (err ? ' err' : '');
  clearTimeout(toast._t); toast._t = setTimeout(() => { t.className = 'toast'; }, 2600);
}
async function api(path, body, method) {
  const opt = body instanceof FormData ? { method: 'POST', body }
    : body !== undefined ? { method: method || 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) } : {};
  const r = await fetch(path, opt);
  let data = null; try { data = await r.json(); } catch { /* sin cuerpo */ }
  if (!r.ok) throw new Error((data && (data.detail || data.error)) || `HTTP ${r.status}`);
  return data;
}
async function copy(text) {
  try { await navigator.clipboard.writeText(text); }
  catch { const ta = document.createElement('textarea'); ta.value = text; document.body.appendChild(ta); ta.select(); document.execCommand('copy'); ta.remove(); }
  toast('Copiado: ' + (text.length > 60 ? text.slice(0, 57) + '…' : text));
}
function chip(id, cls, text, title) { const c = $(id); c.className = 'chip ' + cls; if (text) c.textContent = text; if (title) c.title = title; }
function esc(s) { return String(s ?? '').replace(/[&<>"]/g, (m) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[m])); }

// ------------------------------------------------------------------ modo de misión (dual / solo explorador)
// s.modo (servidor): efectivo = el del modelo de tierra (y(64)) si la telemetría es fresca, si no el de la config.
const esSolo = (s) => !!(s && s.modo && s.modo.efectivo === 'solo_explorador' && [1, 2].includes(s.modo.dron_solo));
const modoTxt = (m) => (!m ? '—' : m.modo === 'solo_explorador' ? `solo explorador (dron ${m.dron_solo})` : 'dual');
// Textos de confirmación: los de siempre (dual) y los del modo solo (dinámicos con el dron)
const CONFIRM_DUAL = {};
document.querySelectorAll('[data-cmd][data-confirm]').forEach((b) => { CONFIRM_DUAL[b.dataset.cmd] = b.dataset.confirm; });
const CONFIRM_SOLO = {
  START: (n) => `¿Iniciar la misión SOLO EXPLORADOR? El dron ${n} se arma y despega a z_E. El otro dron no se usa.`,
  ABORT: (n) => `¿Abortar? El dron ${n} regresa a su home y queda en hover.`,
  LAND_ALL: (n) => `¿Aterrizar el dron ${n} donde está?`,
};
let capDronTocado = false;

// ------------------------------------------------------------------ 3D (si el CDN no responde, la UI sigue con el mapa 2D)
import('./escena3d.js').then(({ Escena3D }) => {
  scene3d = new Escena3D($('three-host'));
  $('three-msg').remove();
  if (S) scene3d.update(S);
}).catch((e) => {
  $('three-msg').textContent = 'Vista 3D no disponible (¿sin internet para cargar three.js?). El mapa 2D sigue funcionando. ' + e.message;
});
document.querySelectorAll('[data-view]').forEach((b) => b.addEventListener('click', () => {
  document.querySelectorAll('[data-view]').forEach((x) => x.classList.toggle('on', x === b));
  if (scene3d) scene3d.setView(b.dataset.view);
}));

// ------------------------------------------------------------------ pestañas
document.querySelectorAll('.tab').forEach((t) => t.addEventListener('click', () => {
  document.querySelectorAll('.tab').forEach((x) => x.classList.toggle('on', x === t));
  document.querySelectorAll('.tabpane').forEach((p) => p.classList.toggle('on', p.id === 'tab-' + t.dataset.tab));
  if (t.dataset.tab === 'conexiones') loadNet();
  if (t.dataset.tab === 'pista') { if (!draftVerts || !draftVerts.length) draftVerts = (cfg?.pista?.vertices || S?.pista || []).map((v) => [...v]); renderVerts(); }
  else { map.draft = null; }
  if (t.dataset.tab === 'pista') map.draft = draftVerts;
  map.draw();
}));

// ------------------------------------------------------------------ WebSocket
let wsDelay = 500;
function connect() {
  const ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
  ws.onopen = () => { wsDelay = 500; chip('chip-ws', 'ok', 'Web'); };
  ws.onmessage = (ev) => { S = JSON.parse(ev.data); render(S); };
  ws.onclose = () => { chip('chip-ws', 'bad', 'Web', 'Sin conexión con la estación; reintentando…'); setTimeout(connect, wsDelay); wsDelay = Math.min(wsDelay * 2, 5000); };
}
connect();

// ------------------------------------------------------------------ render
let lastCmdResult = 0, firstSnap = true;
function render(s) {
  const tel = s.tel;
  // fuente
  const sel = $('sel-fuente'); if (document.activeElement !== sel) sel.value = s.fuente;
  // chips
  const b = s.puente || {};
  chip('chip-bridge', b.fresh ? 'ok' : b.connected ? 'warn' : 'bad', `Puente ${b.rx_hz ?? 0} Hz`,
    `${b.uri}\n${b.connected ? 'conectado' : 'desconectado'}${b.error ? '\n' + b.error : ''}`);
  const c = s.camara || {};
  chip('chip-cam', c.conectada ? 'ok' : (c.fuente && c.fuente[0] === 'ninguna') ? '' : 'bad', `Video ${c.fps ?? 0} fps`,
    `fuente: ${(c.fuente || []).join(' ')}${c.error ? '\n' + c.error : ''}`);
  const v = s.vision || {};
  chip('chip-vision', v.detector && v.detector !== 'ninguno' ? (v.activo ? 'ok' : 'warn') : 'bad', 'Visión',
    `${v.detector}${v.detector_error ? '\n' + v.detector_error : ''}${v.activo ? '\n(detectando)' : '\n(en espera: solo en MISSION)'}`);
  $('chip-version').textContent = tel ? `Modelo ${tel.version_name}` : `Config ${s.version_config}`;
  renderModo(s);
  // estado de misión
  const ms = $('mstate');
  ms.textContent = tel ? `${tel.mission_state_es} · ${tel.mission_state_name}` : 'Sin telemetría';
  ms.className = 'mstate' + (tel ? ' s' + tel.mission_state : '');
  $('cmdinfo').textContent = `cmd ${b.cmd ?? '—'} · cmd_seq ${b.cmd_seq ?? 0} · ack ${b.cmd_seq_ack ?? '—'} · hb ${b.heartbeat ?? 0}`
    + ((b.cola || []).length ? ` · en cola: ${b.cola.join(', ')}` : '');
  // resultado de cada comando (recibido / caducado / descartado / reemplazado)
  for (const r of b.cmd_resultados || []) {
    if (r.n <= lastCmdResult) continue;
    lastCmdResult = r.n;
    if (firstSnap) continue;
    const ok = r.estado === 'recibido';
    toast(`${r.cmd} ${ok ? 'recibido por Simulink' : r.estado.toUpperCase()}${ok ? '' : ': ' + r.detalle}`, !ok);
  }
  firstSnap = false;
  // avisos de seguridad (rojo) y de configuración (amarillo)
  $('avisos').innerHTML = (s.avisos || []).map((a) => `<div class="aviso ${a.nivel}">${esc(a.msg)}</div>`).join('');
  renderAlerta(s);
  $('card-mock').hidden = s.fuente !== 'mock';
  renderMockPruebas(s);
  const solo = esSolo(s);
  $('m-t').textContent = tel ? f2(tel.t, 1) + ' s' : '—';
  $('m-dh').textContent = solo ? 'no aplica' : tel ? f2(tel.dist_horizontal) + ' m' : '—';
  $('m-dz').textContent = solo ? 'no aplica' : tel ? f2(tel.dist_vertical) + ' m' : '—';
  $('metric-dh').title = $('metric-dz').title = solo ? 'Modo solo explorador: un solo dron, no hay distancia E–R' : '';
  const g = $('guard');
  if (tel && solo) { g.textContent = 'GUARD no aplica (un solo dron)'; g.className = 'guard idle'; }
  else if (tel && tel.version === 2) { g.textContent = 'MPC · apilable'; g.className = 'guard idle'; }
  else if (tel) { g.textContent = tel.guard_active ? 'GUARD ACTIVO' : 'GUARD libre'; g.className = 'guard ' + (tel.guard_active ? 'active' : 'idle'); }
  // misión
  if (tel) {
    const e = tel.explorer_id, r = tel.responder_id;
    $('k-e').textContent = e ? `Dron ${e}` : '—'; $('k-r').textContent = solo ? '— (modo solo)' : r ? `Dron ${r}` : '—';
    $('k-known').textContent = tel.n_targets_known; $('k-vis').textContent = tel.n_targets_visited;
    $('k-queue').textContent = tel.queue_length; $('k-rt').textContent = tel.responder_target_id ? '#' + tel.responder_target_id : 'libre';
    const pct = tel.explorer_wp_total ? Math.min(100, 100 * tel.explorer_wp_idx / tel.explorer_wp_total) : 0;
    $('k-wp').textContent = `${tel.explorer_wp_idx}/${tel.explorer_wp_total}${tel.explorer_done ? ' ✓' : ''}`;
    $('bar-wp').style.width = (tel.explorer_done ? 100 : pct) + '%';
    $('k-mark').textContent = Math.round(100 * (tel.marking_progress || 0)) + ' %';
    $('bar-mark').style.width = 100 * (tel.marking_progress || 0) + '%';
    const m = s.mision || {}, mpc = tel.version === 2;
    const dh = tel.dist_horizontal, dz = Math.abs(tel.dist_vertical ?? 0);
    const flying = (tel.drones[1].z ?? 0) > 0.2 && (tel.drones[2].z ?? 0) > 0.2;
    $('s-dh').textContent = f2(dh) + ' m'; $('s-dz').textContent = f2(tel.dist_vertical) + ' m';
    $('s-dh').className = !mpc && flying ? (dh >= m.d_min_h ? 'ok' : 'bad') : '';
    $('s-dz').className = mpc && flying ? (dz >= m.dz_min ? 'ok' : 'bad') : '';
    $('s-dh-th').textContent = mpc ? '(MPC: se permite apilar)' : `mín ${m.d_min_h} m (PID)`;
    $('s-dz-th').textContent = mpc ? `mín ${m.dz_min} m si están apilados` : '';
    $('s-note').textContent = tel.guard_active ? 'El SEPARATION GUARD está reteniendo a un dron.' : '';
    $('sep-block').classList.toggle('active', !!tel.guard_active);
    if (solo) {
      for (const id of ['s-dh', 's-dz']) { $(id).textContent = 'no aplica'; $(id).className = ''; }
      $('s-dh-th').textContent = ''; $('s-dz-th').textContent = '';
      $('s-note').textContent = 'No aplica (un solo dron): sin respondedor no hay separation guard.';
      $('sep-block').classList.remove('active');
    }
  }
  // modo solo: sin respondedor (no hay "visitados" ni marcado)
  $('kv-vis').hidden = $('kv-rt').hidden = $('mark-block').hidden = solo;
  $('k-queue-label').textContent = solo ? 'Detectados (sin respondedor)' : 'En cola';
  renderBotones(s);
  renderDrones(tel, s.actitud_explorador);
  renderTargets(s);
  renderInfo(s);
  // eventos
  $('events').innerHTML = (s.eventos || []).map((e) => `<li><time>${esc(e.t)}</time>${esc(e.msg)}</li>`).join('');
  $('cam-info').textContent = `${(c.fuente || []).join(' ')} · ${c.fps ?? 0} fps · ${v.detector || ''}`;
  map.update(s);
  if (scene3d) scene3d.update(s);
}

// y(55): alerta de Simulink (0 = nada; -1 = valor no finito). "aviso" = amarillo, "grave" = rojo.
function renderAlerta(s) {
  const tel = s.tel, box = $('alerta');
  if (!tel || !tel.alerta) { box.hidden = true; return; }
  const vieja = !(s.puente || {}).fresh;
  box.hidden = false;
  box.className = 'alerta ' + (tel.alerta_nivel === 'aviso' ? 'aviso' : 'grave');
  // -1 = y(55) no finito (NaN/inf): telemetría corrupta, se muestra como grave
  box.innerHTML = `<span class="codigo">ALERTA ${tel.alerta === -1 ? 'NaN' : tel.alerta}</span><span>${esc(tel.alerta_texto)}</span>`
    + (vieja ? '<span class="vieja">(última telemetría recibida; el puente no está fresco)</span>' : '');
}

// Tarjeta "Pruebas con el mock": el selector refleja la alerta que el mock tiene forzada de verdad
// (salvo mientras el operador lo está cambiando).
let mockSelTocado = false;
function renderMockPruebas(s) {
  const sel = $('mock-alerta'), info = $('mock-alerta-actual');
  const f = s.mock ? s.mock.alerta_forzada : null;
  if (!mockSelTocado && document.activeElement !== sel) sel.value = f === null || f === undefined ? '' : String(f);
  info.textContent = s.mock ? (f === null || f === undefined ? 'Ahora: y(55) real del mock' : `Ahora: y(55) forzada a ${f}`) : '';
}

// Distancia horizontal a home. En LANDING cada dron baja EN SU SITIO (qd2_mission_step), así que en
// RETURN_HOME conviene dar LAND ALL cuando los dos marquen "en home" (< mision.tol_xy).
function aHome(d) {
  if (d.x === null || d.y === null || d.home_x === null || d.home_y === null) return '';
  const dist = Math.hypot(d.x - d.home_x, d.y - d.home_y), tol = cfg?.mision?.tol_xy ?? 0.12;
  return dist <= tol ? ' · <span class="flag ok">en home</span>' : ` · a ${f2(dist)} m`;
}

function renderDrones(tel, act) {
  const box = $('drones');
  if (!tel) { box.innerHTML = '<div class="dcard">Sin telemetría del puente.</div>'; return; }
  const solo = esSolo(S);
  box.innerHTML = [1, 2].map((id) => {
    const d = tel.drones[id];
    if (solo && id !== S.modo.dron_solo) return dronNoUsado(id, d);
    const role = d.role === 1 ? '<span class="role e">EXPLORADOR</span>' : d.role === 2 ? '<span class="role r">RESPONDEDOR</span>' : '<span class="role">sin rol</span>';
    const rb = cfg?.optitrack?.[`rigid_body_dron${id}`];
    return `<div class="dcard"><div class="dh"><span class="name">Dron ${id}</span>${role}</div>
      <div class="flag ${d.tracked ? 'ok' : 'bad'}">${d.tracked ? '● rastreado' : '○ sin tracking'} (RB ${rb ?? '?'}) · <span class="flag ${d.comm_ok ? 'ok' : 'bad'}">${d.comm_ok ? 'comm ok' : 'sin comm'}</span></div>
      <div class="flag ${d.fsm_falla ? 'bad' : ''}" title="${tel.version === 2 ? 'FSM a bordo que reporta el dron (§3.2)' : 'Flight State del Flight Plan de tierra (el dron PID no reporta su FSM)'}">${esc(d.fsm_tipo || 'estado')}: ${d.fsm_state} · ${esc(d.fsm_texto || '')}</div>
      <table>
        <tr><td>pos</td><td>${f2(d.x)}, ${f2(d.y)}, ${f2(d.z)} · ψ ${f2((d.yaw ?? 0) * 57.2958, 0)}°</td></tr>
        <tr><td>consigna</td><td>${f2(d.xd)}, ${f2(d.yd)}, ${f2(d.zd)}</td></tr>
        <tr><td>vel</td><td>${f2(d.vx)}, ${f2(d.vy)}, ${f2(d.vz)}</td></tr>
        <tr><td>home</td><td>${f2(d.home_x)}, ${f2(d.home_y)}${aHome(d)}</td></tr>
        <tr><td>batería</td><td>${d.battery_V === null ? '—' : f2(d.battery_V, 1) + ' V'}</td></tr>
        ${id === tel.explorer_id ? `<tr><td title="y(61:62) CRUDOS: roll y pitch del explorador tal como llegan de OptiTrack (prueba del libro)">actitud (cruda)</td><td>${actitudTxt(tel)}</td></tr>${actitudUsadaTxt(act)}` : ''}
      </table></div>`;
  }).join('');
}

// Modo solo explorador: el dron que no se usa (gris, NO USADO). Muestra rastreo y pose (útil para el aviso de
// "sácalo de la arena"), sin "sin comm" en rojo y sin home.
function dronNoUsado(id, d) {
  const rb = cfg?.optitrack?.[`rigid_body_dron${id}`];
  return `<div class="dcard no-usado"><div class="dh"><span class="name">Dron ${id}</span><span class="role nu">NO USADO</span></div>
      <div class="flag">${d.tracked ? '● rastreado (sácalo de la arena)' : '○ sin tracking'} (RB ${rb ?? '?'}) · ${d.comm_ok ? 'conectado (no recibe arm/takeoff)' : 'sin comm'}</div>
      <table>
        <tr><td>pos</td><td>${d.tracked ? `${f2(d.x)}, ${f2(d.y)}, ${f2(d.z)}` : '—'}</td></tr>
        <tr><td>modo</td><td>solo explorador: no recibe arm ni takeoff</td></tr>
      </table></div>`;
}

// y(61:63): roll (φ) y pitch (θ) del explorador TAL COMO LLEGAN; inclinación total = acos(cos φ · cos θ)
function actitudTxt(tel) {
  if (!tel.explorer_att_ok) return '<span class="flag bad">no disponible (y(63) = 0)</span>';
  const r = tel.explorer_roll, p = tel.explorer_pitch;
  const incl = Math.acos(Math.min(1, Math.cos(r) * Math.cos(p))) * 57.2958;
  return `φ ${f2(r * 57.2958, 1)}° · θ ${f2(p * 57.2958, 1)}° · incl. ${f2(incl, 1)}°`;
}

// La actitud que entra a la geolocalización (snapshot.actitud_explorador): solo se muestra si difiere de la
// cruda, es decir con vision.actitud_signos / actitud_offset_deg ajustados o con usar_actitud = false.
function actitudUsadaTxt(act) {
  if (!act || !act.cruda) return '';
  const tip = 'title="Actitud con la que se geolocaliza: signo · (cruda − offset) de vision.actitud_signos / actitud_offset_deg"';
  if (!act.usar) return `<tr><td ${tip}>actitud usada</td><td><span class="flag bad">ignorada (usar_actitud = false): nivelado</span></td></tr>`;
  if (!act.ajustada || !act.usada) return '';
  return `<tr><td ${tip}>actitud usada</td><td>φ ${f2(act.usada[0], 1)}° · θ ${f2(act.usada[1], 1)}° (con signos/offset)</td></tr>`;
}

function renderTargets(s) {
  const list = s.objetivos || [];
  $('tab-count').textContent = list.length;
  $('targets-empty').style.display = list.length ? 'none' : 'block';
  const ORIGEN = { estacion: 'estación', 'estacion+simulink': 'ambos', simulink: 'Simulink' };
  $('tbl-targets').querySelector('tbody').innerHTML = list.map((t) => `<tr>
    <td>${t.id}${t.manual ? '*' : ''}</td><td>${t.clase === 'fuego' ? '🔥 fuego' : '🔧 toma'}</td><td>${f2(t.x)}</td><td>${f2(t.y)}</td>
    <td>${f2(t.conf)}</td><td>${t.n ?? '—'}</td><td><span class="pill ${t.estado}">${t.estado}</span></td>
    <td><span class="origen ${t.origen === 'simulink' ? 'simulink' : ''}"${t.alias_de ? ` title="En Simulink es el objetivo #${t.alias_de}: guardó el id #${t.id} de la estación como alias (mismo lugar)"` : ''}>${ORIGEN[t.origen] || esc(t.origen)}${t.alias_de ? ` (= #${t.alias_de})` : ''}</span></td></tr>`).join('');
  const v = s.vision || {};
  const last = v.ultimo || {};
  const desc = v.descartadas || {};
  $('vision-info').innerHTML = `
    <span>Detector</span><b>${esc(v.detector)}</b>
    ${v.detector_error ? `<span>Errores</span><b>${esc(v.detector_error)}</b>` : ''}
    <span>Activo</span><b>${v.activo ? 'sí (misión en curso)' : 'no (solo en MISSION)'}</b>
    <span>Frames</span><b>${v.frames ?? 0}</b>
    <span>Observaciones</span><b>${v.observaciones ?? 0}</b>
    <span>Candidatos</span><b>${v.candidatos ?? 0} (requieren ${cfg?.vision?.min_observaciones ?? 3} obs.)</b>
    <span title="Filtros físicos de la estación (vision.tam_max_m, vision.dist_max_tubo_toma_m, vision.inclinacion_max_deg = ${v.filtros?.inclinacion_max_deg ?? 'desactivado'}°…): no entran al cluster">Descartadas</span><b>${desc.total ?? 0} (tamaño ${desc['tamaño'] ?? 0}, tubo ${desc['lejos del tubo'] ?? 0}, inclinación ${desc['inclinación'] ?? 0})</b>
    <span title="Roll/pitch del explorador (y(61:63)) en la geolocalización (vision.usar_actitud)">Actitud</span><b${v.actitud && v.actitud.usar && !v.actitud.ok ? ' style="color: var(--warn)"' : ''}>${esc(v.actitud?.texto ?? '—')}${v.actitud?.ok ? ` · φ ${f2(v.actitud.roll_deg, 1)}° θ ${f2(v.actitud.pitch_deg, 1)}°` : ''}</b>
    <span>Último frame</span><b>${(last.dets || []).length} cajas · ${last.ms ?? 0} ms</b>
    ${v.error ? `<span>Error</span><b>${esc(v.error)}</b>` : ''}`;
}

function renderInfo(s) {
  const b = s.puente || {}, c = s.camara || {};
  $('bridge-info').innerHTML = `
    <span>Puente</span><b>${esc(b.uri)} · ${b.connected ? 'conectado' : 'desconectado'}</b>
    <span>Rx / Tx</span><b>${b.rx_hz} Hz / ${b.tx_hz} Hz</b>
    <span>Edad telemetría</span><b>${b.rx_age_s ?? '—'} s</b>
    <span>Reconexiones</span><b>${b.reconnects}</b>
    ${b.error ? `<span>Error</span><b>${esc(b.error)}</b>` : ''}
    <span>Video</span><b>${esc((c.fuente || []).join(' '))} · ${c.conectada ? 'conectado' : 'sin video'}</b>
    ${c.error ? `<span>Error video</span><b>${esc(c.error)}</b>` : ''}`;
}

// ------------------------------------------------------------------ controles
document.querySelectorAll('[data-cmd]').forEach((btn) => btn.addEventListener('click', async () => {
  if (btn.dataset.confirm && !confirm(btn.dataset.confirm)) return;
  try { const r = await api('/api/cmd', { cmd: btn.dataset.cmd }); toast(r.cmd_seq ? `${btn.dataset.cmd} enviado (cmd_seq ${r.cmd_seq}); esperando ack…` : `${btn.dataset.cmd} en cola`); }
  catch (e) { toast(e.message, true); }
}));
$('sel-fuente').addEventListener('change', async (ev) => {
  try { await api('/api/fuente', { fuente: ev.target.value }); map.clearTrails(); scene3d?.clearTrails(); toast('Fuente: ' + ev.target.value); }
  catch (e) { toast(e.message, true); }
});
$('sel-version').addEventListener('change', async (ev) => {
  try { await api('/api/config/version', { version_control: ev.target.value }); toast('version_control = ' + ev.target.value); await loadConfig(); }
  catch (e) { toast(e.message, true); }
});

// anomalías manuales
$('btn-man').addEventListener('click', () => {
  manualMode = !manualMode; manualSimMode = false; $('btn-mock-obj').classList.remove('on');
  $('btn-man').classList.toggle('on', manualMode);
  $('btn-man').textContent = manualMode ? 'Haz clic en el mapa…' : 'Clic en el mapa 2D…';
  $('map-hint').textContent = manualMode ? 'clic = agregar anomalía' : 'x → derecha · y ↑';
});
$('btn-reset-targets').addEventListener('click', async () => {
  if (!confirm('¿Borrar la lista de anomalías de la estación? Solo se permite si Simulink todavía no tiene ninguna (si no, los ids chocarían con su tabla).')) return;
  try { await api('/api/objetivos/reiniciar', {}); toast('Lista reiniciada'); } catch (e) { toast(e.message, true); }
});
$('mock-alerta').addEventListener('change', () => { mockSelTocado = true; });
$('btn-mock-alerta').addEventListener('click', async () => {
  const v = $('mock-alerta').value;
  try { await api('/api/mock/alerta', { codigo: v === '' ? null : Number(v) }); toast(v === '' ? 'Mock: alerta real' : `Mock: y(55) = ${v}`); }
  catch (e) { toast(e.message, true); }
  finally { mockSelTocado = false; }
});
$('btn-mock-obj').addEventListener('click', () => {
  manualSimMode = !manualSimMode; manualMode = false;
  $('btn-mock-obj').classList.toggle('on', manualSimMode);
  $('map-hint').textContent = manualSimMode ? 'clic = objetivo de Simulink (mock)' : 'x → derecha · y ↑';
});
map.onClick = async ([x, y]) => {
  if (manualSimMode) {
    try { const r = await api('/api/mock/objetivo_interno', { cls: Number($('man-cls').value), x, y }); toast(`Objetivo #${r.id} agregado a la tabla del mock (llega por y(56:60))`); }
    catch (e) { toast(e.message, true); }
    manualSimMode = false; $('btn-mock-obj').classList.remove('on'); $('map-hint').textContent = 'x → derecha · y ↑';
    return;
  }
  if (manualMode) {
    try { const r = await api('/api/objetivos/manual', { cls: Number($('man-cls').value), x, y }); toast(`Anomalía #${r.id} publicada en (${x.toFixed(2)}, ${y.toFixed(2)})`); }
    catch (e) { toast(e.message, true); }
    manualMode = false; $('btn-man').classList.remove('on'); $('btn-man').textContent = 'Clic en el mapa 2D…'; $('map-hint').textContent = 'x → derecha · y ↑';
  }
};

// ------------------------------------------------------------------ configuración
async function loadConfig(full = false) {
  cfg = await api('/api/config');
  $('sel-version').value = cfg.version_control || 'PID';
  for (const i of [1, 2]) {
    const ip = cfg.red[`ip_dron${i}`] || '';
    if (full || !$('ip-' + i).dataset.dirty) $('ip-' + i).value = ip;
    $('pend-' + i).textContent = ip ? 'guardada' : 'pendiente'; $('pend-' + i).className = 'pend' + (ip ? ' ok' : '');
  }
  const ot = cfg.optitrack || {};
  const capSel = $('cap-dron').value;
  $('cap-dron').innerHTML = [1, 2].map((i) => `<option value="${i}">Dron ${i} (RB ${ot['rigid_body_dron' + i]})</option>`).join('');
  if (capSel) $('cap-dron').value = capSel;
  if ($('tab-pista').classList.contains('on') && (!draftVerts || !draftVerts.length)) { draftVerts = cfg.pista.vertices.map((v) => [...v]); renderVerts(); }
  if (!full) return;
  $('ot-folder').value = $('ot-folder').value || ot.carpeta || '';
  $('ot-cal-path').value = ot.archivo_cal || ''; $('ot-motive-path').value = ot.archivo_motive || '';
  $('ot-rb1').value = ot.rigid_body_dron1 ?? 1; $('ot-rb2').value = ot.rigid_body_dron2 ?? 2;
}
for (const i of [1, 2]) $('ip-' + i).addEventListener('input', (ev) => { ev.target.dataset.dirty = '1'; });

// --- red
async function loadNet() {
  try {
    const r = await api('/api/red');
    const sel = $('sel-ippc');
    const chosen = cfg?.red?.ip_pc || '';
    $('tbl-net').querySelector('tbody').innerHTML = r.interfaces.map((i) => `<tr class="${i.recomendada ? 'rec' : ''}">
      <td>${esc(i.nombre)}${i.alfa ? ' <b>(Alfa)</b>' : ''}<br><small>${esc(i.descripcion)}${i.activa ? '' : ' · inactiva'}</small></td>
      <td>${esc(i.ip)}</td><td><button class="btn sm ghost" data-copy="${esc(i.ip)}">Copiar</button></td></tr>`).join('')
      || '<tr><td colspan="3">No se encontraron interfaces IPv4.</td></tr>';
    sel.innerHTML = `<option value="">automática (${esc(r.ip_recomendada || 'sin detectar')})</option>` +
      r.interfaces.map((i) => `<option value="${esc(i.ip)}">${esc(i.ip)} — ${esc(i.nombre)}</option>`).join('');
    sel.value = chosen;
    await loadCmds();
  } catch (e) { toast(e.message, true); }
}
async function loadCmds() {
  const r = await api('/api/comandos');
  const rows = (d, k, label) => `<div class="cmd"><div class="cl">${label}</div><div class="cbox"><pre class="${d.pendiente ? 'pending' : ''}">${esc(d[k])}</pre><button class="btn sm ghost" data-copy="${esc(d[k])}">Copiar</button></div></div>`;
  $('cmds').innerHTML = [1, 2].map((i) => {
    const d = r.drones[i];
    if (d.no_usado) return `<div class="dron-cmds"><b>Dron ${i}</b> <span class="pend">no usado (modo solo explorador)</span></div>`;
    return `<div class="dron-cmds"><b>Dron ${i}</b> ${d.pendiente ? '<span class="pend">faltan IPs</span>' : ''}
      ${rows(d, 'uri_host', 'Argumento -URI_Host')}
      ${rows(d, 'quarc_run', 'Lanzar el modelo ' + esc(r.modelo) + ' en el dron')}
      ${rows(d, 'quarc_stop', 'Detener')}
      ${rows(d, 'ext_mode_args', 'Alternativa desde MATLAB (External mode)')}
      ${rows(d, 'ssh_camara', 'Cámara (solo el explorador; en el Jetson)')}</div>`;
  }).join('');
}
$('btn-net').addEventListener('click', loadNet);
$('sel-ippc').addEventListener('change', async (ev) => {
  try { await api('/api/config/red', { ip_pc: ev.target.value }); await loadConfig(); await loadCmds(); toast('IP de la PC: ' + (ev.target.value || 'automática')); }
  catch (e) { toast(e.message, true); }
});
document.querySelectorAll('[data-save-ip]').forEach((b) => b.addEventListener('click', async () => {
  const i = b.dataset.saveIp;
  try { await api('/api/config/red', { ['ip_dron' + i]: $('ip-' + i).value.trim() }); delete $('ip-' + i).dataset.dirty; await loadConfig(); await loadCmds(); toast(`IP del dron ${i} guardada`); }
  catch (e) { toast(e.message, true); }
}));
document.addEventListener('click', (ev) => {
  const t = ev.target.closest('[data-copy]'); if (t) copy(t.dataset.copy);
  const f = ev.target.closest('[data-copy-from]'); if (f) { const v = $(f.dataset.copyFrom).value.trim(); if (v) copy(v); else toast('IP pendiente', true); }
});

// --- Motive
function fileList(box, files, name, current) {
  box.innerHTML = files.length ? files.map((f) => `<label><input type="radio" name="${name}" value="${esc(f.ruta)}" ${f.ruta === current ? 'checked' : ''}>
    <span>${esc(f.nombre)}</span><span class="meta">${esc(f.modificado)} · ${f.kb} kB</span></label>`).join('')
    : '<div class="none">Sin archivos.</div>';
}
$('btn-ot-list').addEventListener('click', async () => {
  try {
    const r = await api('/api/optitrack/archivos?carpeta=' + encodeURIComponent($('ot-folder').value.trim()));
    $('ot-msg').textContent = r.ok ? `${r.cal.length} .cal y ${r.motive.length} .motive en ${r.carpeta}` : `${r.error}: ${r.carpeta}`;
    fileList($('ot-cal'), r.cal || [], 'cal', $('ot-cal-path').value);
    fileList($('ot-motive'), r.motive || [], 'motive', $('ot-motive-path').value);
  } catch (e) { toast(e.message, true); }
});
document.addEventListener('change', (ev) => {
  if (ev.target.name === 'cal') $('ot-cal-path').value = ev.target.value;
  if (ev.target.name === 'motive') $('ot-motive-path').value = ev.target.value;
});
$('ot-upload').addEventListener('change', async (ev) => {
  const f = ev.target.files[0]; if (!f) return;
  const fd = new FormData(); fd.append('archivo', f);
  try { const r = await api('/api/optitrack/subir', fd); $(r.tipo === 'cal' ? 'ot-cal-path' : 'ot-motive-path').value = r.ruta; toast('Archivo subido: ' + r.ruta); }
  catch (e) { toast(e.message, true); }
});
$('btn-ot-save').addEventListener('click', async () => {
  try {
    await api('/api/config/optitrack', {
      carpeta: $('ot-folder').value.trim(), archivo_cal: $('ot-cal-path').value.trim(), archivo_motive: $('ot-motive-path').value.trim(),
      rigid_body_dron1: Number($('ot-rb1').value), rigid_body_dron2: Number($('ot-rb2').value),
    });
    await loadConfig(true); toast('Calibración de Motive guardada en estacion.json');
  } catch (e) { toast(e.message, true); }
});

// --- Pista
function renderVerts() {
  const v = draftVerts || [];
  $('tbl-verts').querySelector('tbody').innerHTML = v.map(([x, y], i) => `<tr>
    <td>${i + 1}</td><td><input data-vi="${i}" data-vk="0" value="${x}"></td><td><input data-vi="${i}" data-vk="1" value="${y}"></td>
    <td><button class="btn sm ghost" data-vup="${i}" title="Subir">↑</button><button class="btn sm ghost" data-vdel="${i}" title="Borrar">✕</button></td></tr>`).join('');
  const sel = $('cap-target'), keep = sel.value;
  sel.innerHTML = '<option value="-1">agregar al final</option>' + v.map((_, i) => `<option value="${i}">reemplazar vértice ${i + 1}</option>`).join('');
  sel.value = [...sel.options].some((o) => o.value === keep) ? keep : '-1';
  map.draft = draftVerts; map.draw();
}
$('tbl-verts').addEventListener('input', (ev) => {
  const i = ev.target.dataset.vi; if (i === undefined) return;
  const val = parseFloat(ev.target.value); if (!Number.isNaN(val)) { draftVerts[i][ev.target.dataset.vk] = val; map.draw(); }
});
$('tbl-verts').addEventListener('click', (ev) => {
  const d = ev.target.dataset;
  if (d.vdel !== undefined) { draftVerts.splice(Number(d.vdel), 1); renderVerts(); }
  if (d.vup !== undefined && Number(d.vup) > 0) { const k = Number(d.vup); [draftVerts[k - 1], draftVerts[k]] = [draftVerts[k], draftVerts[k - 1]]; renderVerts(); }
});
$('btn-v-add').addEventListener('click', () => { if (draftVerts.length >= 16) { toast('Máximo 16 vértices (límite de Simulink)', true); return; } const l = draftVerts[draftVerts.length - 1] || [0, 0]; draftVerts.push([l[0], l[1]]); renderVerts(); });
$('btn-v-reload').addEventListener('click', async () => { await loadConfig(true); draftVerts = cfg.pista.vertices.map((v) => [...v]); renderVerts(); });
$('btn-v-save').addEventListener('click', async () => {
  try { const r = await api('/api/config/pista', { vertices: draftVerts }); draftVerts = r.vertices.map((v) => [...v]); renderVerts(); await loadConfig(); toast('Pista guardada en estacion.json'); }
  catch (e) { toast(e.message, true); }
});
$('cap-dron').addEventListener('change', () => { capDronTocado = true; });
$('btn-cap').addEventListener('click', async () => {
  try {
    const r = await api('/api/pista/capturar', { dron: Number($('cap-dron').value) });
    const k = Number($('cap-target').value);
    const p = [Number(r.x.toFixed(3)), Number(r.y.toFixed(3))];
    if (k >= 0) draftVerts[k] = p; else draftVerts.push(p);
    renderVerts();
    $('cap-msg').textContent = `Capturado del dron ${r.dron} (RB ${r.rigid_body}): (${p[0]}, ${p[1]}), z = ${r.z.toFixed(3)} m. Recuerda “Guardar pista”.`;
  } catch (e) { toast(e.message, true); }
});

// ------------------------------------------------------------------ modo: chip, línea informativa y botones
function renderModo(s) {
  const md = s.modo || {}, solo = esSolo(s), cm = $('chip-modo');
  let lvl = '';
  if (md.coincide === false) lvl = md.config && md.config.modo === 'dual' ? 'warn' : 'bad';
  cm.textContent = solo ? `SOLO EXPLORADOR · dron ${md.dron_solo}` : 'Dual';
  cm.className = 'chip ' + (solo ? 'solo ' : (lvl ? '' : 'neutral ')) + lvl;
  cm.title = `Modelo de tierra (y(64)): ${md.tel ? modoTxt(md.tel) : 'sin telemetría'}`
    + `\nestacion.json de esta estación: ${modoTxt(md.config)}${md.coincide === false ? '\nNO COINCIDEN: START bloqueado' : ''}`;
  const cfgM = md.config || {};
  $('modo-info').innerHTML = `Modo (estacion.json de esta estación): <b>${esc(modoTxt(cfgM))}</b>`
    + (md.tel ? ` · modelo de tierra: <b>${esc(modoTxt(md.tel))}</b>${md.coincide === false ? ' <b style="color: var(--bad)">(no coinciden)</b>' : ''}` : '')
    + (cfgM.aviso ? ` · <span style="color: var(--warn)">${esc(cfgM.aviso)}</span>` : '')
    + '<br><small>El modelo de tierra usa el estacion.json de la PC al compilar (Build): cambiar el modo obliga a copiar'
    + ' el archivo y recompilar. Solo lectura.</small>'
    + (md.remoto ? `<br><small>Puente remoto: ${esc(md.host_puente)}</small>` : '');
  $('pista-remoto').hidden = $('motive-remoto').hidden = !md.remoto;
  // captura de vértices: por defecto el dron solo
  if (solo && !capDronTocado && $('cap-dron').value !== String(md.dron_solo) && document.activeElement !== $('cap-dron')) {
    $('cap-dron').value = String(md.dron_solo);
  }
}

function renderBotones(s) {
  const md = s.modo || {}, solo = esSolo(s), tel = s.tel, n = md.dron_solo;
  document.querySelectorAll('[data-cmd][data-confirm]').forEach((b) => {
    const c = b.dataset.cmd;
    b.dataset.confirm = solo && CONFIRM_SOLO[c] ? CONFIRM_SOLO[c](n) : CONFIRM_DUAL[c];
  });
  const st = $$start();
  let motivo = '';
  if (!tel) motivo = 'Sin telemetría del puente';
  else if (md.coincide === false) motivo = 'El modo de la config local no coincide con el del modelo de tierra (copia estacion.json a la PC y recompila)';
  else if (solo && !(tel.drones[n] || {}).tracked) motivo = `El dron ${n} (solo explorador) no está rastreado por OptiTrack`;
  else if (tel.mission_state !== 2) motivo = 'START solo se acepta en ASSIGN_ROLES (roles asignados)';
  st.disabled = !!motivo;
  st.title = motivo || (solo ? `Inicia la misión solo explorador con el dron ${n}` : 'Inicia la misión');
}
function $$start() { return document.querySelector('[data-cmd="START"]'); }

loadConfig(true).catch((e) => toast(e.message, true));
setInterval(() => loadConfig().catch(() => {}), 10000);

// abrir una pestaña desde la URL (p. ej. http://127.0.0.1:8080/#conexiones)
function openTabFromHash() {
  const t = document.querySelector(`.tab[data-tab="${location.hash.slice(1)}"]`);
  if (t) t.click();
}
window.addEventListener('hashchange', openTabFromHash);
openTabFromHash();
