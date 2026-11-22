// Vista 3D en tiempo real (three.js r160 desde CDN con versión fija).
// Marco OptiTrack RSF: +x adelante, +y izquierda, +z arriba. three.js usa +Y arriba, así que
// (x, y, z)_RSF -> (x, z, -y)_three (rotación propia, conserva la orientación).
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const COL = { e: 0x22d3ee, r: 0xfb923c, none: 0x8b97a6, det: 0xfacc15, asig: 0x60a5fa, vis: 0x4ade80 };
const STATE_COL = { detectada: COL.det, asignada: COL.asig, visitada: COL.vis };
const w2t = (x, y, z) => new THREE.Vector3(x, z, -y);

// Medidas del QDrone 2 (manual): 442.58 x 490.8 x 135.53 mm. Hélices de 7" en guardas.
const QD = { L: 0.4426, W: 0.4908, H: 0.1355, motorX: 0.126, motorY: 0.150, guardR: 0.093 };

function makeLabel(text, color) {
  const c = document.createElement('canvas');
  c.width = 256; c.height = 64;
  const g = c.getContext('2d');
  g.fillStyle = 'rgba(10,13,18,0.75)';
  g.beginPath(); g.roundRect(2, 8, 252, 48, 12); g.fill();
  g.fillStyle = color; g.font = 'bold 30px Segoe UI, sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle';
  g.fillText(text, 128, 33);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
  s.scale.set(0.36, 0.09, 1);
  s.renderOrder = 10;
  s.userData.text = text;
  return s;
}

function setLabel(sprite, text, color) {
  if (sprite.userData.text === text && sprite.userData.color === color) return;
  const c = sprite.material.map.image;
  const g = c.getContext('2d');
  g.clearRect(0, 0, c.width, c.height);
  g.fillStyle = 'rgba(10,13,18,0.75)';
  g.beginPath(); g.roundRect(2, 8, 252, 48, 12); g.fill();
  g.fillStyle = color; g.font = 'bold 30px Segoe UI, sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle';
  g.fillText(text, 128, 33);
  sprite.material.map.needsUpdate = true;
  sprite.userData.text = text; sprite.userData.color = color;
}

// Modelo procedural low-poly del QDrone 2 (grupo en coordenadas three; frente = +x)
function makeDrone() {
  const g = new THREE.Group();
  const dark = new THREE.MeshStandardMaterial({ color: 0x2b2f36, roughness: 0.7, metalness: 0.2, flatShading: true });
  const shell = new THREE.MeshStandardMaterial({ color: COL.none, roughness: 0.5, metalness: 0.1, flatShading: true });
  const guardMat = new THREE.MeshStandardMaterial({ color: 0xd9dde3, roughness: 0.6, flatShading: true });
  const propMat = new THREE.MeshStandardMaterial({ color: 0x111111, transparent: true, opacity: 0.55 });
  // cuerpo central y tapa superior de color de rol
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.21, 0.055, 0.13), dark);
  body.position.y = 0.065; g.add(body);
  const top = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.075, 0.025, 6), shell);
  top.position.y = 0.105; g.add(top);
  // batería debajo
  const bat = new THREE.Mesh(new THREE.BoxGeometry(0.15, 0.03, 0.06), dark);
  bat.position.y = 0.025; g.add(bat);
  // RealSense al frente
  const cam = new THREE.Mesh(new THREE.BoxGeometry(0.025, 0.025, 0.09), new THREE.MeshStandardMaterial({ color: 0x0b0b0b }));
  cam.position.set(0.12, 0.06, 0); g.add(cam);
  // brazos, motores, guardas y hélices
  for (const [sx, sy] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) {
    const mx = sx * QD.motorX, mz = -sy * QD.motorY;
    const len = Math.hypot(mx, mz);
    const arm = new THREE.Mesh(new THREE.BoxGeometry(len, 0.014, 0.022), dark);
    arm.position.set(mx / 2, 0.07, mz / 2);
    arm.rotation.y = -Math.atan2(mz, mx);
    g.add(arm);
    const motor = new THREE.Mesh(new THREE.CylinderGeometry(0.016, 0.016, 0.03, 8), dark);
    motor.position.set(mx, 0.085, mz); g.add(motor);
    const guard = new THREE.Mesh(new THREE.TorusGeometry(QD.guardR, 0.007, 4, 18), guardMat);
    guard.rotation.x = Math.PI / 2; guard.position.set(mx, 0.08, mz); g.add(guard);
    const guardLow = guard.clone(); guardLow.position.y = 0.045; g.add(guardLow);
    const prop = new THREE.Mesh(new THREE.CircleGeometry(QD.guardR * 0.92, 14), propMat);
    prop.rotation.x = -Math.PI / 2; prop.position.set(mx, 0.1, mz); g.add(prop);
    // patas
    const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.005, 0.005, 0.045, 4), dark);
    leg.position.set(mx * 0.55, 0.022, mz * 0.55); g.add(leg);
  }
  g.userData.shell = shell;
  return g;
}

function makeFire(color) {
  const g = new THREE.Group();
  const flame = new THREE.Mesh(new THREE.ConeGeometry(0.045, 0.13, 7),
    new THREE.MeshStandardMaterial({ color: 0xff4d1a, emissive: 0xff3300, emissiveIntensity: 0.7, flatShading: true }));
  flame.position.y = 0.065; g.add(flame);
  const core = new THREE.Mesh(new THREE.ConeGeometry(0.022, 0.07, 6),
    new THREE.MeshStandardMaterial({ color: 0xffd23f, emissive: 0xffaa00, emissiveIntensity: 0.8 }));
  core.position.y = 0.04; g.add(core);
  g.add(stateRing(color));
  return g;
}

function makeToma(color) {
  const g = new THREE.Group();
  const m = new THREE.MeshStandardMaterial({ color: 0x3b82f6, roughness: 0.4, metalness: 0.4, flatShading: true });
  const body = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, 0.07, 8), m);
  body.position.y = 0.035; g.add(body);
  const stem = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, 0.05, 6), m);
  stem.position.y = 0.09; g.add(stem);
  const wheel = new THREE.Mesh(new THREE.TorusGeometry(0.03, 0.006, 4, 12), new THREE.MeshStandardMaterial({ color: 0xef4444 }));
  wheel.rotation.x = Math.PI / 2; wheel.position.y = 0.115; g.add(wheel);
  g.add(stateRing(color));
  return g;
}

function stateRing(color) {
  const ring = new THREE.Mesh(new THREE.RingGeometry(0.075, 0.1, 28),
    new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide, transparent: true, opacity: 0.9 }));
  ring.rotation.x = -Math.PI / 2; ring.position.y = 0.004;
  ring.name = 'ring';
  return ring;
}

function circlePoints(r, n = 64) {
  const pts = [];
  for (let i = 0; i <= n; i++) { const a = (i / n) * Math.PI * 2; pts.push(new THREE.Vector3(Math.cos(a) * r, 0, Math.sin(a) * r)); }
  return pts;
}

export class Escena3D {
  constructor(host) {
    this.host = host;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    host.appendChild(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0a0d12);
    this.camera = new THREE.PerspectiveCamera(45, 1, 0.02, 60);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.5, 0);
    this.controls.enableDamping = true;
    this.view = 'persp';
    this.setView('persp');

    this.scene.add(new THREE.HemisphereLight(0xdde8ff, 0x202020, 1.1));
    const sun = new THREE.DirectionalLight(0xffffff, 1.4);
    sun.position.set(-2, 5, 3); this.scene.add(sun);

    this._arena();
    this.pipe = new THREE.Group(); this.scene.add(this.pipe);
    this.pipeKey = '';
    this.drones = {};
    for (const id of [1, 2]) this.drones[id] = this._droneSet(id);
    this.distLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
      new THREE.LineDashedMaterial({ color: 0xffffff, dashSize: 0.05, gapSize: 0.04, transparent: true, opacity: 0.6 }));
    this.scene.add(this.distLine);
    this.guardRing = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(circlePoints(1)),
      new THREE.LineDashedMaterial({ color: 0xffffff, dashSize: 0.06, gapSize: 0.05, transparent: true, opacity: 0.5 }));
    this.guardRing.computeLineDistances(); this.scene.add(this.guardRing);
    // huella de la cámara del explorador
    this.footGeom = new THREE.BufferGeometry();
    this.footGeom.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(12), 3));
    this.footGeom.setIndex([0, 1, 2, 0, 2, 3]);
    this.foot = new THREE.Mesh(this.footGeom, new THREE.MeshBasicMaterial({ color: COL.e, transparent: true, opacity: 0.13, side: THREE.DoubleSide, depthWrite: false }));
    this.footLine = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints([0, 1, 2, 3].map(() => new THREE.Vector3())),
      new THREE.LineBasicMaterial({ color: COL.e, transparent: true, opacity: 0.7 }));
    this.scene.add(this.foot, this.footLine);
    this.frustum = new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(new Array(8).fill(0).map(() => new THREE.Vector3())),
      new THREE.LineBasicMaterial({ color: COL.e, transparent: true, opacity: 0.35 }));
    this.scene.add(this.frustum);
    this.targets = new Map();
    this.truth = new THREE.Group(); this.scene.add(this.truth); this.truthKey = '';

    this._resize();
    new ResizeObserver(() => this._resize()).observe(host);
    this.renderer.setAnimationLoop(() => this._frame());
  }

  _arena() {
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(3, 3), new THREE.MeshStandardMaterial({ color: 0x1b1f26, roughness: 0.95 }));
    floor.rotation.x = -Math.PI / 2; this.scene.add(floor);
    const outer = new THREE.Mesh(new THREE.PlaneGeometry(9, 9), new THREE.MeshStandardMaterial({ color: 0x0f1318, roughness: 1 }));
    outer.rotation.x = -Math.PI / 2; outer.position.y = -0.002; this.scene.add(outer);
    const grid = new THREE.GridHelper(3, 6, 0x3a4350, 0x2a313b); grid.position.y = 0.001; this.scene.add(grid);
    const b = [[-1.5, -1.5], [1.5, -1.5], [1.5, 1.5], [-1.5, 1.5], [-1.5, -1.5]].map(([x, y]) => w2t(x, y, 0.003));
    this.scene.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(b), new THREE.LineBasicMaterial({ color: 0x8b97a6 })));
    // ejes RSF en el origen: x rojo, y verde, z azul
    const ax = (to, c) => this.scene.add(new THREE.ArrowHelper(to, w2t(0, 0, 0.005), 0.35, c, 0.07, 0.04));
    ax(w2t(1, 0, 0).normalize(), 0xef4444); ax(w2t(0, 1, 0).normalize(), 0x22c55e); ax(w2t(0, 0, 1).normalize(), 0x3b82f6);
  }

  _droneSet(id) {
    const model = makeDrone(); this.scene.add(model);
    const label = makeLabel(`D${id}`, '#e6edf3'); this.scene.add(label);
    const sp = new THREE.Mesh(new THREE.OctahedronGeometry(0.045), new THREE.MeshBasicMaterial({ color: COL.none, wireframe: true }));
    this.scene.add(sp);
    const spLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
      new THREE.LineDashedMaterial({ color: COL.none, dashSize: 0.03, gapSize: 0.03 }));
    this.scene.add(spLine);
    const N = 900;
    const trailGeom = new THREE.BufferGeometry();
    trailGeom.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(N * 3), 3));
    trailGeom.setDrawRange(0, 0);
    const trail = new THREE.Line(trailGeom, new THREE.LineBasicMaterial({ color: COL.none, transparent: true, opacity: 0.8 }));
    this.scene.add(trail);
    const shadow = new THREE.Mesh(new THREE.CircleGeometry(0.2, 24), new THREE.MeshBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.35, depthWrite: false }));
    shadow.rotation.x = -Math.PI / 2; this.scene.add(shadow);
    const home = new THREE.Mesh(new THREE.RingGeometry(0.11, 0.13, 4), new THREE.MeshBasicMaterial({ color: COL.none, side: THREE.DoubleSide }));
    home.rotation.x = -Math.PI / 2; home.rotation.z = Math.PI / 4; this.scene.add(home);
    return { model, label, sp, spLine, trail, trailPts: [], N, shadow, home };
  }

  setView(v) {
    this.view = v;
    if (v === 'persp') { this.camera.position.copy(w2t(-3.3, -2.6, 2.7)); this.controls.target.set(0, 0.4, 0); }
    if (v === 'top') { this.camera.position.set(0, 5.2, 0.001); this.controls.target.set(0, 0, 0); }
    this.controls.update();
  }

  clearTrails() { for (const id of [1, 2]) { this.drones[id].trailPts = []; this.drones[id].trail.geometry.setDrawRange(0, 0); } }

  _resize() {
    const w = this.host.clientWidth, h = this.host.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.renderer.domElement.style.width = w + 'px'; this.renderer.domElement.style.height = h + 'px';
    this.camera.aspect = w / h; this.camera.updateProjectionMatrix();
  }

  _frame() {
    if (this.view === 'follow' && this.followPos) {
      const t = this.followPos;
      this.controls.target.lerp(t, 0.1);
      const off = new THREE.Vector3(-1.6, 1.4, 1.2);
      this.camera.position.lerp(t.clone().add(off), 0.05);
    }
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  _setPipe(verts) {
    const key = JSON.stringify(verts);
    if (key === this.pipeKey) return;
    this.pipeKey = key;
    this.pipe.clear();
    const mat = new THREE.MeshStandardMaterial({ color: 0xf1f3f5, roughness: 0.35 });
    const r = 0.0267 / 2;
    for (let i = 0; i + 1 < verts.length; i++) {
      const a = w2t(verts[i][0], verts[i][1], r), b = w2t(verts[i + 1][0], verts[i + 1][1], r);
      const len = a.distanceTo(b);
      const cyl = new THREE.Mesh(new THREE.CylinderGeometry(r, r, len, 12), mat);
      cyl.position.copy(a.clone().add(b).multiplyScalar(0.5));
      cyl.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), b.clone().sub(a).normalize());
      this.pipe.add(cyl);
    }
    verts.forEach(([x, y]) => { const s = new THREE.Mesh(new THREE.SphereGeometry(r * 1.35, 12, 8), mat); s.position.copy(w2t(x, y, r)); this.pipe.add(s); });
  }

  _setTruth(list) {
    const key = JSON.stringify(list);
    if (key === this.truthKey) return;
    this.truthKey = key; this.truth.clear();
    for (const a of list) {
      const l = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(circlePoints(0.06, 24)),
        new THREE.LineBasicMaterial({ color: 0x9ca3af, transparent: true, opacity: 0.6 }));
      l.position.copy(w2t(a.x, a.y, 0.006)); this.truth.add(l);
    }
  }

  update(s) {
    this._setPipe(s.pista || []);
    this._setTruth(s.verdad_mock || []);
    const tel = s.tel;
    const roleOf = (id) => tel ? (tel.drones[id].role === 1 ? 'e' : tel.drones[id].role === 2 ? 'r' : 'none') : 'none';
    for (const id of [1, 2]) {
      const d = this.drones[id];
      const t = tel && tel.drones[id];
      const visible = !!(t && t.x !== null);
      for (const k of ['model', 'label', 'sp', 'spLine', 'shadow', 'home']) d[k].visible = visible;
      if (!visible) continue;
      const role = roleOf(id), col = COL[role];
      const p = w2t(t.x, t.y, t.z);
      d.model.position.copy(p);
      d.model.rotation.set(0, t.yaw || 0, 0);
      d.model.userData.shell.color.setHex(col);
      d.label.position.copy(p.clone().add(new THREE.Vector3(0, 0.26, 0)));
      const name = `D${id} · ${role === 'e' ? 'EXPLORADOR' : role === 'r' ? 'RESPONDEDOR' : '—'}`;
      setLabel(d.label, name, role === 'e' ? '#22d3ee' : role === 'r' ? '#fb923c' : '#e6edf3');
      d.label.scale.set(role === 'none' ? 0.2 : 0.5, 0.125, 1);
      d.shadow.position.copy(w2t(t.x, t.y, 0.003));
      d.home.position.copy(w2t(t.home_x ?? t.x, t.home_y ?? t.y, 0.004));
      d.home.material.color.setHex(col);
      if (t.xd !== null) {
        const q = w2t(t.xd, t.yd, t.zd);
        d.sp.position.copy(q); d.sp.material.color.setHex(col); d.sp.rotation.y += 0.03;
        d.spLine.geometry.setFromPoints([p, q]); d.spLine.computeLineDistances(); d.spLine.material.color.setHex(col);
      }
      // estela
      const last = d.trailPts[d.trailPts.length - 1];
      if (!last || last.distanceTo(p) > 0.015) {
        d.trailPts.push(p.clone());
        if (d.trailPts.length > d.N) d.trailPts.shift();
        const arr = d.trail.geometry.attributes.position.array;
        d.trailPts.forEach((v, i) => { arr[3 * i] = v.x; arr[3 * i + 1] = v.y; arr[3 * i + 2] = v.z; });
        d.trail.geometry.attributes.position.needsUpdate = true;
        d.trail.geometry.setDrawRange(0, d.trailPts.length);
        d.trail.geometry.computeBoundingSphere();
      }
      d.trail.material.color.setHex(col);
      if (role === 'e') this.followPos = p;
    }
    // distancia E-R y anillo de separación alrededor de E
    const ok = tel && tel.explorer_id && tel.responder_id && tel.drones[tel.explorer_id].x !== null && tel.drones[tel.responder_id].x !== null;
    this.distLine.visible = !!ok; this.guardRing.visible = !!ok && tel.version !== 2;
    if (ok) {
      const e = tel.drones[tel.explorer_id], r = tel.drones[tel.responder_id];
      this.distLine.geometry.setFromPoints([w2t(e.x, e.y, e.z), w2t(r.x, r.y, r.z)]);
      this.distLine.computeLineDistances();
      const guard = tel.guard_active === 1;
      this.distLine.material.color.setHex(guard ? 0xef4444 : 0xffffff);
      const dmin = (s.mision && s.mision.d_min_h) || 0.9;
      this.guardRing.scale.set(dmin, 1, dmin);
      this.guardRing.position.copy(w2t(e.x, e.y, 0.01));
      this.guardRing.material.color.setHex(guard ? 0xef4444 : 0x8b97a6);
    }
    // huella de cámara
    const f = s.huella || [];
    const hasF = f.length === 4;
    this.foot.visible = this.footLine.visible = this.frustum.visible = hasF;
    if (hasF && ok) {
      const pos = this.footGeom.attributes.position.array;
      f.forEach(([x, y], i) => { const v = w2t(x, y, 0.008); pos[3 * i] = v.x; pos[3 * i + 1] = v.y; pos[3 * i + 2] = v.z; });
      this.footGeom.attributes.position.needsUpdate = true; this.footGeom.computeBoundingSphere();
      this.footLine.geometry.setFromPoints(f.map(([x, y]) => w2t(x, y, 0.009)));
      const e = tel.drones[tel.explorer_id];
      const c = w2t(e.x, e.y, e.z - 0.03);
      const segs = []; f.forEach(([x, y]) => { segs.push(c, w2t(x, y, 0.009)); });
      this.frustum.geometry.setFromPoints(segs);
    }
    // anomalías
    const seen = new Set();
    for (const tg of s.objetivos || []) {
      seen.add(tg.id);
      let o = this.targets.get(tg.id);
      const col = STATE_COL[tg.estado] || COL.det;
      if (!o || o.userData.cls !== tg.cls) {
        if (o) this.scene.remove(o);
        o = tg.cls === 0 ? makeFire(col) : makeToma(col);
        o.userData.cls = tg.cls;
        o.userData.label = makeLabel(`#${tg.id} ${tg.clase}`, '#e6edf3'); o.add(o.userData.label);
        o.userData.label.position.y = 0.25; o.userData.label.scale.set(0.34, 0.085, 1);
        this.scene.add(o); this.targets.set(tg.id, o);
      }
      o.position.copy(w2t(tg.x, tg.y, 0));
      o.getObjectByName('ring').material.color.setHex(col);
      setLabel(o.userData.label, `#${tg.id} ${tg.clase}`, '#' + col.toString(16).padStart(6, '0'));
    }
    for (const [id, o] of this.targets) if (!seen.has(id)) { this.scene.remove(o); this.targets.delete(id); }
  }
}
