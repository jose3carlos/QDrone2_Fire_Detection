function res = probar_sim3d(con_capturas)
%PROBAR_SIM3D Pruebas sin ventana del simulador 3D (corre en -batch).
%   res = probar_sim3d()        corre todo y guarda PNG en capturas/
%   res = probar_sim3d(false)   sin capturas (más rápido)
%
%   1. Malla del QDrone 2 contra las medidas del manual (aros concéntricos con los motores).
%   2. Cámara y clusters contra la estación Python (pruebas/referencia_estacion.json,
%      generado con pruebas/generar_referencia_estacion.py desde web/app/geo.py).
%   3. Misión PID completa (más rápido que tiempo real): DONE, todos los objetos
%      visitados y distancia horizontal mínima en vuelo >= d_min_h.
%   4. Misión MPC completa: DONE y separación vertical al apilarse >= dz_min.
%   5. Escenario aleatorio (PID).
%   6. Ritmo de tiempo real sin figura (10 s).
%   7. Costo de dibujar un cuadro (figura invisible) -> fps alcanzables.
%   8. Escenario 'web' = objetos de la cámara sintética de la UI (mock_world), con la N y
%      con la pista real (anomalías sobre pista.tubos, mock_world.anomalias_mock).
%   9. Heartbeat de Python medido en reloj (x16 no pausa; si Python se calla, sí).
%  10. Primer paquete de Python con un comando: no se ejecuta ni se confirma.
%  11. El detector solo trabaja en MISSION/PAUSED (como _vision_gate de la estación).
%  12. Misma corrida con y sin figura (flujo aleatorio propio): resultados idénticos.
%  13. Telemetría y(61:63): roll y pitch de la planta del explorador y válido (misión PID).
%  14. actitud_en_tel = false (emula un modelo de Simulink viejo): y(61:63) = 0 y el
%      detector interno geolocaliza nivelado.
%  15. Geolocalización con roll/pitch, pivote de Motive, signos/offsets y descarte por
%      inclinación contra la estación (casos con actitud de referencia_estacion.json).
%  16. Detector interno, un cuadro sin ruido: ida y vuelta exacta con actitud; error
%      nivelado y con el error del soporte; descarte por inclinación.
%  17. Misión PID con usar_actitud = false y con inclinacion_max_deg = 1.5: error por
%      observación con y sin actitud, y descartadas por inclinación.
%  18. Modo SOLO EXPLORADOR (PID) con la pista REAL del laboratorio (pista.vertices y
%      pista.tubos de config/estacion.json, escenario_solo_lab.json con los objetos sobre los
%      tubos reales) y el dron 2 fuera de OptiTrack: DONE, todos los waypoints (los mismos que
%      qd2_track_waypoints con la ruta de la config), el dron 2 nunca
%      arma ni sube, y(64) = 1, TODOS los objetos detectados (también las tomas de las orillas,
%      a > 0.25 m de la ruta), ninguna observación de un objeto real descartada por "lejos del
%      tubo" y 0 visitados (no hay respondedor). Umbral de confianza: el de la config (0.9).
%  19. Solo explorador con el dron 2 y el dron 1 (no usado) RASTREADO en el piso debajo de
%      la N: el guard nunca se activa y la misión termina (DONE).
%  20. Los mismos objetos con la config SIN pista.tubos (filtro contra la ruta partida en los
%      huecos, como antes de b65603f): las tomas a más de dist_max_tubo_toma_m + 5 cm de la
%      ruta se descartan por "lejos del tubo" y no se publican; las del borde (0.25-0.30 m)
%      pierden la mayoría de sus observaciones; el resto se detecta.
%  21. Tubos reales y filtro "lejos del tubo" contra la estación (geo.tubos_pista,
%      tramos_sin_tubo, dist_tubos, filtros_vision y VisionService.filtrar; casos "tubos" de
%      referencia_estacion.json).
%  22. Pérdida de rastreo en el aire con el stack a bordo emulado (trk_a_bordo): solo explorador
%      (3 s: takeoff 0 a los 0.2 s, a bordo 7 -> 9, sin PAUSED, RESUME no hace nada, DONE con
%      alerta 2), lo mismo con trk_a_bordo = false (autoland), 0.15 s (nada) y dual PID (el
%      otro se aparta, regresa y aterriza: 5 -> 9 -> 7 -> 8).
%  23. Config sin pista.vertices ni pista.tubos: C.tubos vacío y sin filtro de tubo (como la
%      estación); la N por defecto solo se dibuja y sirve para colocar objetos.
%   Las pruebas 3-17 y 22 usan la config real con mision.modo = "dual" y la N de 4 vértices
%   fija sin pista.tubos ni tramos_sin_tubo (el tubo es la N entera), así que no dependen del
%   modo ni de la pista del día en config/estacion.json.
%   La prueba TCP con Python está en prueba_cliente_puente.py (ver README).

if nargin < 1, con_capturas = true; end
aqui = fileparts(mfilename('fullpath'));
addpath(aqui);
raiz = fileparts(aqui);
addpath(fullfile(raiz, 'simulink_qdrone2', 'lib'));
res = struct('nombre', {}, 'ok', {}, 'detalle', {});
base = struct('mostrar', false, 'tiempo_real', false, 'usar_tcp', false, 'auto_start', 3, ...
    'graficas_finales', false, 't_final', 300, 'verbose', false);
% Config de las pruebas: la real con modo dual y la N de 4 vértices fija, sin pista.tubos ni
% tramos_sin_tubo (las pruebas no dependen de mision.modo ni de la pista del día en
% config/estacion.json).
cfg_real = sim3d_leer_json(fileread(fullfile(raiz, 'config', 'estacion.json')));
cfg_dual = cfg_real;
cfg_dual.mision.modo = 'dual';
if isfield(cfg_dual.mision, 'dron_solo'), cfg_dual.mision = rmfield(cfg_dual.mision, 'dron_solo'); end
cfg_dual.pista = struct('vertices', [-1.2 1.2; 1.2 1.2; -1.2 -1.2; 1.2 -1.2]);
base.config = cfg_dual;
% Modo solo explorador con la pista REAL del laboratorio: ruta (pista.vertices) y tubos
% (pista.tubos) tal como están en config/estacion.json, sin copiarlos aquí
cfg_solo = cfg_real;
cfg_solo.mision.modo = 'solo_explorador';
cfg_solo.mision.dron_solo = 1;

% ---- 1. malla
% Motores en (±101.6, ±127) mm (thrust parametrization, tabla 1) con aros concéntricos de 119 mm:
% el manual no se puede cumplir exacto en x y en y a la vez con aros circulares (119.7 vs 118.4 mm),
% así que se acepta hasta 2 mm por eje.
M = sim3d_malla_qdrone2();
manual = [0.44258 0.4908 0.13553];
dif = M.medidas - manual;
err = max(abs(dif));
mot_ok = max(abs(abs(M.motores(:, 1:2)) - [0.1016 0.127]), [], 'all') < 1e-12 ...
    && isequal(M.cen_aros, M.motores(:, 1:2));
lente_ok = M.lente(1) < max(abs(M.motores(:, 1))) + M.r_aro;     % la RealSense no sale de los aros
res = anotar(res, 'malla QDrone 2 = medidas del manual', err < 2e-3 && mot_ok && lente_ok, ...
    sprintf(['malla %.4f x %.4f x %.4f m (manual 0.4426 x 0.4908 x 0.1355; diferencia %+.1f / %+.1f / %+.1f mm), ' ...
    'motores en (±%.1f, ±%.1f) mm con aros concéntricos R %.0f mm = %d, %d caras + 4 hélices'], ...
    M.medidas, 1e3*dif, 1e3*max(abs(M.motores(:, 1))), 1e3*max(abs(M.motores(:, 2))), 1e3*M.r_aro, mot_ok, ...
    sum(arrayfun(@(g) size(g.F, 1), M.grupos))));

% ---- 2. cámara y clusters vs estación
ref_f = fullfile(aqui, 'pruebas', 'referencia_estacion.json');
if exist(ref_f, 'file')
    ref = jsondecode(fileread(ref_f));
    C = sim3d_config(raiz);
    eh = 0;
    for k = 1:numel(ref.huellas)
        h = sim3d_camara('huella', C.cam, ref.huellas(k).pose(:));
        eh = max(eh, max(abs(h(:) - reshape(ref.huellas(k).huella, [], 1))));
    end
    ep = 0;
    for k = 1:numel(ref.pixeles)
        g = sim3d_camara('a_suelo', C.cam, ref.pixeles(k).uv(:), ref.pixeles(k).pose(:));
        ep = max(ep, max(abs(g(:) - ref.pixeles(k).suelo(:))));
    end
    det = sim3d_detector('init', [], C, struct());
    det.radio = ref.clusters.radio; det.min_obs = ref.clusters.min_obs; det.conf_min = 0;
    O = ref.clusters.observaciones;
    for k = 1:size(O, 1)
        det = sim3d_detector('agregar', det, O(k, 1), O(k, 2), O(k, 3), O(k, 4));
    end
    T = sim3d_detector('objetivos', det);
    Tr = ref.clusters.objetivos;
    ec = inf;
    if isequal(size(T), size(Tr)), ec = max(abs(T(:) - Tr(:))); end
    res = anotar(res, 'huella de cámara = web/app/geo.py', eh < 2e-3, sprintf('error máx. %.2g m (Python redondea a mm)', eh));
    res = anotar(res, 'píxel -> suelo = web/app/geo.py', ep < 1e-9, sprintf('error máx. %.2g m', ep));
    res = anotar(res, 'clusters = Clusterer de la estación', ec < 1e-9, ...
        sprintf('%d objetivos (Python %d), error máx. %.2g', size(T, 1), size(Tr, 1), ec));
    % ---- 15. geolocalización con actitud vs estación
    if isfield(ref, 'pose_geo')
        res = prueba_geo_actitud(res, ref, C, raiz);
    else
        res = anotar(res, 'geolocalización con actitud = estación', false, ...
            'referencia_estacion.json sin casos con actitud: córrela con pruebas/generar_referencia_estacion.py');
    end
    % ---- 8. escenario 'web'
    if isfield(ref, 'anomalias_web')
        aw = ref.anomalias_web;
        if ~iscell(aw), aw = num2cell(aw); end
        escw = sim3d_escenario(sim3d_opciones(struct('escenario', 'web')), sim3d_config(raiz, cfg_dual));
        ew = inf;
        if numel(escw.obj) == numel(aw)
            ew = 0;
            for k = 1:numel(aw)
                ew = max([ew, abs(escw.obj(k).x - aw{k}.x), abs(escw.obj(k).y - aw{k}.y), ...
                    1e3*abs(escw.obj(k).clase - aw{k}.cls)]);
            end
        end
        res = anotar(res, 'escenario ''web'' = cámara sintética de la UI', ew < 1e-9, ...
            sprintf('%d objetos (Python %d), error máx. %.2g m', numel(escw.obj), numel(aw), ew));
    end
    % ---- 8b. escenario 'web' con la pista REAL (cámara sintética sobre pista.tubos)
    if isfield(ref, 'anomalias_web_pista_real')
        aw = ref.anomalias_web_pista_real;
        if ~iscell(aw), aw = num2cell(aw); end
        Cr = sim3d_config(raiz, cfg_solo);
        escw = sim3d_escenario(sim3d_opciones(struct('escenario', 'web')), Cr);
        ew = inf; dt = NaN;
        if numel(escw.obj) == numel(aw)
            ew = 0;
            for k = 1:numel(aw)
                ew = max([ew, abs(escw.obj(k).x - aw{k}.x), abs(escw.obj(k).y - aw{k}.y), ...
                    1e3*abs(escw.obj(k).clase - aw{k}.cls)]);
            end
            dt = max(arrayfun(@(ob) sim3d_tubos('dist', ob.x, ob.y, Cr.tubos), escw.obj));
        end
        res = anotar(res, 'escenario ''web'' con la pista real = cámara sintética (sobre los tubos)', ...
            ew < 1e-9 && dt < 1e-3, sprintf(['%d objetos (Python %d), error máx. %.2g m, distancia máx. ' ...
            'al tubo real %.4f m'], numel(escw.obj), numel(aw), ew, dt));
    end
    % ---- 21. tubos reales y filtro "lejos del tubo" vs estación
    if isfield(ref, 'tubos')
        res = prueba_tubos(res, ref.tubos, raiz, cfg_real);
    else
        res = anotar(res, 'tubos y filtro de tubo = estación', false, ...
            'referencia_estacion.json sin casos de tubos: córrela con pruebas/generar_referencia_estacion.py');
    end
else
    res = anotar(res, 'referencia de la estación', false, 'falta pruebas/referencia_estacion.json');
end

% ---- 9. heartbeat de Python en tiempo de reloj (estación sola, sin TCP)
oo = sim3d_opciones(struct('mostrar', false));
est = sim3d_estacion('init', [], oo);
pq = @(hb, cmd, seq) [hb; cmd; seq; zeros(29, 1)];
vel = 16; dt = 0.02; t = 0; tr = 0; hb_py = 0; t_tx = 0; hb_v = []; t_v = [];
rng(5);
while tr < 6
    t = t + dt; tr = tr + dt/vel;
    P = zeros(32, 0);
    if tr < 4 && tr >= t_tx              % Python manda a ~20 Hz de reloj con jitter de hasta 0.2 s
        hb_py = hb_py + 1; P = pq(hb_py, 0, 0);
        t_tx = tr + 0.05 + 0.15*rand;
    end
    [est, py] = sim3d_estacion('paso', est, t, P, [], zeros(0, 5), oo, tr);
    hb_v(end+1) = py(1); t_v(end+1) = tr; %#ok<AGROW>
end
% hueco máximo (en s SIMULADOS) sin que suba el heartbeat mientras Python manda
sube = [true diff(hb_v) > 0];
ts = (1:numel(hb_v))*dt;
vivo = t_v < 4;
hueco = max(diff(ts(sube & vivo)));
t_para = t_v(find(sube, 1, 'last'));
ok = hueco < 0.05 && t_para > 4 && t_para < 4 + oo.t_hb_python + 0.06;
res = anotar(res, 'heartbeat de Python medido en reloj (x16)', ok, sprintf(['con Python a 20 Hz de reloj y ' ...
    'jitter de 0.2 s: hueco máx. %.2f s simulados (antes ~0.8-3 s -> PAUSED); Python se calla a 4.00 s de ' ...
    'reloj y el heartbeat deja de subir a %.2f s'], hueco, t_para));

% ---- 10. primer paquete con START: no se ejecuta ni se confirma
est = sim3d_estacion('init', [], oo);
[est, py] = sim3d_estacion('paso', est, 0.02, pq(1, 1, 7), [], zeros(0, 5), oo, 0.02);
ack1 = est.ack_py; c1 = py(2);
[est, py] = sim3d_estacion('paso', est, 0.04, pq(2, 1, 7), [], zeros(0, 5), oo, 0.04);
c2 = py(2);
[est, py] = sim3d_estacion('paso', est, 0.06, pq(3, 1, 8), [], zeros(0, 5), oo, 0.06);
est = sim3d_estacion('ack', est, py(3));
ok = ack1 == 6 && c1 == 0 && c2 == 0 && py(2) == 1 && est.ack_py == 8;
res = anotar(res, 'primer paquete con START: ni se ejecuta ni se confirma', ok, sprintf(['ack %d al START ' ...
    'seq 7 del primer paquete (cmd a la misión %d, %d); el siguiente START seq 8 sí: cmd %d, ack %d'], ...
    ack1, c1, c2, py(2), est.ack_py));


% ---- 3. misión PID
o = base;
if con_capturas
    o.capturas = [2.5 2.6 30 30.1 60 95]; o.capturas_vistas = [1 4 1 2 1 1];
    o.graficas_finales = true; o.prefijo_capturas = 'prueba';
end
R = sim3d_correr(o);
m = R.met;
ok = m.estado_final == 8 && m.reales_visitados == m.n_reales && m.dh_min_aire >= R.p.d_min_h;
res = anotar(res, 'misión PID completa', ok, sprintf(['DONE en %.1f s sim (%.1f s reloj), %d/%d objetos ' ...
    'visitados, dh mín. en vuelo %.3f m (>= %.2f), error de posición medio %.3f m, falsos publicados %d'], ...
    m.t_sim, m.t_reloj, m.reales_visitados, m.n_reales, m.dh_min_aire, R.p.d_min_h, mean(m.err_pos), ...
    m.n_falsos_publicados));
% ---- 11. el detector solo trabaja en MISSION / PAUSED
L = R.log;
nuevo = [L.det_cuadros(1) > 0, diff(L.det_cuadros) > 0];
est_det = unique(L.estado(nuevo));
res = anotar(res, 'detector solo en MISSION/PAUSED', all(ismember(est_det, [5 10])) && any(nuevo), ...
    sprintf('%d cuadros procesados, estados en los que hubo cuadros: %s', m.det_cuadros, mat2str(est_det)));
% ---- 13. telemetría y(61:63) = [roll pitch válido] del explorador
% La telemetría del paso k usa la actitud medida al inicio del paso = la de la planta
% registrada al final del paso k-1 (Lg.att); sin pérdidas de rastreo, válido = hay explorador.
kt = find(~isnan(L.tel_att(1, :)));
kt = kt(kt > 1);
eid = L.explorer_id(kt);
val = eid == 1 | eid == 2;
err = 0;
for jj = find(val)
    ka = kt(jj);
    err = max(err, max(abs(L.tel_att(1:2, ka) - L.att(1:2, ka - 1, eid(jj)))));
end
ok_val = isequal(L.tel_att(3, kt) == 1, val) && all(L.tel_att(:, kt(~val)) == 0, 'all');
amp = max(abs(L.tel_att(1:2, kt(val))), [], 'all');
res = anotar(res, 'telemetría y(61:63) actitud del explorador', any(val) && ok_val && err < 1e-12 && amp > 0.005, ...
    sprintf(['%d mensajes (%d con explorador): y(63) coherente = %d, error vs planta %.1e rad, ' ...
    '|roll|,|pitch| máx. %.2f°'], numel(kt), sum(val), ok_val, err, rad2deg(amp)));
% ---- 14. actitud_en_tel = false (modelo de Simulink viejo): y(61:63) = 0 aunque haya explorador
o = base; o.t_final = 30; o.actitud_en_tel = false;
Rv = sim3d_correr(o);
Lv = Rv.log;
kv = find(~isnan(Lv.tel_att(1, :)));
ev = Lv.explorer_id(kv);
amp_v = max(abs(Lv.att(1:2, :, :)), [], 'all');   % la planta sí se inclinó
mv = Rv.met;
res = anotar(res, 'actitud_en_tel = false: y(61:63) = 0 (modelo viejo)', ...
    any(ev == 1 | ev == 2) && all(Lv.tel_att(:, kv) == 0, 'all') && amp_v > 0.005 ...
    && mv.det_con_actitud == 0 && mv.det_nivelado == mv.det_cuadros, ...
    sprintf(['%d mensajes (%d con explorador), todos con y(61:63) = 0; la planta llegó a %.2f°; ' ...
    'detector interno: %d cuadros, %d nivelados, %d con actitud'], numel(kv), sum(ev == 1 | ev == 2), ...
    rad2deg(amp_v), mv.det_cuadros, mv.det_nivelado, mv.det_con_actitud));
% ---- 16. detector interno: un cuadro sin ruido con el dron inclinado
res = prueba_cuadro_actitud(res, raiz);
% ---- 17. misión PID sin actitud (usar_actitud = false) y con inclinacion_max_deg = 1.5
mc = R.met;
o = base; o.usar_actitud = false;
Rn = sim3d_correr(o);
mn = Rn.met;
o = base; o.inclinacion_max_deg = 1.5; o.t_final = 60;
Ri = sim3d_correr(o);
mi = Ri.met;
ok = mc.det_con_actitud > 0 && mc.det_nivelado == 0 && mc.det_desc_incl == 0 ...
    && mn.det_con_actitud == 0 && mn.det_nivelado > 0 && mn.estado_final == 8 ...
    && median(mc.err_obs) < median(mn.err_obs) && mi.det_desc_incl > 0 && mi.det_con_actitud > 0;
res = anotar(res, 'misión PID con y sin actitud (detector interno)', ok, sprintf(['error por observación ' ...
    'mediana/p95/máx.: con actitud %.1f/%.1f/%.1f mm (n=%d, %d cuadros, incl. máx. %.2f°, %d descartadas), ' ...
    'usar_actitud=false %.1f/%.1f/%.1f mm (n=%d, %d cuadros nivelados; DONE %d); con inclinacion_max_deg=1.5 ' ...
    '(60 s): %d descartadas por inclinación, %d observaciones al cluster'], 1e3*median(mc.err_obs), ...
    1e3*pct(mc.err_obs, 95), 1e3*max(mc.err_obs), numel(mc.err_obs), mc.det_con_actitud, mc.det_incl_max_deg, ...
    mc.det_desc_incl, 1e3*median(mn.err_obs), 1e3*pct(mn.err_obs, 95), 1e3*max(mn.err_obs), numel(mn.err_obs), ...
    mn.det_nivelado, mn.estado_final == 8, mi.det_desc_incl, mi.det_obs));
% ---- 12. misma corrida con y sin figura
if con_capturas
    Rb = sim3d_correr(base);
    mb = Rb.met;
    ok = mb.t_sim == m.t_sim && mb.dh_min_aire == m.dh_min_aire && mb.n_objetivos == m.n_objetivos ...
        && isequal(mb.err_pos, m.err_pos);
    res = anotar(res, 'misma corrida con y sin figura', ok, sprintf(['con figura: DONE %.2f s, dh %.4f m, ' ...
        '%d objetivos; sin figura: DONE %.2f s, dh %.4f m, %d objetivos'], m.t_sim, m.dh_min_aire, ...
        m.n_objetivos, mb.t_sim, mb.dh_min_aire, mb.n_objetivos));
end
% ---- 4. misión MPC
o = base; o.version = 'MPC';
if con_capturas
    o.capturas = 62.5; o.capturas_vistas = 1; o.graficas_finales = true; o.prefijo_capturas = 'prueba';
end
R2 = sim3d_correr(o);
m = R2.met;
L = R2.log;
ap = L.dh < 0.3 & squeeze(L.pos(3,:,1)) > 0.2 & squeeze(L.pos(3,:,2)) > 0.2;
ok = m.estado_final == 8 && m.reales_visitados == m.n_reales && (~any(ap) || m.dz_min_apilados >= R2.p.dz_min - 0.05);
res = anotar(res, 'misión MPC completa (apilados)', ok, sprintf(['DONE en %.1f s sim, %d/%d visitados, ' ...
    '%.1f s apilados (dh < 0.3 m) con dz mín. %.3f m (dz_min %.2f)'], m.t_sim, m.reales_visitados, m.n_reales, ...
    0.02*sum(ap), m.dz_min_apilados, R2.p.dz_min));
% ---- 5. escenario aleatorio
o = base; o.escenario = 'aleatorio'; o.semilla = 11; o.n_fuegos = 3; o.n_tomas = 3;
R3 = sim3d_correr(o);
m = R3.met;
ok = m.estado_final == 8 && m.dh_min_aire >= R3.p.d_min_h && m.reales_detectados == m.n_reales;
res = anotar(res, 'escenario aleatorio (semilla 11, 6 objetos)', ok, sprintf(['DONE en %.1f s, detectados %d/%d, ' ...
    'visitados %d/%d, dh mín. %.3f m'], m.t_sim, m.reales_detectados, m.n_reales, m.reales_visitados, m.n_reales, m.dh_min_aire));
% ---- 18. solo explorador PID: pista REAL del laboratorio (ruta + tubos), dron 2 fuera de OptiTrack
o = base; o.config = cfg_solo; o.escenario = 'escenarios/escenario_solo_lab.json';
if con_capturas
    o.capturas = [45 45.1]; o.capturas_vistas = [1 2]; o.graficas_finales = true; o.prefijo_capturas = 'prueba_solo';
end
R5 = sim3d_correr(o);
m = R5.met; L = R5.log; s5 = R5.s_final;
kt = ~isnan(L.y64);
z_aus = max(L.pos(3, :, 2));
C5 = sim3d_config(raiz, cfg_solo);
d_ruta = arrayfun(@(ob) sim3d_tubos('dist_polilinea', ob.x, ob.y, C5.ruta), R5.esc.obj);
d_tubo = arrayfun(@(ob) sim3d_tubos('dist', ob.x, ob.y, C5.tubos), R5.esc.obj);
% waypoints esperados: la N que arma qd2_mission_step en START con la ruta de la config
[wp5, n_wp5] = qd2_track_waypoints(C5.p.vertices, C5.p.wp_spacing, C5.p.z_E, C5.p.n_vertices);
wp_ok = n_wp5 > 1 && s5.n_wp == n_wp5 && max(abs(s5.wp(1:n_wp5, 1:2) - wp5(1:n_wp5, 1:2)), [], 'all') < 1e-12;
ok = R5.modo.solo && m.estado_final == 8 && wp_ok && s5.wp_idx == s5.n_wp && s5.explorer_done == 1 ...
    && z_aus == 0 && max(L.arm(2, :)) == 0 && max(L.takeoff(2, :)) == 0 && all(L.explorer_id == 1) ...
    && any(kt) && all(L.y64(kt) == 1) && C5.tubos_reales && m.tubos_reales ...
    && m.reales_detectados == m.n_reales && all(propia_clase(R5)) && all(m.det_desc_tubo_obj == 0) ...
    && all(m.det_obs_obj >= R5.det.min_obs) ...
    && m.n_visitados == 0 && m.reales_visitados == 0;
res = anotar(res, 'solo explorador PID (pista real del lab con tubos, dron 2 fuera)', ok, sprintf(['DONE en %.1f s sim, ' ...
    '%d/%d wp (config: %d; ruta de %d vértices, %.2f m; %d tubos reales); detectados %d/%d (%d objetivos, error %s mm; ' ...
    'a %s m de la ruta y %s m del tubo), observaciones por objeto %s, descartadas por tubo: %d de objetos reales, ' ...
    '%d en total, confianza_min %.2f; 0 visitados = %d; dron 2: z máx. %.3f m, arm máx. %d, takeoff máx. %d; ' ...
    'y(64) = 1 en %d/%d mensajes'], m.t_sim, s5.wp_idx, s5.n_wp, n_wp5, size(C5.ruta, 1), ...
    sum(sqrt(sum(diff(C5.ruta).^2, 2))), numel(C5.tubos), m.reales_detectados, m.n_reales, m.n_objetivos, ...
    mat2str(round(1e3*m.err_pos)), mat2str(round(d_ruta, 2)), mat2str(round(d_tubo, 2)), mat2str(m.det_obs_obj), ...
    sum(m.det_desc_tubo_obj), m.det_desc_tubo, C5.vis.conf_min, m.n_visitados == 0, z_aus, max(L.arm(2, :)), ...
    max(L.takeoff(2, :)), sum(L.y64(kt) == 1), sum(kt)));
% ---- 20. los mismos objetos con el filtro contra la RUTA (config sin pista.tubos)
res = prueba_sin_tubos(res, R5, cfg_solo, base, raiz);
% ---- 19. solo explorador con el dron 2; el dron 1 (no usado) rastreado en el piso bajo la N
o = base; o.modo_mision = 'solo_explorador'; o.dron_solo = 2; o.dron_ausente = 'piso';
o.homes = [0.10 0.02; 1.25 0];
R6 = sim3d_correr(o);
m = R6.met; L = R6.log; s6 = R6.s_final;
kt = ~isnan(L.y64);
ok = R6.modo.solo && m.estado_final == 8 && s6.wp_idx == s6.n_wp && all(L.guard == 0) ...
    && max(L.arm(1, :)) == 0 && max(L.takeoff(1, :)) == 0 && max(L.pos(3, :, 1)) == 0 ...
    && all(L.explorer_id == 2) && any(kt) && all(L.y64(kt) == 2);
d_paso = min(hypot(L.pos(1, L.estado == 5, 2) - 0.10, L.pos(2, L.estado == 5, 2) - 0.02));
res = anotar(res, 'solo explorador dron 2 (dron 1 en el piso bajo la N)', ok, sprintf(['DONE en %.1f s sim, %d/%d wp, ' ...
    'guard activo %d pasos, dron 1: arm máx. %d, z máx. %.3f m; E pasó a %.2f m (horizontal) del dron 1; y(64) = 2 en %d/%d'], ...
    m.t_sim, s6.wp_idx, s6.n_wp, sum(L.guard > 0), max(L.arm(1, :)), max(L.pos(3, :, 1)), d_paso, sum(L.y64(kt) == 2), sum(kt)));
% ---- 22. pérdida de rastreo en el aire: aterriza (stack a bordo emulado, trk_a_bordo)
res = prueba_rastreo(res, base);
% ---- 23. config sin pista.vertices ni pista.tubos: C.tubos vacío (la estación no filtra)
res = prueba_sin_pista(res, cfg_real, raiz);
% ---- 6. tiempo real sin figura
close all force; drawnow; pause(1);     % que la limpieza de figuras anteriores no caiga en la medición
o = base; o.tiempo_real = true; o.t_final = 10; o.parar_en_done = false;
R4 = sim3d_correr(o);
f = R4.met.t_sim/R4.met.t_reloj;
res = anotar(res, 'ritmo de tiempo real (10 s, sin figura)', abs(f - 1) < 0.03, sprintf(['%.2f s sim en %.2f s de reloj ' ...
    '(factor %.3f, re-sincronizaciones %d)'], R4.met.t_sim, R4.met.t_reloj, f, R4.met.n_atrasos));

% ---- 7. costo de dibujo
C = sim3d_config(raiz, cfg_dual);
oo = sim3d_opciones(struct('mostrar', false));
esc = sim3d_escenario(oo, C);
G = sim3d_escena('crear', [], C, esc, oo, raiz);
E = cuadro_falso(R, esc, C, G);
n = 60;
G = sim3d_escena('actualizar', G, E); drawnow;
t_act = 0; t_draw = 0;
for k = 1:n
    E.t = E.t + 0.04; E.X(9,1) = E.X(9,1) + 0.01; E.X(1,2) = E.X(1,2) + 0.002;
    t0 = tic;
    G = sim3d_escena('actualizar', G, E);
    t_act = t_act + toc(t0);
    t0 = tic;
    drawnow;
    t_draw = t_draw + toc(t0);
end
ms = 1000*(t_act + t_draw)/n;
close(G.fig);
% Con la figura en pantalla el bucle dibuja a lo más opc.fps (30) y calcula la física según el
% reloj, así que un cuadro de hasta ~50 ms solo baja los fps, no el tiempo real.
res = anotar(res, 'costo de un cuadro (figura invisible)', ms < 50, sprintf(['%.1f ms por cuadro (~%.0f fps): ' ...
    'actualizar %.1f ms + drawnow %.1f ms'], ms, 1000/ms, 1000*t_act/n, 1000*t_draw/n));
fprintf('\n===== RESUMEN probar_sim3d =====\n');
for k = 1:numel(res)
    fprintf('[%s] %s: %s\n', ternario(res(k).ok, ' OK ', 'FALLO'), res(k).nombre, res(k).detalle);
end
fprintf('%d de %d pruebas pasan\n', sum([res.ok]), numel(res));
end

% -------------------------------------------------------------------------
function res = prueba_geo_actitud(res, ref, C, raiz)
% Casos con actitud generados con web/app/geo.py y vision_service.pose_para_geo (ver
% pruebas/generar_referencia_estacion.py).
ok_piv = abs(C.cam.pivote - ref.camara.pivote) < 1e-12;
ep = 0; nn = 0;
for k = 1:numel(ref.pixeles_actitud)
    c = ref.pixeles_actitud(k);
    g = sim3d_camara('a_suelo', C.cam, c.uv(:), c.pose(:));
    [e, n1] = comparar_suelo(g, c.suelo);
    ep = max(ep, e); nn = nn + n1;
end
eh = 0;
for k = 1:numel(ref.huellas_actitud)
    c = ref.huellas_actitud(k);
    h = sim3d_camara('huella', C.cam, c.pose(:));
    hr = c.huella;
    if iscell(hr) || isempty(h)
        eh = inf;          % una esquina no corta el piso: estos casos no deberían tenerla
    else
        eh = max(eh, max(abs(h - hr), [], 'all'));
    end
end
casos = ref.pose_geo;
if ~iscell(casos), casos = num2cell(casos); end
eq = 0; ei = 0; es = 0; n_mal = 0; n_incl = 0; n_ok = 0; n_desc = 0; nn2 = 0;
n_var = numel(unique(cellfun(@(c) jsonencode(struct('v', c.vision, 'o', c.optitrack)), casos, 'UniformOutput', false)));
for k = 1:numel(casos)
    c = casos{k};
    cfg = C.cfg;
    vj = c.vision;
    if isfield(c, 'vision_json'), vj = sim3d_leer_json(c.vision_json); end   % null y [] distintos
    f = fieldnames(vj);
    for i = 1:numel(f), cfg.vision.(f{i}) = vj.(f{i}); end
    f = fieldnames(c.optitrack);
    for i = 1:numel(f), cfg.optitrack.(f{i}) = c.optitrack.(f{i}); end
    Cv = sim3d_config(raiz, cfg);
    [q, act] = sim3d_detector('pose_geo', [], c.pose(:), Cv.vis);
    eq = max([eq, max(abs(q(:) - c.q6(:))), abs(Cv.cam.pivote - c.pivote)]);
    lim_ok = isequal(isempty(Cv.vis.incl_max_deg), isempty(c.lim_incl_deg)) ...
        && (isempty(c.lim_incl_deg) || Cv.vis.incl_max_deg == c.lim_incl_deg);
    if act.ok ~= c.ok || act.disponible ~= c.disponible || act.inclinado ~= c.inclinado || ~lim_ok
        n_mal = n_mal + 1;
    end
    if c.ok, ei = max(ei, abs(rad2deg(act.incl) - c.incl_deg)); end
    g = sim3d_camara('a_suelo', Cv.cam, c.uv(:), q);
    [e, n1] = comparar_suelo(g, c.suelo);
    es = max(es, e); nn2 = nn2 + n1;
    n_ok = n_ok + c.ok; n_incl = n_incl + c.inclinado; n_desc = n_desc + ~c.disponible;
end
ok = ok_piv && ep < 1e-9 && eh < 1e-9 && eq < 1e-12 && ei < 1e-9 && es < 1e-9 && n_mal == 0 && nn == 0 && nn2 == 0;
res = anotar(res, 'geolocalización con actitud = estación (geo.py + pose_para_geo)', ok, sprintf(['pivote %.3f m; ' ...
    '%d píxeles con roll/pitch hasta 20°: error máx. %.2g m; %d huellas: %.2g m; %d casos con %d variantes de ' ...
    'vision/optitrack (%d con actitud, %d sin y(63), %d descartados por inclinación): pose %.2g, inclinación ' ...
    '%.2g°, suelo %.2g m, %d diferencias de ok/disponible/inclinado/límite; NaN distintos %d'], C.cam.pivote, ...
    numel(ref.pixeles_actitud), ep, numel(ref.huellas_actitud), eh, numel(casos), n_var, n_ok, n_desc, n_incl, eq, ei, ...
    es, n_mal, nn + nn2));
end

function [e, n_mal] = comparar_suelo(g, ref)
% g 2x1 (NaN si no corta) contra la referencia de Python ([] = None)
n_mal = 0; e = 0;
if isempty(ref)
    n_mal = double(all(isfinite(g)));
elseif any(~isfinite(g))
    n_mal = 1;
else
    e = max(abs(g(:) - ref(:)));
end
end

function res = prueba_cuadro_actitud(res, raiz)
% Un cuadro del detector con el explorador inclinado 5° (roll 4°, pitch -3°), sin ruido ni falsos
% positivos y objetos de 4 cm de alto (su centro está en el plano z = 0.02 m de la estación): con la
% actitud, la estación debe devolver los objetos EXACTOS.
C = sim3d_config(raiz);
C.vis.dist_toma = [];          % objetos en una retícula bajo el dron, no sobre los tubos: sin filtro de tubo
% Umbral de prueba: el detector sintético da confianzas de 0.55 a 0.95 y vision.confianza_min de la
% config real es 0.9 (separa la planta de hojas rojas del detector real); con 0.9 solo ~1/8 de las
% observaciones de un cuadro pasarían y la prueba (que cuenta las 12) fallaba desde 853cd9d.
C.vis.conf_min = 0.5;
xr = [0.1; -0.2; 1.5; 0; 0; 0; deg2rad(4); deg2rad(-3); 0.7];
qr = xr([1:3 9 7 8]);
[uu, vv] = meshgrid(linspace(170, 470, 4), linspace(130, 350, 3));
G = sim3d_camara('a_suelo', C.cam, [uu(:) vv(:)].', qr);
obj = struct('clase', num2cell(mod(1:size(G, 2), 2)), 'x', num2cell(G(1, :)), 'y', num2cell(G(2, :)), ...
    'h', 0.04, 'r', 0.05);
esc.obj = obj;
opc = sim3d_opciones(struct('p_deteccion', 1, 'ruido_px', 0, 'p_falso', 0, 'p_confusion', 0, ...
    'sesgo_montaje_deg', [0 0]));
err_de = @(det) max(hypot(det.ultimas(:, 2) - [obj.x].', det.ultimas(:, 3) - [obj.y].'));
uno = @(Cx, o) sim3d_detector('cuadro', setfield(sim3d_detector('init', [], Cx, o), 'rs', ...
    RandStream('mt19937ar', 'Seed', 1)), Cx, esc, xr, qr, 0, o); %#ok<SFLD>
d1 = uno(C, opc);                               % con actitud
Cn = C; Cn.vis.usar_actitud = false;
d2 = uno(Cn, opc);                              % estación sin actitud (nivelado)
d3 = sim3d_detector('cuadro', setfield(sim3d_detector('init', [], C, opc), 'rs', ...
    RandStream('mt19937ar', 'Seed', 1)), C, esc, xr, qr(1:4), 0, opc); %#ok<SFLD>  sin y(61:63)
os = opc; os.sesgo_montaje_deg = [0.8 -0.6];
d4 = uno(C, os);                                % con actitud, error del soporte
Ci = C; Ci.vis.incl_max_deg = 4.9;
d5 = uno(Ci, opc);                              % 5.0° > 4.9°: se descarta todo
n = numel(obj);
e1 = err_de(d1); e2 = err_de(d2); e3 = err_de(d3); e4 = err_de(d4);
ok = size(d1.ultimas, 1) == n && d1.n_obs == n && e1 < 1e-9 && d1.n_con_actitud == 1 ...
    && e2 > 0.08 && d2.n_nivelado == 1 && abs(e3 - e2) < 1e-12 && d3.n_nivelado == 1 ...
    && e4 > 0.01 && e4 < 0.05 && d5.n_desc_incl == n && d5.n_obs == 0 && isempty(d5.cl);
res = anotar(res, 'detector interno con actitud (cuadro sin ruido)', ok, sprintf(['explorador a 1.5 m, roll 4°, ' ...
    'pitch -3° (%.2f° de inclinación), %d objetos: con actitud error máx. %.1e m; nivelado (usar_actitud=false) ' ...
    '%.3f m, sin y(61:63) %.3f m; con el error del soporte [0.8 -0.6]° %.3f m; inclinacion_max_deg=4.9: %d de %d ' ...
    'descartadas, %d al cluster'], rad2deg(d1.incl_max), n, e1, e2, e3, e4, d5.n_desc_incl, n, d5.n_obs));
end

function res = prueba_sin_tubos(res, R5, cfg_solo, base, raiz)
% Prueba 20: los objetos del escenario del lab en su lugar REAL (sobre pista.tubos), pero la config
% sin pista.tubos: el filtro mide contra la ruta partida en tramos_sin_tubo (como antes de b65603f y
% como en los vuelos 5 y 6 del 24-nov). Las tomas a más de dist_max_tubo_toma_m de esa ruta se pierden.
cfg = cfg_solo;
cfg.pista = rmfield(cfg.pista, 'tubos');
Cs = sim3d_config(raiz, cfg);
obj = R5.esc.obj;
L = cell(1, numel(obj));
for j = 1:numel(obj)
    L{j} = struct('clase', ternario(obj(j).clase == 1, 'toma', 'fuego'), 'x', obj(j).x, 'y', obj(j).y, ...
        'yaw_deg', rad2deg(obj(j).yaw), 'variante', obj(j).variante);
end
js = struct('homes', R5.esc.home, 'yaw0_deg', rad2deg(R5.esc.yaw0));
js.objetos = L;
f = [tempname '_escenario_lab_absoluto.json'];
fid = fopen(f, 'w'); fwrite(fid, jsonencode(js), 'char'); fclose(fid);
limpiar = onCleanup(@() delete(f)); %#ok<NASGU>
o = base; o.config = cfg; o.escenario = f;
R = sim3d_correr(o);
m = R.met;
d_ruta = arrayfun(@(ob) sim3d_tubos('dist', ob.x, ob.y, Cs.tubos), obj);
% Tomas fuera del límite: "lejos" con margen de 5 cm (2 veces el error de geolocalización) no se
% publican; las del "borde" (límite < d <= límite + 5 cm, como la de 0.26 m del vuelo 6) pierden la
% mayoría de sus observaciones y pueden o no confirmarse; el resto se detecta.
% Una observación de toma confundida como fuego (p_confusion) no tiene filtro de tubo (fuego: null),
% igual que en la estación; por eso se mira el objetivo publicado de la MISMA clase.
toma = [obj.clase] == 1;
fuera = toma & d_ruta > Cs.vis.dist_toma;
lejos = toma & d_ruta > Cs.vis.dist_toma + 0.05;
borde = fuera & ~lejos;
pc = propia_clase(R);
ok = ~Cs.tubos_reales && ~m.tubos_reales && m.estado_final == 8 && any(lejos) ...
    && ~any(pc(lejos)) && all(m.det_desc_tubo_obj(fuera) > m.det_obs_obj(fuera)) && all(pc(~fuera));
res = anotar(res, 'sin pista.tubos: las tomas de las orillas se pierden (filtro contra la ruta)', ok, sprintf(['%d tubos ' ...
    '(ruta partida en %s); tomas a %s m de la ruta (límite %.2f): fuera del límite %s, de ellas lejos (> +5 cm) %s ' ...
    'sin publicar = %d; descartadas por tubo / aceptadas de las de fuera: %s / %s (borde %s publicada = %d); ' ...
    'objetos con objetivo de su clase %d/%d (con pista.tubos: %d/%d)'], numel(Cs.tubos), mat2str(Cs.sin_tubo), ...
    mat2str(round(d_ruta(toma), 3)), Cs.vis.dist_toma, mat2str(find(fuera)), mat2str(find(lejos)), ~any(pc(lejos)), ...
    mat2str(m.det_desc_tubo_obj(fuera)), mat2str(m.det_obs_obj(fuera)), mat2str(find(borde)), any(pc(borde)), ...
    sum(pc), numel(pc), sum(propia_clase(R5)), numel(obj)));
end

function res = prueba_rastreo(res, base)
% Prueba 22: pérdida de rastreo en el aire con mision.t_perdida_aterrizar_s = 0.2 (config) y el
% stack a bordo emulado (trk_a_bordo = true, por defecto): a los 0.2 s tierra le baja takeoff al
% perdido (alerta 2, sin PAUSED) y a bordo pasa a 7 (rampa) y luego a 9; nunca más takeoff, RESUME
% no hace nada. Dual: el otro se aparta, regresa y aterriza (9 -> 7 -> 8). Una pérdida de 0.15 s no
% hace nada. Con trk_a_bordo = false el perdido baja por autoland (6 -> 9).
tl = 40;
o = base; o.modo_mision = 'solo_explorador'; o.dron_solo = 1;
o.perdida_rastreo = [1 tl tl + 3]; o.comandos = [tl + 6 5];          % RESUME a los 46 s
Rs = sim3d_correr(o);
[ok_s, ds] = revisar_perdida(Rs, 1, tl);
o.trk_a_bordo = false;
Rf = sim3d_correr(o);
[ok_f, df] = revisar_perdida(Rf, 1, tl);
ok_f = ok_f && isnan(df.t7) && any(Rf.log.fsm(1, :) == 6);
o = base; o.modo_mision = 'solo_explorador'; o.dron_solo = 1;
o.perdida_rastreo = [1 tl tl + 0.15];
Rc = sim3d_correr(o);
Lc = Rc.log; sc = Rc.s_final;
ok_c = Rc.met.estado_final == 8 && isnan(Rc.met.t_trk7(1)) && ~any(Lc.alerta == 2) && ~any(Lc.estado == 10) ...
    && all(sc.falla == 0) && sc.wp_idx == sc.n_wp;
o = base; o.perdida_rastreo = [2 tl tl + 5];                       % dual PID (la config de las pruebas)
Rd = sim3d_correr(o);
[ok_d, dd] = revisar_perdida(Rd, 2, tl);
Ld = Rd.log;
k_aire = Ld.t >= tl & squeeze(Ld.pos(3, :, 1)) > 0.2 & squeeze(Ld.pos(3, :, 2)) > 0.2;
dh_aire = min([Ld.dh(k_aire) inf]);
ok_d = ok_d && isequal(dd.seq, [5 9 7 8]) && Ld.pos(3, end, 1) < 0.05 && dh_aire >= Rd.p.d_min_h;
ok = ok_s && isequal(ds.seq, [5 7 8]) && ok_f && ok_c && ok_d;
res = anotar(res, 'pérdida de rastreo en el aire: aterriza (stack a bordo emulado)', ok, sprintf(['solo, pérdida ' ...
    '[%g %g): takeoff 0 a +%.2f s, a bordo 7 a +%.2f s y 9 a +%.2f s, z %.2f m -> suelo a +%.2f s a %.2f m ' ...
    'de donde se perdió; estados %s, alerta final %d, PAUSED %d, takeoff de vuelta %d (RESUME a los %g s); ' ...
    'trk_a_bordo = false: takeoff 0 a +%.2f s, autoland, suelo a +%.2f s, estados %s; 0.15 s: nada (DONE, ' ...
    'alerta 2 %d, falla %s); dual PID, dron 2 [%g %g): takeoff 0 a +%.2f s, 7 a +%.2f s, estados %s, ' ...
    'alerta %d, dron 1 z final %.3f m, dh mín. con los dos arriba de 0.2 m %.2f m'], tl, tl + 3, ds.t_drop, ...
    ds.t7, ds.t9, ds.z0, ds.t_suelo, ds.deriva, mat2str(ds.seq), ds.alerta, ds.pausa, ds.resube, tl + 6, ...
    df.t_drop, df.t_suelo, mat2str(df.seq), any(Lc.alerta == 2), mat2str(sc.falla), tl, tl + 5, dd.t_drop, ...
    dd.t7, mat2str(dd.seq), dd.alerta, Ld.pos(3, end, 1), dh_aire));
end

function [ok, d] = revisar_perdida(R, dr, tl)
% Verificaciones comunes de una pérdida de rastreo del dron dr desde tl (> 0.2 s)
L = R.log;
k_drop = find(L.t >= tl & L.takeoff(dr, :) == 0, 1);
d.t_drop = L.t(k_drop) - tl;
d.t7 = R.met.t_trk7(dr) - tl;
k9 = find(L.t >= tl & L.fsm(dr, :) == 9, 1);
d.t9 = L.t(k9) - tl;
k0 = find(L.t >= tl, 1);
d.z0 = L.pos(3, k0, dr);
k_suelo = find(L.t >= tl & L.pos(3, :, dr) < 0.02, 1);
d.t_suelo = L.t(k_suelo) - tl;
d.deriva = norm(L.pos(1:2, k_suelo, dr) - L.pos(1:2, k0, dr));
e = L.estado(k0:end);
d.seq = [L.estado(k0 - 1) e([true diff(e) ~= 0])];
d.seq = d.seq([true diff(d.seq) ~= 0]);
d.alerta = L.alerta(end);
d.pausa = any(L.estado == 10);
d.resube = any(L.takeoff(dr, k_drop:end) > 0);
ok = ~isempty(k_drop) && abs(d.t_drop - 0.2) < 0.03 && R.met.estado_final == 8 && d.alerta == 2 && ~d.pausa ...
    && ~d.resube && L.fsm(dr, end) == 9 && L.pos(3, end, dr) < 0.05 && R.s_final.falla(dr) == 2 ...
    && (isnan(d.t7) || abs(d.t7 - 0.2) < 0.03);
end

function res = prueba_sin_pista(res, cfg_real, raiz)
% Prueba 23: sin pista.vertices ni pista.tubos la estación no tiene tubos (geo.tubos_pista da [])
% y no filtra; el simulador igual (C.tubos vacío). La N por defecto (C.ruta) solo se dibuja y sirve
% para colocar los objetos. Con solo pista.tubos, los tubos son esos.
cfg = cfg_real;
cfg = rmfield(cfg, 'pista');
C1 = sim3d_config(raiz, cfg);
esc1 = sim3d_escenario(sim3d_opciones(struct('escenario', 'aleatorio', 'semilla', 3)), C1);
det1 = sim3d_detector('init', [], C1, struct());
[lejos1, d1] = sim3d_detector('tubo', det1, 1, 1.4, 1.4);
cfg.pista = struct('tubos', {{[0 0; 1 0]}});
C2 = sim3d_config(raiz, cfg);
det2 = sim3d_detector('init', [], C2, struct());
[lejos2, d2] = sim3d_detector('tubo', det2, 1, 0.5, 1.0);
ok = isempty(C1.tubos) && ~C1.tubos_reales && ~lejos1 && isempty(d1) && numel(esc1.tubos) == 1 ...
    && isequal(esc1.tubos{1}, C1.ruta) && numel(esc1.obj) == 4 && numel(C2.tubos) == 1 && C2.tubos_reales ...
    && lejos2 && abs(d2 - 1.0) < 1e-12;
res = anotar(res, 'config sin pista: sin tubos ni filtro (como la estación)', ok, sprintf(['sin pista: %d tubos, ' ...
    'toma en (1.4, 1.4) lejos = %d (distancia %s); dibujo y objetos sobre la ruta de %d vértices (%d objetos); ' ...
    'solo pista.tubos: %d tubo, toma a %.2f m -> lejos = %d'], numel(C1.tubos), lejos1, mat2str(d1), ...
    size(C1.ruta, 1), numel(esc1.obj), numel(C2.tubos), d2, lejos2));
end

function pc = propia_clase(R)
% por objeto real: hay un objetivo publicado por el detector interno de su MISMA clase a < 0.35 m
T = R.objetivos_detector;
pc = false(1, numel(R.esc.obj));
for j = 1:numel(R.esc.obj)
    o = R.esc.obj(j);
    k = T(:, 2) == o.clase;
    pc(j) = any(hypot(T(k, 3) - o.x, T(k, 4) - o.y) < 0.35);
end
end

function res = prueba_tubos(res, tb, raiz, cfg_real)
% Prueba 21: casos de pruebas/generar_referencia_estacion.py con geo.tubos_pista, tramos_sin_tubo,
% dist_tubos, filtros_vision (dist_max_tubo_*) y VisionService.filtrar (decisión "lejos del tubo").
P = tb.pistas; if ~iscell(P), P = num2cell(P); end
n_t = 0; n_s = 0; et = 0; ed = 0; n_none = 0; n_pts = 0;
for k = 1:numel(P)
    c = P{k};
    pista = sim3d_leer_json(c.pista_json);
    T = sim3d_tubos('leer', pista);
    Tr = tubos_ref(c.tubos);
    if numel(T) ~= numel(Tr)
        n_t = n_t + 1;
    else
        for i = 1:numel(T)
            if ~isequal(size(T{i}), size(Tr{i})), n_t = n_t + 1; else, et = max(et, max(abs(T{i} - Tr{i}), [], 'all')); end
        end
    end
    S = sim3d_tubos('sin_tubo', pista);
    if ~isequal(S(:), double(c.sin_tubo(:))), n_s = n_s + 1; end
    Q = c.puntos;
    for i = 1:size(Q, 1)
        d = sim3d_tubos('dist', Q(i, 1), Q(i, 2), T);
        n_pts = n_pts + 1;
        if isnan(Q(i, 3))
            n_none = n_none + 1;
            if ~isempty(d), n_t = n_t + 1; end
        elseif isempty(d)
            n_t = n_t + 1;
        else
            ed = max(ed, abs(d - Q(i, 3)));
        end
    end
end
% umbrales dist_max_tubo_toma_m / dist_max_tubo_fuego_m
Lm = tb.limites; if ~iscell(Lm), Lm = num2cell(Lm); end
n_l = 0;
for k = 1:numel(Lm)
    c = Lm{k};
    cfg = cfg_vision(cfg_real, c.vision_json);
    Cv = sim3d_config(raiz, cfg);
    if ~mismo(Cv.vis.dist_toma, c.toma) || ~mismo(Cv.vis.dist_fuego, c.fuego), n_l = n_l + 1; end
end
% decisión "lejos del tubo" de VisionService.filtrar con la cámara real
F = tb.filtrar; if ~iscell(F), F = num2cell(F); end
n_f = 0; eg = 0; edf = 0; n_lejos = 0;
for k = 1:numel(F)
    c = F{k};
    cfg = cfg_vision(cfg_real, c.vision_json);
    cfg.pista = sim3d_leer_json(c.pista_json);
    Cv = sim3d_config(raiz, cfg);
    g = sim3d_camara('a_suelo', Cv.cam, c.uv(:), c.pose(:));
    eg = max(eg, max(abs(g(:) - c.suelo(:))));
    det = sim3d_detector('init', [], Cv, struct());
    [lejos, d] = sim3d_detector('tubo', det, c.cls, g(1), g(2));
    if lejos ~= c.lejos || isempty(d) ~= isempty(c.dist), n_f = n_f + 1; end
    if ~isempty(d) && ~isempty(c.dist), edf = max(edf, abs(d - c.dist)); end
    n_lejos = n_lejos + c.lejos;
end
ok = n_t == 0 && n_s == 0 && et < 1e-12 && ed < 1e-12 && n_l == 0 && n_f == 0 && eg < 1e-9 && edf < 1e-9;
res = anotar(res, 'tubos y filtro de tubo = estación (tubos_pista, dist_tubos, filtrar)', ok, sprintf(['%d pistas ' ...
    '(con y sin pista.tubos, huecos, valores inválidos): %d diferencias de tubos/distancia, %d de tramos_sin_tubo, ' ...
    'error máx. vértices %.2g m, distancia %.2g m (%d puntos, %d sin tubos); %d variantes de umbral: %d diferencias; ' ...
    '%d casos de filtrar (%d lejos del tubo): %d diferencias, suelo %.2g m, distancia %.2g m'], numel(P), n_t, n_s, ...
    et, ed, n_pts, n_none, numel(Lm), n_l, numel(F), n_lejos, n_f, eg, edf));
end

function cfg = cfg_vision(cfg, vision_json)
% la config real con los campos de vision del caso (texto JSON: null y listas como la estación)
v = sim3d_leer_json(['{"vision": ' vision_json '}']);
v = v.vision;
f = fieldnames(v);
for i = 1:numel(f), cfg.vision.(f{i}) = v.(f{i}); end
end

function ok = mismo(a, b)
ok = isequal(isempty(a), isempty(b)) && (isempty(a) || abs(a - b) < 1e-12);
end

function T = tubos_ref(v)
% lista de polilíneas de la referencia (jsondecode) -> cell de Nx2
T = {};
if iscell(v)
    T = cellfun(@double, reshape(v, 1, []), 'UniformOutput', false);
elseif ndims(v) == 3
    for k = 1:size(v, 1), T{end+1} = reshape(v(k, :, :), size(v, 2), size(v, 3)); end %#ok<AGROW>
end
end

function v = pct(x, p)
% percentil sin Statistics Toolbox (interpolación lineal, como numpy por defecto)
x = sort(x(:));
if isempty(x), v = NaN; return; end
r = 1 + (numel(x) - 1)*p/100;
i = floor(r); f = r - i;
v = x(i);
if i < numel(x), v = v + f*(x(i + 1) - x(i)); end
end

function res = anotar(res, nombre, ok, detalle)
res(end+1) = struct('nombre', nombre, 'ok', logical(ok), 'detalle', detalle);
fprintf('[%s] %s: %s\n', ternario(ok, ' OK ', 'FALLO'), nombre, detalle);
end

function E = cuadro_falso(R, esc, C, G)
% estado de dibujo a media misión, tomado del final de una corrida
L = R.log;
k = find(L.estado == 5, 1) + 500;
E.t = L.t(k); E.vel = 1; E.tiempo_real = true; E.factor_real = 1;
E.X = zeros(10, 2);
E.X(1:3, 1) = L.pos(:, k, 1); E.X(1:3, 2) = L.pos(:, k, 2); E.X(10, :) = 14.7;
E.sp = [L.sp(:, k, 1).'; L.sp(:, k, 2).']; E.fsm = [5 5]; E.bat = [16 16];
E.s = R.s_final; E.s.estado = 5; E.tel = R.tel_final; E.ga = 0; E.p = R.p; E.esc = esc; E.version = 1;
E.dh = L.dh(k); E.dz = 1; E.dh_min = 1.2;
E.estela = {repmat(E.X(1:3,1).', 400, 1), repmat(E.X(1:3,2).', 400, 1)};
E.huella = sim3d_camara('huella', C.cam, [E.X(1:3, 1); 0]);
E.lente = E.X(1:3, 1).' + G.malla.lente;
E.obs_ult = zeros(0, 4);
E.estado_real = zeros(1, numel(esc.obj));
E.err_obj = repmat({'0.03 m'}, 1, max(E.s.n_obj, 1));
E.n_pend = 0; E.n_falsos = 0; E.txt_vision = 'prueba'; E.txt_estacion = 'interna'; E.txt_tcp = 'TCP apagado';
end

function r = ternario(c, a, b)
if c, r = a; else, r = b; end
end
