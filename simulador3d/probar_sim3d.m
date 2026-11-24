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
%   8. Escenario 'web' = objetos de la cámara sintética de la UI (mock_world).
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
%  18. Modo SOLO EXPLORADOR (PID) con la pista del laboratorio y el dron 2 fuera de
%      OptiTrack: DONE, recorrido completo, el dron 2 nunca arma ni sube, y(64) = 1,
%      >= 3 de 4 objetos detectados y 0 visitados (no hay respondedor).
%  19. Solo explorador con el dron 2 y el dron 1 (no usado) RASTREADO en el piso debajo de
%      la N: el guard nunca se activa y la misión termina (DONE).
%   Las pruebas 3-17 usan la config real con mision.modo = "dual" y la N de 4 vértices
%   fija (no dependen del modo ni de la pista del día en config/estacion.json).
%   La prueba TCP con Python está en prueba_cliente_puente.py (ver README).

if nargin < 1, con_capturas = true; end
aqui = fileparts(mfilename('fullpath'));
addpath(aqui);
raiz = fileparts(aqui);
addpath(fullfile(raiz, 'simulink_qdrone2', 'lib'));
res = struct('nombre', {}, 'ok', {}, 'detalle', {});
base = struct('mostrar', false, 'tiempo_real', false, 'usar_tcp', false, 'auto_start', 3, ...
    'graficas_finales', false, 't_final', 300, 'verbose', false);
% Config de las pruebas: la real con modo dual y la N de 4 vértices fija (las pruebas no
% dependen de mision.modo ni de la pista del día en config/estacion.json).
cfg_real = sim3d_leer_json(fileread(fullfile(raiz, 'config', 'estacion.json')));
cfg_dual = cfg_real;
cfg_dual.mision.modo = 'dual';
if isfield(cfg_dual.mision, 'dron_solo'), cfg_dual.mision = rmfield(cfg_dual.mision, 'dron_solo'); end
cfg_dual.pista.vertices = [-1.2 1.2; 1.2 1.2; -1.2 -1.2; 1.2 -1.2];
base.config = cfg_dual;
% Modo solo explorador con la pista del laboratorio (23-nov)
cfg_solo = cfg_real;
cfg_solo.mision.modo = 'solo_explorador';
cfg_solo.mision.dron_solo = 1;
cfg_solo.pista.vertices = [1.02 0.41; 0.99 1.20; -0.90 1.10; -0.27 0.29; 0.47 -0.25; 0.99 -1.01; -1.02 -1.20];

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
% ---- 18. solo explorador PID: pista del laboratorio, dron 2 fuera de OptiTrack
o = base; o.config = cfg_solo; o.escenario = 'escenarios/escenario_solo_lab.json';
if con_capturas
    o.capturas = [45 45.1]; o.capturas_vistas = [1 2]; o.graficas_finales = true; o.prefijo_capturas = 'prueba_solo';
end
R5 = sim3d_correr(o);
m = R5.met; L = R5.log; s5 = R5.s_final;
kt = ~isnan(L.y64);
z_aus = max(L.pos(3, :, 2));
ok = R5.modo.solo && m.estado_final == 8 && s5.n_wp == 35 && s5.wp_idx == s5.n_wp && s5.explorer_done == 1 ...
    && z_aus == 0 && max(L.arm(2, :)) == 0 && max(L.takeoff(2, :)) == 0 && all(L.explorer_id == 1) ...
    && any(kt) && all(L.y64(kt) == 1) && m.reales_detectados >= 3 && m.n_visitados == 0 && m.reales_visitados == 0;
res = anotar(res, 'solo explorador PID (pista del lab, dron 2 fuera)', ok, sprintf(['DONE en %.1f s sim, %d/%d wp, ' ...
    'detectados %d/%d (%d objetivos, 0 visitados = %d), dron 2: z máx. %.3f m, arm máx. %d, takeoff máx. %d; ' ...
    'y(64) = 1 en %d/%d mensajes'], m.t_sim, s5.wp_idx, s5.n_wp, m.reales_detectados, m.n_reales, m.n_objetivos, ...
    m.n_visitados == 0, z_aus, max(L.arm(2, :)), max(L.takeoff(2, :)), sum(L.y64(kt) == 1), sum(kt)));
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
