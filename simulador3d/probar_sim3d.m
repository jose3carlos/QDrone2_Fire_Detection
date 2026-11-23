function res = probar_sim3d(con_capturas)
%PROBAR_SIM3D Pruebas sin ventana del simulador 3D (corre en -batch).
%   res = probar_sim3d()        corre todo y guarda PNG en capturas/
%   res = probar_sim3d(false)   sin capturas (más rápido)
%
%   1. Malla del QDrone 2 contra las medidas del manual.
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
%   La prueba TCP con Python está en prueba_cliente_puente.py (ver README).

if nargin < 1, con_capturas = true; end
aqui = fileparts(mfilename('fullpath'));
addpath(aqui);
raiz = fileparts(aqui);
addpath(fullfile(raiz, 'simulink_qdrone2', 'lib'));
res = struct('nombre', {}, 'ok', {}, 'detalle', {});
base = struct('mostrar', false, 'tiempo_real', false, 'usar_tcp', false, 'auto_start', 3, ...
    'graficas_finales', false, 't_final', 300, 'verbose', false);

% ---- 1. malla
M = sim3d_malla_qdrone2();
manual = [0.44258 0.4908 0.13553];
err = max(abs(M.medidas - manual));
res = anotar(res, 'malla QDrone 2 = medidas del manual', err < 1e-3, ...
    sprintf('malla %.4f x %.4f x %.4f m (manual 0.4426 x 0.4908 x 0.1355), %d caras + 4 hélices', ...
    M.medidas, sum(arrayfun(@(g) size(g.F, 1), M.grupos))));

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
    % ---- 8. escenario 'web'
    if isfield(ref, 'anomalias_web')
        aw = ref.anomalias_web;
        if ~iscell(aw), aw = num2cell(aw); end
        escw = sim3d_escenario(sim3d_opciones(struct('escenario', 'web')), C);
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
% ---- 6. tiempo real sin figura
close all force; drawnow; pause(1);     % que la limpieza de figuras anteriores no caiga en la medición
o = base; o.tiempo_real = true; o.t_final = 10; o.parar_en_done = false;
R4 = sim3d_correr(o);
f = R4.met.t_sim/R4.met.t_reloj;
res = anotar(res, 'ritmo de tiempo real (10 s, sin figura)', abs(f - 1) < 0.03, sprintf(['%.2f s sim en %.2f s de reloj ' ...
    '(factor %.3f, re-sincronizaciones %d)'], R4.met.t_sim, R4.met.t_reloj, f, R4.met.n_atrasos));

% ---- 7. costo de dibujo
C = sim3d_config(raiz);
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
