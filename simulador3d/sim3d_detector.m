function [det, info] = sim3d_detector(accion, det, varargin)
%SIM3D_DETECTOR Detector simulado de la cámara del explorador + clusters.
%
%   det = sim3d_detector('init', [], C, opc)
%   [det, info] = sim3d_detector('cuadro', det, C, esc, x_real, pose_med, t, opc)
%       x_real   : estado real del explorador [x y z vx vy vz roll pitch yaw]
%       pose_med : pose medida [x y z yaw] de OptiTrack, o [x y z yaw roll pitch] con el roll y
%                  el pitch que llegan a la estación en y(61:62) (NaN = y(63) = 0, sin actitud)
%   T = sim3d_detector('objetivos', det)   -> Kx5 [id clase x y conf] publicados
%   det = sim3d_detector('agregar', det, clase, x, y, conf)   una observación al Clusterer
%   [q6, act] = sim3d_detector('pose_geo', [], pose_med, C.vis)
%       pose_para_geo de web/app/vision_service.py: q6 = [x y z yaw roll pitch] con la que se
%       geolocaliza y act (.ok, .disponible, .roll, .pitch, .incl en rad, .inclinado)
%   [lejos, dist] = sim3d_detector('tubo', det, clase, x, y)
%       filtro de distancia al tubo de vision_service.filtrar: dist = distancia (m) al tubo REAL
%       más cercano (C.tubos = geo.tubos_pista; [] sin tubos) y lejos = true si pasa del límite de
%       su clase (vision.dist_max_tubo_toma_m / dist_max_tubo_fuego_m; [] = sin filtro)
%
%   Cadena (igual a la de la estación, ARQUITECTURA §7):
%   1. Cada objeto real se proyecta con la pose REAL de la cámara (roll, pitch y
%      yaw del dron, pivote de Motive y un error del soporte, opc.sesgo_montaje_deg,
%      que gira solo la cámara) -> píxel de la caja.
%   2. Si cae dentro de la imagen y su tamaño aparente es suficiente, se
%      detecta con probabilidad p_deteccion (menor cerca del borde), con
%      ruido de píxel, confusión de clase y confianza aleatoria.
%   3. Además hay falsos positivos con probabilidad p_falso por cuadro.
%   4. El píxel se geolocaliza como la estación (VisionService.process): rayo con la
%      pose MEDIDA y, si llega actitud y vision.usar_actitud = true, con el roll y el
%      pitch de y(61:62) corregidos con vision.actitud_signos / actitud_offset_deg
%      (sin actitud: nivelado), intersecado con z = 0.02 m. Si la inclinación supera
%      vision.inclinacion_max_deg la observación se descarta (no entra al cluster).
%      Luego el filtro de tubo de la estación (VisionService.filtrar con dist_tubos): una toma a
%      más de vision.dist_max_tubo_toma_m (0.25 m) del tubo REAL más cercano (pista.tubos, o la
%      ruta partida en pista.tramos_sin_tubo) se descarta; los fuegos, solo con
%      dist_max_tubo_fuego_m (null por defecto). El filtro de tamaño no se simula (sin cajas).
%      La estación no conoce el error del soporte: ese error queda en la posición.
%   5. Clusters con las reglas de web/app/geo.py (Clusterer): misma clase,
%      centroide más cercano dentro de radio_cluster_m, se publica con
%      min_observaciones y confianza >= confianza_min; ids 1..32.

switch accion
    case 'init'
        C = varargin{1};
        det.radio = C.vis.radio;
        det.min_obs = C.vis.min_obs;
        det.conf_min = C.vis.conf_min;
        det.max_obj = 32;
        det.next_id = 1;
        % clusters: [clase sx sy sconf n id]
        det.cl = zeros(0, 6);
        det.n_cuadros = 0;
        det.n_obs = 0;
        det.n_falsos = 0;
        det.vistos = [];           % índices de objetos reales en el último cuadro
        det.ultimas = zeros(0, 4); % [clase x y conf] del último cuadro
        det.n_con_actitud = 0;     % cuadros geolocalizados con roll/pitch
        det.n_nivelado = 0;        % cuadros geolocalizados nivelados (sin actitud o usar_actitud = false)
        det.n_desc_incl = 0;       % observaciones (conf >= confianza_min) descartadas por inclinación
        det.incl_max = 0;          % rad, inclinación máxima usada en la geolocalización
        det.err_obs = zeros(1, 0); % m, error horizontal de cada observación real que entró al cluster
        det.ult_act = [];          % actitud usada en el último cuadro (pose_geo)
        % filtro de distancia al tubo (vision_service.filtrar): tubos reales y límites por clase
        det.tubos = {};
        if isfield(C, 'tubos'), det.tubos = C.tubos; end
        det.lim_tubo = {[], 0.25};  % {fuego, toma}
        if isfield(C.vis, 'dist_toma'), det.lim_tubo = {C.vis.dist_fuego, C.vis.dist_toma}; end
        det.n_desc_tubo = 0;       % observaciones (conf >= confianza_min) descartadas por "lejos del tubo"
        det.desc_tubo_obj = zeros(1, 0);  % descartadas por tubo de cada objeto real (índice de esc.obj)
        det.obs_obj = zeros(1, 0);        % observaciones de cada objeto real que entraron al cluster
        det.dist_tubo_obs = zeros(1, 0);  % m, distancia al tubo de cada observación que entró al cluster
        info = [];
    case 'cuadro'
        [det, info] = cuadro(det, varargin{:});
    case 'objetivos'
        det = objetivos(det);
        info = [];
    case 'agregar'          % det = sim3d_detector('agregar', det, clase, x, y, conf)
        det = agregar(det, varargin{:});
        info = [];
    case 'pose_geo'         % [q6, act] = sim3d_detector('pose_geo', [], pose_med, C.vis)
        [det, info] = pose_geo(varargin{:});
    case 'tubo'             % [lejos, dist] = sim3d_detector('tubo', det, clase, x, y)
        [lejos, info] = filtro_tubo(det, varargin{:});
        det = lejos;
    otherwise
        error('sim3d:detector', 'Acción desconocida: %s', accion);
end
end

% =========================================================================
function [det, info] = cuadro(det, C, esc, xr, pm, ~, opc)
cam = C.cam;
% flujo aleatorio propio del simulador (det.rs); si no hay, el global
if isfield(det, 'rs') && ~isempty(det.rs)
    rs = det.rs;
else
    rs = RandStream.getGlobalStream();
end
det.n_cuadros = det.n_cuadros + 1;
% cámara REAL: actitud real del dron + error del soporte (gira solo la cámara; la estación no lo conoce)
sb = deg2rad(opc.sesgo_montaje_deg);
R_wb = sim3d_camara('rwb', xr(9), xr(7), xr(8));
R_m = rot_zyx(0, sb(2), sb(1));
pos = xr(1:3);
% pose con la que geolocaliza la estación (medida + actitud de la telemetría) y descarte por inclinación
[q, act] = pose_geo(pm, C.vis);
if act.ok
    det.n_con_actitud = det.n_con_actitud + 1;
    det.incl_max = max(det.incl_max, act.incl);
else
    det.n_nivelado = det.n_nivelado + 1;
end
obs = zeros(0, 5);           % [clase x y conf j] (j = objeto real, 0 = falso positivo)
vistos = [];
nobj = numel(esc.obj);
if nobj > 0
    P = [[esc.obj.x]; [esc.obj.y]; 0.5*[esc.obj.h]];
    [uv, del] = sim3d_camara('proyectar', cam, P, pos, R_wb, R_m);
    [~, p_c] = sim3d_camara('pose', cam, pos, R_wb, R_m);
    for j = 1:nobj
        if ~del(j), continue; end
        u = uv(1,j); v = uv(2,j);
        dist = norm(P(:,j) - p_c);
        tam = cam.fx*2*esc.obj(j).r/max(dist, 0.05);       % px, diámetro aparente
        m = 0.3*tam;                                        % basta con ver ~80 % de la pieza
        if u < -m || u > cam.W - 1 + m || v < -m || v > cam.H - 1 + m
            continue;
        end
        vistos(end+1) = j; %#ok<AGROW>
        pd = opc.p_deteccion*min(1, max(0, (tam - opc.tam_min_px)/opc.tam_min_px));
        borde = min([u, cam.W - 1 - u, v, cam.H - 1 - v]);
        if borde < tam/2
            pd = 0.6*pd;
        end
        if rand(rs) >= pd, continue; end
        cls = esc.obj(j).clase;
        if rand(rs) < opc.p_confusion, cls = 1 - cls; end
        uu = min(max(u + opc.ruido_px*randn(rs), 0), cam.W - 1);
        vv = min(max(v + opc.ruido_px*randn(rs), 0), cam.H - 1);
        g = sim3d_camara('a_suelo', cam, [uu; vv], q);
        if any(~isfinite(g)), continue; end
        obs(end+1, :) = [cls g(1) g(2) 0.55 + 0.4*rand(rs) j]; %#ok<AGROW>
    end
end
if rand(rs) < opc.p_falso
    uu = rand(rs)*(cam.W - 1); vv = rand(rs)*(cam.H - 1);
    g = sim3d_camara('a_suelo', cam, [uu; vv], q);
    if all(isfinite(g))
        obs(end+1, :) = [double(rand(rs) < 0.5) g(1) g(2) 0.5 + 0.15*rand(rs) 0];
        det.n_falsos = det.n_falsos + 1;
    end
end
for k = 1:size(obs, 1)
    if obs(k, 4) >= det.conf_min
        if act.inclinado
            % como la estación: demasiado inclinado, la proyección no es fiable y no entra al cluster
            det.n_desc_incl = det.n_desc_incl + 1;
            continue;
        end
        j = obs(k, 5);
        [lejos, dist] = filtro_tubo(det, obs(k, 1), obs(k, 2), obs(k, 3));
        if lejos
            % como la estación: "lejos del tubo" (una toma va abrazada al tubo real)
            det.n_desc_tubo = det.n_desc_tubo + 1;
            if j > 0
                if numel(det.desc_tubo_obj) < j, det.desc_tubo_obj(j) = 0; end
                det.desc_tubo_obj(j) = det.desc_tubo_obj(j) + 1;
            end
            continue;
        end
        det = agregar(det, obs(k, 1), obs(k, 2), obs(k, 3), obs(k, 4));
        det.n_obs = det.n_obs + 1;
        if ~isempty(dist), det.dist_tubo_obs(end+1) = dist; end
        if j > 0
            det.err_obs(end+1) = hypot(obs(k, 2) - esc.obj(j).x, obs(k, 3) - esc.obj(j).y);
            if numel(det.obs_obj) < j, det.obs_obj(j) = 0; end
            det.obs_obj(j) = det.obs_obj(j) + 1;
        end
    end
end
det.vistos = vistos;
det.ultimas = obs(:, 1:4);
det.ult_act = act;
info.n = size(obs, 1);
info.actitud = act;
end

function [lejos, dist] = filtro_tubo(det, cls, x, y)
% parte de distancia al tubo de VisionService.filtrar: dist_tubos(g, tubos) y límite de la clase
dist = [];
if isfield(det, 'tubos'), dist = sim3d_tubos('dist', x, y, det.tubos); end
lim = [];
if isfield(det, 'lim_tubo') && (cls == 0 || cls == 1), lim = det.lim_tubo{cls + 1}; end
lejos = ~isempty(dist) && ~isempty(lim) && dist > lim;
end

function [q, act] = pose_geo(pm, vis)
% pose_para_geo + descarte por inclinación de web/app/vision_service.py (VisionService.process).
% pm = [x y z yaw] o [x y z yaw roll pitch] con roll/pitch crudos de y(61:62) (NaN = sin actitud).
pm = pm(:).';
roll = NaN; pitch = NaN;
if numel(pm) >= 6
    roll = pm(5); pitch = pm(6);
end
act.disponible = isfinite(roll) && isfinite(pitch);
act.ok = logical(vis.usar_actitud) && act.disponible;
if act.ok
    % corregir_actitud: signo·(ángulo - offset)
    roll = vis.signos(1)*(roll - vis.offset_att(1));
    pitch = vis.signos(2)*(pitch - vis.offset_att(2));
    act.incl = acos(max(-1, min(1, cos(roll)*cos(pitch))));     % geo.inclinacion
else
    roll = 0; pitch = 0;
    act.incl = NaN;
end
act.roll = roll; act.pitch = pitch;
act.inclinado = act.ok && ~isempty(vis.incl_max_deg) && rad2deg(act.incl) > vis.incl_max_deg;
q = [pm(1:4) roll pitch];
end

function det = agregar(det, cls, x, y, conf)
% Igual que Clusterer.add de web/app/geo.py
best = 0; best_d = inf;
for i = 1:size(det.cl, 1)
    if det.cl(i, 1) ~= cls, continue; end
    n = det.cl(i, 5);
    d = hypot(det.cl(i, 2)/n - x, det.cl(i, 3)/n - y);
    if d <= det.radio && d < best_d
        best = i; best_d = d;
    end
end
if best == 0
    det.cl(end+1, :) = [cls 0 0 0 0 0];
    best = size(det.cl, 1);
end
det.cl(best, 2:5) = det.cl(best, 2:5) + [x y conf 1];
if det.cl(best, 6) == 0 && det.cl(best, 5) >= det.min_obs && det.next_id <= det.max_obj
    det.cl(best, 6) = det.next_id;
    det.next_id = det.next_id + 1;
end
end

function T = objetivos(det)
% Kx5 [id clase x y conf] de los clusters publicados, ordenados por id
k = det.cl(:, 6) > 0;
c = det.cl(k, :);
n = c(:, 5);
T = [c(:, 6) c(:, 1) c(:, 2)./n c(:, 3)./n c(:, 4)./n];
T = sortrows(T, 1);
end

function R = rot_zyx(yaw, pitch, roll)
cy = cos(yaw); sy = sin(yaw); cp = cos(pitch); sp = sin(pitch); cr = cos(roll); sr = sin(roll);
R = [cy -sy 0; sy cy 0; 0 0 1]*[cp 0 sp; 0 1 0; -sp 0 cp]*[1 0 0; 0 cr -sr; 0 sr cr];
end
