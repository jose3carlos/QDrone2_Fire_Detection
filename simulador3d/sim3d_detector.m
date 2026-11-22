function [det, info] = sim3d_detector(accion, det, varargin)
%SIM3D_DETECTOR Detector simulado de la cámara del explorador + clusters.
%
%   det = sim3d_detector('init', [], C, opc)
%   [det, info] = sim3d_detector('cuadro', det, C, esc, x_real, pose_med, t, opc)
%       x_real   : estado real del explorador [x y z vx vy vz roll pitch yaw]
%       pose_med : pose medida por OptiTrack [x y z yaw] (lo que usa la estación)
%   T = sim3d_detector('objetivos', det)   -> Kx5 [id clase x y conf] publicados
%   det = sim3d_detector('agregar', det, clase, x, y, conf)   una observación al Clusterer
%
%   Cadena (igual a la de la estación, ARQUITECTURA §7):
%   1. Cada objeto real se proyecta con la pose REAL de la cámara (incluye
%      roll y pitch del dron y un error del soporte, opc.sesgo_montaje_deg)
%      -> píxel de la caja.
%   2. Si cae dentro de la imagen y su tamaño aparente es suficiente, se
%      detecta con probabilidad p_deteccion (menor cerca del borde), con
%      ruido de píxel, confusión de clase y confianza aleatoria.
%   3. Además hay falsos positivos con probabilidad p_falso por cuadro.
%   4. El píxel se geolocaliza como la estación: rayo con la pose MEDIDA,
%      solo yaw (roll = pitch = 0), intersecado con z = 0.02 m. Por eso la
%      inclinación del dron produce el mismo error que en el sistema real.
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
        info = [];
    case 'cuadro'
        [det, info] = cuadro(det, varargin{:});
    case 'objetivos'
        det = objetivos(det);
        info = [];
    case 'agregar'          % det = sim3d_detector('agregar', det, clase, x, y, conf)
        det = agregar(det, varargin{:});
        info = [];
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
% actitud real del dron + error del soporte de la cámara (desconocido para la estación)
sb = deg2rad(opc.sesgo_montaje_deg);
R_wb = rot_zyx(xr(9), xr(8), xr(7))*rot_zyx(0, sb(2), sb(1));
pos = xr(1:3);
obs = zeros(0, 4);
vistos = [];
nobj = numel(esc.obj);
if nobj > 0
    P = [[esc.obj.x]; [esc.obj.y]; 0.5*[esc.obj.h]];
    [uv, del] = sim3d_camara('proyectar', cam, P, pos, R_wb);
    [~, p_c] = sim3d_camara('pose', cam, pos, R_wb);
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
        g = sim3d_camara('a_suelo', cam, [uu; vv], pm);
        if any(~isfinite(g)), continue; end
        obs(end+1, :) = [cls g(1) g(2) 0.55 + 0.4*rand(rs)]; %#ok<AGROW>
    end
end
if rand(rs) < opc.p_falso
    uu = rand(rs)*(cam.W - 1); vv = rand(rs)*(cam.H - 1);
    g = sim3d_camara('a_suelo', cam, [uu; vv], pm);
    if all(isfinite(g))
        obs(end+1, :) = [double(rand(rs) < 0.5) g(1) g(2) 0.5 + 0.15*rand(rs)];
        det.n_falsos = det.n_falsos + 1;
    end
end
for k = 1:size(obs, 1)
    if obs(k, 4) >= det.conf_min
        det = agregar(det, obs(k, 1), obs(k, 2), obs(k, 3), obs(k, 4));
        det.n_obs = det.n_obs + 1;
    end
end
det.vistos = vistos;
det.ultimas = obs;
info.n = size(obs, 1);
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
