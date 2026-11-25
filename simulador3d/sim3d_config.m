function C = sim3d_config(raiz, cfg)
%SIM3D_CONFIG Lee config/estacion.json y arma parámetros de misión, cámara y visión.
%   C = sim3d_config(raiz)
%   C = sim3d_config(raiz, cfg)   usa el struct cfg (ya decodificado) en lugar del archivo
%   C.cfg  : estacion.json decodificado
%   C.p    : qd2_params_from_config(cfg) (pista, alturas, d_min_h, ...)
%   C.ruta : ruta de vuelo (pista.vertices, la de Simulink) · C.tubos: tubos reales (geo.tubos_pista;
%            {} sin pista.vertices ni pista.tubos, como la estación)
%   C.cam  : modelo de cámara igual al de la estación (web/app/geo.py), con el pivote de Motive
%   C.vis  : radio de cluster, mínimo de observaciones, confianza mínima, uso de la actitud
%            (usar_actitud, signos, offset, inclinación máxima) y distancia máxima al tubo de
%            tomas y fuegos (dist_toma, dist_fuego) como web/app/config.py
%   C.puerto : red.puerto_puente

if isempty(which('qd2_params_from_config'))
    addpath(fullfile(raiz, 'simulink_qdrone2', 'lib'));
end
ruta = fullfile(raiz, 'config', 'estacion.json');
if nargin >= 2 && isstruct(cfg)
    ruta = '(struct dado)';
elseif exist(ruta, 'file')
    cfg = sim3d_leer_json(fileread(ruta));       % [] y null distintos, como config.py
else
    warning('sim3d:config', 'No existe %s: se usan los valores por defecto.', ruta);
    cfg = struct();
end
C.cfg = cfg;
C.ruta_cfg = ruta;
C.p = qd2_params_from_config(cfg);

% ---------------- Pista: ruta de vuelo y tubos REALES ---------------------
% C.ruta  : pista.vertices = ruta de VUELO que compila Simulink (C.p, máx. 16 vértices)
% C.tubos : tubos reales en el piso, cell de polilíneas Nx2, con las reglas de geo.tubos_pista:
%           pista.tubos (polilíneas válidas, >= 2 puntos) o, si no hay, la ruta partida en
%           pista.tramos_sin_tubo. Es lo que usa el filtro de tomas del detector interno (como
%           vision_service.filtrar). Sin pista.vertices ni pista.tubos en la config queda VACÍO,
%           igual que en la estación (tubos_pista da [] y no filtra): la N por defecto de C.ruta
%           solo se dibuja (sim3d_escenario la usa como esc.tubos para el dibujo y la colocación).
% C.tubos_reales : true si salen de pista.tubos (false = ruta partida en los huecos)
pista = struct();
if isfield(cfg, 'pista') && isstruct(cfg.pista), pista = cfg.pista; end
C.ruta = C.p.vertices(1:C.p.n_vertices, :);
C.tubos = sim3d_tubos('leer', pista);
C.sin_tubo = sim3d_tubos('sin_tubo', pista);
C.tubos_reales = false;
if isfield(pista, 'tubos')
    solo_tubos.tubos = pista.tubos;
    C.tubos_reales = ~isempty(sim3d_tubos('leer', solo_tubos));
end

% ---------------- Cámara (misma convención que web/app/geo.py) ------------
cam.W = 640; cam.H = 480;
cam.hfov = 69.4; cam.vfov = 42.5;
cam.rot = 0;
cam.offset = [0.20; 0.0; -0.05];
cc = struct();
if isfield(cfg, 'camara'), cc = cfg.camara; end
if isfield(cc, 'resolucion') && numel(cc.resolucion) == 2
    cam.W = double(cc.resolucion(1)); cam.H = double(cc.resolucion(2));
end
if isfield(cc, 'hfov_deg'), cam.hfov = double(cc.hfov_deg); end
if isfield(cc, 'vfov_deg'), cam.vfov = double(cc.vfov_deg); end
if isfield(cc, 'rotacion_montaje'), cam.rot = double(cc.rotacion_montaje); end
if isfield(cc, 'offset_camara_m') && numel(cc.offset_camara_m) == 3
    cam.offset = double(cc.offset_camara_m(:));
end
% Intrínsecos: camara.intrinsecos["WxH"] > vision.intrinsecos_nominales["WxH"] > HFOV/VFOV
clave = matlab.lang.makeValidName(sprintf('%dx%d', cam.W, cam.H));   % jsondecode: "640x480" -> x640x480
cam.fx = (cam.W/2)/tand(cam.hfov/2);
cam.fy = (cam.H/2)/tand(cam.vfov/2);
cam.cx = (cam.W - 1)/2;
cam.cy = (cam.H - 1)/2;
cam.origen = 'HFOV/VFOV';
fuentes = {{'camara', 'intrinsecos'}, {'vision', 'intrinsecos_nominales'}};
for k = 1:numel(fuentes)
    sec = fuentes{k}{1}; sub = fuentes{k}{2};
    if isfield(cfg, sec) && isfield(cfg.(sec), sub) && isstruct(cfg.(sec).(sub)) ...
            && isfield(cfg.(sec).(sub), clave)
        d = cfg.(sec).(sub).(clave);
        if all(isfield(d, {'fx', 'fy', 'cx', 'cy'}))
            cam.fx = d.fx; cam.fy = d.fy; cam.cx = d.cx; cam.cy = d.cy;
            cam.origen = [sec '.' sub];
            break;
        end
    end
end
% Rotación cuerpo <- cámara: montaje hacia abajo, arriba de la imagen = +x del dron
R_BODY_CAM0 = [0 -1 0; -1 0 0; 0 0 -1];
a = deg2rad(cam.rot);
cam.R_bc = R_BODY_CAM0*[cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
cam.z_plano = 0.02;            % m, plano de intersección de la estación
% Pivote de Motive (geo.pivote_de_config): la pose es el punto corregido pivote + bias_z·ẑ, así que
% el pivote queda -bias_z m ARRIBA de ella. Promedio de bias_z_dron1/2; un valor no numérico -> 0.07.
piv = [0.07 0.07];
if isfield(cfg, 'optitrack') && isstruct(cfg.optitrack)
    nb = {'bias_z_dron1', 'bias_z_dron2'};
    for k = 1:2
        if isfield(cfg.optitrack, nb{k})
            v = cfg.optitrack.(nb{k});
            if es_num(v), piv(k) = -double(v); end
        end
    end
end
cam.pivote = mean(piv);
C.cam = cam;

% ---------------- Visión ---------------------------------------------------
vis.radio = 0.25; vis.min_obs = 3; vis.conf_min = 0.5;
if isfield(cfg, 'vision')
    v = cfg.vision;
    if isfield(v, 'radio_cluster_m'), vis.radio = double(v.radio_cluster_m); end
    if isfield(v, 'min_observaciones'), vis.min_obs = double(v.min_observaciones); end
    if isfield(v, 'confianza_min'), vis.conf_min = double(v.confianza_min); end
end
% Igual que _vision_gate de web/app/server.py: solo se geolocaliza con el explorador
% por encima de web.z_min_deteccion (0.4 m por defecto en web/app/config.py) y en MISSION o PAUSED.
vis.z_min = 0.4;
vis.estados = [5 10];
if isfield(cfg, 'web') && isstruct(cfg.web) && isfield(cfg.web, 'z_min_deteccion')
    vis.z_min = double(cfg.web.z_min_deteccion);
end
% Actitud del explorador en la geolocalización, con las MISMAS reglas que actitud_vision y
% filtros_vision de web/app/config.py:
%   usar_actitud       : solo true/false (otro valor -> true)
%   actitud_signos     : [roll pitch], cada uno ±1 (otro valor -> 1; elemento por elemento,
%                        también si el JSON mezcla tipos, p. ej. [true, -1] -> [1 -1])
%   actitud_offset_deg : [roll pitch], |a| <= 30 (otro valor -> 0); se guarda en rad
%   inclinacion_max_deg: ausente o no numérico -> 15; null, <= 0 o no finito -> [] (sin descarte).
%                        Un [] en el JSON es una lista (no un número) -> 15: sim3d_leer_json lo
%                        cambia por "[]" al leer el archivo; con un struct ya decodificado,
%                        [] numérico se toma como null.
vis.usar_actitud = true;
vis.signos = [1 1];
vis.offset_att = [0 0];
vis.incl_max_deg = 15;
if isfield(cfg, 'vision') && isstruct(cfg.vision)
    v = cfg.vision;
    if isfield(v, 'usar_actitud') && islogical(v.usar_actitud) && isscalar(v.usar_actitud)
        vis.usar_actitud = v.usar_actitud;
    end
    if isfield(v, 'actitud_signos')
        vis.signos = par(v.actitud_signos, [1 1], @(a) a == 1 || a == -1);
    end
    if isfield(v, 'actitud_offset_deg')
        vis.offset_att = deg2rad(par(v.actitud_offset_deg, [0 0], @(a) abs(a) <= 30));
    end
    if isfield(v, 'inclinacion_max_deg')
        a = v.inclinacion_max_deg;
        if isempty(a) && isnumeric(a)                 % null en el JSON
            vis.incl_max_deg = [];
        elseif isnumeric(a) && isscalar(a) && isreal(a) && ~islogical(a)
            if isfinite(a) && a > 0
                vis.incl_max_deg = double(a);
            else
                vis.incl_max_deg = [];
            end
        end
    end
end
% Filtro de distancia al tubo (filtros_vision de web/app/config.py): toma 0.25 m y fuego sin filtro
% por defecto; null, <= 0 o no finito -> [] (sin filtro); bool, texto o lista -> el valor por defecto
% (sim3d_leer_json cambia una lista por el texto "[]" al leer el archivo).
vis.dist_toma = 0.25;
vis.dist_fuego = [];
if isfield(cfg, 'vision') && isstruct(cfg.vision)
    vis.dist_toma = lim_filtro(cfg.vision, 'dist_max_tubo_toma_m', vis.dist_toma);
    vis.dist_fuego = lim_filtro(cfg.vision, 'dist_max_tubo_fuego_m', vis.dist_fuego);
end
C.vis = vis;

C.puerto = 18700;
if isfield(cfg, 'red') && isfield(cfg.red, 'puerto_puente')
    C.puerto = double(cfg.red.puerto_puente);
end
end

% =========================================================================
function ok = es_num(v)
% número real finito (no logical), como el isinstance(v, (int, float)) de Python sin bool
ok = isnumeric(v) && isscalar(v) && isreal(v) && isfinite(v);
end

function out = lim_filtro(v, clave, dflt)
% un umbral de filtros_vision: ausente -> dflt; null -> []; número real (no bool) -> él mismo si es
% finito y > 0, si no []; cualquier otra cosa -> dflt
out = dflt;
if ~isfield(v, clave), return; end
a = v.(clave);
if isempty(a) && isnumeric(a)
    out = [];
elseif isnumeric(a) && isscalar(a) && isreal(a)
    if isfinite(a) && a > 0
        out = double(a);
    else
        out = [];
    end
end
end

function out = par(val, dflt, valido)
% [a b] de estacion.json elemento por elemento (config.actitud_vision de la estación): si no es
% una lista de 2 elementos se usa dflt completo; un elemento que no es número real finito (bool,
% texto, null, lista) o que no cumple 'valido' -> el de dflt. jsondecode da un cell cuando el
% arreglo mezcla tipos ([true, -1], [-30, "a"]) y un logical cuando son todos bool.
out = dflt;
if isnumeric(val) || islogical(val)
    val = num2cell(val);
elseif ~iscell(val)
    return;                                       % texto, struct, ...
end
if numel(val) ~= 2
    return;
end
for k = 1:2
    a = val{k};
    if es_num(a) && ~islogical(a) && valido(double(a))
        out(k) = double(a);
    end
end
end
