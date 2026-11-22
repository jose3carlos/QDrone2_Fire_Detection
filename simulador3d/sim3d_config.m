function C = sim3d_config(raiz)
%SIM3D_CONFIG Lee config/estacion.json y arma parámetros de misión, cámara y visión.
%   C = sim3d_config(raiz)
%   C.cfg  : estacion.json decodificado
%   C.p    : qd2_params_from_config(cfg) (pista, alturas, d_min_h, ...)
%   C.cam  : modelo de cámara igual al de la estación (web/app/geo.py)
%   C.vis  : radio de cluster, mínimo de observaciones y confianza mínima
%   C.puerto : red.puerto_puente

if isempty(which('qd2_params_from_config'))
    addpath(fullfile(raiz, 'simulink_qdrone2', 'lib'));
end
ruta = fullfile(raiz, 'config', 'estacion.json');
if exist(ruta, 'file')
    cfg = jsondecode(fileread(ruta));
else
    warning('sim3d:config', 'No existe %s: se usan los valores por defecto.', ruta);
    cfg = struct();
end
C.cfg = cfg;
C.ruta_cfg = ruta;
C.p = qd2_params_from_config(cfg);

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
C.vis = vis;

C.puerto = 18700;
if isfield(cfg, 'red') && isfield(cfg.red, 'puerto_puente')
    C.puerto = double(cfg.red.puerto_puente);
end
end
