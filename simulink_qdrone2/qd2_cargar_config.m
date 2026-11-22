function [cfg, P] = qd2_cargar_config(mdl, ruta_json)
%QD2_CARGAR_CONFIG Lee config/estacion.json para los modelos Simulink de tierra.
%   qd2_cargar_config()          crea QD2CFG y QD2P en el workspace base.
%   qd2_cargar_config(mdl)       además aplica la configuración al modelo
%                                cargado 'mdl' (archivos .cal/.motive del
%                                bloque OptiTrack Trackables). Es lo que
%                                llama el InitFcn de QD2_MissionCtrl_Dual_*.
%   [cfg, P] = qd2_cargar_config(..., ruta_json)  usa otro archivo JSON.
%
%   config/estacion.json es la ÚNICA fuente de verdad (compartida con la UI
%   web). Esta función solo la lee; nunca la escribe.
%
%   QD2CFG (struct, workspace base):
%     .version_control        'PID' / 'MPC' (informativo)
%     .optitrack.rb_ids       [rb_dron1 rb_dron2] -> rigid_body_ids del
%                             bloque OptiTrack Trackables (parámetro evaluado)
%     .optitrack.bias_d1/d2   [0 0 bias_z] -> bloque Bias de cada
%                             Measurement Conditioning (altura del pivot)
%     .optitrack.archivo_cal / archivo_motive  rutas completas ('' = dejar
%                             las del modelo del Equipo2)
%     .uri_dron1/.uri_dron2   tcpip://localhost:1800i (Stream Server de cada dron)
%     .uri_puente             tcpip://localhost:18700 (PYTHON BRIDGE)
%     .ext_mode_args          argumentos de external mode (-URI_Host/-URI_Host2)
%     .version_lib            1 = PID, 2 = MPC (según el modelo que la llame)
%   QD2P (struct): qd2_params_from_config(cfg) -> parámetro 'P' de los
%   bloques de QD2_Mission_Lib (alturas, d_min_h, pista, tolerancias...).
%
%   Nota sobre OptiTrack Trackables (QUARC): 'rigid_body_ids' es un
%   parámetro evaluado, así que acepta directamente QD2CFG.optitrack.rb_ids.
%   'calibration_file' y 'rigid_body_file' son parámetros literales (no
%   evaluados, "&" en la máscara): no aceptan variables. Por eso esta
%   función los escribe con set_param desde el InitFcn cuando el JSON
%   trae rutas (si vienen vacías se conservan las del modelo).

raiz = fileparts(mfilename('fullpath'));           % .../simulink_qdrone2
dir_lib = fullfile(raiz, 'lib');
if ~contains(path, dir_lib)
    addpath(dir_lib);
end
if nargin < 2 || isempty(ruta_json)
    ruta_json = fullfile(fileparts(raiz), 'config', 'estacion.json');
end
if ~exist(ruta_json, 'file')
    error('qd2_cargar_config:sin_json', 'No encuentro %s', ruta_json);
end
c = jsondecode(fileread(ruta_json));

cfg = struct();
cfg.archivo_json = ruta_json;
cfg.version_control = leer(c, {'version_control'}, 'PID');

% ---- Red ------------------------------------------------------------------
p1 = leer(c, {'red', 'puerto_dron1'}, 18001);
p2 = leer(c, {'red', 'puerto_dron2'}, 18002);
pp = leer(c, {'red', 'puerto_puente'}, 18700);
cfg.puerto_dron1 = double(p1);
cfg.puerto_dron2 = double(p2);
cfg.puerto_puente = double(pp);
cfg.uri_dron1 = sprintf('tcpip://localhost:%d', cfg.puerto_dron1);
cfg.uri_dron2 = sprintf('tcpip://localhost:%d', cfg.puerto_dron2);
cfg.uri_puente = sprintf('tcpip://localhost:%d', cfg.puerto_puente);
cfg.ext_mode_args = sprintf('-w -d %%d -uri %%u -URI_Host %s -URI_Host2 %s', ...
    cfg.uri_dron1, cfg.uri_dron2);

% ---- OptiTrack --------------------------------------------------------------
o = struct();
o.carpeta = char(leer(c, {'optitrack', 'carpeta'}, ''));
o.rb_ids = double([leer(c, {'optitrack', 'rigid_body_dron1'}, 1), ...
                   leer(c, {'optitrack', 'rigid_body_dron2'}, 2)]);
o.bias_d1 = [0 0 double(leer(c, {'optitrack', 'bias_z_dron1'}, -0.07))];
o.bias_d2 = [0 0 double(leer(c, {'optitrack', 'bias_z_dron2'}, -0.07))];
o.archivo_cal = ruta_completa(o.carpeta, char(leer(c, {'optitrack', 'archivo_cal'}, '')));
o.archivo_motive = ruta_completa(o.carpeta, char(leer(c, {'optitrack', 'archivo_motive'}, '')));
if numel(unique(o.rb_ids)) < 2
    % Error (no aviso): con el mismo rigid body los dos drones recibirían la
    % misma pose y la separación/roles no tendrían sentido.
    error('qd2_cargar_config:rb', ...
        'Los rigid body de dron 1 y dron 2 son iguales (%d). Corrige optitrack.rigid_body_dron1/2 en %s.', ...
        o.rb_ids(1), ruta_json);
end
cfg.optitrack = o;

% ---- Versión (según el modelo que llama) --------------------------------------
cfg.version_lib = 1;
if strcmpi(cfg.version_control, 'MPC'), cfg.version_lib = 2; end
if nargin >= 1 && ~isempty(mdl) && contains(char(mdl), 'MPC')
    cfg.version_lib = 2;
elseif nargin >= 1 && ~isempty(mdl) && contains(char(mdl), 'PID')
    cfg.version_lib = 1;
end

% ---- Parámetros de la librería -------------------------------------------------
P = qd2_params_from_config(c);

assignin('base', 'QD2CFG', cfg);
assignin('base', 'QD2P', P);

% ---- Aplicar al modelo (InitFcn) -------------------------------------------------
if nargin >= 1 && ~isempty(mdl) && bdIsLoaded(char(mdl))
    aplicar_modelo(char(mdl), cfg);
end
end

% =========================================================================
function aplicar_modelo(mdl, cfg)
bl = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
    'BlockType', 'S-Function', 'FunctionName', 'optitrack_rigid_body_position_block');
for i = 1:numel(bl)
    try
        if ~isempty(cfg.optitrack.archivo_cal) && ...
                ~strcmp(get_param(bl{i}, 'calibration_file'), cfg.optitrack.archivo_cal)
            set_param(bl{i}, 'calibration_file', cfg.optitrack.archivo_cal);
            fprintf('qd2_cargar_config: calibration_file <- %s\n', cfg.optitrack.archivo_cal);
        end
        if ~isempty(cfg.optitrack.archivo_motive) && ...
                ~strcmp(get_param(bl{i}, 'rigid_body_file'), cfg.optitrack.archivo_motive)
            set_param(bl{i}, 'rigid_body_file', cfg.optitrack.archivo_motive);
            fprintf('qd2_cargar_config: rigid_body_file <- %s\n', cfg.optitrack.archivo_motive);
        end
    catch e
        warning('qd2_cargar_config:optitrack', ...
            'No pude escribir las rutas de Motive en %s: %s', bl{i}, e.message);
    end
end
% Argumentos de external mode (-URI_Host / -URI_Host2) según los puertos del JSON
try
    if ~strcmp(get_param(mdl, 'ExtModeMexArgs'), cfg.ext_mode_args)
        set_param(mdl, 'ExtModeMexArgs', cfg.ext_mode_args);
        fprintf('qd2_cargar_config: ExtModeMexArgs <- %s\n', cfg.ext_mode_args);
    end
catch e
    warning('qd2_cargar_config:extmode', 'No pude actualizar ExtModeMexArgs: %s', e.message);
end
end

function v = leer(s, campos, defecto)
v = defecto;
for k = 1:numel(campos)
    if isstruct(s) && isfield(s, campos{k})
        s = s.(campos{k});
    else
        return
    end
end
if ~isempty(s)
    v = s;
end
end

function r = ruta_completa(carpeta, archivo)
r = archivo;
if isempty(archivo)
    return
end
es_absoluta = numel(archivo) > 2 && (archivo(2) == ':' || startsWith(archivo, '\\') || startsWith(archivo, '/'));
if ~es_absoluta && ~isempty(carpeta)
    r = fullfile(carpeta, archivo);
end
end
