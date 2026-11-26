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
%     .modo_mision            'dual' o 'solo_explorador' (mision.modo)
%     .dron_solo              0 (dual) o 1/2 (mision.dron_solo en solo explorador)
%   QD2P (struct): qd2_params_from_config(cfg) -> parámetro 'P' de los
%   bloques de QD2_Mission_Lib (alturas, d_min_h, pista, tolerancias,
%   modo_mision, dron_solo...). P no es ajustable: cambiar el modo o la pista
%   obliga a recompilar (Build).
%   Imprime una línea con el modo de misión (y un warning si mision.modo o
%   mision.dron_solo no son válidos -> se usa dual, o si es solo explorador
%   con la versión MPC, que solo está validado en PID).
%
%   Sello del build (26-nov): con el modelo cargado (InitFcn de Update
%   Diagram / Build / Connect, iniciar_tierra_lab) agrega una línea JSON a
%   <carpeta del .slx>/<modelo>.config_usada.jsonl (gitignored; últimas 30
%   más la última anterior al <modelo>.rt-win64 actual, la de su build):
%   hora (t_posix UTC), sha256 y texto del JSON usado, sha256 de lib/*.m +
%   qd2_cargar_config.m, ruta, modo y rb_ids. web/preflight_lab.py toma la
%   última línea anterior a la fecha del <modelo>.rt-win64 y compara
%   CONTENIDOS (git pull cambia las fechas; de la config, solo las claves
%   que lee el modelo). Si no se puede escribir, solo
%   un warning (qd2_cargar_config:sello): nunca detiene el modelo.
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
% QUARC lee ExtModeMexArgs como una lista de cadenas de MATLAB: van ENTRE COMILLAS, como en el
% original del Equipo2 ('-w -d %d -uri %u -URI_Host tcpip://localhost:18001'). Sin ellas,
% qc_get_model_arguments / qc_get_target_uri fallan con "This statement is incomplete."
% El InitFcn NO basta: QUARC lee el valor GUARDADO al abrir el modelo y al empezar el build,
% antes del InitFcn. Por eso construir_modelos lo guarda con comillas y lo comprueba en cada
% .slx de tierra (qd2_ext_mode_args_slx); aquí solo se mantiene al día con los puertos del JSON.
cfg.ext_mode_args = sprintf('''-w -d %%d -uri %%u -URI_Host %s -URI_Host2 %s''', ...
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
[P, aviso] = qd2_params_from_config(c);

% ---- Modo de misión (mision.modo / mision.dron_solo) ------------------------------
if P.modo_mision > 0.5
    ds = P.dron_solo;
    cfg.modo_mision = 'solo_explorador';
    cfg.dron_solo = ds;
    puertos = [cfg.puerto_dron1 cfg.puerto_dron2];
    fprintf(['qd2_cargar_config: MODO SOLO EXPLORADOR -> dron %d (rigid body %d, Stream Server :%d). ' ...
        'El dron %d no recibe arm/takeoff.\n'], ds, o.rb_ids(ds), puertos(ds), 3 - ds);
    if cfg.version_lib == 2
        warning('qd2_cargar_config:solo_mpc', ...
            'Modo solo explorador con la versión MPC: solo explorador está validado solo en PID.');
    end
else
    cfg.modo_mision = 'dual';
    cfg.dron_solo = 0;
    fprintf('qd2_cargar_config: modo dual (2 drones)\n');
end
if ~isempty(aviso)
    warning('qd2_cargar_config:modo', '%s', aviso);
end
% ---- Envolvente de las consignas xy (mision.limites_x / limites_y) ------------------
if any(isfinite([P.env_x P.env_y]))
    [lo, hi] = qd2_limites_xy(P);
    fprintf('qd2_cargar_config: envolvente de consignas x [%.2f %.2f], y [%.2f %.2f] m (mision.limites_x / limites_y)\n', ...
        lo(1), hi(1), lo(2), hi(2));
end

assignin('base', 'QD2CFG', cfg);
assignin('base', 'QD2P', P);

% ---- Aplicar al modelo (InitFcn) -------------------------------------------------
if nargin >= 1 && ~isempty(mdl) && bdIsLoaded(char(mdl))
    aplicar_modelo(char(mdl), cfg);
    try
        escribir_sello(char(mdl), cfg, raiz);
    catch e
        warning('qd2_cargar_config:sello', 'No pude escribir el sello del build: %s', e.message);
    end
end
end

% =========================================================================
function escribir_sello(mdl, cfg, raiz)
% Una línea JSON por llamada en <carpeta del .slx>/<mdl>.config_usada.jsonl (ver la ayuda).
archivo = get_param(mdl, 'FileName');
if isempty(archivo)
    return                                          % modelo sin guardar: no hay carpeta
end
sello = fullfile(fileparts(archivo), [mdl '.config_usada.jsonl']);
e = struct();
e.formato = 1;
e.t_posix = posixtime(datetime('now', 'TimeZone', 'UTC'));
e.fecha = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
e.modelo = mdl;
e.matlab = version('-release');
e.config = struct('ruta', cfg.archivo_json, 'sha256', sha256_archivo(cfg.archivo_json), ...
    'texto', texto_archivo(cfg.archivo_json));      % el preflight dice qué clave cambió
e.modo = cfg.modo_mision;
e.dron_solo = cfg.dron_solo;
e.rb_ids = cfg.optitrack.rb_ids;
e.archivo_motive = cfg.optitrack.archivo_motive;
d = dir(fullfile(raiz, 'lib', '*.m'));
rutas = [{'qd2_cargar_config.m'}, strcat('lib/', sort({d.name}))];
arch = cell(1, numel(rutas));
for k = 1:numel(rutas)
    arch{k} = struct('ruta', rutas{k}, 'sha256', sha256_archivo(fullfile(raiz, rutas{k})));
end
e.archivos = arch;                                  % celda -> arreglo JSON (aunque sea de 1)
lineas = {};
if exist(sello, 'file')
    fid = fopen(sello, 'r', 'n', 'UTF-8');
    if fid >= 0
        txt = fread(fid, inf, '*char')';
        fclose(fid);
        lineas = strtrim(splitlines(txt));
        lineas = lineas(~cellfun(@isempty, lineas));
        lineas = recortar_sello(lineas, fullfile(fileparts(archivo), [mdl '.rt-win64']));
    end
end
lineas{end + 1} = jsonencode(e);
fid = fopen(sello, 'w', 'n', 'UTF-8');
if fid < 0
    error('qd2_cargar_config:sello', 'No puedo escribir %s', sello);
end
fprintf(fid, '%s\n', lineas{:});
fclose(fid);
fprintf('qd2_cargar_config: sello %s (config sha256 %s...)\n', sello, e.config.sha256(1:min(12, end)));
end

function lineas = recortar_sello(lineas, rt)
% Últimas 29 líneas + la última anterior al .rt-win64 actual (la de su build, la que usa el preflight), aunque
% desde entonces haya habido muchos Connect / Update Diagram sin recompilar.
n = numel(lineas);
quedan = max(1, n - 28):n;
d = dir(rt);
if numel(d) == 1 && n > 0
    t_rt = posixtime(datetime(d.datenum, 'ConvertFrom', 'datenum', 'TimeZone', 'local'));
    for k = n:-1:1
        try
            s = jsondecode(lineas{k});
            t = double(s.t_posix);
        catch
            continue                                % línea ilegible: el preflight también la ignora
        end
        if t <= t_rt + 2
            quedan = unique([k quedan]);
            break
        end
    end
end
lineas = lineas(quedan);
end

function txt = texto_archivo(ruta)
txt = '';
fid = fopen(ruta, 'r', 'n', 'UTF-8');
if fid >= 0
    txt = fread(fid, inf, '*char')';
    fclose(fid);
end
end

function h = sha256_archivo(ruta)
fid = fopen(ruta, 'r');
if fid < 0
    h = '';
    return
end
b = fread(fid, inf, '*uint8');
fclose(fid);
md = java.security.MessageDigest.getInstance('SHA-256');
if ~isempty(b)
    md.update(typecast(b, 'int8'));
end
h = sprintf('%02x', typecast(md.digest(), 'uint8'));
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
