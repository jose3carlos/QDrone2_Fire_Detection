function [p, aviso] = qd2_params_from_config(cfg)
%QD2_PARAMS_FROM_CONFIG qd2_params() + valores de config/estacion.json.
%   p = qd2_params_from_config()            lee config/estacion.json del repo
%   p = qd2_params_from_config(ruta_json)
%   p = qd2_params_from_config(cfg_struct)  (ya decodificado con jsondecode)
%   [p, aviso] = qd2_params_from_config(...)   aviso = '' o el texto del
%                                              problema con mision.modo / dron_solo
%
%   Solo MATLAB (usa jsondecode): llamarla en InitFcn / en el script de
%   construcción y pasar 'p' a los bloques MATLAB Function como parámetro
%   (o usar sus valores para generar constantes). No es para codegen.
%   Claves usadas: mision.{z_explorador, z_respondedor, t_mark, d_min_h,
%   dz_min, espaciado_waypoints, tol_xy, tol_z, auto_land, limites_xy,
%   modo, dron_solo}, pista.vertices (máx. 16) y vision.radio_cluster_m.
%
%   Modo de misión (mismas reglas que config.mision_modo de la estación):
%     mision.modo      "dual" | "solo_explorador" (sin distinguir mayúsculas,
%                      sin espacios a los lados). Ausente, null, [] o "" ->
%                      dual sin aviso. Otro texto o algo que no es texto ->
%                      dual con aviso.
%     mision.dron_solo 1 | 2 (solo con modo solo_explorador). Ausente, null o
%                      [] -> 1. Otro valor (0, 3, 1.5, "1", true, NaN, [1 2])
%                      -> dual con aviso.
%   Resultado: p.modo_mision (0 dual, 1 solo) y p.dron_solo (1 o 2; en dual
%   queda el valor por defecto 1, que no se usa). Basura = dual: con un solo
%   dron, dual espera los DOS rastreados y nunca despega.

p = qd2_params();
aviso = '';
if nargin < 1 || isempty(cfg)
    raiz = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    cfg = fullfile(raiz, 'config', 'estacion.json');
end
if ischar(cfg) || isstring(cfg)
    cfg = jsondecode(fileread(char(cfg)));
end
if isfield(cfg, 'mision')
    m = cfg.mision;
    p = copiar(p, m, 'z_explorador', 'z_E');
    p = copiar(p, m, 'z_respondedor', 'z_R');
    p = copiar(p, m, 't_mark', 't_mark');
    p = copiar(p, m, 'd_min_h', 'd_min_h');
    p = copiar(p, m, 'dz_min', 'dz_min');
    p = copiar(p, m, 'espaciado_waypoints', 'wp_spacing');
    p = copiar(p, m, 'tol_xy', 'tol_xy');
    p = copiar(p, m, 'tol_z', 'tol_z');
    if isfield(m, 'auto_land')
        p.auto_land = double(logical(m.auto_land));
    end
    if isfield(m, 'limites_xy') && numel(m.limites_xy) == 2
        p.xy_lim = reshape(double(m.limites_xy), 1, 2);
    end
end
if isfield(cfg, 'pista') && isfield(cfg.pista, 'vertices')
    V = double(cfg.pista.vertices);
    if size(V, 2) == 2 && size(V, 1) >= 2
        nv = min(size(V, 1), p.max_vertices);
        p.vertices = zeros(p.max_vertices, 2);
        p.vertices(1:nv, :) = V(1:nv, :);
        p.n_vertices = nv;
    end
end
if isfield(cfg, 'vision') && isfield(cfg.vision, 'radio_cluster_m')
    p.radio_cluster = double(cfg.vision.radio_cluster_m);
end
m = struct();
if isfield(cfg, 'mision') && isstruct(cfg.mision)
    m = cfg.mision;
end
[modo, ds, aviso] = modo_mision(m);
p.modo_mision = modo;
p.dron_solo = ds;
end

function [modo, ds, aviso] = modo_mision(m)
% Reglas de mision.modo / mision.dron_solo (contrato §1.2, = config.mision_modo de la estación).
modo = 0;
ds = 1;
aviso = '';
if ~isfield(m, 'modo')
    return;
end
v = m.modo;
if (ischar(v) && (isempty(v) || isrow(v))) || (isstring(v) && isscalar(v))
    s = lower(strtrim(char(v)));
    if isempty(s) && isempty(char(v))
        return;                                   % "" = no definido
    elseif strcmp(s, 'dual')
        return;
    elseif ~strcmp(s, 'solo_explorador')
        aviso = sprintf('mision.modo desconocido (''%s''): se usa dual', char(v));
        return;
    end
elseif isempty(v) && (isnumeric(v) || iscell(v))
    return;                                       % null o [] = no definido
else
    aviso = sprintf('mision.modo desconocido (''%s''): se usa dual', texto_valor(v));
    return;
end
% solo explorador
if ~isfield(m, 'dron_solo')
    modo = 1;
    return;
end
d = m.dron_solo;
if isempty(d) && (isnumeric(d) || iscell(d))
    modo = 1;                                     % null o [] -> dron 1
    return;
end
if isnumeric(d) && ~islogical(d) && isscalar(d) && isreal(d) && isfinite(d) && (d == 1 || d == 2)
    modo = 1;
    ds = double(d);
    return;
end
aviso = sprintf('mision.dron_solo inválido (''%s''): se usa dual', texto_valor(d));
end

function t = texto_valor(v)
% Texto corto de un valor de la config para el aviso.
if ischar(v) || isstring(v)
    t = char(v);
    return;
end
try
    t = jsonencode(v);
catch
    t = class(v);
end
end

function p = copiar(p, m, clave, campo)
if isfield(m, clave) && ~isempty(m.(clave))
    p.(campo) = double(m.(clave));
end
end
