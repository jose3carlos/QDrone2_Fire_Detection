function p = qd2_params_from_config(cfg)
%QD2_PARAMS_FROM_CONFIG qd2_params() + valores de config/estacion.json.
%   p = qd2_params_from_config()            lee config/estacion.json del repo
%   p = qd2_params_from_config(ruta_json)
%   p = qd2_params_from_config(cfg_struct)  (ya decodificado con jsondecode)
%
%   Solo MATLAB (usa jsondecode): llamarla en InitFcn / en el script de
%   construcción y pasar 'p' a los bloques MATLAB Function como parámetro
%   (o usar sus valores para generar constantes). No es para codegen.
%   Claves usadas: mision.{z_explorador, z_respondedor, t_mark, d_min_h,
%   dz_min, espaciado_waypoints, tol_xy, tol_z, auto_land, limites_xy},
%   pista.vertices (máx. 16) y vision.radio_cluster_m.

p = qd2_params();
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
end

function p = copiar(p, m, clave, campo)
if isfield(m, clave) && ~isempty(m.(clave))
    p.(campo) = double(m.(clave));
end
end
