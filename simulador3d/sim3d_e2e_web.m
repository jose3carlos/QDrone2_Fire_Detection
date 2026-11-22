function R = sim3d_e2e_web(versiones, opc_extra)
%SIM3D_E2E_WEB Prueba punta a punta: simulador 3D en tiempo real manejado por la estación web.
%   R = sim3d_e2e_web()                 % PID y luego MPC
%   R = sim3d_e2e_web({'MPC'})
%   R = sim3d_e2e_web({'PID'}, struct('mostrar', true))   % con la ventana 3D
%
%   Escenario 'web' (los mismos objetos que dibuja la cámara sintética de la UI),
%   use_vision_python = true (los objetivos los publica la estación), puente
%   §3.3 en config/estacion.json -> red.puerto_puente (18700) y tiempo real. La
%   estación (web/e2e/e2e_estacion.py --fuente sim3d, o la UI a mano con
%   camara.fuente = "sintetica") da START, PAUSE, RESUME y LAND ALL.
%   Ver docs/PRUEBA_E2E.md. Devuelve R(i).met de cada corrida.

if nargin < 1 || isempty(versiones), versiones = {'PID', 'MPC'}; end
if ischar(versiones) || isstring(versiones), versiones = cellstr(versiones); end
if nargin < 2, opc_extra = struct(); end
aqui = fileparts(mfilename('fullpath'));
addpath(aqui);
R = struct('version', {}, 'met', {});
for i = 1:numel(versiones)
    v = upper(versiones{i});
    opc = struct('version', v, 'escenario', 'web', 'usar_tcp', true, 'use_vision_python', true, ...
        'mostrar', false, 'tiempo_real', true, 'velocidad', 1, 't_final', 330, 'graficas_finales', false);
    f = fieldnames(opc_extra);
    for k = 1:numel(f)
        opc.(f{k}) = opc_extra.(f{k});
    end
    fprintf('\n===== E2E sim3d %s %s: esperando a la estación =====\n', v, datestr(now, 'HH:MM:SS'));
    r = sim3d_correr(opc);
    M = r.met;
    fprintf(['===== E2E sim3d %s %s: estado final %d (8 = DONE), estados %s, objetivos %d (visitados %d), ' ...
        'reales detectados %d de %d, dh mín en el aire %.3f m, paquetes de Python %d, comandos %d\n'], v, ...
        datestr(now, 'HH:MM:SS'), M.estado_final, mat2str(M.estados_vistos), M.n_objetivos, M.n_visitados, ...
        M.reales_detectados, M.n_reales, M.dh_min_aire, M.paquetes_python, M.comandos_python);
    R(end+1) = struct('version', v, 'met', M); %#ok<AGROW>
    pause(5);
end
end
