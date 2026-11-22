function R = qd2_e2e_web(corridas, op)
%QD2_E2E_WEB Prueba punta a punta: QD2_Sim_Dual en TIEMPO REAL manejado por la estación web.
%   R = qd2_e2e_web()                          % corridas 'pid', 'mpc' y 'falla', una tras otra
%   R = qd2_e2e_web({'mpc'})                   % solo una
%   R = qd2_e2e_web({'pid'}, struct('detector', true))
%
%   Corre QD2_Sim_Dual con COMMAND SOURCE = 2 (estación Python en el puerto del
%   PYTHON BRIDGE, config/estacion.json -> red.puerto_puente, 18700) y Simulation
%   Pacing. La estación (web/e2e/e2e_estacion.py o la UI a mano) da START, PAUSE,
%   RESUME y LAND ALL y publica los objetivos. Ver docs/PRUEBA_E2E.md.
%
%   Corridas:
%     'pid'    versión PID (nunca apilados, SEPARATION GUARD)
%     'mpc'    versión MPC (apilados)
%     'falla'  versión MPC con falla a bordo del dron 2 en t = op.t_falla (45 s):
%              su FSM simulada pasa a 7 -> alerta 9, el sano se aparta y aterrizan.
%   op.detector  false (defecto) = el SIMULATED DETECTOR no publica nada: solo
%                publica la estación (sus ids y los del detector serían
%                independientes; ver sim/README.md). true = deja los objetivos
%                por defecto de QD2_SIM (publica el detector mientras la
%                estación no publique).
%   op.t_final   s simulados como máximo por corrida (330).
%   op.t_falla   s de la falla en la corrida 'falla' (45).
%
%   Cada corrida termina sola 3 s después de DONE; si la estación no da START,
%   se queda en ASSIGN_ROLES hasta op.t_final. Devuelve las métricas de
%   qd2_correr_sim por corrida.

if nargin < 1 || isempty(corridas), corridas = {'pid', 'mpc', 'falla'}; end
if ischar(corridas) || isstring(corridas), corridas = cellstr(corridas); end
if nargin < 2, op = struct(); end
d = struct('detector', false, 't_final', 330, 't_falla', 45);
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(op, f{i}), op.(f{i}) = d.(f{i}); end
end
aqui = fileparts(mfilename('fullpath'));
addpath(aqui);
addpath(fullfile(fileparts(aqui), 'lib'));
R = struct('corrida', {}, 'res', {});
for i = 1:numel(corridas)
    c = lower(corridas{i});
    ov = struct();
    if ~op.detector
        ov.objetivos = zeros(0, 3);
    end
    switch c
        case 'pid', v = 1;
        case 'mpc', v = 2;
        case 'falla', v = 2; ov.falla_fsm = [2 op.t_falla];
        otherwise, error('qd2_e2e_web:corrida', 'Corrida desconocida "%s" (pid, mpc o falla).', c);
    end
    fprintf('\n===== E2E %s (versión %d) %s: esperando a la estación en el PYTHON BRIDGE =====\n', ...
        c, v, datestr(now, 'HH:MM:SS'));
    r = qd2_correr_sim(v, struct('fuente', 2, 'pacing', true, 't_final', op.t_final, 'override', ov));
    T = r.datos.tel;
    fprintf(['===== E2E %s %s: DONE=%d (t = %.1f s) | estados %s | alertas %s | visitados %d de %d (mask %d) | ' ...
        'dh mín en el aire %.3f m | FSM final %s\n'], c, datestr(now, 'HH:MM:SS'), r.completada, r.t_done, ...
        mat2str(r.estados), mat2str(r.alertas), r.n_visitados, r.n_conocidos, r.visited_mask, r.dh_min_aire, ...
        mat2str(r.fsm_final));
    fprintf('      final: dron 1 [%.2f %.2f %.2f] (home [%.2f %.2f]) | dron 2 [%.2f %.2f %.2f] (home [%.2f %.2f])\n', ...
        r.datos.x1(end, 1:3), T(end, 29:30), r.datos.x2(end, 1:3), T(end, 47:48));
    R(end+1) = struct('corrida', c, 'res', rmfield(r, 'datos')); %#ok<AGROW>
    pause(5);   % la estación ve el reinicio de t entre corridas
end
end
