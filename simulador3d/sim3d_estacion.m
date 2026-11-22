function [est, py] = sim3d_estacion(accion, est, varargin)
%SIM3D_ESTACION Estación de tierra del simulador: arma los 32 doubles de §3.3.
%
%   est = sim3d_estacion('init', [], opc)
%   [est, py] = sim3d_estacion('paso', est, t, paquetes, cmds_internos, objetivos, opc, t_reloj)
%       t             : s de simulación
%       paquetes      : 32xN paquetes recibidos de Python en este paso (o [])
%       cmds_internos : vector de comandos de la figura / programados (1..5)
%       objetivos     : Kx5 [id clase x y conf] del detector simulado
%       t_reloj       : s de RELOJ (tic/toc) desde que arrancó el simulador
%       py            : 32x1 para qd2_mission_step
%   est = sim3d_estacion('ack', est, cmd_seq_ack_mision)
%       Traduce el ack de la misión al cmd_seq de Python (índice 54).
%
%   Fuentes:
%   - Sin estación Python (o sin TCP): la estación interna da el heartbeat
%     en cada paso y los comandos vienen de los botones/teclas de la figura
%     o de opc.auto_start / opc.comandos.
%   - Con la estación Python conectada: el heartbeat lo da Python, pero se
%     mide en TIEMPO DE RELOJ (Python manda a 20 Hz de reloj aunque el
%     simulador corra x16 o sin tiempo real). Mientras haya llegado un
%     heartbeat nuevo de Python en los últimos opc.t_hb_python s de reloj,
%     el heartbeat que ve la misión sube en cada paso; si Python deja de
%     mandar, deja de subir y la misión pasa a PAUSED tras hb_timeout (1 s
%     simulado), como en el sistema real. A velocidad x1 la pausa llega
%     ~t_hb_python + 1 s después de que Python se calla (en el real, ~1 s).
%     Tras opc.t_toma_interna s de RELOJ sin paquetes, la estación interna
%     retoma el heartbeat para que puedas dar RESUME desde la figura.
%     Cada comando nuevo de Python (cmd_seq distinto) se reenvía a la misión
%     con un cmd_seq interno y su ack se devuelve a Python con el cmd_seq de
%     Python. El primer paquete solo se registra (como el "primer contacto"
%     de la misión): un START viejo de la estación no hace despegar a los
%     drones. Si ese primer paquete trae un comando, NO se confirma (ack =
%     cmd_seq - 1) para que BridgeClient lo reporte como caducado y no como
%     "recibido".
%   - Objetivos: del detector simulado, salvo que opc.use_vision_python sea
%     true y Python esté conectado (entonces se reenvían sus ranuras 4-19).
%   - Overrides 20-23: de Python si está conectado; si no, opc.overrides.

switch accion
    case 'init'
        est.hb = 0;
        est.seq = 0;               % cmd_seq interno (el que ve la misión)
        est.cmd = 0;
        est.cola = zeros(0, 2);    % [cmd seq_python] pendientes de mandar (seq_python = -1 si es interno)
        est.mapa = zeros(0, 2);    % [seq_interno seq_python] esperando ack
        est.py_visto = false;
        est.t_py = -inf;           % s de RELOJ del último paquete de Python
        est.t_hb = -inf;           % s de RELOJ del último heartbeat NUEVO de Python
        est.py_hb = NaN;
        est.py_seq = NaN;
        est.py_ult = zeros(32, 1);
        est.ack_py = 0;
        est.rot = 0;
        est.modo_python = false;
        est.n_py = 0;
        est.n_cmd_py = 0;
        py = [];
    case 'paso'
        [est, py] = paso(est, varargin{:});
    case 'ack'
        ack = varargin{1};
        k = est.mapa(:, 1) <= ack;
        if any(k)
            m = est.mapa(k, :);
            est.ack_py = m(end, 2);
            est.mapa = est.mapa(~k, :);
        end
        py = [];
    otherwise
        error('sim3d:estacion', 'Acción desconocida: %s', accion);
end
end

% =========================================================================
function [est, py] = paso(est, t, paquetes, cmds_int, objetivos, opc, t_reloj)
if nargin < 7, t_reloj = t; end
hb_py_nuevo = false;
for k = 1:size(paquetes, 2)
    pk = paquetes(:, k);
    pk(~isfinite(pk)) = 0;
    est.t_py = t_reloj;
    est.n_py = est.n_py + 1;
    if ~est.py_visto
        est.py_visto = true;
        est.py_seq = pk(3);
        % primer contacto: se registra pero NO se ejecuta. Si trae un comando,
        % no se confirma (BridgeClient lo dará por caducado, no por "recibido").
        if round(pk(2)) ~= 0
            est.ack_py = max(pk(3) - 1, 0);
        else
            est.ack_py = pk(3);
        end
        est.py_hb = pk(1);
        hb_py_nuevo = true;
    else
        if pk(3) ~= est.py_seq
            est.py_seq = pk(3);
            c = round(pk(2));
            if c >= 1 && c <= 5
                est.cola(end+1, :) = [c pk(3)];
                est.n_cmd_py = est.n_cmd_py + 1;
            elseif ~isempty(est.mapa) || any(est.cola(:, 2) >= 0)
                % Python canceló un comando que caducó: cmd = 0 con cmd_seq nuevo
                est.cola = est.cola(est.cola(:, 2) < 0, :);
                est.cola(end+1, :) = [0 pk(3)];
            else
                est.ack_py = pk(3);
            end
        end
        if pk(1) ~= est.py_hb
            est.py_hb = pk(1);
            hb_py_nuevo = true;
        end
    end
    est.py_ult = pk;
end
for k = 1:numel(cmds_int)
    est.cola(end+1, :) = [cmds_int(k) -1];
end
if hb_py_nuevo
    est.t_hb = t_reloj;
end
% todo en tiempo de RELOJ: Python manda a 20 Hz de reloj sin importar la velocidad del simulador
est.modo_python = est.py_visto && (t_reloj - est.t_py) <= opc.t_toma_interna;

% heartbeat
if est.modo_python
    if (t_reloj - est.t_hb) <= opc.t_hb_python
        est.hb = est.hb + 1;     % Python está vivo (en reloj): la misión ve el heartbeat subir
    end
else
    est.hb = est.hb + 1;
end
% un comando por paso (la misión lo confirma en el mismo paso)
if ~isempty(est.cola)
    est.seq = est.seq + 1;
    est.cmd = est.cola(1, 1);
    if est.cola(1, 2) >= 0
        est.mapa(end+1, :) = [est.seq est.cola(1, 2)];
    end
    est.cola(1, :) = [];
end

py = zeros(32, 1);
py(1) = est.hb;
py(2) = est.cmd;
py(3) = est.seq;
if opc.use_vision_python
    % los objetivos SOLO vienen de Python (si se desconecta, ranuras vacías:
    % la misión conserva su tabla)
    if est.modo_python
        py(4:19) = est.py_ult(4:19);
    end
else
    n = size(objetivos, 1);
    py(4) = min(n, 32);
    if n > 0
        if n <= 3
            sel = 1:n;
        else
            sel = mod(est.rot + (0:2), n) + 1;
            est.rot = mod(est.rot + 3, n);
        end
        for j = 1:numel(sel)
            py(5 + 5*(j-1) : 9 + 5*(j-1)) = objetivos(sel(j), 1:5).';
        end
    end
end
if est.modo_python
    py(20:23) = est.py_ult(20:23);
else
    py(20:23) = opc.overrides(:);
end
end
