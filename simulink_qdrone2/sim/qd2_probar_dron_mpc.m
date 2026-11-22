function R = qd2_probar_dron_mpc(escenarios, conservar)
%QD2_PROBAR_DRON_MPC Prueba en lazo cerrado del COMMANDER (ANGLE MODE) del
%   modelo a bordo QD2_DroneStack_MPC.slx (el bloque real, copiado tal cual).
%
%   R = qd2_probar_dron_mpc()            % escenarios por defecto
%   R = qd2_probar_dron_mpc(E, true)     % E: struct array, conserva el .slx
%
%   El modelo completo del dron no se puede simular en modo Normal (los
%   bloques HIL/Stream de QUARC no hacen nada fuera del target). Por eso se
%   arma un arnés en tempdir que contiene una COPIA del subsistema
%   COMMANDER (ANGLE MODE) del modelo MPC (FSM, THROTTLE SWITCHBOX, PIV,
%   MPC a 50 Hz con sus Rate Transition y el switch PID/MPC) y lo cierra con:
%     - planta qd2_quad_plant a 1 kHz (el STABILIZER se aproxima con la
%       actitud de primer orden de la planta; retardo de actitud opcional),
%     - estimación IDEAL (pose/velocidad verdaderas, sin ruido ni latencia:
%       los errores de z de este arnés son una cota optimista; la
%       simulación QD2_Sim_Dual sí usa OptiTrack con ruido y latencia),
%     - emulación del Pose Switchbox con los límites [vel, acel] de los
%       "Continuous Sigmoid" originales (lento en TAKEOFF_HEIGHT, rápido en
%       FLIGHT; z_ref = 0 en AUTOLAND),
%     - Goto globales QD2_Datos_MPC / QD2_Actitud_Estimada, como en la raíz,
%     - un "otro dron" 1 m arriba durante [42, 55] s con downwash x1.3.
%   Guion: arm 2.5 s, takeoff 4 s (z = 1 m), hover hasta 30 s (error de z
%   transitorio en [15, 20) s y asentado en [25, 30) s), escalón xy a
%   (0.5, 0.3) en 30 s, baja a z = 0.7 en 38 s, el otro dron llega encima
%   en 42 s y se va en 55 s, takeoff = 0 (AUTOLAND) en 60 s.
%   El bloque "To Host File" (caja negra) de la copia se comenta y la
%   simulación corre en tempdir: no deja logs .mat en el repositorio.
%
%   Campos de cada escenario: nombre, modo (mpc_mode: 0 PIV, 1 MPC, o
%   [t_cambio modo_inicial modo_final]), retardo (pasos de 1 ms de retardo
%   de actitud), dw (factor del downwash real), masa (kg, opcional: masa
%   real de la planta y valor de "Drone Weight"; por defecto 1.504).

if nargin < 1 || isempty(escenarios)
    escenarios = struct('nombre', {'MPC', 'PIV', 'MPC_retardo100ms', 'PIV_retardo100ms', 'cambio_PIV_a_MPC'}, ...
        'modo', {1, 0, 1, 0, [25 0 1]}, 'retardo', {0, 0, 100, 100, 0}, 'dw', {1.3, 1.3, 1.3, 1.3, 1.3}, ...
        'masa', {1.504, 1.504, 1.504, 1.504, 1.504});
end
if nargin < 2, conservar = false; end
raiz = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(raiz, 'lib'));
addpath(fullfile(raiz, 'equipo2_original'));
if exist('qc_check_arguments', 'file') == 0 && exist('C:/Program Files/Quanser/QUARC/quarc', 'dir')
    addpath('C:/Program Files/Quanser/QUARC/quarc', '-end');
end
md = 'QD2_DroneStack_MPC';
load_system(fullfile(raiz, 'mpc', [md '.slx']));

h = 'qd2_arnes_commander';
dir_tmp = fullfile(tempdir, 'qd2_arnes_commander');
if ~exist(dir_tmp, 'dir'), mkdir(dir_tmp); end
if bdIsLoaded(h), close_system(h, 0); end
new_system(h);
set_param(h, 'SolverType', 'Fixed-step', 'Solver', 'ode1', 'FixedStep', '1/1000', ...
    'StopTime', '75', 'SolverMode', 'MultiTasking', 'EnableMultiTasking', 'on', ...
    'AutoInsertRateTranBlk', 'off', 'ReturnWorkspaceOutputs', 'on', 'SaveTime', 'off', ...
    'SaveOutput', 'off', 'SignalLogging', 'off');
cm = [h '/COMMANDER (ANGLE MODE)'];
add_block([md '/COMMANDER (ANGLE MODE)'], cm, 'Position', [600 100 800 420]);
% La caja negra (To Host File, ~16 MB por corrida) no hace falta en el arnés
th = find_system(cm, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Regexp', 'on', 'Name', '^To Host File');
for i = 1:numel(th)
    set_param(th{i}, 'Commented', 'on');
end

% ---------------- entorno ---------------------------------------------------
env = [h '/ENTORNO'];
add_block('simulink/User-Defined Functions/MATLAB Function', env, 'Position', [200 100 420 420]);
cfg = get_param(env, 'MATLABFunctionConfiguration');
cfg.FunctionScript = codigo_entorno();
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', env);
tam = {'trig', '4'; 'hf_rate', '4'; 'hf_err', '4'; 'hsig', '4'; 'ifpose', '8'; 'datos', '8'; 'att', '9'; 'xlog', '14'};
for j = 1:size(tam, 1)
    dd = ch.find('-isa', 'Stateflow.Data', 'Name', tam{j,1});
    dd.Props.Array.Size = tam{j,2};
end
add_block('simulink/Sources/Digital Clock', [h '/Clock'], 'SampleTime', '1/1000', 'Position', [40 100 70 130]);
add_block('simulink/Sources/Constant', [h '/MODO'], 'SampleTime', '1/1000', 'Value', '[1e9 1 1]', 'Position', [40 150 100 170]);
add_block('simulink/Sources/Constant', [h '/RETARDO'], 'SampleTime', '1/1000', 'Value', '0', 'Position', [40 190 100 210]);
add_block('simulink/Sources/Constant', [h '/DW'], 'SampleTime', '1/1000', 'Value', '1.3', 'Position', [40 230 100 250]);
add_block('simulink/Discrete/Memory', [h '/M_T'], 'Position', [880 160 910 190]);
add_block('simulink/Discrete/Memory', [h '/M_att'], 'Position', [880 210 910 240]);
add_block('simulink/Discrete/Memory', [h '/M_fsm'], 'Position', [880 360 910 390]);
L = @(a, b) add_line(h, a, b, 'autorouting', 'on');
L('Clock/1', 'ENTORNO/1');
L('M_T/1', 'ENTORNO/2');
L('M_att/1', 'ENTORNO/3');
L('M_fsm/1', 'ENTORNO/4');
L('MODO/1', 'ENTORNO/5');
L('RETARDO/1', 'ENTORNO/6');
L('DW/1', 'ENTORNO/7');
add_block('simulink/Sources/Constant', [h '/MASA'], 'SampleTime', '1/1000', 'Value', '1.504', 'Position', [40 270 100 290]);
L('MASA/1', 'ENTORNO/8');
L('COMMANDER (ANGLE MODE)/3', 'M_T/1');
L('COMMANDER (ANGLE MODE)/4', 'M_att/1');
L('COMMANDER (ANGLE MODE)/6', 'M_fsm/1');
% Constantes del COMMANDER (como en la raíz del modelo)
cte = {'LowBat', '0', 1; 'SensFail', '0', 2; 'CommIssue', '0', 9; 'Weight', '1.504', 10; 'MaxThr', '10.79*4', 11; 'isTrk', '1', 4};
for i = 1:size(cte, 1)
    add_block('simulink/Sources/Constant', [h '/' cte{i,1}], 'Value', cte{i,2}, ...
        'Position', [480 440+30*i 520 460+30*i], 'SampleTime', '1/1000');
    L([cte{i,1} '/1'], sprintf('COMMANDER (ANGLE MODE)/%d', cte{i,3}));
end
L('ENTORNO/1', 'COMMANDER (ANGLE MODE)/3');
L('ENTORNO/2', 'COMMANDER (ANGLE MODE)/5');
L('ENTORNO/3', 'COMMANDER (ANGLE MODE)/6');
L('ENTORNO/4', 'COMMANDER (ANGLE MODE)/7');
L('ENTORNO/5', 'COMMANDER (ANGLE MODE)/8');
add_block('simulink/Signal Routing/Goto', [h '/G_datos'], 'GotoTag', 'QD2_Datos_MPC', ...
    'TagVisibility', 'global', 'Position', [470 40 560 60]);
add_block('simulink/Signal Routing/Goto', [h '/G_att'], 'GotoTag', 'QD2_Actitud_Estimada', ...
    'TagVisibility', 'global', 'Position', [470 70 560 90]);
L('ENTORNO/6', 'G_datos/1');
L('ENTORNO/7', 'G_att/1');
add_block('simulink/Signal Routing/From', [h '/F_telem'], 'GotoTag', 'QD2_Telem_Cmd', 'Position', [880 30 970 50]);
logs = {'ENTORNO/8', 'x'; 'F_telem/1', 'telem'; 'COMMANDER (ANGLE MODE)/6', 'fsm'; ...
    'COMMANDER (ANGLE MODE)/3', 'T'; 'COMMANDER (ANGLE MODE)/4', 'attc'};
for i = 1:size(logs, 1)
    b = sprintf('W%d', i);
    add_block('simulink/Sinks/To Workspace', [h '/' b], 'VariableName', logs{i,2}, ...
        'SaveFormat', 'Array', 'Decimation', '10', 'Position', [1050 40*i 1110 40*i+25]);
    L(logs{i,1}, [b '/1']);
end
add_block('simulink/Sinks/Terminator', [h '/T1'], 'Position', [880 420 900 440]);
add_block('simulink/Sinks/Terminator', [h '/T2'], 'Position', [880 450 900 470]);
add_block('simulink/Sinks/Terminator', [h '/T3'], 'Position', [880 480 900 500]);
L('COMMANDER (ANGLE MODE)/1', 'T1/1');
L('COMMANDER (ANGLE MODE)/2', 'T2/1');
L('COMMANDER (ANGLE MODE)/5', 'T3/1');
archivo = fullfile(dir_tmp, [h '.slx']);
save_system(h, archivo);

R = struct('nombre', {}, 'ok', {}, 'fsm_seq', {}, 't_vuelo', {}, 'err_z_hover', {}, ...
    'err_z_hover_asentado', {}, 'sobrepaso_xy', {}, 't_asent_xy', {}, 'err_z_apilado', {}, 'z_max_aterrizaje', {}, ...
    'fsm_final', {}, 't_fin', {}, 'estados_mpc', {}, 'salto_T_cambio', {}, 't_reloj', {});
for k = 1:numel(escenarios)
    e = escenarios(k);
    if isscalar(e.modo)
        set_param([h '/MODO'], 'Value', sprintf('[1e9 %g %g]', e.modo, e.modo));
    else
        set_param([h '/MODO'], 'Value', mat2str(e.modo));
    end
    set_param([h '/RETARDO'], 'Value', num2str(e.retardo));
    set_param([h '/DW'], 'Value', num2str(e.dw));
    masa = 1.504;
    if isfield(e, 'masa') && ~isempty(e.masa), masa = e.masa; end
    set_param([h '/MASA'], 'Value', num2str(masa));
    set_param([h '/Weight'], 'Value', num2str(masa));
    t0 = tic;
    dir_ant = pwd;
    cd(dir_tmp);     % cualquier archivo que escriba la simulación queda fuera del repo
    try
        out = sim(h);
    catch err
        cd(dir_ant);
        fprintf(2, '%s\n', getReport(err, 'extended', 'hyperlinks', 'off'));
        rethrow(err);
    end
    cd(dir_ant);
    r.nombre = e.nombre;
    r.t_reloj = toc(t0);
    X = a_filas(out.get('x'));          % [t x y z vx vy vz roll pitch yaw T ref_xyz]
    F = a_filas(out.get('fsm'));
    Tl = a_filas(out.get('telem'));
    t = X(:,1);
    f = F(:);
    cambios = [true; diff(f) ~= 0];
    r.fsm_seq = f(cambios)';
    r.ok = all(isfinite(X(:)));
    i5 = find(f == 5, 1);
    r.t_vuelo = NaN; if ~isempty(i5), r.t_vuelo = t(i5); end
    ih = t >= 15 & t < 20;
    r.err_z_hover = max(abs(X(ih,4) - X(ih,14)));          % transitorio tras el despegue
    ihs = t >= 25 & t < 30;
    r.err_z_hover_asentado = max(abs(X(ihs,4) - X(ihs,14)));
    ixy = t >= 30 & t < 38;
    ex = X(ixy,2) - 0.5; ey = X(ixy,3) - 0.3;
    r.sobrepaso_xy = max(0, max(X(ixy,2)) - 0.5);
    dxy = hypot(ex, ey);
    ia = find(dxy > 0.03, 1, 'last');
    r.t_asent_xy = NaN; if ~isempty(ia), r.t_asent_xy = t(find(ixy, 1) + ia - 1) - 30; end
    iap = t >= 42 & t < 57;            % incluye la llegada y la salida del otro dron
    r.err_z_apilado = max(abs(X(iap,4) - X(iap,14)));
    r.fsm_final = f(end);
    r.t_fin = t(end);
    iat = t >= 60;
    r.z_max_aterrizaje = max(X(iat,4));
    r.estados_mpc = unique(Tl(:,2))';
    r.salto_T_cambio = NaN;
    if ~isscalar(e.modo)
        ic = t >= e.modo(1) - 0.5 & t < e.modo(1) + 2;
        r.salto_T_cambio = max(abs(X(ic,4) - X(ic,14)));   % error de z alrededor del cambio
    end
    R(k) = r; %#ok<AGROW>
    fprintf('[arnés] %-18s fsm %s | vuelo %.2f s | z hover %.4f m (asentado %.4f m) | xy sobrepaso %.3f m, asent. %.2f s | z apilado %.4f m | fin fsm %d en %.1f s | estados MPC %s | reloj %.1f s\n', ...
        r.nombre, mat2str(r.fsm_seq), r.t_vuelo, r.err_z_hover, r.err_z_hover_asentado, r.sobrepaso_xy, r.t_asent_xy, ...
        r.err_z_apilado, r.fsm_final, r.t_fin, mat2str(r.estados_mpc), r.t_reloj);
    if ~isnan(r.salto_T_cambio)
        fprintf('         error de z máximo en +-1 s del cambio PIV->MPC: %.4f m\n', r.salto_T_cambio);
    end
end
if ~conservar
    close_system(h, 0);
end
end

function A = a_filas(A)
if ndims(A) == 3
    A = permute(A, [3 1 2]);
end
if size(A, 2) == 1 && size(A, 1) > 1
    return;
end
end

function s = codigo_entorno()
s = sprintf([ ...
'function [trig, hf_rate, hf_err, hsig, ifpose, datos, att, xlog] = fcn(t, T, attc, fsm, modo, retardo, dwf, masa)\n' ...
'%%#codegen\n' ...
'%% Entorno del arnés: planta + estimación ideal + Pose Switchbox (sigmoides).\n' ...
'persistent x p r rv buf k\n' ...
'dt = 0.001;\n' ...
'if isempty(p)\n' ...
'    p = qd2_params();\n' ...
'    x = zeros(10,1);\n' ...
'    r = zeros(4,1);\n' ...
'    rv = zeros(4,1);\n' ...
'    buf = zeros(3,200);\n' ...
'    k = 1;\n' ...
'end\n' ...
'p.planta.masa = masa;   %% masa real de la planta (escenario)\n' ...
'%% guion\n' ...
'arm = double(t >= 2.5);\n' ...
'tk = double(t >= 4 && t < 60);\n' ...
'trig = [arm; tk; 0; 0];\n' ...
'des = [0; 0; 1.0; 0];\n' ...
'if t >= 30, des = [0.5; 0.3; 1.0; 0]; end\n' ...
'if t >= 38, des = [0.5; 0.3; 0.7; 0]; end\n' ...
'mm = modo(2); if t >= modo(1), mm = modo(3); end\n' ...
'%% otro dron encima en [42, 55] s\n' ...
'otro = nan(6,1);\n' ...
'if t >= 42 && t < 55\n' ...
'    otro = [0.5; 0.3; 1.7; 0; 0; 0];\n' ...
'end\n' ...
'datos = [otro; mm; 0];\n' ...
'%% Pose Switchbox: "Continuous Sigmoid" por eje con [vel máx, acel máx]\n' ...
'%% (lento en TAKEOFF_HEIGHT, rápido en FLIGHT; AUTOLAND: z_ref = 0).\n' ...
'lento  = [2 0.2; 2 0.2; 3 4.5; 2 0.5];\n' ...
'rapido = [2 10; 2 10; 3 10; 2 2];\n' ...
'if fsm <= 3 || fsm >= 7\n' ...
'    r = [x(1); x(2); 0; x(9)];\n' ...
'    rv = zeros(4,1);\n' ...
'else\n' ...
'    obj = des;\n' ...
'    lim = lento;\n' ...
'    if fsm == 5, lim = rapido; end\n' ...
'    if fsm == 6, obj(3) = 0; end\n' ...
'    for a = 1:4\n' ...
'        e = obj(a) - r(a);\n' ...
'        vd = sign(e)*min(lim(a,1), sqrt(2*lim(a,2)*abs(e)));\n' ...
'        ac = max(min((vd - rv(a))/dt, lim(a,2)), -lim(a,2));\n' ...
'        rv(a) = rv(a) + ac*dt;\n' ...
'        r(a) = r(a) + rv(a)*dt;\n' ...
'    end\n' ...
'end\n' ...
'%% planta (comando con retardo de actitud opcional)\n' ...
'buf(:, k) = attc(:);\n' ...
'n = min(max(round(retardo), 0), 199);\n' ...
'j = k - n; if j < 1, j = j + 200; end\n' ...
'ad = buf(:, j);\n' ...
'k = k + 1; if k > 200, k = 1; end\n' ...
'F = zeros(3,1);\n' ...
'if all(isfinite(otro))\n' ...
'    [F, ~] = qd2_downwash(x(1:3), otro(1:3), p.dw);\n' ...
'    F = dwf*F;\n' ...
'end\n' ...
'x = qd2_quad_plant(x, [T; ad(1); ad(2); ad(3)], F, dt, p.planta);\n' ...
'%% estimación ideal\n' ...
'psi = x(9); c = cos(psi); s = sin(psi);\n' ...
'e = r(1:3) - x(1:3);\n' ...
'hf_err = [c*e(1) + s*e(2); -s*e(1) + c*e(2); e(3); r(4) - psi];\n' ...
'hf_rate = [c*x(4) + s*x(5); -s*x(4) + c*x(5); x(6); ad(3)];\n' ...
'hsig = [r(3); x(3); 2.5; des(3)];\n' ...
'ifpose = [x(1); x(2); x(3); psi; r];\n' ...
'att = [x(7); x(8); x(9); 0; 0; 0; 0; 0; 0];\n' ...
'xlog = [t; x; r(1:3)];\n']);
end
