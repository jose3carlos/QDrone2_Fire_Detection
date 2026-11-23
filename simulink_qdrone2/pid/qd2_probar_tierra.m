function R = qd2_probar_tierra(ver, t_final, conservar, t_arm_sw, opt)
%QD2_PROBAR_TIERRA Prueba funcional (SIL) del modelo de tierra generado.
%   R = qd2_probar_tierra('PID')   o   qd2_probar_tierra('MPC', 230)
%   R = qd2_probar_tierra('MPC', 80, false, 0, struct('to_low_t', 62.9))
%
%   Copia QD2_MissionCtrl_Dual_<ver>.slx a una carpeta temporal y, SOLO EN
%   LA COPIA, sustituye lo que necesita hardware por modelos simples:
%     - OptiTrack Trackables  -> bloque SIM DRONES (2 drones cinemáticos que
%       siguen la Desired Pose de cada dron cuando arm y takeoff = 1,
%       aterrizan solos con takeoff = 0 y caen con E-stop).
%     - En Communication QDrone 1/2 solo el Stream Server (y su Model
%       Argument) se cambia por STREAM SERVER STUB: conectado, eco del
%       timestamp y, en MPC, telemetría a bordo de 8 doubles. Así corre el
%       camino real de los paquetes (16 doubles PID; Mux 24 / Timestamp [1] /
%       QDroneX_Telemetry en MPC) y se registra lo que se envía a cada dron.
%     - PYTHON BRIDGE se comenta; los comandos de Python los genera PYTHON
%       STUB (heartbeat, START, un objetivo a los 30 s y LAND ALL cuando la
%       misión lleva 6 s en RETURN_HOME).
%     - Sinks QUARC (To Host File, Show Message, Stop with Message) se
%       comentan; el paro se mide con la señal Stop_Allowed.
%     - Switches Arm/Disarm y Takeoff/Land del operador en 1 (permiso).
%   Todo lo demás (áreas 1-10, librería QD2_Mission_Lib, Goto/From,
%   OPERATOR SAFETY LAYER, OPERATOR COMMAND MERGE, SEPARATION GUARD, rate
%   limiter, TELEMETRY PACK, MISSION END CHECK) corre tal como está.
%   El modelo original y el generado NO se modifican.
%
%   opt (struct, casos del operador; tiempos en s):
%     .estop_t    Emergency Stop sube en t
%     .to_low_t   Takeoff/Land baja en t
%     .arm_low_t  Arm/Disarm baja en t
%     .manual_t   Command Source pasa a MANUAL (1) en t
%     .sw_low_t   Arm/Disarm y Takeoff/Land bajan en t (con manual_t: el
%                 operador baja los switches para permitir el cambio de modo)
%     .py_off_t   la estación Python deja de mandar heartbeat en t
%     .trk2_off   [t1 t2]: OptiTrack pierde al dron 2 en ese intervalo
%     .homes      [x1 y1; x2 y2]: posición inicial (en el piso) de los drones 1 y
%                 2; por defecto [-1.0 0.3; 0.9 -0.3]. El de menor x es el
%                 explorador: con [0.9 -0.3; -1.0 0.3] el explorador es el dron 2
%                 y trk2_off quita el rastreo del EXPLORADOR (y(63) = 0)
%     .falla      [dron t]: (MPC) el stack a bordo de ese dron entra a Error en t:
%                 reporta FSM 7 (y 9 a los 4 s) en su respuesta de 8 doubles y cae
%                 a 0.5 m/s sin obedecer las consignas
%     .proto_mal  [dron t1 t2]: (MPC) ese dron responde sin la etiqueta y(8) = 2
%     .enlace_off [dron t]: (MPC) enlace caído en los DOS sentidos desde t: el
%                 Stream Server retiene la última respuesta (eco congelado, el
%                 loopback crece) y el dron, sin paquetes, pasa a Error a los
%                 0.5 s y cae sin obedecer (tierra nunca ve esa FSM 7)

if nargin < 1, ver = 'PID'; end
if nargin < 2, t_final = 230; end
if nargin < 3, conservar = false; end
if nargin < 4, t_arm_sw = 0; end   % > 0: el switch Arm/Disarm se sube en t = t_arm_sw
if nargin < 5, opt = struct(); end
ver = upper(ver);
raiz = fileparts(fileparts(mfilename('fullpath')));
addpath(raiz); addpath(fullfile(raiz, 'lib'));
if isempty(which('qc_get_step_size')) && exist('C:\Program Files\Quanser\QUARC\quarc', 'dir')
    addpath('C:\Program Files\Quanser\QUARC\quarc');
end
addpath(fullfile(raiz, lower(ver)));
% Artefactos de compilación (slprj, .slxc) fuera del repositorio
dc = fullfile(tempdir, 'qd2_slcache');
if ~exist(dc, 'dir'), mkdir(dc); end
Simulink.fileGenControl('set', 'CacheFolder', dc, 'CodeGenFolder', dc);
qd2_cargar_config();
src = fullfile(raiz, lower(ver), ['QD2_MissionCtrl_Dual_' ver '.slx']);
m = ['qd2_tierra_prueba_' lower(ver)];
dtmp = fullfile(tempdir, ['qd2_tierra_' char(java.util.UUID.randomUUID)]);
mkdir(dtmp);
copyfile(src, fullfile(dtmp, [m '.slx']));
if bdIsLoaded(m), close_system(m, 0); end
load_system(fullfile(dtmp, [m '.slx']));
limpiar = onCleanup(@() terminar(m, dtmp, conservar));
MC = [m '/MISSION CONTROL QDRONE POSITION CONTROL'];
es_mpc = strcmp(ver, 'MPC');

% ---------------- raíz: Stream Servers -> stubs ----------------------------
for i = 1:2
    stub_stream_server(sprintf('%s/Communication QDrone %d', m, i), i, es_mpc, campo(opt, 'falla', [0 -1]), campo(opt, 'proto_mal', [0 -1 -1]), ...
        campo(opt, 'enlace_off', [0 -1]));
end
set_param([m '/PYTHON BRIDGE'], 'Commented', 'on');
lh = get_param([m '/Goto Python Commands'], 'LineHandles');
delete_line(lh.Inport(1));
add_block('simulink/Sources/Digital Clock', [m '/Clock Stub'], 'Position', [-1300 900 -1260 920], 'SampleTime', '0.001');
add_block('simulink/Signal Routing/From', [m '/From Tel Stub'], 'Position', [-1400 940 -1260 960], 'GotoTag', 'Python_Telemetry');
add_block('simulink/Discrete/Memory', [m '/Memory Stub'], 'Position', [-1230 935 -1200 965]);
add_block('simulink/User-Defined Functions/MATLAB Function', [m '/PYTHON STUB'], 'Position', [-1150 890 -1000 970]);
poner_codigo([m '/PYTHON STUB'], codigo_python(campo(opt, 'py_off_t', inf)), {'py', '[32 1]'});
add_line(m, 'Clock Stub/1', 'PYTHON STUB/1');
add_line(m, 'From Tel Stub/1', 'Memory Stub/1');
add_line(m, 'Memory Stub/1', 'PYTHON STUB/2');
add_line(m, 'PYTHON STUB/1', 'Goto Python Commands/1', 'autorouting', 'smart');

% ---------------- MC: OptiTrack -> SIM DRONES -----------------------------
set_param([MC '/Localization - Optitrack'], 'Commented', 'on');
lh = get_param([MC '/Manual Switch'], 'LineHandles');
delete_line(lh.Inport(1));
x0 = 1750; y0 = 300;
add_block('simulink/Sources/Digital Clock', [MC '/Clock Sim'], 'Position', [x0 y0 x0+40 y0+20], 'SampleTime', '0.001');
tags = {'Triggers_QDrone1', 'Triggers_QDrone2', 'Desired_Pose', 'Desired_Pose_QDrone2'};
for k = 1:4
    add_block('simulink/Signal Routing/From', [MC sprintf('/From Sim %d', k)], ...
        'Position', [x0-120 y0+30+30*k x0+40 y0+50+30*k], 'GotoTag', tags{k});
end
add_block('simulink/User-Defined Functions/MATLAB Function', [MC '/SIM DRONES'], 'Position', [x0+100 y0 x0+260 y0+180]);
fd = campo(opt, 'falla', [0 -1]);
eo = campo(opt, 'enlace_off', [0 -1]);
if eo(2) >= 0
    fd = [eo(1) eo(2) + 0.5];          % sin paquetes 0.5 s -> Error a bordo: cae
end
homes = campo(opt, 'homes', [-1.0 0.3; 0.9 -0.3]);
poner_codigo([MC '/SIM DRONES'], codigo_drones(campo(opt, 'trk2_off', [-1 -1]), fd, homes), {'data', '14'; 'pos', '[6 1]'});
add_line(MC, 'Clock Sim/1', 'SIM DRONES/1');
for k = 1:4
    add_line(MC, sprintf('From Sim %d/1', k), sprintf('SIM DRONES/%d', k+1));
end
% retardo de 1 paso: OptiTrack es una fuente (rompe el lazo algebraico de la prueba)
add_block('simulink/Discrete/Unit Delay', [MC '/Delay Sim'], 'Position', [x0+300 y0+20 x0+330 y0+50], ...
    'SampleTime', '0.001', 'InitialCondition', sprintf('[%g %g 0.07 0 0 0 1 %g %g 0.07 0 0 0 1]''', homes(1,1), homes(1,2), homes(2,1), homes(2,2)));
add_line(MC, 'SIM DRONES/1', 'Delay Sim/1');
add_line(MC, 'Delay Sim/1', 'Manual Switch/1', 'autorouting', 'smart');

% sinks QUARC que no corren en simulación normal
comentar = {[MC '/To Host File'], [MC '/To Host File Dual'], [MC '/Show Message on Host'], ...
    [MC '/Show Message on Host '], [MC '/Show Message on Host QDrone 2'], ...
    [MC '/Subsystem/Stop with Message'], [MC '/Subsystem QDrone 2/Stop with Message']};
for k = 1:numel(comentar)
    set_param(comentar{k}, 'Commented', 'on');
end
% switches del operador: Arm y Takeoff en 1 (permiso), E-stop en 0
for b = {'Arm//Disarm', 'Takeoff//Land'}
    set_param([MC '/' b{1}], 'InitFcn', '', 'sw', '1');
end
if t_arm_sw > 0
    escalon(MC, 'Arm//Disarm', 'Step Arm', t_arm_sw, 0, 1, [20 140 50 170]);
end
% ---------------- casos del operador --------------------------------------
if isfield(opt, 'estop_t')
    set_param([MC '/Emergency Stop'], 'InitFcn', '', 'sw', '1');
    escalon(MC, 'Emergency Stop', 'Step ES', opt.estop_t, 0, 1, [20 240 50 270]);
end
if isfield(opt, 'to_low_t') || isfield(opt, 'sw_low_t')
    escalon(MC, 'Takeoff//Land', 'Step TO', campo(opt, 'to_low_t', campo(opt, 'sw_low_t', inf)), 1, 0, [20 190 50 220]);
end
if isfield(opt, 'arm_low_t') || isfield(opt, 'sw_low_t')
    escalon(MC, 'Arm//Disarm', 'Step ARM2', campo(opt, 'arm_low_t', campo(opt, 'sw_low_t', inf)), 1, 0, [20 100 50 130]);
end
if isfield(opt, 'manual_t')
    set_param([MC '/Command Source'], 'InitFcn', '', 'sw', '1');
    escalon(MC, 'Command Source', 'Step CS', opt.manual_t, 0, 1, [700 2000 730 2030]);
end

% ---------------- registros ----------------------------------------------
reg = {'Python_Telemetry', 'tel'; 'Triggers_QDrone1', 'trig1'; 'Triggers_QDrone2', 'trig2'; ...
       'Stop_Allowed', 'stop_ok'; 'Desired_Pose', 'des1'; 'Desired_Pose_QDrone2', 'des2'; ...
       'Fligth_Plan', 'fp1'; 'Fligth_Plan_D2', 'fp2'; 'Command_Source_Effective', 'cse'; ...
       'Operator_Request', 'opreq'; 'Communication_Loopback_Time_D2', 'lb2'};
for k = 1:size(reg, 1)
    yk = 2700 + 40*k;
    add_block('simulink/Signal Routing/From', [MC '/From Log ' reg{k,2}], 'Position', [200 yk 360 yk+20], 'GotoTag', reg{k,1});
    add_block('simulink/Sinks/To Workspace', [MC '/Log ' reg{k,2}], 'Position', [420 yk 500 yk+20], ...
        'VariableName', reg{k,2}, 'SaveFormat', 'Array', 'Decimation', '20');
    add_line(MC, ['From Log ' reg{k,2} '/1'], ['Log ' reg{k,2} '/1']);
end
add_block('simulink/Sinks/To Workspace', [MC '/Log pos'], 'Position', [x0+320 y0+120 x0+400 y0+140], ...
    'VariableName', 'pos', 'SaveFormat', 'Array', 'Decimation', '20');
add_line(MC, 'SIM DRONES/2', 'Log pos/1');
add_block('simulink/Sources/Digital Clock', [MC '/Clock Log'], 'Position', [200 3200 240 3220], 'SampleTime', '0.001');
add_block('simulink/Sinks/To Workspace', [MC '/Log t'], 'Position', [420 3200 500 3220], ...
    'VariableName', 'tt', 'SaveFormat', 'Array', 'Decimation', '20');
add_line(MC, 'Clock Log/1', 'Log t/1');

set_param(m, 'StopTime', num2str(t_final), 'SimulationMode', 'normal', 'ReturnWorkspaceOutputs', 'on');
t0 = tic;
out = sim(m);
R.t_reloj = toc(t0);
R.version = ver;
R.opt = opt;
R.t = col(out.get('tt'));
R.tel = filas(out.get('tel'));
R.trig1 = filas(out.get('trig1'));
R.trig2 = filas(out.get('trig2'));
R.stop_ok = col(out.get('stop_ok'));
R.lb2 = col(out.get('lb2'));          % loopback del dron 2 (s)
R.pos = filas(out.get('pos'));
R.des1 = filas(out.get('des1'));
R.des2 = filas(out.get('des2'));
R.fp = [col(out.get('fp1')) col(out.get('fp2'))];
R.cse = col(out.get('cse'));
R.opreq = filas(out.get('opreq'));
R.tx1 = filas(out.get('tx1'));
R.tx2 = filas(out.get('tx2'));
R = resumir(R);
end

% =========================================================================
function stub_stream_server(C, i, es_mpc, falla, proto, eo)
% Cambia el Stream Server de Communication QDrone i por un stub (solo en la copia).
SS = [C '/Stream Server'];
lh = get_param(SS, 'LineHandles');
src = get_param(lh.Inport(1), 'SrcPortHandle');
dst = cell(1, 5);
for k = [1 2 4]
    dst{k} = get_param(lh.Outport(k), 'DstPortHandle');
end
for k = 1:numel(lh.Inport), if lh.Inport(k) > 0, delete_line(lh.Inport(k)); end, end
for k = 1:numel(lh.Outport), if lh.Outport(k) > 0, delete_line(lh.Outport(k)); end, end
delete_block(SS);
% Model Argument (string QUARC) y lo que cuelga de él
ma = [C '/Model Argument'];
lm = get_param(ma, 'LineHandles');
for k = 1:numel(lm.Outport)
    if lm.Outport(k) > 0
        d = get_param(lm.Outport(k), 'DstBlockHandle');
        delete_line(lm.Outport(k));
        for j = 1:numel(d), delete_block(d(j)); end
    end
end
delete_block(ma);
sd = find_system(C, 'SearchDepth', 1, 'Regexp', 'on', 'Name', '^String Display');
for k = 1:numel(sd), delete_block(sd{k}); end
n = 1 + 7*double(es_mpc);
b = [C '/STREAM SERVER STUB'];
add_block('simulink/User-Defined Functions/MATLAB Function', b, 'Position', [295 22 395 138]);
poner_codigo(b, codigo_stream(n, i, falla, proto, eo), {'st', '1'; 'er', '1'; 'rx', sprintf('[%d 1]', n)});
pb = get_param(b, 'PortHandles');
add_line(C, src, pb.Inport(1), 'autorouting', 'smart');
add_block('simulink/Sources/Digital Clock', [C '/Clock Stub'], 'Position', [220 140 260 160], 'SampleTime', '0.001');
add_line(C, 'Clock Stub/1', 'STREAM SERVER STUB/2', 'autorouting', 'smart');
for k = 1:2
    for j = 1:numel(dst{k}), add_line(C, pb.Outport(k), dst{k}(j), 'autorouting', 'smart'); end
end
for j = 1:numel(dst{4}), add_line(C, pb.Outport(3), dst{4}(j), 'autorouting', 'smart'); end
% registro de lo que se envía al dron (16 doubles PID / 24 MPC)
lg = add_block('simulink/Sinks/To Workspace', [C '/Log tx'], 'Position', [295 170 375 190], ...
    'VariableName', sprintf('tx%d', i), 'SaveFormat', 'Array', 'Decimation', '20');
pl = get_param(lg, 'PortHandles');
add_line(C, src, pl.Inport(1), 'autorouting', 'smart');
end

function c = codigo_stream(n, i, falla, proto, eo)
c = sprintf([ ...
'function [st, er, rx] = fcn(tx, t)\n' ...
'%% Stream Server simulado: conectado (estado 2), sin error, eco del timestamp (tx(16)).\n' ...
'%% MPC: respuesta de 8 doubles [eco FSM batería estado T roll pitch 2] (§3.2).\n' ...
'persistent ult\n' ...
'if isempty(ult)\n' ...
'    ult = zeros(%d, 1);\n' ...
'end\n' ...
'st = 2;\n' ...
'er = 0;\n' ...
'rx = zeros(%d, 1);\n' ...
'rx(1) = tx(16);\n' ...
'if numel(rx) >= 8\n' ...
'    rx(2) = 5;      %% FSM a bordo\n' ...
'    rx(3) = 16.0;   %% batería (V)\n' ...
'    rx(8) = 2;      %% etiqueta protocolo MPC v1\n' ...
'    if %d == %g && %g >= 0 && t >= %g\n' ...
'        rx(2) = 7;  %% falla a bordo: Error (rampa de emergencia)\n' ...
'        if t >= %g + 4, rx(2) = 9; end\n' ...
'    end\n' ...
'    if %d == %g && t >= %g && t < %g\n' ...
'        rx(8) = 0;  %% sin etiqueta\n' ...
'    end\n' ...
'end\n' ...
'if %d == %g && %g >= 0 && t >= %g\n' ...
'    rx = ult;       %% enlace caído: "most recent" retiene la última respuesta\n' ...
'else\n' ...
'    ult = rx;\n' ...
'end\n'], n, n, i, falla(1), falla(2), falla(2), falla(2), i, proto(1), proto(2), proto(3), i, eo(1), eo(2), eo(2));
end

function escalon(MC, blk, nombre, t, antes, despues, pos)
lh = get_param([MC '/' blk], 'LineHandles');
delete_line(lh.Inport(1));
add_block('simulink/Sources/Step', [MC '/' nombre], 'Position', pos, ...
    'Time', num2str(t), 'Before', num2str(antes), 'After', num2str(despues), 'SampleTime', '0.001');
add_line(MC, [nombre '/1'], [blk '/1'], 'autorouting', 'smart');
end

function v = campo(s, f, defecto)
if isfield(s, f), v = s.(f); else, v = defecto; end
end

function R = resumir(R)
est = R.tel(:, 2);
R.estados = unique(est(:))';
R.explorer_id = R.tel(end, 4);
[~, ia] = unique(est, 'stable');
R.t_estados = [est(ia) R.t(ia)];
p1 = R.pos(:, 1:3); p2 = R.pos(:, 4:6);
aire = p1(:,3) > 0.15 & p2(:,3) > 0.15;
dh = sqrt(sum((p1(:,1:2) - p2(:,1:2)).^2, 2));
R.dh = dh;
R.dh_min_aire = minv(dh(aire));
R.dz_aire = [minv(abs(p1(aire,3) - p2(aire,3))) maxv(abs(p1(aire,3) - p2(aire,3)))];
dd = sqrt(sum((R.des1(:,1:2) - R.des2(:,1:2)).^2, 2));
R.dh_min_consignas_aire = minv(dd(aire));
R.z_max = [max(p1(:,3)) max(p2(:,3))];
cerca = aire & dh < 0.9;
if any(cerca), R.dz_min_cerca = min(abs(p1(cerca,3) - p2(cerca,3))); else, R.dz_min_cerca = NaN; end
% "uno encima del otro": alguno de los dos arriba (> 0.15 m) y a < 0.5 m en
% horizontal (huellas de 0.44 x 0.49 m) con |dz| < 0.3 m -> se tocarían
uno_arriba = p1(:,3) > 0.15 | p2(:,3) > 0.15;
R.choque = any(uno_arriba & dh < 0.5 & abs(p1(:,3) - p2(:,3)) < 0.3);
% arm/takeoff efectivos
R.t_arm = [primero(R.t, R.trig1(:,1) > 0.5) primero(R.t, R.trig2(:,1) > 0.5)];
R.t_takeoff = [primero(R.t, R.trig1(:,2) > 0.5) primero(R.t, R.trig2(:,2) > 0.5)];
R.arm_y_takeoff_juntos = any(diff(R.trig1(:,2)) > 0.5 & R.trig1(2:end,1) > 0.5 & R.trig1(1:end-1,1) < 0.5) || ...
    any(diff(R.trig2(:,2)) > 0.5 & R.trig2(2:end,1) > 0.5 & R.trig2(1:end-1,1) < 0.5);
R.n_despegues = [sum(diff(R.trig1(:,2)) > 0.5) sum(diff(R.trig2(:,2)) > 0.5)];
R.t_done = primero(R.t, est == 8);
R.t_suelo = [primero_desde(R.t, p1(:,3) <= 0.001, R.t_takeoff(1)), primero_desde(R.t, p2(:,3) <= 0.001, R.t_takeoff(2))];
R.dh_al_aterrizar = [valor_en(R.t, dh, R.t_suelo(1)) valor_en(R.t, dh, R.t_suelo(2))];
R.t_stop_ok = primero(R.t, R.stop_ok > 0.5);
% Stop Allowed con algún dron rastreado en el aire (no debe pasar nunca)
R.stop_con_dron_en_aire = any(R.stop_ok > 0.5 & (p1(:,3) >= 0.10 | p2(:,3) >= 0.10));
R.max_tel_nan = any(isnan(R.tel(:, [1:14 51:55])), 'all');
% y(61:63): roll, pitch y válido del explorador. El stub SIM DRONES manda la
% actitud conocida de codigo_drones; válido = explorador 1/2 rastreado (o+1).
tt = R.tel(:, 1);
A = [0.04*sin(0.5*tt) -0.03*cos(0.3*tt) 0.05*sin(0.7*tt) 0.035*sin(0.2*tt + 1)];
eid = R.tel(:, 4);
val = (eid == 1 & R.tel(:, 15) > 0.5) | (eid == 2 & R.tel(:, 33) > 0.5);
R.att_y63_ok = isequal(R.tel(:, 63) > 0.5, val) && all(ismember(R.tel(:, 63), [0 1]));
R.att_cero_sin_valido = all(R.tel(~val, 61:63) == 0, 'all') && all(R.tel(:, 64) == 0);
k1 = val & eid == 1; k2 = val & eid == 2;
err = [abs(R.tel(k1, 61:62) - A(k1, 1:2)); abs(R.tel(k2, 61:62) - A(k2, 3:4))];
R.att_err_max = maxv(err(:));
R.att_n_valido = [sum(k1) sum(k2) sum(~val)];
R.alerta_max = max(R.tel(:, 55));
R.alertas = unique(R.tel(:, 55))';
R.n_visitados = max(R.tel(:, 10));
R.modo_final = R.cse(end);
R.t_modo = primero(R.t, abs(diff([R.cse(1); R.cse])) > 0.5);
fprintf('[%s] %.0f s simulados en %.1f s de reloj  opt = %s\n', R.version, R.t(end), R.t_reloj, texto_opt(R.opt));
fprintf('  estados: %s | explorer_id = %d\n', mat2str(R.estados), R.explorer_id);
fprintf('  t por estado: %s\n', mat2str(R.t_estados', 4));
fprintf('  arm efectivo t = %s, takeoff efectivo t = %s, despegues = %s, arm+takeoff en el mismo paso = %d\n', ...
    mat2str(R.t_arm, 4), mat2str(R.t_takeoff, 4), mat2str(R.n_despegues), R.arm_y_takeoff_juntos);
fprintf('  distancia horizontal mínima (ambos en el aire): poses %.3f m, consignas %.3f m; |dz| en el aire [%.2f %.2f] m\n', ...
    R.dh_min_aire, R.dh_min_consignas_aire, R.dz_aire);
fprintf('  |dz| mínimo con distancia horizontal < 0.9 m (ambos en el aire): %.3f m (NaN = nunca se acercaron)\n', R.dz_min_cerca);
fprintf('  uno encima del otro (dh < 0.5 m y |dz| < 0.3 m con alguno arriba): %d\n', R.choque);
fprintf('  objetivos visitados = %d, alertas y(55) = %s, DONE t = %.2f\n', R.n_visitados, mat2str(R.alertas), R.t_done);
fprintf('  y(61:63) actitud del explorador: muestras válidas E1/E2/no válidas = %s, y(63) coherente = %d, 0 0 0 sin válido = %d, error max = %.2e rad\n', ...
    mat2str(R.att_n_valido), R.att_y63_ok, R.att_cero_sin_valido, R.att_err_max);
fprintf('  en el suelo t = %s (dh al aterrizar %s m), Stop Allowed t = %.2f, Stop con dron en el aire = %d\n', ...
    mat2str(R.t_suelo, 5), mat2str(R.dh_al_aterrizar, 3), R.t_stop_ok, R.stop_con_dron_en_aire);
fprintf('  Flight State final = %s, Command Source efectivo final = %d (cambio en t = %.2f), peticiones operador = %d\n', ...
    mat2str(R.fp(end,:)), R.modo_final, R.t_modo, max(R.opreq(:,1)));
end

function s = texto_opt(o)
f = fieldnames(o);
s = '';
for k = 1:numel(f), s = [s sprintf('%s=%s ', f{k}, mat2str(o.(f{k}), 4))]; end %#ok<AGROW>
if isempty(s), s = '(nominal)'; end
end

function v = minv(x)
if isempty(x), v = NaN; else, v = min(x); end
end

function v = maxv(x)
if isempty(x), v = NaN; else, v = max(x); end
end

function v = valor_en(tt, x, t)
if isnan(t), v = NaN; return, end
v = x(find(tt >= t, 1));
end

function t = primero(tt, c)
i = find(c, 1);
if isempty(i), t = NaN; else, t = tt(i); end
end

function t = primero_desde(tt, c, t0)
if isnan(t0), t = NaN; return, end
i = find(c & tt > t0 + 5, 1);
if isempty(i), t = NaN; else, t = tt(i); end
end

function A = filas(A)
% To Workspace (Array): señal 1-D -> N x ancho; matriz -> a x b x N.
if ndims(A) == 3
    A = squeeze(permute(A, [3 1 2]));
end
end

function v = col(v)
v = squeeze(v); v = v(:);
end

function poner_codigo(blk, codigo, tam)
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk);
ch.Script = codigo;
for k = 1:size(tam, 1)
    d = ch.find('-isa', 'Stateflow.Data', 'Name', tam{k, 1});
    d.Props.Array.Size = tam{k, 2};
end
end

function terminar(m, dtmp, conservar)
if bdIsLoaded(m), close_system(m, 0); end
if ~conservar
    try, rmdir(dtmp, 's'); catch, end
end
end

function c = codigo_drones(trk2_off, falla, homes)
c = sprintf([ ...
'function [data, pos] = fcn(t, trig1, trig2, des1, des2)\n' ...
'%% 2 drones cinemáticos (solo para la prueba): siguen la consigna con arm y takeoff.\n' ...
'persistent p tp\n' ...
'if isempty(p)\n' ...
'    p = [%g %g 0; %g %g 0];\n' ...
'    tp = t;\n' ...
'end\n' ...
'dt = t - tp; tp = t;\n' ...
'tau = 0.6;\n' ...
'T = [trig1(:)''; trig2(:)''];\n' ...
'D = [des1(1) des1(2) des1(3); des2(1) des2(2) des2(3)];\n' ...
'for i = 1:2\n' ...
'    if i == %g && %g >= 0 && t >= %g\n' ...
'        p(i,3) = max(p(i,3) - 0.5*dt, 0);            %% falla a bordo: cae, sin control\n' ...
'    elseif T(i,3) > 0.5\n' ...
'        p(i,3) = 0;                                  %% E-stop: motores cortados\n' ...
'    elseif T(i,1) > 0.5 && T(i,2) > 0.5\n' ...
'        p(i,:) = p(i,:) + (D(i,:) - p(i,:))*min(1, dt/tau);\n' ...
'    elseif p(i,3) > 0\n' ...
'        p(i,3) = max(p(i,3) - 0.4*dt, 0);            %% autoland a bordo\n' ...
'    end\n' ...
'end\n' ...
'trk2 = 1;\n' ...
'if t >= %g && t < %g\n' ...
'    trk2 = 0;                                        %% OptiTrack pierde al dron 2\n' ...
'end\n' ...
'%% roll y pitch conocidos (rad, < 3°) para verificar y(61:63) del TELEMETRY PACK\n' ...
'att = [0.04*sin(0.5*t) -0.03*cos(0.3*t) 0.05*sin(0.7*t) 0.035*sin(0.2*t + 1)];\n' ...
'%% Data [7n] en RSF como sale de Localization - Optitrack (z incluye el pivot de 0.07 m)\n' ...
'data = [p(1,1); p(1,2); p(1,3)+0.07; att(1); att(2); 0; 1; p(2,1); p(2,2); p(2,3)+0.07; att(3); att(4); 0; trk2];\n' ...
'pos = [p(1,:)''; p(2,:)''];\n'], homes(1,1), homes(1,2), homes(2,1), homes(2,2), ...
    falla(1), falla(2), falla(2), trk2_off(1), trk2_off(2));
end

function c = codigo_python(t_off)
c = sprintf([ ...
'function py = fcn(t, tel)\n' ...
'%% Estación Python simulada (ARQUITECTURA §3.3).\n' ...
'persistent t6 seq cmd hb\n' ...
'if isempty(t6), t6 = -1; seq = 0; cmd = 0; hb = 0; end\n' ...
'py = zeros(32, 1);\n' ...
'if t < %g\n' ...
'    hb = floor(t/0.05) + 1;                %% heartbeat (se congela si Python se cae)\n' ...
'end\n' ...
'py(1) = hb;\n' ...
'if t >= 6 && seq == 0, seq = 1; cmd = 1; end   %% START\n' ...
'if tel(2) == 6\n' ...
'    if t6 < 0, t6 = t; end\n' ...
'    if t - t6 > 6 && seq == 1, seq = 2; cmd = 3; end   %% LAND ALL\n' ...
'else\n' ...
'    t6 = -1;\n' ...
'end\n' ...
'py(2) = cmd; py(3) = seq;\n' ...
'if t >= 30\n' ...
'    py(4) = 1;\n' ...
'    py(5:9) = [1; 1; 0.6; -0.6; 0.9];        %% objetivo id 1 (toma) en (0.6, -0.6)\n' ...
'end\n'], t_off);
end
