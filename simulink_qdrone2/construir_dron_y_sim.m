function R = construir_dron_y_sim(op)
%CONSTRUIR_DRON_Y_SIM Genera de forma programática los modelos del dron MPC
%   y de la simulación SIL dual, y los exporta a versiones anteriores.
%
%   R = construir_dron_y_sim()            % todo
%   R = construir_dron_y_sim(struct('sim', false))   % solo el dron, etc.
%
%   Campos de op (todos opcionales, por defecto true):
%     dron      : genera mpc/QD2_DroneStack_MPC.slx a partir de una COPIA de
%                 equipo2_original/QD2_DroneStack_PID_2021a.slx (el original
%                 no se toca). Dentro de COMMANDER (ANGLE MODE) agrega el MPC
%                 de posición a 50 Hz (qd2_mpc_position) con Rate Transition
%                 y un switch PID/MPC; extiende Communication a 24/8 doubles
%                 (ARQUITECTURA.md §3.2).
%     sim       : genera sim/QD2_Sim_Dual.slx (ver sim/qd2_construir_sim.m).
%     exportar  : exporta ambos a versiones/{R2022a,R2022b,R2023a,R2023b,R2025a}.
%     capturas  : guarda PNG del nivel superior (y de COMMANDER/Communication)
%                 en capturas/.
%
%   Requisitos: MATLAB + Simulink (R2022a o posterior) con QUARC instalado
%   (las librerías de QUARC deben estar en el path para abrir el modelo del
%   dron). No necesita Stateflow ni Simulink Coder para construir.

if nargin < 1, op = struct(); end
d = struct('dron', true, 'sim', true, 'exportar', true, 'capturas', true);
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(op, f{i}), op.(f{i}) = d.(f{i}); end
end
raiz = fileparts(mfilename('fullpath'));          % .../simulink_qdrone2
addpath(fullfile(raiz, 'lib'));
addpath(fullfile(raiz, 'sim'));
addpath(fullfile(raiz, 'equipo2_original'));      % Motor_Mapping_7_Inch.m
asegurar_quarc();
% Archivos generados por Simulink (slprj, .slxc) fuera del repositorio
cache = fullfile(tempdir, 'qd2_cache_simulink');
if ~exist(cache, 'dir'), mkdir(cache); end
Simulink.fileGenControl('set', 'CacheFolder', cache, 'CodeGenFolder', cache);
R = struct();
if op.dron
    R.dron = construir_dron_mpc(raiz);
end
if op.sim
    R.sim = qd2_construir_sim(raiz);
end
if op.capturas
    R.capturas = capturas(raiz, op);
end
if op.exportar
    R.versiones = exportar(raiz, op);
end
end

% =========================================================================
function asegurar_quarc()
% Si MATLAB arrancó sin el startup de QUARC (p. ej. -batch con un startup.m
% que corta matlabrc) la carpeta principal de QUARC no queda en el path.
if exist('qc_check_arguments', 'file') == 0
    c = 'C:/Program Files/Quanser/QUARC/quarc';
    if exist(c, 'dir')
        addpath(c, '-end');
    end
end
end

% =========================================================================
function info = construir_dron_mpc(raiz)
src = fullfile(raiz, 'equipo2_original', 'QD2_DroneStack_PID_2021a.slx');
dst_dir = fullfile(raiz, 'mpc');
if ~exist(dst_dir, 'dir'), mkdir(dst_dir); end
m = 'QD2_DroneStack_MPC';
dst = fullfile(dst_dir, [m '.slx']);
if bdIsLoaded(m), close_system(m, 0); end
if bdIsLoaded('QD2_DroneStack_PID_2021a'), close_system('QD2_DroneStack_PID_2021a', 0); end
copyfile(src, dst, 'f');
fileattrib(dst, '+w');
load_system(dst);
fprintf('[dron] copia cargada: %s\n', dst);

% ---------------- Parámetros del modelo ------------------------------------
% InitFcn: busca simulink_qdrone2/lib subiendo carpetas (funciona desde mpc/
% y desde versiones/RXXXX/) y luego corre el mapeo de motores original.
initfcn = sprintf([ ...
    '%% QD2_DroneStack_MPC: rutas de la librería qd2_* y del mapeo de motores\n' ...
    'qd2_d = fileparts(get_param(bdroot, ''FileName''));\n' ...
    'for qd2_k = 1:4\n' ...
    '    if exist(fullfile(qd2_d, ''lib'', ''qd2_params.m''), ''file''), break; end\n' ...
    '    qd2_d = fileparts(qd2_d);\n' ...
    'end\n' ...
    'addpath(fullfile(qd2_d, ''lib''));\n' ...
    'addpath(fullfile(qd2_d, ''equipo2_original''));\n' ...
    'clear qd2_d qd2_k\n' ...
    'Motor_Mapping_7_Inch\n']);
set_param(m, 'InitFcn', initfcn);

% Título "Drone Stack (PID)" -> "Drone Stack (MPC)"
an = find_system(m, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'annotation');
for i = 1:numel(an)
    t = get_param(an(i), 'Text');
    if contains(t, 'Drone Stack (PID)')
        set_param(an(i), 'Text', strrep(t, 'Drone Stack (PID)', 'Drone Stack (MPC)'));
    end
end
% Carpeta de logs propia (To Host File)
th = find_system(m, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Regexp', 'on', 'Name', '^To Host File');
for i = 1:numel(th)
    dp = fieldnames(get_param(th{i}, 'DialogParameters'));
    for k = 1:numel(dp)
        v = get_param(th{i}, dp{k});
        if ischar(v) && contains(v, 'QD2_DroneStack_PID_2021a')
            set_param(th{i}, dp{k}, strrep(v, 'QD2_DroneStack_PID_2021a', m));
        end
    end
end

modificar_communication(m);
modificar_commander(m);
modificar_raiz(m);

% Verificación: actualizar el diagrama (compila los MATLAB Function y
% revisa tamaños, tiempos de muestreo y lazos algebraicos).
info.actualizado = false;
try
    set_param(m, 'SimulationCommand', 'update');
    info.actualizado = true;
    fprintf('[dron] update diagram OK\n');
catch e
    info.error_update = getReport(e, 'basic');
    fprintf(2, '[dron] update diagram FALLÓ:\n%s\n', info.error_update);
end
save_system(m, dst);
if exist([dst '.r2022a'], 'file'), delete([dst '.r2022a']); end   % respaldo automático de Simulink
info.archivo = dst;
fprintf('[dron] guardado %s\n', dst);
end

% =========================================================================
function modificar_communication(m)
c = [m '/Communication'];
% 24 doubles de entrada (el tamaño de la salida rcv lo fija default_value)
set_param([c '/Stream Client'], 'default_value', 'zeros(1,24)');
% Selector del timestamp en Detect Communication Loss (si fija el ancho)
sl = find_system([c '/Detect Communication Loss'], 'LookUnderMasks', 'all', 'BlockType', 'Selector');
for i = 1:numel(sl)
    if ~strcmp(get_param(sl{i}, 'InputPortWidth'), '-1')
        set_param(sl{i}, 'InputPortWidth', '-1');
    end
end
b = find_system(c, 'SearchDepth', 1, 'BlockType', 'Selector');
for i = 1:numel(b)
    if ~strcmp(get_param(b{i}, 'InputPortWidth'), '-1')
        set_param(b{i}, 'InputPortWidth', '-1');
    end
end

% ---- Recepción: índices 17-24 -> Goto global QD2_Datos_MPC ---------------
add_block('simulink/Signal Routing/Selector', [c '/Select MPC data'], ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', '[17:24]', ...
    'InputPortWidth', '-1', 'Position', [475 301 515 339]);
add_block('simulink/Signal Routing/Goto', [c '/Goto MPC data'], ...
    'GotoTag', 'QD2_Datos_MPC', 'TagVisibility', 'global', ...
    'Position', [565 308 695 332], 'ShowName', 'off');
add_line(c, 'Stream Client/4', 'Select MPC data/1', 'autorouting', 'on');
add_line(c, 'Select MPC data/1', 'Goto MPC data/1');

% ---- Envío: [ts, FSM, batería, estado MPC, T, roll, pitch, 0] ------------
mem = [c '/Memory'];
phm = get_param(mem, 'PortHandles');
delete_line(get_param(phm.Outport(1), 'Line'));
add_block('simulink/Signal Routing/From', [c '/From Telem Cmd'], 'GotoTag', 'QD2_Telem_Cmd', ...
    'Position', [-395 -84 -275 -66], 'ShowName', 'off');
add_block('simulink/Signal Routing/From', [c '/From Battery'], 'GotoTag', 'QD2_Bateria_V', ...
    'Position', [-395 -54 -275 -36], 'ShowName', 'off');
% Índice 8 (reservado en §3.2) = 2: etiqueta "protocolo MPC v1". Permite
% que la estación de tierra compruebe que habla con el binario MPC.
add_block('simulink/Sources/Constant', [c '/Reserved'], 'Value', '2', ...
    'Position', [-300 -24 -280 -6], 'ShowName', 'off');
add_block('simulink/Signal Routing/Mux', [c '/Mux Telemetry'], 'Inputs', '3', ...
    'Position', [-240 -90 -235 0], 'ShowName', 'off');
add_block('simulink/Signal Attributes/Data Type Conversion', [c '/convert1'], ...
    'OutDataTypeStr', 'double', 'Position', [-200 -55 -150 -35], 'ShowName', 'off');
add_block('simulink/Discrete/Memory', [c '/Memory1'], 'X0', '[1 3 0 0 0 0 0]', ...
    'Position', [-55 -60 -25 -30], 'ShowName', 'off');
add_block('simulink/Signal Routing/Mux', [c '/Mux Send'], 'Inputs', '2', ...
    'Position', [45 -50 50 70], 'ShowName', 'off');
add_block('simulink/Signal Routing/Selector', [c '/Send order'], ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', '[8 1 6 2 3 4 5 7]', ...
    'InputPortWidth', '8', 'Position', [100 22 140 58]);
set_param([c '/Send order'], 'Name', sprintf('Telemetry [8]\n(ARQUITECTURA 3.2)'));
so = [c '/' get_param([c '/' sprintf('Telemetry [8]\n(ARQUITECTURA 3.2)')], 'Name')];
add_line(c, 'From Telem Cmd/1', 'Mux Telemetry/1');
add_line(c, 'From Battery/1', 'Mux Telemetry/2');
add_line(c, 'Reserved/1', 'Mux Telemetry/3');
add_line(c, 'Mux Telemetry/1', 'convert1/1');
add_line(c, 'convert1/1', 'Memory1/1');
add_line(c, 'Memory1/1', 'Mux Send/1');
add_line(c, 'Memory/1', 'Mux Send/2');
ph_so = get_param(so, 'PortHandles');
ph_mx = get_param([c '/Mux Send'], 'PortHandles');
ph_sc = get_param([c '/Stream Client'], 'PortHandles');
add_line(c, ph_mx.Outport(1), ph_so.Inport(1));
add_line(c, ph_so.Outport(1), ph_sc.Inport(1));

% ---- PROTOCOL CHECK: valida cada trama de 24 doubles ----------------------
% Si el dron MPC se empareja por error con un MissionCtrl de 16 doubles,
% QUARC lee unidades de 192 B sobre un flujo de 128 B y las tramas quedan
% desalineadas (pose y timestamp caerían en arm/takeoff/e-stop). La trama se
% acepta solo si los campos binarios valen exactamente 0 o 1 y el índice 24
% vale 2 (etiqueta de protocolo MPC v1; estricto desde que tierra la manda). Si no, se repite la
% última trama válida: el timestamp se congela y la "Detect Communication
% Loss" original declara Communication Issue (0.5 s) -> Error -> estado 7,
% exactamente como una pérdida de enlace real. Sin ninguna trama válida
% quedan los ceros iniciales (arm = 0, isTracking = 0): el dron no arma.
phc = get_param([c '/Stream Client'], 'PortHandles');
lr = get_param(phc.Outport(4), 'Line');
dst = get_param(lr, 'DstPortHandle');
delete_line(lr);
pc = [c '/PROTOCOL CHECK'];
add_block('simulink/User-Defined Functions/MATLAB Function', pc, ...
    'Position', [322 104 352 131], 'ShowName', 'off', 'BackgroundColor', 'orange');
cfg = get_param(pc, 'MATLABFunctionConfiguration');
cfg.FunctionScript = codigo_protocolo();
% salida 1-D de 24 (como la salida rcv del Stream Client): los selectores
% aguas abajo y las ganancias del PIV esperan vectores 1-D
fijar_tam(pc, {'rcv_ok', '24'});
php = get_param(pc, 'PortHandles');
add_line(c, phc.Outport(4), php.Inport(1));
for k = 1:numel(dst)
    add_line(c, php.Outport(1), dst(k), 'autorouting', 'on');
end
nota(c, sprintf(['MPC (ARQUITECTURA.md 3.2): recibe 24 doubles (17-19 pos. del otro dron, 20-22 vel., 23 mpc_mode, 24 = 2 etiqueta MPC v1, obligatoria)\n' ...
    'y envía 8: [eco ts, FSM, batería (V), estado MPC, T, roll, pitch, 2 = etiqueta protocolo MPC v1]. Las señales nuevas viajan por Goto/From globales.\n' ...
    'PROTOCOL CHECK (bloque naranja junto a rcv): descarta tramas desalineadas (p. ej. un MissionCtrl de 16 doubles) y repite la última válida,\n' ...
    'así el timestamp se congela y la detección de pérdida de comunicación original lleva la FSM a Error (estado 7).']), ...
    [-400 360 720 420]);
end

function s = codigo_protocolo()
s = sprintf([ ...
'function rcv_ok = fcn(rcv)\n' ...
'%%#codegen\n' ...
'%% PROTOCOL CHECK (versión MPC, 24 doubles, ARQUITECTURA.md 3.2).\n' ...
'%% Acepta la trama solo si: arm, takeoff, e-stop, joystick_issue,\n' ...
'%% isTracking (índices 1-5) y mpc_mode (23) valen exactamente 0 o 1, el\n' ...
'%% índice 24 vale 2 (etiqueta de protocolo MPC v1, la manda tierra) y el\n' ...
'%% timestamp (16) es finito y >= 0. Si no, repite la última trama válida\n' ...
'%% (el timestamp congelado dispara la detección de pérdida de enlace).\n' ...
'persistent ult\n' ...
'if isempty(ult)\n' ...
'    ult = zeros(24, 1);\n' ...
'end\n' ...
'ok = true;\n' ...
'idx = [1 2 3 4 5 23];\n' ...
'for k = 1:6\n' ...
'    v = double(rcv(idx(k)));\n' ...
'    if ~(v == 0 || v == 1)\n' ...
'        ok = false;\n' ...
'    end\n' ...
'end\n' ...
'v24 = double(rcv(24));\n' ...
'if ~(v24 == 2)\n' ...
'    ok = false;\n' ...
'end\n' ...
'ts = double(rcv(16));\n' ...
'if ~(isfinite(ts) && ts >= 0)\n' ...
'    ok = false;\n' ...
'end\n' ...
'if ok\n' ...
'    ult = double(rcv(:));\n' ...
'end\n' ...
'rcv_ok = ult;\n']);
end

% =========================================================================
function modificar_raiz(m)
% Batería (DAQ salida 2): el Terminator2 se cambia por un Goto global.
t2 = [m '/Terminator2'];
pos = get_param(t2, 'Position');
pht = get_param(t2, 'PortHandles');
delete_line(get_param(pht.Inport(1), 'Line'));
delete_block(t2);
add_block('simulink/Signal Routing/Goto', [m '/Goto Battery'], 'GotoTag', 'QD2_Bateria_V', ...
    'TagVisibility', 'global', 'Position', [pos(1) pos(2) pos(1)+110 pos(4)], 'ShowName', 'off');
add_line(m, 'QDrone2 DAQ/2', 'Goto Battery/1');
% Actitud estimada (DAQ salida 8) -> Goto global para el MPC
add_block('simulink/Signal Routing/Goto', [m '/Goto Attitude'], 'GotoTag', 'QD2_Actitud_Estimada', ...
    'TagVisibility', 'global', 'Position', [90 1000 230 1020], 'ShowName', 'off');
add_line(m, 'QDrone2 DAQ/8', 'Goto Attitude/1', 'autorouting', 'on');
nota(m, sprintf(['VERSIÓN MPC (copia de QD2_DroneStack_PID_2021a):\n' ...
    '- COMMANDER: MPC de posición a 50 Hz (qd2_mpc_position) en lugar del PIV\n' ...
    '  cuando mpc_mode = 1 (índice 23); con 0, PIV original de respaldo.\n' ...
    '- Communication: 24 doubles de entrada y 8 de salida (ARQUITECTURA 3.2).\n' ...
    '- Goto/From globales nuevos: QD2_Datos_MPC, QD2_Actitud_Estimada,\n' ...
    '  QD2_Bateria_V, QD2_Telem_Cmd.\n' ...
    '- Requiere el MissionCtrl MPC (24 doubles); NO usar con el PID de tierra.\n' ...
    '  Si se empareja mal, Communication/PROTOCOL CHECK descarta las tramas\n' ...
    '  y la FSM original pasa a Error (no arma, o estado 7 en vuelo).']), ...
    [-1745 930 -1500 1060]);
end

% =========================================================================
function modificar_commander(m)
c = [m '/COMMANDER (ANGLE MODE)'];
% --- bloque Product del mapeo XYZYaw -> TRPY (nombre en blanco) ----------
pr = find_system(c, 'SearchDepth', 1, 'BlockType', 'Product');
pmap = '';
for i = 1:numel(pr)
    if isequal(get_param(pr{i}, 'Position'), [-790 906 -745 979])
        pmap = pr{i};
    end
end
assert(~isempty(pmap), 'No encontré el Product del mapeo de comandos');
ph = get_param(pmap, 'PortHandles');
lp = get_param(ph.Outport(1), 'Line');
dsts = get_param(lp, 'DstPortHandle');
delete_line(lp);

% --- Switch PID / MPC -------------------------------------------------------
% Va en el hueco entre el área "2" (Command Mapping, termina en x = -734) y
% las áreas 3/4 (empiezan en x = -681), sin tocar el Matrix Multiply. Su
% entrada 3 (PIV) queda a la altura de la salida del Product (y = 942).
sw = [c '/PID-MPC Switch'];
add_block('simulink/Signal Routing/Switch', sw, 'Criteria', 'u2 > Threshold', ...
    'Threshold', '0.5', 'Position', [-706 897 -688 951], 'ShowName', 'off');
phs = get_param(sw, 'PortHandles');
add_line(c, ph.Outport(1), phs.Inport(3), 'autorouting', 'on');
for k = 1:numel(dsts)
    add_line(c, phs.Outport(1), dsts(k), 'autorouting', 'on');
end
lpid = get_param(phs.Inport(3), 'Line');
set_param(lpid, 'Name', 'PIV');
% El comando del MPC y la bandera "usar MPC" llegan por From locales desde
% el área 5 (sin cables largos que crucen las áreas 3 y 4). Se colocan en el
% hueco libre bajo el scope "Generalized commands".
add_block('simulink/Signal Routing/From', [c '/From MPC_Cmd'], 'GotoTag', 'MPC_Cmd', ...
    'Position', [-860 1072 -770 1088], 'ShowName', 'off');
add_block('simulink/Signal Routing/From', [c '/From MPC_Use'], 'GotoTag', 'MPC_Use', ...
    'Position', [-860 1097 -770 1113], 'ShowName', 'off');
pin1 = get_param(phs.Inport(1), 'Position');
pin2 = get_param(phs.Inport(2), 'Position');
pf1 = get_param([c '/From MPC_Cmd'], 'PortHandles');
pf2 = get_param([c '/From MPC_Use'], 'PortHandles');
q1 = get_param(pf1.Outport(1), 'Position');
q2 = get_param(pf2.Outport(1), 'Position');
add_line(c, [q1; -716 q1(2); -716 pin1(2); pin1]);
add_line(c, [q2; -712 q2(2); -712 pin2(2); pin2]);

% --- Goto locales junto a las fuentes --------------------------------------
inp = @(n) find_system(c, 'SearchDepth', 1, 'BlockType', 'Inport', 'Port', n);
x = inp('8'); goto_local(c, x{1}, 1, 'MPC_IF_Pose', [-610 838 -520 854]);
% bajo el inport 5, a la izquierda de la etiqueta de la ganancia "d" y del
% área "3. Takeoff height state check"
x = inp('5'); goto_local(c, x{1}, 1, 'MPC_HF_Rate', [-1810 1072 -1730 1088]);
goto_local(c, [c '/Memory'], 1, 'MPC_FSM', [60 412 140 428]);
goto_local(c, [c '/Product'], 1, 'MPC_Hover_T', [-600 1086 -520 1102]);
goto_local(c, [c '/THROTTLE SWITCHBOX '], 1, 'MPC_Throttle', [-200 1160 -120 1176]);
goto_local(c, [c '/THROTTLE SWITCHBOX '], 2, 'MPC_AttCmd', [-200 1185 -120 1201]);

% --- Área 5: MPC ------------------------------------------------------------
y0 = 1265;
froms = {'MPC_IF_Pose', 'MPC_HF_Rate', 'QD2_Actitud_Estimada', 'QD2_Datos_MPC', 'MPC_FSM', 'MPC_Hover_T'};
etiq  = {'IF Pose Info [8]', 'HF Pose Rate [4]', 'Estimated Attitude [9]', 'Other drone + mpc_mode [8]', 'FSM State', 'Hover thrust (N)'};
add_block('simulink/Signal Routing/Mux', [c '/MPC Inputs'], 'Inputs', '6', ...
    'Position', [-1090 y0+15 -1085 y0+205], 'ShowName', 'off');
for i = 1:numel(froms)
    yy = y0 + 15 + (i-0.5)*190/6;
    b = [c '/From ' froms{i}];
    add_block('simulink/Signal Routing/From', b, 'GotoTag', froms{i}, ...
        'Position', [-1420 yy-9 -1270 yy+9], 'ShowName', 'off');
    lf = add_line(c, ['From ' froms{i} '/1'], sprintf('MPC Inputs/%d', i));
    set_param(lf, 'Name', etiq{i});
end
rt1 = [c '/RT 1 kHz to 50 Hz'];
add_block('simulink/Signal Attributes/Rate Transition', rt1, 'Integrity', 'on', ...
    'Deterministic', 'on', 'OutPortSampleTime', '20/1000', 'Position', [-1050 y0+92 -1010 y0+128]);
set_param(rt1, 'Name', sprintf('1 kHz -> 50 Hz'));
rt1 = [c '/1 kHz -> 50 Hz'];
mf = [c '/MPC POSITION CONTROLLER (50 Hz)'];
add_block('simulink/User-Defined Functions/MATLAB Function', mf, ...
    'Position', [-960 y0+55 -730 y0+165], 'BackgroundColor', 'orange');
cfg = get_param(mf, 'MATLABFunctionConfiguration');
cfg.FunctionScript = codigo_mpc_onboard();
fijar_tam(mf, {'y', '[7 1]'});
rt2 = [c '/RT 50 Hz to 1 kHz'];
add_block('simulink/Signal Attributes/Rate Transition', rt2, 'Integrity', 'on', ...
    'Deterministic', 'off', 'X0', '[0;0;0;0;0;3;0]', 'OutPortSampleTime', '1/1000', ...
    'Position', [-705 y0+92 -665 y0+128]);
set_param(rt2, 'Name', sprintf('50 Hz -> 1 kHz'));
add_block('simulink/Signal Routing/Demux', [c '/MPC Demux'], 'Outputs', '[4 1 1 1]', ...
    'Position', [-605 y0+60 -600 y0+160], 'ShowName', 'off');
add_block('simulink/Discontinuities/Saturation', [c '/Saturate MPC Command'], ...
    'UpperLimit', '[35 pi/4 pi/4 100*pi/180]', 'LowerLimit', '[-35 -pi/4 -pi/4 -100*pi/180]', ...
    'Position', [-575 y0+38 -545 y0+68]);
add_line(c, 'MPC Inputs/1', '1 kHz -> 50 Hz/1');
add_line(c, '1 kHz -> 50 Hz/1', 'MPC POSITION CONTROLLER (50 Hz)/1');
add_line(c, 'MPC POSITION CONTROLLER (50 Hz)/1', '50 Hz -> 1 kHz/1');
% vector 1-D como la salida del mapeo original (el STABILIZER lo espera así)
add_block('simulink/Math Operations/Reshape', [c '/to 1-D'], 'OutputDimensionality', '1-D array', ...
    'Position', [-650 y0+98 -625 y0+122], 'ShowName', 'off');
add_line(c, '50 Hz -> 1 kHz/1', 'to 1-D/1');
add_line(c, 'to 1-D/1', 'MPC Demux/1');
add_line(c, 'MPC Demux/1', 'Saturate MPC Command/1');
pss = get_param([c '/Saturate MPC Command'], 'PortHandles');
pdm = get_param([c '/MPC Demux'], 'PortHandles');
% Comando y bandera hacia el PID-MPC Switch (área 2) por Goto locales
add_block('simulink/Signal Routing/Goto', [c '/Goto MPC_Cmd'], 'GotoTag', 'MPC_Cmd', ...
    'TagVisibility', 'local', 'Position', [-515 y0+45 -425 y0+61], 'ShowName', 'off');
pgc = get_param([c '/Goto MPC_Cmd'], 'PortHandles');
add_line(c, pss.Outport(1), pgc.Inport(1));
pd2 = get_param(pdm.Outport(2), 'Position');
add_block('simulink/Signal Routing/Goto', [c '/Goto MPC_Use'], 'GotoTag', 'MPC_Use', ...
    'TagVisibility', 'local', 'Position', [-575 pd2(2)-8 -485 pd2(2)+8], 'ShowName', 'off');
pgu = get_param([c '/Goto MPC_Use'], 'PortHandles');
add_line(c, pdm.Outport(2), pgu.Inport(1));
% Telemetría [FSM, estado MPC, T, roll, pitch] -> Goto global QD2_Telem_Cmd
add_block('simulink/Signal Routing/From', [c '/From FSM telem'], 'GotoTag', 'MPC_FSM', ...
    'Position', [-470 y0+120 -390 y0+136], 'ShowName', 'off');
add_block('simulink/Signal Routing/From', [c '/From Throttle telem'], 'GotoTag', 'MPC_Throttle', ...
    'Position', [-470 y0+170 -390 y0+186], 'ShowName', 'off');
add_block('simulink/Signal Routing/From', [c '/From Att telem'], 'GotoTag', 'MPC_AttCmd', ...
    'Position', [-470 y0+195 -390 y0+211], 'ShowName', 'off');
add_block('simulink/Signal Routing/Selector', [c '/Roll Pitch'], 'IndexOptions', 'Index vector (dialog)', ...
    'Indices', '[1 2]', 'InputPortWidth', '3', 'Position', [-365 y0+193 -335 y0+213], 'ShowName', 'off');
add_block('simulink/Signal Attributes/Data Type Conversion', [c '/double FSM'], ...
    'OutDataTypeStr', 'double', 'Position', [-370 y0+120 -330 y0+136], 'ShowName', 'off');
add_block('simulink/Signal Routing/Mux', [c '/Mux Telem'], 'Inputs', '4', ...
    'Position', [-290 y0+110 -285 y0+220], 'ShowName', 'off');
add_block('simulink/Signal Routing/Goto', [c '/Goto Telem'], 'GotoTag', 'QD2_Telem_Cmd', ...
    'TagVisibility', 'global', 'Position', [-255 y0+155 -135 y0+175]);
set_param([c '/Goto Telem'], 'Name', sprintf('[FSM, MPC status, T, roll, pitch]\nto Communication'));
gt = [c '/' sprintf('[FSM, MPC status, T, roll, pitch]\nto Communication')];
add_line(c, 'From FSM telem/1', 'double FSM/1');
add_line(c, 'double FSM/1', 'Mux Telem/1');
add_line(c, 'MPC Demux/3', 'Mux Telem/2', 'autorouting', 'on');
add_line(c, 'From Throttle telem/1', 'Mux Telem/3');
add_line(c, 'From Att telem/1', 'Roll Pitch/1');
add_line(c, 'Roll Pitch/1', 'Mux Telem/4');
pmx = get_param([c '/Mux Telem'], 'PortHandles');
pgt = get_param(gt, 'PortHandles');
add_line(c, pmx.Outport(1), pgt.Inport(1));
% Monitor del MPC
add_block('simulink/Sinks/Scope', [c '/MPC monitor'], 'NumInputPorts', '2', ...
    'Position', [-380 y0+20 -350 y0+60], 'BackgroundColor', 'green');
hsc = get_param([c '/MPC monitor'], 'Handle');
set_param(hsc, 'Name', sprintf('MPC status,\nd_hat (m/s^2)'));
psc = get_param(hsc, 'PortHandles');
add_line(c, pdm.Outport(3), psc.Inport(1), 'autorouting', 'on');
add_line(c, pdm.Outport(4), psc.Inport(2), 'autorouting', 'on');
area(c, '5. MPC Position Controller (50 Hz) - versión MPC', [-1440 y0-25 -60 y0+262]);
nota(c, sprintf(['qd2_mpc_position (simulink_qdrone2/lib) a 50 Hz entre Rate Transitions (modelo MultiTasking: corre en su propio hilo).\n' ...
    'Se usa solo si mpc_mode = 1 (índice 23 del paquete), FSM en 4-6 y sin entradas inválidas; si no, pasa el PIV original.\n' ...
    'Estado MPC: 0 ok | 1 FISTA sin converger (se aplica) | 2 entrada inválida -> PIV | 3 inactivo (FSM fuera de 4-6) | 4 PIV elegido (MPC en sombra).']), ...
    [-1420 y0+215 -300 y0+255]);
end

% =========================================================================
function goto_local(c, fuente, puerto, tag, pos)
b = [c '/Goto ' tag];
add_block('simulink/Signal Routing/Goto', b, 'GotoTag', tag, 'TagVisibility', 'local', ...
    'Position', pos, 'ShowName', 'off');
pf = get_param(fuente, 'PortHandles');
pg = get_param(b, 'PortHandles');
add_line(c, pf.Outport(puerto), pg.Inport(1), 'autorouting', 'on');
end

function fijar_tam(blk, lista)
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk);
for j = 1:size(lista, 1)
    dd = ch.find('-isa', 'Stateflow.Data', 'Name', lista{j,1});
    dd.Props.Array.Size = lista{j,2};
end
end

function area(sys, texto, pos)
add_block('built-in/Area', [sys '/' strrep(texto, '/', '//')], 'Position', pos);
end

function nota(sys, texto, pos)
h = Simulink.Annotation([sys '/' strrep(texto, '/', '//')]);
h.Position = pos;
h.HorizontalAlignment = 'left';
try
    h.FontSize = 11;
catch
end
end

% =========================================================================
function s = codigo_mpc_onboard()
s = sprintf([ ...
'function y = fcn(u)\n' ...
'%%#codegen\n' ...
'%% MPC DE POSICIÓN A BORDO (50 Hz). Sustituye al PIV del COMMANDER cuando\n' ...
'%% mpc_mode = 1 (índice 23 del paquete PC->dron, ARQUITECTURA.md 3.2).\n' ...
'%% u (31x1): [IF Pose Info(8) = medida x y z yaw + deseada x y z yaw,\n' ...
'%%            HF Pose Rate(4) = vx vy vz (horizon frame) + yaw rate,\n' ...
'%%            Actitud estimada(9) = roll pitch yaw ..., datos MPC(8) =\n' ...
'%%            otro dron x y z vx vy vz + mpc_mode + reservado,\n' ...
'%%            estado FSM, empuje de hover (N)]\n' ...
'%% y (7x1):  [comando (4) en el formato del PIV: T-trim, roll, pitch,\n' ...
'%%            yaw_rate; usar_mpc (0/1); estado MPC; d_hat (m/s^2)]\n' ...
'%% Estado MPC: 0 ok, 1 FISTA sin converger (se aplica igual), 2 entrada\n' ...
'%% inválida (pasa el PIV), 3 inactivo (FSM fuera de 4-6), 4 PIV elegido\n' ...
'%% (mpc_mode = 0, el MPC corre en sombra).\n' ...
'%% Parámetros: qd2_params() se evalúa al COMPILAR (coder.const), así que el\n' ...
'%% Jetson no corre la Riccati, expm ni los waypoints en la primera llamada.\n' ...
'P0 = coder.const(@feval, ''qd2_params'');\n' ...
'persistent p mst activo_prev modo_prev masa_ok\n' ...
'if isempty(p)\n' ...
'    p = P0;\n' ...
'    mst = qd2_mpc_init(p);\n' ...
'    activo_prev = 0;\n' ...
'    modo_prev = 0;\n' ...
'    masa_ok = 0;\n' ...
'end\n' ...
'pose_if = u(1:8);\n' ...
'v_hf    = u(9:11);\n' ...
'att     = u(13:21);\n' ...
'otro    = u(22:27);\n' ...
'modo    = double(u(28) > 0.5);\n' ...
'fsm     = u(30);\n' ...
'T_hover = u(31);\n' ...
'%% El MPC sigue al "Drone Weight" del modelo (mismo trim que el PIV). Si la\n' ...
'%% masa difiere de qd2_params, se reconstruye UNA vez todo el modelo del MPC\n' ...
'%% (trim, masa, matrices del eje z y cotas) con qd2_mpc_build. Solo se hace\n' ...
'%% en tierra (FSM 1-2), nunca en vuelo.\n' ...
'if masa_ok < 0.5 && fsm <= 2 && isfinite(T_hover) && T_hover > 1\n' ...
'    masa_ok = 1;\n' ...
'    if abs(T_hover - p.T_trim) > 1e-3\n' ...
'        b.g = p.g;\n' ...
'        b.masa = T_hover/p.g;\n' ...
'        b.T_trim = T_hover;\n' ...
'        b.planta.tau_att = p.planta.tau_att;\n' ...
'        b.pid.Kp = p.pid.Kp;\n' ...
'        p.mpc = qd2_mpc_build(b);\n' ...
'        p.T_trim = T_hover;\n' ...
'        p.masa = b.masa;\n' ...
'        mst = qd2_mpc_init(p);\n' ...
'    end\n' ...
'end\n' ...
'activo = (fsm >= 4) && (fsm <= 6);\n' ...
'%% Reinicio al entrar a TAKEOFF_HEIGHT (como el integrador del PIV) y al\n' ...
'%% pasar de PIV a MPC en vuelo (el observador en sombra no vio el empuje real)\n' ...
'if activo && (activo_prev < 0.5 || (modo > 0.5 && modo_prev < 0.5))\n' ...
'    mst = qd2_mpc_init(p);\n' ...
'end\n' ...
'psi = pose_if(4);\n' ...
'c = cos(psi); s = sin(psi);\n' ...
'%% velocidad del horizon frame al marco inercial (yaw medido)\n' ...
'x_est = [pose_if(1); pose_if(2); pose_if(3); ...\n' ...
'         c*v_hf(1) - s*v_hf(2); s*v_hf(1) + c*v_hf(2); v_hf(3); ...\n' ...
'         att(1); att(2); psi];\n' ...
'ref = [pose_if(5); pose_if(6); pose_if(7); pose_if(8)];\n' ...
'cmd = zeros(4,1);\n' ...
'usar = 0;\n' ...
'estado = 3;\n' ...
'd_hat = 0;\n' ...
'if activo\n' ...
'    [uu, mst, info] = qd2_mpc_position(x_est, ref, otro, p, mst);\n' ...
'    estado = uu(5);\n' ...
'    d_hat = info.d_hat;\n' ...
'    cmd = [uu(1) - p.T_trim; uu(2); uu(3); uu(4)];\n' ...
'    if estado < 1.5 && all(isfinite(cmd))\n' ...
'        if modo > 0.5\n' ...
'            usar = 1;\n' ...
'        else\n' ...
'            estado = 4;\n' ...
'        end\n' ...
'    else\n' ...
'        estado = 2;\n' ...
'        cmd = zeros(4,1);\n' ...
'    end\n' ...
'end\n' ...
'activo_prev = double(activo);\n' ...
'modo_prev = modo;\n' ...
'y = [cmd; usar; estado; d_hat];\n']);
end

% =========================================================================
function C = capturas(raiz, op)
od = fullfile(raiz, 'capturas');
if ~exist(od, 'dir'), mkdir(od); end
C = {};
% La simulación primero: al imprimir después de cargar el modelo del dron
% (con imágenes de QUARC) a veces se colaban sus logotipos en la captura.
if op.sim || exist(fullfile(raiz, 'sim', 'QD2_Sim_Dual.slx'), 'file')
    load_system(fullfile(raiz, 'sim', 'QD2_Sim_Dual.slx'));
    lista = {'QD2_Sim_Dual', 'QD2_Sim_Dual_raiz'};
    sub = find_system('QD2_Sim_Dual', 'SearchDepth', 1, 'BlockType', 'SubSystem');
    for i = 1:numel(sub)
        if isempty(get_param(sub{i}, 'ReferenceBlock')) && strcmp(get_param(sub{i}, 'SFBlockType'), 'NONE')
            n = regexprep(get_param(sub{i}, 'Name'), '[^A-Za-z0-9]+', '_');
            lista = [lista; {sub{i}, ['QD2_Sim_Dual_' n]}]; %#ok<AGROW>
        end
    end
    C = [C imprimir(lista, od)];
    close_system('QD2_Sim_Dual', 0);
end
if op.dron || exist(fullfile(raiz, 'mpc', 'QD2_DroneStack_MPC.slx'), 'file')
    load_system(fullfile(raiz, 'mpc', 'QD2_DroneStack_MPC.slx'));
    lista = {'QD2_DroneStack_MPC', 'QD2_DroneStack_MPC_raiz'; ...
        'QD2_DroneStack_MPC/COMMANDER (ANGLE MODE)', 'QD2_DroneStack_MPC_COMMANDER'; ...
        'QD2_DroneStack_MPC/Communication', 'QD2_DroneStack_MPC_Communication'};
    C = [C imprimir(lista, od)];
end
end

function C = imprimir(lista, od)
C = {};
for i = 1:size(lista, 1)
    f = fullfile(od, [lista{i,2} '.png']);
    try
        print(['-s' lista{i,1}], '-dpng', '-r110', f);
        C{end+1} = f; %#ok<AGROW>
    catch e
        fprintf(2, 'captura %s falló: %s\n', lista{i,1}, e.message);
    end
end
end

% =========================================================================
function V = exportar(raiz, op)
vers = {'R2022a', 'R2022b', 'R2023a', 'R2023b', 'R2025a'};
mods = {};
if op.dron || exist(fullfile(raiz, 'mpc', 'QD2_DroneStack_MPC.slx'), 'file')
    mods{end+1} = fullfile(raiz, 'mpc', 'QD2_DroneStack_MPC.slx');
end
if op.sim || exist(fullfile(raiz, 'sim', 'QD2_Sim_Dual.slx'), 'file')
    mods{end+1} = fullfile(raiz, 'sim', 'QD2_Sim_Dual.slx');
end
actual = ['R' version('-release')];
V = {};
for j = 1:numel(mods)
    [~, nm] = fileparts(mods{j});
    load_system(mods{j});
    for i = 1:numel(vers)
        od = fullfile(raiz, 'versiones', vers{i});
        if ~exist(od, 'dir'), mkdir(od); end
        f = fullfile(od, [nm '.slx']);
        if exist(f, 'file'), delete(f); end
        try
            es_sim = strcmp(nm, 'QD2_Sim_Dual');
            if strcmpi(vers{i}, actual)
                if es_sim
                    % copia autocontenida (sin enlaces a QD2_Mission_Lib)
                    save_system(nm, f, 'BreakUserLinks', true);
                    close_system(nm, 0);
                    load_system(mods{j});
                else
                    copyfile(mods{j}, f, 'f');
                end
            else
                % El nombre del archivo exportado no puede coincidir con un
                % modelo cargado: se exporta a temporal y se copia.
                tmp = fullfile(tempdir, ['qd2exp_' vers{i}]);
                if ~exist(tmp, 'dir'), mkdir(tmp); end
                ft = fullfile(tmp, [nm '.slx']);
                if exist(ft, 'file'), delete(ft); end
                if es_sim
                    % los bloques de QD2_Mission_Lib se copian dentro del
                    % modelo: la versión exportada no depende de la librería
                    Simulink.exportToVersion(nm, ft, vers{i}, 'BreakUserLinks', true);
                else
                    Simulink.exportToVersion(nm, ft, vers{i});
                end
                copyfile(ft, f, 'f');
            end
            V{end+1} = f; %#ok<AGROW>
            fprintf('[exportar] %s\n', f);
        catch e
            fprintf(2, '[exportar] %s %s falló: %s\n', nm, vers{i}, e.message);
        end
    end
end
end
