function R = construir_modelos(varargin)
%CONSTRUIR_MODELOS Genera los modelos Simulink de tierra para 2 QDrone 2.
%   R = construir_modelos()  hace todo:
%     1. lib/QD2_Mission_Lib.slx  (librería con los subsistemas nuevos)
%     2. pid/QD2_MissionCtrl_Dual_PID.slx  (tierra, versión PID)
%     3. mpc/QD2_MissionCtrl_Dual_MPC.slx  (tierra, versión MPC)
%     4. Update Diagram de los dos modelos (verificación)
%     5. versiones/{R2022a,R2022b,R2023a,R2023b,R2025a}/ (ExportToVersion)
%     6. capturas/*.png (original vs. nuevo)
%   Opciones (nombre, valor): 'verificar', 'exportar', 'capturas' (true/false).
%
%   Los modelos NUEVOS son COPIAS del modelo de tierra del Equipo2
%   (equipo2_original/QD2_MissionCtrl_NoJoystick_2021a.slx, que no se toca)
%   editadas con la API de Simulink. Todo bloque original se queda donde
%   está (posición, nombre, color); lo nuevo va en espacio libre, en áreas
%   numeradas 7-10 que siguen la convención de las áreas 1-6 originales.
%   El script se puede correr las veces que se quiera: siempre parte del
%   original.
%
%   Requisitos: MATLAB/Simulink R2022a+ y QUARC instalado (bloques de
%   quarc_library). Ver pid/README.md.

op = struct('verificar', true, 'exportar', true, 'capturas', true);
for k = 1:2:numel(varargin)
    op.(lower(varargin{k})) = varargin{k+1};
end

raiz = fileparts(mfilename('fullpath'));
preparar_rutas(raiz);
qd2_cargar_config();                 % QD2CFG, QD2P en el workspace base
orig = fullfile(raiz, 'equipo2_original', 'QD2_MissionCtrl_NoJoystick_2021a.slx');

R = struct();
t0 = tic;
cerrar({'QD2_MissionCtrl_Dual_PID', 'QD2_MissionCtrl_Dual_MPC', 'QD2_Mission_Lib'});
R.lib = construir_libreria(raiz);
R.pid = construir_tierra(raiz, orig, 'PID');
R.mpc = construir_tierra(raiz, orig, 'MPC');
fprintf('Construcción: %.1f s\n', toc(t0));

if op.verificar
    R.verif_pid = verificar(R.pid);
    R.verif_mpc = verificar(R.mpc);
end
if op.capturas
    capturas(raiz, orig, {R.pid, R.mpc});
end
if op.exportar
    R.versiones = exportar(raiz, {R.lib, R.pid, R.mpc});
end
cerrar({'QD2_MissionCtrl_Dual_PID', 'QD2_MissionCtrl_Dual_MPC', 'QD2_Mission_Lib'});
fprintf('Total: %.1f s\n', toc(t0));
end

% #########################################################################
%  LIBRERÍA
% #########################################################################
function archivo = construir_libreria(raiz)
lib = 'QD2_Mission_Lib';
archivo = fullfile(raiz, 'lib', [lib '.slx']);
if exist(archivo, 'file'), delete(archivo); end
new_system(lib, 'Library');
set_param(lib, 'Lock', 'off');

B = bloques_lib();
for i = 1:numel(B)
    crear_bloque_lib(lib, B(i));
end
nota(lib, [40 -150 900 -40], sprintf([ ...
    'QD2_Mission_Lib: subsistemas de misión para 2 QDrone 2 (versión PID y MPC).\n' ...
    'Cada bloque es un MATLAB Function que llama a las funciones de simulink_qdrone2/lib (qd2_*.m).\n' ...
    'Parámetros de máscara: P = QD2P (qd2_params_from_config, lo crea qd2_cargar_config en InitFcn),\n' ...
    'version = 1 (PID) o 2 (MPC), Ts = paso de muestreo del bloque (s). Contrato: docs/ARQUITECTURA.md.']));
set_param(lib, 'Lock', 'on');
save_system(lib, archivo);
fprintf('Librería: %s\n', archivo);
end

function B = bloques_lib()
% Definición de cada bloque: nombre, entradas, salidas, código, tamaños de
% salida, parámetros (además de las entradas) y posición en la librería.
B = struct('nombre', {}, 'ent', {}, 'sal', {}, 'cod', {}, 'tam', {}, 'par', {}, 'pos', {}, 'desc', {}, 'sat', {});

B(end+1) = def('ROLE & HOME ASSIGNMENT', ...
    {'Pose Data QDrone 1 [7]', 'Pose Data QDrone 2 [7]'}, ...
    {'Pose QDrone 1 [4]', 'Pose QDrone 2 [4]', 'isTracking [2]', 'Roles & Homes Preview [8]'}, ...
    {'[4 1]', '[4 1]', '[2 1]', '[8 1]'}, {'P', 'version'}, [40 40 260 160], ...
    ['Separa [x y z yaw] e isTracking de cada dron (Corrected Pose Data [7] = [x y z roll pitch yaw isTracking]) ' ...
     'y muestra en vivo la asignación de roles de qd2_assign_roles: [explorer_id responder_id home1(2) home2(2) ' ...
     'd_homes start_ok]. El dron con menor x es el EXPLORADOR. El MISSION SUPERVISOR fija roles y homes al recibir START; ' ...
     'start_ok = 0 avisa ANTES de START que los homes están demasiado cerca (START se rechazaría, alerta 1).'], ...
    codigo_roles(), '');

B(end+1) = def('MISSION SUPERVISOR', ...
    {'Time (s)', 'Pose QDrone 1 [4]', 'Pose QDrone 2 [4]', 'isTracking [2]', 'Python Cmd [32]', 'Onboard Status [4]'}, ...
    {'Setpoint QDrone 1 [4]', 'Setpoint QDrone 2 [4]', 'Guard Context [7]', 'Mission Info [40]', ...
     'Mission State', 'Cmd QD1 [3]', 'Cmd QD2 [3]'}, ...
    {'[4 1]', '[4 1]', '[7 1]', '[40 1]', '1', '[3 1]', '[3 1]'}, {'P', 'version'}, [340 40 600 260], ...
    ['FSM de misión (qd2_mission_step, ARQUITECTURA §3.3). Commands = [arm takeoff estop] por dron: el ' ...
     'OPERATOR SAFETY LAYER los combina con los switches del operador. Guard Context = [explorer_id ctx_guard(6)]. ' ...
     'Onboard Status [4] = [fsm1 fsm2 cod1 cod2] del MPC LINK CHECK (versión MPC; en PID constante 0 = ' ...
     'desconocido; cod 1 = protocolo MPC incorrecto, 2 = enlace perdido): un dron en vuelo con FSM a bordo 7/8/9 o ' ...
     'sin enlace (o protocolo incorrecto) -> alerta 9 (10), el sano se aparta >= d_min_h a su altura, regresa y ' ...
     'aterriza; DONE. En LANDING, R no baja hasta que E está a >= d_min_h en horizontal. ' ...
     'Mission Info [40] = campos de telemetría para el TELEMETRY PACK (ver codigo_supervisor en construir_modelos.m).'], ...
    codigo_supervisor(), '');

B(end+1) = def('SEPARATION GUARD', ...
    {'Pose QD1 [4]', 'Pose QD2 [4]', 'SP QD1 [4]', 'SP QD2 [4]', 'Guard Context [7]'}, ...
    {'Safe SP QD1 [4]', 'Safe SP QD2 [4]', 'Guard Active'}, ...
    {'[4 1]', '[4 1]', '1'}, {'P'}, [680 40 900 200], ...
    ['Versión PID: nunca uno encima del otro (qd2_separation_guard). Distancia horizontal >= d_min_h entre poses, ' ...
     'consignas y caminos rectos. Traduce dron <-> rol con explorer_id (Guard Context(1)) y reinicia su estado si los ' ...
     'roles cambian. Después de este bloque la consigna DEBE limitarse por norma (SETPOINT RATE LIMITER).'], ...
    codigo_guard(), '');

B(end+1) = def('STACKING INFO', ...
    {'Pose Data QDrone 1 [7]', 'Pose Data QDrone 2 [7]', 'Velocity QDrone 1 [3]', 'Velocity QDrone 2 [3]', 'MPC Mode'}, ...
    {'MPC Extra QDrone 1 [8]', 'MPC Extra QDrone 2 [8]', 'dz (m)'}, ...
    {'[1 8]', '[1 8]', '1'}, {}, [680 260 900 400], ...
    ['Versión MPC: índices 17-24 del mensaje de 24 doubles (§3.2) para cada dron = [x y z vx vy vz del OTRO dron, ' ...
     'mpc_mode, 2]. El índice 24 = 2 es la etiqueta "protocolo MPC v1": el PROTOCOL CHECK a bordo solo acepta ' ...
     'tramas con 24 = 2. Si el otro no está rastreado se mandan NaN (el MPC a bordo desactiva el feedforward de downwash).'], ...
    codigo_stacking(), '');

B(end+1) = def('MPC LINK CHECK', ...
    {'Time (s)', 'Telemetry QD1 [8]', 'Telemetry QD2 [8]', 'Loopback QD1 (s)', 'Loopback QD2 (s)', ...
     'Comm Status QD1', 'Comm Status QD2'}, {'Onboard Status [4]'}, {'[4 1]'}, {'P', 'version'}, [1580 260 1800 460], ...
    ['Versión MPC (qd2_enlace_mpc): valida la respuesta de 8 doubles de cada dron (§3.2): válida si y(8) = 2 ' ...
     '(etiqueta protocolo MPC v1) y la FSM y(2) es un entero 1..9. Si el dron está conectado (loopback < 0.5 s y sin ' ...
     'pérdida de enlace) y responde mal más de 0.5 s seguidos -> protocolo incorrecto (cod 1, alerta 10, dron no ' ...
     'disponible). Sin conexión después de haber estado conectado -> enlace perdido (cod 2; la FSM retenida no se ' ...
     'usa: fsm = 0). Onboard Status = [fsm1 fsm2 cod1 cod2] (fsm 0 = desconocida) -> MISSION SUPERVISOR. Con version = 1 ' ...
     '(PID, el dron solo devuelve el eco) la salida es 0.'], ...
    codigo_enlace(), '');

B(end+1) = def('OPERATOR SAFETY LAYER', ...
    {'Time (s)', 'Switches [4]', 'Cmd QD1 [3]', 'Cmd QD2 [3]', 'Source (0,1)'}, ...
    {'Triggers QD1 [4]', 'Triggers QD2 [4]', 'Source Effective'}, ...
    {'[1 4]', '[1 4]', '1'}, {}, [980 40 1200 200], ...
    ['Versión SIMPLE (la usa QD2_Sim_Dual; interfaz de 5/3 puertos sin cambios). Los modelos de tierra usan ' ...
     'OPERATOR SAFETY LAYER DUAL. En modo MISIÓN (Command Source = 0) arm_i = switch AND misión, takeoff_i = ' ...
     'switch AND misión (solo tras >= 1 s armado), estop_i = switch OR misión. En modo MANUAL Equipo2 (1) el dron 1 ' ...
     'recibe los switches (takeoff también >= 1 s después de armar) y el dron 2 solo el E-stop. El modo solo cambia ' ...
     'con los dos drones desarmados Y los switches Arm y Takeoff abajo (nunca se dispara solo).'], ...
    codigo_operador(false), '');

B(end+1) = def('OPERATOR SAFETY LAYER DUAL', ...
    {'Time (s)', 'Switches [4]', 'Cmd QD1 [3]', 'Cmd QD2 [3]', 'Source (0,1)', 'Flight State [2]', ...
     'Pose Data QDrone 1 [7]', 'Pose Data QDrone 2 [7]', 'Mission Info [40]'}, ...
    {'Triggers QD1 [4]', 'Triggers QD2 [4]', 'Source Effective', 'Operator Request [2]'}, ...
    {'[1 4]', '[1 4]', '1', '[2 1]'}, {'P'}, [1280 260 1500 520], ...
    ['Capa de seguridad del operador (modelos de tierra). Switches = autoridad final. MISIÓN: arm_i = switch AND ' ...
     'misión; Takeoff/Land arriba = PERMISO de despegue (takeoff >= 1 s después de armar); en vuelo, BAJARLO pide ' ...
     'LAND ALL al supervisor (Operator Request -> OPERATOR COMMAND MERGE): aterrizaje ordenado, R primero y E se ' ...
     'aparta; el takeoff de cada dron sigue al supervisor. Respaldo crudo si el supervisor no acepta en 2 s o tras ' ...
     '90 s: suelta el takeoff, pero NUNCA el de un dron que tiene al otro debajo a < d_min_h. E-stop = corte ' ...
     'inmediato. Arm abajo / E-stop en vuelo también piden LAND ALL. MANUAL: como el original (solo dron 1) con ' ...
     'takeoff >= 1 s después de armar. El modo solo cambia en tierra: Flight State [0 0], desarmados, switches ' ...
     'Arm y Takeoff abajo. Operator Request = [contador de peticiones, alerta 5 E-stop / 6 Takeoff abajo / ' ...
     '7 Arm abajo / 8 respaldo crudo].'], ...
    codigo_operador(true), '');

B(end+1) = def('OPERATOR COMMAND MERGE', ...
    {'Python Cmd [32]', 'Operator Request [2]'}, {'Merged Cmd [32]'}, ...
    {'[32 1]'}, {}, [1580 40 1800 140], ...
    ['Inyecta LAND ALL (cmd 3) en el flujo de comandos de Python cuando el OPERATOR SAFETY LAYER DUAL lo pide ' ...
     '(contador Operator Request(1) cambia). Paso 1: heartbeat y cmd_seq distintos -> el supervisor lo acepta ' ...
     'aunque la estación Python esté caída. Paso 2: vuelve a la secuencia real de Python SIN comando, así el ' ...
     'último comando de Python no se re-ejecuta. La entrada Operator Request debe llegar con 1 paso de retardo.'], ...
    codigo_merge(), '');

B(end+1) = def('SETPOINT RATE LIMITER', ...
    {'Time (s)', 'Setpoint [4]', 'Triggers [4]', 'Mission Info [40]'}, {'Desired Pose [4]'}, ...
    {'[1 4]'}, {'P', 'dron'}, [980 260 1200 360], ...
    ['Saturation (como Saturation2 original, con z hasta P.z_max) + limitador de razón por NORMA en x,y ' ...
     '(qd2_rate_limit(...,1)): 1/3 m/s xy, 1/6 m/s z, 30 °/s yaw. Sustituye al Rate Limiter2 por eje (que hace ' ...
     'caminos en L y rompe la separación). Desarmado (Triggers(1) = 0) la consigna pasa sin limitar. ' ...
     'Mientras este dron se aparta de un dron en falla a bordo (Mission Info(37 + dron) = 1) el límite xy es ' ...
     'P.falla_rate_xy (1 m/s). Parámetro dron = 1 o 2.'], ...
    codigo_rate(), 'si');

B(end+1) = def('VELOCITY ESTIMATE', ...
    {'Time (s)', 'Pose Data [7]'}, {'Velocity (m/s) [3]'}, {'[3 1]'}, {}, [40 300 260 380], ...
    ['Velocidad [vx vy vz] a partir de OptiTrack: derivada entre muestras nuevas (100 Hz) + filtro de primer orden ' ...
     '(tau = 0.04 s). Sin rastreo -> 0.'], ...
    codigo_velocidad(), '');

B(end+1) = def('TELEMETRY PACK', ...
    {'Time (s)', 'Mission Info [40]', 'Guard Active (0,1)', 'Pose Data QDrone 1 [7]', 'Pose Data QDrone 2 [7]', ...
     'Desired Pose QDrone 1 [4]', 'Desired Pose QDrone 2 [4]', 'Velocity QDrone 1 [3]', 'Velocity QDrone 2 [3]', ...
     'Drone Status [8]'}, {'Telemetry [64]'}, {'[64 1]'}, {'version'}, [340 300 600 520], ...
    ['Arma los 64 doubles Simulink -> Python (§3.3) con qd2_telemetry_pack. Drone Status = [fsm1 fsm2 bat1 bat2 ' ...
     'comm_issue1 comm_issue2 loopback1 loopback2 (n_peticiones alerta_operador)]; comm_ok = sin issue y loopback ' ...
     '< 0.5 s. y(55) = alerta: la del operador (5-8) si hay, si no la del supervisor (0-4, 9 falla a bordo, ' ...
     '10 protocolo MPC incorrecto). y(56:60) = un objetivo por mensaje, rotando: [id clase x y estado].'], ...
    codigo_telemetria(), '');

B(end+1) = def('MISSION END CHECK', ...
    {'Time (s)', 'Flight State [2]', 'Pose Data QDrone 1 [7]', 'Pose Data QDrone 2 [7]'}, ...
    {'Stop Allowed (0,1)', 'QDrone Finished [2]'}, {'1', '[2 1]'}, {}, [1280 40 1500 180], ...
    ['CORRECCIÓN del paro automático del original (paraba 3 s después de Land aunque el dron siguiera en el aire). ' ...
     'Un dron "terminó" si su Flight_State es 3 (Land) o 9 (MS Stop) Y está en el suelo (rastreado, z < 0.10 m) ' ...
     'durante 3 s seguidos, o lleva 60 s SEGUIDOS sin rastreo en 3/9 (respaldo). Un dron rastreado con z >= 0.10 m ' ...
     'NUNCA permite el paro. Stop Allowed = los dos terminaron (o nunca se armaron) y al menos uno se armó. ' ...
     'El Stop with Message del Flight Plan solo actúa con Stop Allowed = 1.'], ...
    codigo_fin(), '');
end

function b = def(nombre, ent, sal, tam, par, pos, desc, cod, sat)
b = struct('nombre', nombre, 'ent', {ent}, 'sal', {sal}, 'cod', cod, 'tam', {tam}, ...
    'par', {par}, 'pos', pos, 'desc', desc, 'sat', sat);
end

function crear_bloque_lib(lib, b)
blk = [lib '/' b.nombre];
add_block('simulink/Ports & Subsystems/Subsystem', blk, 'Position', b.pos);
delete_line(blk, 'In1/1', 'Out1/1');
delete_block([blk '/In1']);
delete_block([blk '/Out1']);
n_in = numel(b.ent); n_out = numel(b.sal);
alto = max(n_in, n_out)*40 + 40;
x_fn = 260;
if strcmp(b.sat, 'si'), x_fn = 330; end
fn = [blk '/MATLAB Function'];
add_block('simulink/User-Defined Functions/MATLAB Function', fn, 'Position', [x_fn 20 x_fn+220 20+alto]);
for i = 1:n_in
    y = 20 + alto*(i - 0.5)/n_in;
    add_block('simulink/Sources/In1', [blk '/' esc(b.ent{i})], 'Position', [40 y-7 70 y+7]);
end
for i = 1:n_out
    y = 20 + alto*(i - 0.5)/n_out;
    add_block('simulink/Sinks/Out1', [blk '/' esc(b.sal{i})], 'Position', [x_fn+320 y-7 x_fn+350 y+7]);
end
% código y datos del MATLAB Function
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', fn);
ch.Script = b.cod;
% (El paso de muestreo NO se pone en el chart: en R2025a la propiedad
% EMChart.SampleTime ignora la asignación y queda en -1. Se fija en el
% subsistema atómico, ver abajo.)
for k = 1:numel(b.par)
    d = ch.find('-isa', 'Stateflow.Data', 'Name', b.par{k});
    d.Scope = 'Parameter';
    d.Tunable = false;
end
datos_sal = ch.find('-isa', 'Stateflow.Data', 'Scope', 'Output');
for k = 1:numel(datos_sal)
    % el orden de salida es el de la firma
    idx = datos_sal(k).Port;
    datos_sal(k).Props.Array.Size = b.tam{idx};
end
% líneas
for i = 1:n_in
    if strcmp(b.sat, 'si') && i == 2
        sat = [blk '/Saturation'];
        y = 20 + alto*(i - 0.5)/n_in;
        add_block('simulink/Discontinuities/Saturation', sat, 'Position', [200 y-15 230 y+15], ...
            'UpperLimit', '[P.xy_lim(2); P.xy_lim(2); P.z_max; 3*pi]', ...
            'LowerLimit', '[P.xy_lim(1); P.xy_lim(1); 0; -3*pi]', 'ShowName', 'off');
        add_line(blk, [esc(b.ent{i}) '/1'], 'Saturation/1', 'autorouting', 'smart');
        add_line(blk, 'Saturation/1', sprintf('MATLAB Function/%d', i), 'autorouting', 'smart');
    else
        add_line(blk, [esc(b.ent{i}) '/1'], sprintf('MATLAB Function/%d', i), 'autorouting', 'smart');
    end
end
for i = 1:n_out
    add_line(blk, sprintf('MATLAB Function/%d', i), [esc(b.sal{i}) '/1'], 'autorouting', 'smart');
end
% máscara
mk = Simulink.Mask.create(blk);
mk.Type = 'QD2 Mission Lib';
mk.Description = b.desc;
if any(strcmp(b.par, 'P')) || strcmp(b.sat, 'si')
    mk.addParameter('Type', 'edit', 'Name', 'P', 'Prompt', ...
        'Parámetros de misión (struct de qd2_params_from_config):', 'Value', 'QD2P', 'Tunable', 'off');
end
if any(strcmp(b.par, 'version'))
    mk.addParameter('Type', 'edit', 'Name', 'version', 'Prompt', ...
        'Versión (1 = PID, 2 = MPC):', 'Value', '1', 'Tunable', 'off');
end
if any(strcmp(b.par, 'dron'))
    mk.addParameter('Type', 'edit', 'Name', 'dron', 'Prompt', ...
        'QDrone (1 o 2):', 'Value', '1', 'Tunable', 'off');
end
mk.addParameter('Type', 'edit', 'Name', 'Ts', 'Prompt', 'Paso de muestreo (s):', ...
    'Value', '0.001', 'Tunable', 'off');
% Paso de muestreo DISCRETO explícito: subsistema atómico con
% SystemSampleTime = Ts (el MATLAB Function hereda -1 de aquí). Sin esto el
% bloque heredaba el tiempo continuo del solver ode2 cuando sus entradas
% eran constantes (switches) y la compilación fallaba por las variables
% persistentes.
set_param(blk, 'TreatAsAtomicUnit', 'on', 'SystemSampleTime', 'Ts');
set_param(blk, 'BackgroundColor', 'white');
end

% ------------------------- código de los bloques --------------------------
function c = codigo_roles()
c = sprintf([ ...
'function [pose1, pose2, trk, prev] = fcn(d1, d2, P, version)\n' ...
'%%#codegen\n' ...
'%% ROLE & HOME ASSIGNMENT. d1/d2 = Corrected Pose Data [x y z roll pitch yaw isTracking].\n' ...
'%% prev = [explorer_id responder_id home1x home1y home2x home2y d_homes start_ok] (vista previa;\n' ...
'%% el MISSION SUPERVISOR fija roles y homes con START).\n' ...
'pose1 = [d1(1); d1(2); d1(3); d1(6)];\n' ...
'pose2 = [d2(1); d2(2); d2(3); d2(6)];\n' ...
'trk = [d1(7); d2(7)];\n' ...
'[e, r, h1, h2, ok] = qd2_assign_roles(pose1, pose2, trk(1), trk(2));\n' ...
'dh = sqrt((h1(1) - h2(1))^2 + (h1(2) - h2(2))^2);\n' ...
'if version > 1.5\n' ...
'    umbral = P.d_home_min_mpc;\n' ...
'else\n' ...
'    umbral = P.d_min_h + P.guard_margen;\n' ...
'end\n' ...
'prev = [e; r; h1(1); h1(2); h2(1); h2(2); dh; double(ok > 0.5 && dh >= umbral)];\n']);
end

function c = codigo_supervisor()
c = sprintf([ ...
'function [sp1, sp2, ctx, info, estado, cmd1, cmd2] = fcn(t, pose1, pose2, trk, py, ab, P, version)\n' ...
'%%#codegen\n' ...
'%% MISSION SUPERVISOR: qd2_mission_step (FSM de ARQUITECTURA §3.3) para los 2 drones.\n' ...
'%% ab = Onboard Status [fsm1 fsm2 cod1 cod2] (MPC LINK CHECK; fsm 0 = desconocida;\n' ...
'%% cod 0 bien, 1 protocolo MPC incorrecto, 2 enlace perdido).\n' ...
'persistent s\n' ...
'if isempty(s)\n' ...
'    s = qd2_mission_init(P);\n' ...
'end\n' ...
'in.t = t;\n' ...
'in.pose1 = [pose1(1) pose1(2) pose1(3) pose1(4)];\n' ...
'in.pose2 = [pose2(1) pose2(2) pose2(3) pose2(4)];\n' ...
'in.tracked1 = trk(1);\n' ...
'in.tracked2 = trk(2);\n' ...
'in.py = py(:);\n' ...
'in.version = version;\n' ...
'in.fsm_a_bordo = [ab(1) ab(2)];\n' ...
'in.proto_mal = [ab(3) ab(4)];\n' ...
'[s, ~, ~, tel] = qd2_mission_step(s, in, P);\n' ...
'sp1 = [tel.sp1(1); tel.sp1(2); tel.sp1(3); tel.sp1(4)];\n' ...
'sp2 = [tel.sp2(1); tel.sp2(2); tel.sp2(3); tel.sp2(4)];\n' ...
'cmd1 = [tel.arm(1); tel.takeoff(1); tel.estop(1)];\n' ...
'cmd2 = [tel.arm(2); tel.takeoff(2); tel.estop(2)];\n' ...
'ctx = zeros(7, 1);\n' ...
'ctx(1) = tel.explorer_id;\n' ...
'for k = 1:6\n' ...
'    ctx(k+1) = tel.ctx_guard(k);\n' ...
'end\n' ...
'%% Mission Info [40] (orden fijo, lo leen el TELEMETRY PACK, el OPERATOR SAFETY LAYER\n' ...
'%% (info(1)) y el SETPOINT RATE LIMITER (info(38:39)))\n' ...
'info = zeros(40, 1);\n' ...
'info(1) = tel.mission_state;      info(2) = tel.explorer_id;\n' ...
'info(3) = tel.responder_id;       info(4) = tel.explorer_wp_idx;\n' ...
'info(5) = tel.explorer_wp_total;  info(6) = tel.responder_target_id;\n' ...
'info(7) = tel.n_targets_known;    info(8) = tel.n_targets_visited;\n' ...
'info(9) = tel.explorer_done;      info(10) = tel.visited_mask;\n' ...
'info(11) = tel.marking_progress;  info(12) = tel.queue_length;\n' ...
'info(13) = tel.cmd_seq_ack;\n' ...
'info(14) = tel.home1(1);  info(15) = tel.home1(2);\n' ...
'info(16) = tel.home2(1);  info(17) = tel.home2(2);\n' ...
'info(18) = tel.role1;     info(19) = tel.role2;\n' ...
'info(20) = tel.alerta;    info(21) = tel.home_ok;\n' ...
'info(22) = tel.pausa_por_trk;  info(23) = tel.hb_ok;\n' ...
'info(24) = tel.pausa_por_hb;   info(25) = tel.z_E;\n' ...
'info(26) = tel.z_R;       info(27) = tel.d_min_h;\n' ...
'info(28) = tel.t_mark;    info(29) = tel.r_marking;\n' ...
'%% falla a bordo por dron (0, 9, 10), fase de la reacción, objetivo rotando y(56:60)\n' ...
'info(30) = tel.falla(1);  info(31) = tel.falla(2);\n' ...
'info(32) = tel.falla_fase;\n' ...
'for k = 1:5\n' ...
'    info(32 + k) = tel.obj_rot(k);\n' ...
'end\n' ...
'%% 1 = el dron se aparta de un dron en falla (límite xy P.falla_rate_xy)\n' ...
'info(38) = tel.escape(1); info(39) = tel.escape(2);\n' ...
'estado = tel.mission_state;\n']);
end

function c = codigo_enlace()
c = strjoin({
'function est = fcn(t, tel1, tel2, lb1, lb2, cs1, cs2, P, version)'
'%#codegen'
'% MPC LINK CHECK (qd2_enlace_mpc): valida la respuesta de 8 doubles de cada dron'
'% (§3.2: y(8) = 2, FSM entera 1..9). est = [fsm1 fsm2 cod1 cod2]'
'% (cod 0 bien, 1 protocolo incorrecto sostenido, 2 enlace perdido).'
'persistent tm'
'if isempty(tm)'
'    tm = [-1 -1 0 0];'
'end'
'y1 = zeros(1, 8);'
'y2 = zeros(1, 8);'
'for k = 1:8'
'    y1(k) = tel1(k);'
'    y2(k) = tel2(k);'
'end'
'[e, tm] = qd2_enlace_mpc(tm, t, y1, y2, [lb1(1) lb2(1)], [cs1(1) cs2(1)], P, version);'
'est = [e(1); e(2); e(3); e(4)];'
}, newline);
c = [c newline];
end

function c = codigo_guard()
c = sprintf([ ...
'function [sp1o, sp2o, activo] = fcn(pose1, pose2, sp1, sp2, ctx, P)\n' ...
'%%#codegen\n' ...
'%% SEPARATION GUARD (versión PID): qd2_separation_guard por rol.\n' ...
'persistent gs eid_prev\n' ...
'if isempty(gs)\n' ...
'    gs = qd2_guard_init();\n' ...
'    eid_prev = ctx(1);\n' ...
'end\n' ...
'if ctx(1) ~= eid_prev\n' ...
'    gs = qd2_guard_init();   %% los roles cambiaron: el estado ya no aplica\n' ...
'    eid_prev = ctx(1);\n' ...
'end\n' ...
'c6 = [ctx(2) ctx(3) ctx(4) ctx(5) ctx(6) ctx(7)];\n' ...
'p1 = [pose1(1) pose1(2) pose1(3)];\n' ...
'p2 = [pose2(1) pose2(2) pose2(3)];\n' ...
's1 = [sp1(1) sp1(2) sp1(3) sp1(4)];\n' ...
's2 = [sp2(1) sp2(2) sp2(3) sp2(4)];\n' ...
'if ctx(1) == 2\n' ...
'    [eO, rO, act, gs] = qd2_separation_guard(p2, p1, s2, s1, P, gs, c6);\n' ...
'    sp1o = [rO(1); rO(2); rO(3); rO(4)];\n' ...
'    sp2o = [eO(1); eO(2); eO(3); eO(4)];\n' ...
'else\n' ...
'    [eO, rO, act, gs] = qd2_separation_guard(p1, p2, s1, s2, P, gs, c6);\n' ...
'    sp1o = [eO(1); eO(2); eO(3); eO(4)];\n' ...
'    sp2o = [rO(1); rO(2); rO(3); rO(4)];\n' ...
'end\n' ...
'activo = double(act);\n']);
end

function c = codigo_stacking()
c = sprintf([ ...
'function [x1, x2, dz] = fcn(d1, d2, v1, v2, modo)\n' ...
'%%#codegen\n' ...
'%% STACKING INFO (versión MPC): estado del OTRO dron para el MPC a bordo (§3.2, índices 17-24):\n' ...
'%% [x y z vx vy vz del otro, mpc_mode, 2].\n' ...
'x1 = zeros(1, 8);\n' ...
'x2 = zeros(1, 8);\n' ...
'if d2(7) > 0.5\n' ...
'    x1(1:3) = [d2(1); d2(2); d2(3)];\n' ...
'    x1(4:6) = [v2(1); v2(2); v2(3)];\n' ...
'else\n' ...
'    x1(1:6) = NaN;\n' ...
'end\n' ...
'if d1(7) > 0.5\n' ...
'    x2(1:3) = [d1(1); d1(2); d1(3)];\n' ...
'    x2(4:6) = [v1(1); v1(2); v1(3)];\n' ...
'else\n' ...
'    x2(1:6) = NaN;\n' ...
'end\n' ...
'x1(7) = modo;\n' ...
'x2(7) = modo;\n' ...
'%% índice 24 = 2: etiqueta "protocolo MPC v1" (el PROTOCOL CHECK a bordo solo acepta 2)\n' ...
'x1(8) = 2;\n' ...
'x2(8) = 2;\n' ...
'dz = d1(3) - d2(3);\n']);
end

function c = codigo_operador(ext)
% ext = false: OPERATOR SAFETY LAYER (5/3 puertos, la usa QD2_Sim_Dual)
% ext = true : OPERATOR SAFETY LAYER DUAL (modelos de tierra)
if ext
    cab = {
'function [trig1, trig2, modo_ef, opreq] = fcn(t, sw, cmd1, cmd2, modo_sel, fsm, d1, d2, info, P)'
'%#codegen'
'% OPERATOR SAFETY LAYER DUAL. sw = [Arm Takeoff EStop 0] de los manual switches.'
'% modo 0 = MISIÓN (2 drones), 1 = MANUAL Equipo2 (solo dron 1, modelo original).'
'% fsm = Flight State de tierra de cada dron (1 paso de retardo), d1/d2 = Corrected'
'% Pose Data [x y z roll pitch yaw isTracking], info(1) = estado del MISSION SUPERVISOR.'
'EXT = true;'
'D_MIN = P.d_min_h;'
'estado = info(1);'
    };
else
    cab = {
'function [trig1, trig2, modo_ef] = fcn(t, sw, cmd1, cmd2, modo_sel)'
'%#codegen'
'% OPERATOR SAFETY LAYER (versión simple, interfaz de la simulación QD2_Sim_Dual).'
'% sw = [Arm Takeoff EStop 0]. modo 0 = MISIÓN, 1 = MANUAL Equipo2 (solo dron 1).'
'EXT = false;'
'D_MIN = 0.9;'
'estado = -1;'
'fsm = [0 0];'
'd1 = zeros(7, 1);'
'd2 = zeros(7, 1);'
    };
end
cuerpo = {
'T_ARM_MIN = 1.0;    % s armado antes de permitir takeoff (a bordo: trigger_2 en IDLE = Error)'
'T_ACEPTA = 2.0;     % s para que el supervisor acepte el LAND ALL del operador'
'T_RESPALDO = 90.0;  % s máximos con Takeoff/Land abajo antes del aterrizaje crudo'
'Z_SUELO = 0.10;     % m'
'persistent modo t_arm a_prev to_prev sw_prev t_bajo n_req lat_to lat_arm'
'if isempty(modo)'
'    modo = double(modo_sel > 0.5);'
'    t_arm = [-1 -1];'
'    a_prev = [0 0];'
'    to_prev = [0 0];'
'    sw_prev = [double(sw(1) > 0.5) double(sw(2) > 0.5) double(sw(3) > 0.5)];'
'    t_bajo = -1;'
'    n_req = 0;'
'    lat_to = 0;'
'    lat_arm = 0;'
'end'
'sw_arm = sw(1) > 0.5;'
'sw_to = sw(2) > 0.5;'
'sw_es = sw(3) > 0.5;'
'z = [d1(3) d2(3)];'
'rastreo = [d1(7) > 0.5 && isfinite(d1(3)), d2(7) > 0.5 && isfinite(d2(3))];'
'armado = a_prev(1) > 0.5 || a_prev(2) > 0.5;'
'volando = to_prev(1) > 0.5 || to_prev(2) > 0.5;'
''
'% ---- Cambio de modo: SOLO en tierra, con los dos Flight State en Idle (0),'
'% los dos desarmados y los switches Arm y Takeoff ABAJO. Así un cambio del'
'% selector hecho en vuelo queda pendiente y nunca dispara arm/takeoff solo:'
'% el operador tiene que bajar los switches y volver a subirlos.'
'sel = double(modo_sel > 0.5);'
'suelo = true;'
'for i = 1:2'
'    if rastreo(i) && z(i) >= Z_SUELO'
'        suelo = false;'
'    end'
'end'
'if sel ~= modo && fsm(1) == 0 && fsm(2) == 0 && ~armado && ~volando && ~sw_arm && ~sw_to && suelo'
'    modo = sel;'
'    t_arm = [-1 -1];'
'end'
''
'% ---- Peticiones del operador en modo MISIÓN (flancos de los switches)'
'if EXT && modo < 0.5'
'    if sw_prev(2) > 0.5 && ~sw_to && (armado || volando)'
'        n_req = n_req + 1;      % Takeoff/Land abajo -> LAND ALL ordenado'
'        lat_to = 1;'
'        t_bajo = t;'
'    end'
'    if sw_prev(1) > 0.5 && ~sw_arm && armado'
'        n_req = n_req + 1;      % Arm abajo (corte, como el original) -> LAND ALL'
'        lat_arm = 1;'
'    end'
'    if sw_prev(3) < 0.5 && sw_es && armado'
'        n_req = n_req + 1;      % E-stop -> LAND ALL (la misión no sigue)'
'    end'
'end'
'if sw_to'
'    lat_to = 0;'
'    t_bajo = -1;'
'end'
'if sw_arm'
'    lat_arm = 0;'
'end'
'sw_prev = [double(sw_arm) double(sw_to) double(sw_es)];'
''
'% ---- Disparos pedidos'
'if modo > 0.5'
'    a = [double(sw_arm) 0];'
'    to_req = [sw_to false];'
'    es = [double(sw_es) double(sw_es)];'
'else'
'    a = [double(sw_arm && cmd1(1) > 0.5), double(sw_arm && cmd2(1) > 0.5)];'
'    es = [double(sw_es || cmd1(3) > 0.5), double(sw_es || cmd2(3) > 0.5)];'
'    if EXT'
'        % Takeoff/Land = PERMISO para despegar. En vuelo, bajarlo NO corta el'
'        % takeoff de golpe (cada dron aterrizaría en su x,y: E caería sobre R'
'        % en la versión MPC); pide LAND ALL y el takeoff sigue al supervisor.'
'        to_req = [cmd1(2) > 0.5 && (sw_to || to_prev(1) > 0.5), ...'
'                  cmd2(2) > 0.5 && (sw_to || to_prev(2) > 0.5)];'
'    else'
'        to_req = [sw_to && cmd1(2) > 0.5, sw_to && cmd2(2) > 0.5];'
'    end'
'end'
''
'% ---- Respaldo crudo: el supervisor no aceptó el LAND ALL (estado distinto de'
'% LANDING/DONE/en tierra) en T_ACEPTA, o pasaron T_RESPALDO con Takeoff abajo.'
'% Suelta el takeoff, pero NUNCA el de un dron que tiene al otro DEBAJO a'
'% menos de d_min_h en horizontal (aterrizaría encima / en su downwash).'
'respaldo = 0;'
'if EXT && modo < 0.5 && t_bajo >= 0 && volando'
'    acepto = (estado == 7) || (estado == 8) || (estado <= 2);'
'    if (~acepto && (t - t_bajo) >= T_ACEPTA) || ((t - t_bajo) >= T_RESPALDO)'
'        respaldo = 1;'
'        dh = sqrt((d1(1) - d2(1))^2 + (d1(2) - d2(2))^2);'
'        for i = 1:2'
'            o = 3 - i;'
'            debajo = rastreo(o) && rastreo(i) && dh < D_MIN && z(o) < z(i) - 0.15;'
'            if ~debajo'
'                to_req(i) = false;'
'            end'
'        end'
'    end'
'end'
''
'% ---- Takeoff efectivo: >= T_ARM_MIN después de armar (también en MANUAL).'
'to = [0 0];'
'for i = 1:2'
'    if a(i) > 0.5'
'        if t_arm(i) < 0'
'            t_arm(i) = t;'
'        end'
'    else'
'        t_arm(i) = -1;'
'    end'
'    if modo > 0.5'
'        % manual: igual que el original una vez en vuelo (el takeoff sigue al'
'        % switch aunque se baje Arm); para despegar exige 1 s armado'
'        if to_req(i) && (to_prev(i) > 0.5 || (a(i) > 0.5 && (t - t_arm(i)) >= T_ARM_MIN))'
'            to(i) = 1;'
'        end'
'    elseif to_req(i) && a(i) > 0.5 && (t - t_arm(i)) >= T_ARM_MIN'
'        to(i) = 1;'
'    end'
'end'
''
'% ---- Alerta del operador para la telemetría (y(55))'
'codigo = 0;'
'if sw_es'
'    codigo = 5;'
'elseif respaldo > 0.5'
'    codigo = 8;'
'elseif lat_arm > 0.5'
'    codigo = 7;'
'elseif lat_to > 0.5'
'    codigo = 6;'
'end'
'a_prev = a;'
'to_prev = to;'
'trig1 = [a(1) to(1) es(1) 0];'
'trig2 = [a(2) to(2) es(2) 0];'
'modo_ef = modo;'
    };
if ext
    pie = {'opreq = [n_req; codigo];'};
else
    pie = {};
end
c = strjoin([cab; cuerpo; pie(:)], newline);
c = [c newline];
end

function c = codigo_merge()
c = strjoin({
'function pyo = fcn(py, opreq)'
'%#codegen'
'% OPERATOR COMMAND MERGE: inyecta LAND ALL (cmd 3) en los comandos de Python'
'% cuando el OPERATOR SAFETY LAYER DUAL lo pide (opreq(1) = contador).'
'persistent n_prev fase'
'if isempty(n_prev)'
'    n_prev = opreq(1);'
'    fase = 0;'
'end'
'pyo = zeros(32, 1);'
'for k = 1:32'
'    if isfinite(py(k))'
'        pyo(k) = py(k);'
'    end'
'end'
'if opreq(1) ~= n_prev'
'    n_prev = opreq(1);'
'    fase = 1;'
'end'
'if fase == 1'
'    % heartbeat y cmd_seq distintos de los últimos vistos por el supervisor:'
'    % acepta el LAND ALL aunque la estación Python esté caída'
'    pyo(1) = pyo(1) + 0.5;'
'    pyo(2) = 3;'
'    pyo(3) = pyo(3) + 0.5;'
'    fase = 2;'
'elseif fase == 2'
'    % vuelta a la secuencia real de Python SIN comando: el último comando de'
'    % Python (p. ej. un START viejo) no se vuelve a ejecutar'
'    pyo(2) = 0;'
'    fase = 0;'
'end'
}, newline);
c = [c newline];
end

function c = codigo_rate()
c = sprintf([ ...
'function y = fcn(t, u, trig, info, P, dron)\n' ...
'%%#codegen\n' ...
'%% SETPOINT RATE LIMITER: limita x,y por norma (camino recto), z y yaw por eje.\n' ...
'%% info(37 + dron) = 1: el dron se aparta de un dron en falla -> límite xy P.falla_rate_xy.\n' ...
'persistent yp tp\n' ...
'uu = [u(1); u(2); u(3); u(4)];\n' ...
'if isempty(yp)\n' ...
'    yp = uu;\n' ...
'    tp = t;\n' ...
'end\n' ...
'dt = t - tp;\n' ...
'if dt < 0\n' ...
'    dt = 0;\n' ...
'end\n' ...
'tp = t;\n' ...
'if ~all(isfinite(uu))\n' ...
'    uu = yp;\n' ...
'end\n' ...
'if trig(1) < 0.5\n' ...
'    yp = uu;   %% desarmado: sigue la consigna del supervisor sin limitar\n' ...
'else\n' ...
'    rate = [P.pid.rate_xy; P.pid.rate_xy; P.pid.rate_z; P.pid.rate_yaw];\n' ...
'    ie = min(max(round(37 + dron), 38), 39);\n' ...
'    if info(ie) > 0.5\n' ...
'        rate(1) = P.falla_rate_xy;\n' ...
'        rate(2) = P.falla_rate_xy;\n' ...
'    end\n' ...
'    yp = qd2_rate_limit(yp, uu, rate, dt, 1);\n' ...
'end\n' ...
'%% fila [1x4], igual que la salida del Rate Limiter2 original\n' ...
'y = [yp(1) yp(2) yp(3) yp(4)];\n']);
end

function c = codigo_velocidad()
c = sprintf([ ...
'function v = fcn(t, d)\n' ...
'%%#codegen\n' ...
'%% VELOCITY ESTIMATE: derivada entre muestras nuevas de OptiTrack + filtro de 1er orden.\n' ...
'TAU = 0.04;\n' ...
'persistent pl tl vt vf tp\n' ...
'x = [d(1); d(2); d(3)];\n' ...
'if isempty(pl)\n' ...
'    pl = x;\n' ...
'    tl = t - 1;\n' ...
'    vt = zeros(3, 1);\n' ...
'    vf = zeros(3, 1);\n' ...
'    tp = t;\n' ...
'end\n' ...
'dtp = t - tp;\n' ...
'if dtp < 0\n' ...
'    dtp = 0;\n' ...
'end\n' ...
'tp = t;\n' ...
'if all(isfinite(x)) && d(7) > 0.5\n' ...
'    if any(x ~= pl)\n' ...
'        dtm = t - tl;\n' ...
'        if dtm > 1e-4 && dtm < 0.5\n' ...
'            vt = (x - pl)/dtm;\n' ...
'        else\n' ...
'            vt = zeros(3, 1);\n' ...
'        end\n' ...
'        pl = x;\n' ...
'        tl = t;\n' ...
'    elseif t - tl > 0.05\n' ...
'        vt = zeros(3, 1);\n' ...
'    end\n' ...
'else\n' ...
'    vt = zeros(3, 1);\n' ...
'    tl = t - 1;\n' ...
'end\n' ...
'a = dtp/(TAU + dtp);\n' ...
'vf = vf + a*(vt - vf);\n' ...
'v = vf;\n']);
end

function c = codigo_telemetria()
c = sprintf([ ...
'function y = fcn(t, info, guard, d1, d2, des1, des2, v1, v2, st, version)\n' ...
'%%#codegen\n' ...
'%% TELEMETRY PACK: 64 doubles Simulink -> Python (ARQUITECTURA §3.3) con qd2_telemetry_pack.\n' ...
'tel.mission_state = info(1);\n' ...
'tel.explorer_id = info(2);\n' ...
'tel.responder_id = info(3);\n' ...
'tel.explorer_wp_idx = info(4);\n' ...
'tel.explorer_wp_total = info(5);\n' ...
'tel.responder_target_id = info(6);\n' ...
'tel.n_targets_known = info(7);\n' ...
'tel.n_targets_visited = info(8);\n' ...
'tel.explorer_done = info(9);\n' ...
'tel.visited_mask = info(10);\n' ...
'tel.marking_progress = info(11);\n' ...
'tel.queue_length = info(12);\n' ...
'tel.cmd_seq_ack = info(13);\n' ...
'tel.home1 = [info(14) info(15)];\n' ...
'tel.home2 = [info(16) info(17)];\n' ...
'tel.role1 = info(18);\n' ...
'tel.role2 = info(19);\n' ...
'%% y(56:60): objetivo de la tabla que toca en este mensaje [id clase x y estado]\n' ...
'tel.obj_rot = [info(33) info(34) info(35) info(36) info(37)];\n' ...
'%% alerta: la del operador (Drone Status(10): 5 E-stop, 6 Takeoff abajo, 7 Arm abajo,\n' ...
'%% 8 respaldo crudo) tiene prioridad sobre la del supervisor (0-4, 9 falla a bordo,\n' ...
'%% 10 protocolo MPC incorrecto)\n' ...
'alerta = info(20);\n' ...
'if numel(st) >= 10\n' ...
'    ov = st(min(10, numel(st)));\n' ...
'    if ov > 0.5\n' ...
'        alerta = ov;\n' ...
'    end\n' ...
'end\n' ...
'tel.alerta = alerta;\n' ...
'pose1 = [d1(1); d1(2); d1(3); d1(6)];\n' ...
'pose2 = [d2(1); d2(2); d2(3); d2(6)];\n' ...
'%% comm_ok = sin Communication Issue y loopback < 0.5 s\n' ...
'comm1 = double(st(5) < 0.5 && st(7) < 0.5);\n' ...
'comm2 = double(st(6) < 0.5 && st(8) < 0.5);\n' ...
'aux = [d1(7) d2(7) st(1) st(2) st(3) st(4) comm1 comm2];\n' ...
'y = qd2_telemetry_pack(t, tel, version, guard, pose1, pose2, ...\n' ...
'    [des1(1) des1(2) des1(3) des1(4)], [des2(1) des2(2) des2(3) des2(4)], ...\n' ...
'    [v1(1) v1(2) v1(3)], [v2(1) v2(2) v2(3)], aux);\n']);
end

function c = codigo_fin()
c = strjoin({
'function [permitido, fin] = fcn(t, fsm, d1, d2)'
'%#codegen'
'% MISSION END CHECK: el modelo de tierra solo se detiene con los DOS drones en el suelo.'
'% Un dron terminó si su Flight_State es 3 (Land) o 9 (MS Stop) y:'
'%   - está rastreado en el suelo (z < Z_SUELO) T_SUELO s SEGUIDOS, o'
'%   - lleva T_SIN_RASTREO s SEGUIDOS sin rastreo (respaldo: no se puede confirmar).'
'% Un dron rastreado con z >= Z_SUELO NUNCA permite el paro (reinicia los dos relojes).'
'T_SUELO = 3.0;         % s'
'T_SIN_RASTREO = 60.0;  % s'
'Z_SUELO = 0.10;        % m'
'persistent t_suelo t_sin armado'
'if isempty(armado)'
'    t_suelo = [-1 -1];'
'    t_sin = [-1 -1];'
'    armado = [0 0];'
'end'
'z = [d1(3) d2(3)];'
'trk = [d1(7) d2(7)];'
'fin = zeros(2, 1);'
'ok = [false false];'
'for i = 1:2'
'    if fsm(i) > 0.5'
'        armado(i) = 1;'
'    end'
'    termino = (fsm(i) == 3) || (fsm(i) == 9);'
'    rastreado = trk(i) > 0.5 && isfinite(z(i));'
'    en_suelo = rastreado && z(i) < Z_SUELO;'
'    if termino && en_suelo'
'        if t_suelo(i) < 0'
'            t_suelo(i) = t;'
'        end'
'    else'
'        t_suelo(i) = -1;'
'    end'
'    if termino && ~rastreado'
'        if t_sin(i) < 0'
'            t_sin(i) = t;'
'        end'
'    else'
'        t_sin(i) = -1;'
'    end'
'    fin(i) = double(termino && ((t_suelo(i) >= 0 && (t - t_suelo(i)) >= T_SUELO) || ...'
'        (t_sin(i) >= 0 && (t - t_sin(i)) >= T_SIN_RASTREO)));'
'    en_aire = rastreado && z(i) >= Z_SUELO;'
'    ok(i) = fin(i) > 0.5 || (armado(i) < 0.5 && ~en_aire);'
'end'
'permitido = double(ok(1) && ok(2) && (armado(1) > 0.5 || armado(2) > 0.5));'
}, newline);
c = [c newline];
end

% #########################################################################
%  MODELOS DE TIERRA
% #########################################################################
function archivo = construir_tierra(raiz, orig, ver)
es_mpc = strcmp(ver, 'MPC');
mdl = ['QD2_MissionCtrl_Dual_' ver];
carpeta = fullfile(raiz, lower(ver));
if ~exist(carpeta, 'dir'), mkdir(carpeta); end
archivo = fullfile(carpeta, [mdl '.slx']);
if exist(archivo, 'file'), delete(archivo); end
copyfile(orig, archivo, 'f');
fileattrib(archivo, '+w');
w = warning('off', 'all');
load_system(archivo);
warning(w);
nver = 1 + double(es_mpc);
MC = [mdl '/MISSION CONTROL QDRONE POSITION CONTROL'];
lib = 'QD2_Mission_Lib';
load_system(lib);

% =================== NIVEL RAÍZ ==========================================
C1 = [mdl '/Communication QDrone 1'];
set_param([mdl '/Communication'], 'Name', 'Communication QDrone 1');
if es_mpc
    comm_mpc(C1, 1);
end
C2 = [mdl '/Communication QDrone 2'];
add_block(C1, C2, 'Position', [-855 640 -640 720]);
set_param([C2 '/Model Argument'], 'argument_name', 'URI_Host2');
an = find_system(C2, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'annotation');
for k = 1:numel(an)
    txt = get_param(an(k), 'Text');
    if contains(txt, 'URI_Host')
        set_param(an(k), 'Text', strrep(strrep(txt, 'URI_Host', 'URI_Host2'), '18001', '18002'));
    end
end
if es_mpc
    set_param([C2 '/From QDrone MPC Extra'], 'GotoTag', 'QDrone2_MPC_Extra');
    set_param([C2 '/Goto QDrone Telemetry'], 'GotoTag', 'QDrone2_Telemetry');
end
% From / Goto / Display de la raíz para el dron 2 (misma convención: el
% bloque Communication mira a la izquierda)
from_(mdl, 'From QDrone2 Data In', [-595 670 -435 690], 'QDrone2_Data_In', 'left');
goto_(mdl, 'Goto QDrone2 Communication Issue', [-1115 650 -925 670], 'QDrone2_Communication_Issue', 'global', 'left');
goto_(mdl, 'Goto QDrone2 Timestamp Received', [-1115 690 -925 710], 'QDrone2_Timestamp_Received', 'global', 'left');
add_block('simulink/Sinks/Display', [mdl '/Display QDrone 2'], 'Position', [-900 595 -840 615], 'ShowName', 'off');
L(mdl, 'From QDrone2 Data In/1', 'Communication QDrone 2/1');
L(mdl, 'Communication QDrone 2/1', 'Goto QDrone2 Communication Issue/1');
L(mdl, 'Communication QDrone 2/2', 'Goto QDrone2 Timestamp Received/1');
L(mdl, 'Communication QDrone 2/1', 'Display QDrone 2/1');
nota(mdl, [-899 615 -820 626], 'Comm Issue QDrone 2', 'Arial', 8);

% PYTHON BRIDGE (raíz, debajo de las comunicaciones de los drones)
PB = [mdl '/PYTHON BRIDGE'];
construir_puente(PB, [-855 790 -640 870]);
from_(mdl, 'From Python Telemetry', [-595 820 -435 840], 'Python_Telemetry', 'left');
goto_(mdl, 'Goto Python Commands', [-1115 800 -925 820], 'Python_Commands', 'global', 'left');
add_block('simulink/Sinks/Display', [mdl '/Display Python'], 'Position', [-945 835 -885 855], 'ShowName', 'off', 'Orientation', 'left');
L(mdl, 'From Python Telemetry/1', 'PYTHON BRIDGE/1');
L(mdl, 'PYTHON BRIDGE/1', 'Goto Python Commands/1');
L(mdl, 'PYTHON BRIDGE/2', 'Display Python/1');
nota(mdl, [-945 857 -870 868], 'Python Connected', 'Arial', 8);
nota(mdl, [-400 560 -20 960], texto_raiz(es_mpc));
% Aviso junto a las instrucciones del Equipo2 (los pasos 10-13 cambian en modo misión)
h = nota(mdl, [-1444 625 -1130 745], sprintf([ ...
    '2 QDRONES - modo MISIÓN (Command Source = 0):\n' ...
    'los pasos 10-13 cambian. Arm/Disarm y Takeoff/Land\n' ...
    'arriba = PERMISO; despega el START de la UI.\n' ...
    'Bajar Takeoff/Land en vuelo = LAND ALL ordenado\n' ...
    '(R primero, E se aparta). Emergency Stop = corte.\n' ...
    'El modelo se detiene solo con los 2 en el suelo.\n' ...
    'Detalles: nota DOS QDRONES (a la derecha).']));
h.ForegroundColor = 'red';

% Parámetros del modelo
cfg = evalin('base', 'QD2CFG');
set_param(mdl, 'ExtModeMexArgs', cfg.ext_mode_args);
set_param(mdl, 'PreLoadFcn', preload_fcn());
set_param(mdl, 'PostLoadFcn', 'try, qd2_cargar_config(); catch qd2_e, warning(qd2_e.message); end');
set_param(mdl, 'InitFcn', 'qd2_cargar_config(bdroot);');
set_param(mdl, 'Description', sprintf(['Modelo de tierra para 2 QDrone 2 (versión %s). Copia extendida de ' ...
    'QD2_MissionCtrl_NoJoystick_2021a del Equipo2, generada por simulink_qdrone2/construir_modelos.m. ' ...
    'Contrato: docs/ARQUITECTURA.md.'], ver));

% =================== MISSION CONTROL (MC) ================================
% ---- 2. Localization Data Server: 2 rigid bodies -------------------------
OT = [MC '/Localization - Optitrack/OptiTrack Trackables'];
set_param(OT, 'rigid_body_ids', 'QD2CFG.optitrack.rb_ids');
% isTracking? del dron 1 sigue siendo el elemento 7, ahora de un vector de 14
set_param([MC '/isTracking'], 'InputPortWidth', '14');
bias1 = find_system([MC '/Measurement Conditioning'], 'SearchDepth', 1, 'BlockType', 'Bias');
set_param(bias1{1}, 'Bias', 'QD2CFG.optitrack.bias_d1');
% Selector [1:7] en la línea Manual Switch -> Measurement Conditioning
borrar_linea_a([MC '/Measurement Conditioning'], 1);
add_block('simulink/Signal Routing/Selector', [MC '/QDrone 1 [1:7]'], 'Position', [690 503 720 517], ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', '1:7', 'InputPortWidth', '14');
add_block('simulink/Signal Routing/Selector', [MC '/QDrone 2 [8:14]'], 'Position', [560 573 590 587], ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', '8:14', 'InputPortWidth', '14');
goto_(MC, 'Goto Optitrack QDrone2', [610 570 770 590], 'Optitrack_Data_QDrone2', 'local');
L(MC, 'Manual Switch/1', 'QDrone 1 [1:7]/1');
L(MC, 'QDrone 1 [1:7]/1', 'Measurement Conditioning/1');
L(MC, 'Manual Switch/1', 'QDrone 2 [8:14]/1');
L(MC, 'QDrone 2 [8:14]/1', 'Goto Optitrack QDrone2/1');

% ---- 1. Joystick Interface: disparos efectivos del dron 1 ----------------
% Los switches (Joystick Signals) siguen yendo al display, al log y a la
% home pose; el dron 1 y su Flight Plan reciben los disparos efectivos que
% calcula el OPERATOR SAFETY LAYER (From Triggers_QDrone1).
borrar_linea_a([MC '/Subsystem'], 1);
borrar_linea_a([MC '/Triggers (0,1) [4]'], 1);
from_(MC, 'From Triggers QDrone1', [330 218 490 240], 'Triggers_QDrone1');
from_(MC, 'From Triggers QDrone1 Out', [445 327 525 343], 'Triggers_QDrone1');
L(MC, 'From Triggers QDrone1/1', 'Subsystem/1');
L(MC, 'From Triggers QDrone1 Out/1', 'Triggers (0,1) [4]/1');

% ---- Flight Plan (Subsystem): paro solo con los 2 drones en el suelo ------
modificar_flight_plan([MC '/Subsystem']);

% ---- 4. Command Generation: fuente de la consigna del dron 1 -------------
rl = [MC '/Rate Limiter2'];
lh = get_param(rl, 'LineHandles');
delete_line(lh.Outport(1));
add_block('simulink/Signal Routing/Switch', [MC '/Command Source Switch'], 'Position', [1580 405 1610 465], ...
    'Criteria', 'u2 > Threshold', 'Threshold', '0.5', 'ShowName', 'off');
from_(MC, 'From Command Source', [1400 426 1555 444], 'Command_Source_Effective');
from_(MC, 'From Desired Pose Mission QDrone1', [1400 452 1555 470], 'Desired_Pose_Mission_QDrone1');
L(MC, 'Rate Limiter2/1', 'Command Source Switch/1');
L(MC, 'From Command Source/1', 'Command Source Switch/2');
L(MC, 'From Desired Pose Mission QDrone1/1', 'Command Source Switch/3');
outd = find_system(MC, 'SearchDepth', 1, 'BlockType', 'Outport', 'Port', '4');
ph_sw = get_param([MC '/Command Source Switch'], 'PortHandles');
ph_o4 = get_param(outd{1}, 'PortHandles');
ph_g1 = get_param([MC '/Goto1'], 'PortHandles');
add_line(MC, ph_sw.Outport(1), ph_o4.Inport(1), 'autorouting', 'smart');
add_line(MC, ph_sw.Outport(1), ph_g1.Inport(1), 'autorouting', 'smart');

% ---- Áreas nuevas 7-10 ---------------------------------------------------
area_(MC, [96 980 951 1300], '7. Localization Data Server - QDrone 2');
area_(MC, [976 980 1661 1300], '8. Communication, Signal Check & Flight Plan - QDrone 2');
area_(MC, [96 1325 1661 2205], '9. Mission Supervision - 2 QDrones (Python commands -> setpoints)');
area_(MC, [96 2230 1661 2600], '10. Python Bridge Telemetry & Blackbox - 2 QDrones');
nota(MC, [990 -1 1500 127], texto_grupos(es_mpc));

area7(MC);
area8(MC);
area9(MC, lib, es_mpc, nver);
area10(MC, lib, es_mpc, nver);

% Ts de los bloques de librería = paso del modelo (como los bloques QUARC originales)
bl = find_system(MC, 'SearchDepth', 1, 'MaskType', 'QD2 Mission Lib');
for k = 1:numel(bl)
    set_param(bl{k}, 'Ts', 'qc_get_step_size');
end

save_system(mdl, archivo);
if exist([archivo '.r2022a'], 'file'), delete([archivo '.r2022a']); end
fprintf('Modelo %s: %s\n', ver, archivo);
end

% -------------------------------------------------------------------------
function comm_mpc(C, i)
% Versión MPC del subsistema Communication: envía 24 doubles (16 + 8 extra
% de STACKING INFO) y recibe 8 (eco del timestamp + telemetría, §3.2).
SS = [C '/Stream Server'];
% Sin conexión: timestamp 0 (como el original) y NaN en la telemetría a
% bordo (fsm, batería...), como pide §3.3 para valores desconocidos.
set_param(SS, 'default_value', '[0; NaN(7,1)]');
lh = get_param(SS, 'LineHandles');
delete_line(lh.Inport(1));
add_block('simulink/Signal Routing/Mux', [C '/Mux 24'], 'Position', [250 22 255 58], 'Inputs', '2', 'ShowName', 'off');
from_(C, 'From QDrone MPC Extra', [165 50 235 64], sprintf('QDrone%d_MPC_Extra', i));
L(C, 'Data In/1', 'Mux 24/1');
L(C, 'From QDrone MPC Extra/1', 'Mux 24/2');
L(C, 'Mux 24/1', 'Stream Server/1');
delete_line(lh.Outport(4));
add_block('simulink/Signal Routing/Selector', [C '/Timestamp [1]'], 'Position', [500 98 530 112], ...
    'IndexOptions', 'Index vector (dialog)', 'Indices', '1', 'InputPortWidth', '8', 'ShowName', 'off');
goto_(C, 'Goto QDrone Telemetry', [470 145 640 165], sprintf('QDrone%d_Telemetry', i), 'global');
L(C, 'Stream Server/4', 'Timestamp [1]/1');
L(C, 'Timestamp [1]/1', 'Timestamp Received (s)/1');
L(C, 'Stream Server/4', 'Goto QDrone Telemetry/1');
nota(C, [450 170 700 210], sprintf(['Versión MPC: envía 24 doubles (16 originales + 8 de STACKING INFO)\n' ...
    '(índice 24 = 2, etiqueta MPC v1) y recibe 8: [timestamp fsm bateria mpc_status T roll pitch 2] (§3.2).']));
end

function construir_puente(PB, pos)
add_block('simulink/Ports & Subsystems/Subsystem', PB, 'Position', pos, ...
    'BackgroundColor', 'orange', 'Orientation', 'left');
set_param(PB, 'ContentPreviewEnabled', 'off');
delete_line(PB, 'In1/1', 'Out1/1');
delete_block([PB '/In1']);
delete_block([PB '/Out1']);
add_block('simulink/Sources/In1', [PB '/Telemetry [64]'], 'Position', [40 33 70 47]);
add_block('simulink/Signal Attributes/Rate Transition', [PB '/Rate Transition1'], 'Position', [130 30 150 50], ...
    'OutPortSampleTime', 'qc_get_step_size*20', 'ShowName', 'off');
add_block('simulink/Sources/Constant', [PB '/Constant'], 'Position', [200 72 210 88], 'Value', '1', 'ShowName', 'off');
SS = [PB '/Stream Server'];
add_block('quarc_library/Communications/Basic/Stream Server', SS, 'Position', [295 22 395 138]);
set_param(SS, 'uri_source', 'Specify via dialog (evaluate)', 'uri', 'QD2CFG.uri_puente', ...
    'send_buffer_size', '1460', 'receive_buffer_size', '1460', ...
    'byte_order', 'little endian (Intel - LSB first)', 'optimization', 'Minimum latency', ...
    'send_options', 'Send most recent data', 'receive_options', 'Receive most recent data', ...
    'default_value', 'zeros(32,1)', 'sample_time', 'qc_get_step_size*20', ...
    'implementation', 'Use blocking I/O in a separate thread', 'data_type', 'double');
add_block('simulink/Logic and Bit Operations/Compare To Constant', [PB '/Connected'], 'Position', [470 18 500 42], ...
    'relop', '==', 'const', '2', 'ShowName', 'off');
add_block('simulink/Signal Attributes/Data Type Conversion', [PB '/Data Type Conversion'], 'Position', [530 23 570 37], ...
    'OutDataTypeStr', 'double', 'ShowName', 'off');
add_block('simulink/Signal Attributes/Rate Transition', [PB '/Rate Transition2'], 'Position', [470 95 490 115], ...
    'OutPortSampleTime', 'qc_get_step_size', 'ShowName', 'off');
add_block('simulink/Sinks/Terminator', [PB '/Terminator1'], 'Position', [420 50 430 60], 'ShowName', 'off');
add_block('simulink/Sinks/Terminator', [PB '/Terminator2'], 'Position', [420 75 430 85], 'ShowName', 'off');
add_block('simulink/Sinks/Terminator', [PB '/Terminator3'], 'Position', [420 125 430 135], 'ShowName', 'off');
add_block('simulink/Sinks/Out1', [PB '/Python Commands [32]'], 'Position', [665 98 695 112]);
add_block('simulink/Sinks/Out1', [PB '/Python Connected (0,1)'], 'Position', [665 23 695 37]);
L(PB, 'Telemetry [64]/1', 'Rate Transition1/1');
L(PB, 'Rate Transition1/1', 'Stream Server/1');
L(PB, 'Constant/1', 'Stream Server/2');
L(PB, 'Stream Server/1', 'Connected/1');
L(PB, 'Connected/1', 'Data Type Conversion/1');
L(PB, 'Data Type Conversion/1', 'Python Connected (0,1)/1');
L(PB, 'Stream Server/2', 'Terminator1/1');
L(PB, 'Stream Server/3', 'Terminator2/1');
L(PB, 'Stream Server/4', 'Rate Transition2/1');
L(PB, 'Rate Transition2/1', 'Python Commands [32]/1');
L(PB, 'Stream Server/5', 'Terminator3/1');
nota(PB, [60 170 700 290], sprintf([ ...
    'PYTHON BRIDGE (ARQUITECTURA §3.3): Simulink es el Stream Server en QD2CFG.uri_puente = tcpip://localhost:18700\n' ...
    '(puerto red.puerto_puente de config/estacion.json). La estación Python (web/) es el cliente TCP.\n' ...
    'Envía 64 doubles de telemetría a 50 Hz y recibe 32 doubles de comandos/detecciones (most recent data).\n' ...
    'Sin cliente conectado la salida queda en ceros (heartbeat = 0): el MISSION SUPERVISOR no arranca la misión\n' ...
    'y, si ya estaba en MISSION, pasa a PAUSED al perder el heartbeat más de 1 s.']));
end

function modificar_flight_plan(S)
% Inserta la condición "los dos drones en el suelo" (MISSION END CHECK)
% antes del Delay de 3 s que dispara el Stop with Message original.
lh = get_param([S '/OR'], 'LineHandles');
delete_line(lh.Outport(1));
add_block('simulink/Logic and Bit Operations/Logical Operator', [S '/AND Stop Allowed'], ...
    'Position', [445 160 470 195], 'Operator', 'AND', 'Inputs', '2', 'ShowName', 'off');
from_(S, 'From Stop Allowed', [330 222 435 238], 'Stop_Allowed');
L(S, 'OR/1', 'AND Stop Allowed/1');
L(S, 'From Stop Allowed/1', 'AND Stop Allowed/2');
L(S, 'AND Stop Allowed/1', 'Delay/1');
nota(S, [330 250 640 300], sprintf(['CORRECCIÓN (2 drones): el modelo de tierra ya no se detiene 3 s después de Land.\n' ...
    'Stop Allowed (MISSION END CHECK) = los DOS drones en el suelo >= 3 s (o 60 s seguidos sin rastreo).\n' ...
    'Después sigue el Delay original de 3 s y el Stop with Message.']));
end

% -------------------------------------------------------------------------
function area7(MC)
% 7. Localization Data Server - QDrone 2 (+ paquete de 16/24 doubles del dron 2)
from_(MC, 'From Optitrack QDrone2', [125 1100 295 1120], 'Optitrack_Data_QDrone2');
MS2 = [MC '/Measurement Conditioning QDrone 2'];
add_block([MC '/Measurement Conditioning'], MS2, 'Position', [340 1081 460 1139]);
b2 = find_system(MS2, 'SearchDepth', 1, 'BlockType', 'Bias');
set_param(b2{1}, 'Bias', 'QD2CFG.optitrack.bias_d2');
add_block([MC '/isTracking?'], [MC '/isTracking QDrone 2?'], 'Position', [540 1010 585 1058]);
goto_(MC, 'Goto Corrected Measured Pose D2', [520 1225 705 1255], 'Corrected_Measured_Pose_D2', 'local');
add_block('simulink/Signal Routing/Mux', [MC '/Mux QDrone 2'], 'Position', [800 1020 805 1220], 'Inputs', '5', 'ShowName', 'off');
from_(MC, 'From Triggers QDrone2', [610 1030 770 1050], 'Triggers_QDrone2');
from_(MC, 'From Desired Pose QDrone2', [610 1150 770 1170], 'Desired_Pose_QDrone2');
add_block('quarc_library/Sources/Time/Time', [MC '/Time QDrone 2'], 'Position', [700 1185 745 1217], ...
    'time_source', 'Wall clock', 'sample_time', 'qc_get_step_size');
goto_(MC, 'Goto QDrone2 Data In', [830 1110 945 1130], 'QDrone2_Data_In', 'global');
L(MC, 'From Optitrack QDrone2/1', 'Measurement Conditioning QDrone 2/1');
L(MC, 'From Triggers QDrone2/1', 'Mux QDrone 2/1');
L(MC, 'Measurement Conditioning QDrone 2/2', 'Mux QDrone 2/2');
L(MC, 'Measurement Conditioning QDrone 2/2', 'isTracking QDrone 2?/1');
L(MC, 'Measurement Conditioning QDrone 2/1', 'Mux QDrone 2/3');
L(MC, 'From Desired Pose QDrone2/1', 'Mux QDrone 2/4');
L(MC, 'Time QDrone 2/1', 'Mux QDrone 2/5');
L(MC, 'Measurement Conditioning QDrone 2/3', 'Goto Corrected Measured Pose D2/1');
L(MC, 'Mux QDrone 2/1', 'Goto QDrone2 Data In/1');
nota(MC, [125 1170 470 1235], sprintf(['QDrone 2 = elementos [8:14] del Data [7n] de OptiTrack\n' ...
    '(rigid_body_ids = QD2CFG.optitrack.rb_ids, de config/estacion.json).\n' ...
    'Bias de altura propio: QD2CFG.optitrack.bias_d2.\n' ...
    'Mux QDrone 2 = mismo paquete de 16 doubles que el dron 1:\n' ...
    '[triggers(4) isTracking pose(6) desired(4) timestamp].']));
end

function area8(MC)
% 8. Communication, Signal Check & Flight Plan - QDrone 2 (copias de 3 y 5)
from_(MC, 'From QDrone2 Communication Issue', [990 1032 1120 1048], 'QDrone2_Communication_Issue');
add_block([MC '/Product'], [MC '/Product QDrone 2'], 'Position', [1145 1029 1165 1051]);
add_block([MC '/Triggered Subsystem'], [MC '/Triggered Subsystem QDrone 2'], 'Position', [1040 1080 1130 1122]);
add_block([MC '/Communication Issue ' newline 'with Commander'], [MC '/Communication Issue ' newline 'with Commander QDrone 2'], ...
    'Position', [1215 1000 1265 1016]);
goto_(MC, 'Goto Communication Status D2', [1215 1058 1380 1082], 'Communication_Status_D2', 'local');
add_block([MC '/Show Message on Host'], [MC '/Show Message on Host QDrone 2'], 'Position', [1215 1100 1305 1130]);
set_param([MC '/Show Message on Host QDrone 2'], 'message', 'Mission Control: No communication with QDrone 2');
L(MC, 'From QDrone2 Communication Issue/1', 'Product QDrone 2/1');
L(MC, 'From QDrone2 Communication Issue/1', 'Triggered Subsystem QDrone 2/Trigger');
L(MC, 'Triggered Subsystem QDrone 2/1', 'Product QDrone 2/2');
L(MC, 'Product QDrone 2/1', ['Communication Issue ' newline 'with Commander QDrone 2/1']);
L(MC, 'Product QDrone 2/1', 'Goto Communication Status D2/1');
L(MC, 'Product QDrone 2/1', 'Show Message on Host QDrone 2/1');
% Signal Check del dron 2
add_block([MC '/Time'], [MC '/Time Loopback QDrone 2'], 'Position', [1000 1175 1045 1207]);
from_(MC, 'From QDrone2 Timestamp Received', [990 1225 1120 1241], 'QDrone2_Timestamp_Received');
add_block([MC '/Subtract'], [MC '/Subtract QDrone 2'], 'Position', [1150 1192 1180 1223]);
goto_(MC, 'Goto Loopback Time D2', [1215 1197 1400 1219], 'Communication_Loopback_Time_D2', 'local');
L(MC, 'Time Loopback QDrone 2/1', 'Subtract QDrone 2/1');
L(MC, 'From QDrone2 Timestamp Received/1', 'Subtract QDrone 2/2');
L(MC, 'Subtract QDrone 2/1', 'Goto Loopback Time D2/1');
% Flight Plan del dron 2 (copia del Subsystem ya corregido)
add_block([MC '/Subsystem'], [MC '/Subsystem QDrone 2'], 'Position', [1520 1010 1650 1063]);
from_(MC, 'From Triggers QDrone2 FP', [1400 1015 1505 1031], 'Triggers_QDrone2');
from_(MC, 'From Communication Status D2', [1400 1042 1505 1058], 'Communication_Status_D2');
goto_(MC, 'Goto Flight Plan D2', [1500 1085 1650 1105], 'Fligth_Plan_D2', 'local');
L(MC, 'From Triggers QDrone2 FP/1', 'Subsystem QDrone 2/1');
L(MC, 'From Communication Status D2/1', 'Subsystem QDrone 2/2');
L(MC, 'Subsystem QDrone 2/1', 'Goto Flight Plan D2/1');
nota(MC, [1430 1150 1655 1230], sprintf(['Flight Plan del QDrone 2: copia de "Subsystem"\n' ...
    '(máquina de estados de tierra 0 Idle, 1 Armed,\n2 Takeoff, 3 Land, 9 MS Stop) con los disparos\n' ...
    'efectivos del dron 2.']));
end

function area9(MC, lib, es_mpc, nver)
% 9. Mission Supervision - 2 QDrones
y0 = 1325;
% --- Fila superior: roles -> supervisor -> guard / stacking -> rate limiter
from_(MC, 'From Corrected Pose D1', [120 y0+55 280 y0+75], 'Corrected_Measured_Pose');
from_(MC, 'From Corrected Pose D2', [120 y0+105 280 y0+125], 'Corrected_Measured_Pose_D2');
add_block([lib '/ROLE & HOME ASSIGNMENT'], [MC '/ROLE & HOME ASSIGNMENT'], 'Position', [320 y0+40 500 y0+140]);
set_param([MC '/ROLE & HOME ASSIGNMENT'], 'version', num2str(nver));
add_block('simulink/Sinks/Display', [MC '/Roles & Homes Preview'], 'Position', [380 y0+170 450 y0+330], 'BackgroundColor', 'yellow');
add_block('simulink/Sources/Digital Clock', [MC '/Mission Clock'], 'Position', [590 y0+42 630 y0+62], 'SampleTime', 'qc_get_step_size');
SUP = [MC '/MISSION SUPERVISOR'];
add_block([lib '/MISSION SUPERVISOR'], SUP, 'Position', [660 y0+30 860 y0+250]);
set_param(SUP, 'version', num2str(nver));
% comandos de Python + LAND ALL del operador (OPERATOR COMMAND MERGE, fila inferior)
ys5 = puerto_y(SUP, 'Inport', 5);
ys6 = puerto_y(SUP, 'Inport', 6);
from_(MC, 'From Python Commands Merged', [480 ys5-9 630 ys5+9], 'Python_Commands_Merged');
% estado a bordo de cada dron (MPC: MPC LINK CHECK; PID: el dron solo devuelve el eco)
if es_mpc
    from_(MC, 'From Onboard Status', [480 ys6-9 630 ys6+9], 'Onboard_Status');
    L(MC, 'From Onboard Status/1', 'MISSION SUPERVISOR/6');
else
    add_block('simulink/Sources/Constant', [MC '/Onboard Status N//A (PID)'], 'Position', [560 ys6-8 630 ys6+8], ...
        'Value', '[0; 0; 0; 0]');
    L(MC, 'Onboard Status N//A (PID)/1', 'MISSION SUPERVISOR/6');
end
add_block('simulink/Sinks/Display', [MC '/Mission State'], 'Position', [885 y0+163 935 y0+179], 'BackgroundColor', 'yellow');
goto_(MC, 'Goto Mission Info', [885 y0+130 985 y0+150], 'Mission_Info', 'local');
L(MC, 'From Corrected Pose D1/1', 'ROLE & HOME ASSIGNMENT/1');
L(MC, 'From Corrected Pose D2/1', 'ROLE & HOME ASSIGNMENT/2');
L(MC, 'ROLE & HOME ASSIGNMENT/4', 'Roles & Homes Preview/1');
L(MC, 'Mission Clock/1', 'MISSION SUPERVISOR/1');
L(MC, 'ROLE & HOME ASSIGNMENT/1', 'MISSION SUPERVISOR/2');
L(MC, 'ROLE & HOME ASSIGNMENT/2', 'MISSION SUPERVISOR/3');
L(MC, 'ROLE & HOME ASSIGNMENT/3', 'MISSION SUPERVISOR/4');
L(MC, 'From Python Commands Merged/1', 'MISSION SUPERVISOR/5');
L(MC, 'MISSION SUPERVISOR/4', 'Goto Mission Info/1');
L(MC, 'MISSION SUPERVISOR/5', 'Mission State/1');
nota(MC, [120 y0+150 360 y0+215], sprintf(['Vista previa antes de START:\n' ...
    '[explorer responder home1(x,y) home2(x,y)\n d_homes start_ok]  (start_ok = 0 ->\n separar los drones, START se rechazaría)']));

if ~es_mpc
    G = [MC '/SEPARATION GUARD'];
    add_block([lib '/SEPARATION GUARD'], G, 'Position', [1010 y0+30 1180 y0+170]);
    L(MC, 'ROLE & HOME ASSIGNMENT/1', 'SEPARATION GUARD/1');
    L(MC, 'ROLE & HOME ASSIGNMENT/2', 'SEPARATION GUARD/2');
    L(MC, 'MISSION SUPERVISOR/1', 'SEPARATION GUARD/3');
    L(MC, 'MISSION SUPERVISOR/2', 'SEPARATION GUARD/4');
    L(MC, 'MISSION SUPERVISOR/3', 'SEPARATION GUARD/5');
    goto_(MC, 'Goto Guard Active', [1210 y0+190 1330 y0+210], 'Guard_Active', 'local');
    add_block('simulink/Sinks/Display', [MC '/Guard Active'], 'Position', [1210 y0+225 1260 y0+245], 'BackgroundColor', 'yellow');
    L(MC, 'SEPARATION GUARD/3', 'Goto Guard Active/1');
    L(MC, 'SEPARATION GUARD/3', 'Guard Active/1');
    src1 = 'SEPARATION GUARD/1'; src2 = 'SEPARATION GUARD/2';
else
    S = [MC '/STACKING INFO'];
    add_block([lib '/STACKING INFO'], S, 'Position', [1080 y0+190 1260 y0+300]);
    from_(MC, 'From Corrected Pose D1 S', [900 y0+196 1050 y0+212], 'Corrected_Measured_Pose');
    from_(MC, 'From Corrected Pose D2 S', [900 y0+218 1050 y0+234], 'Corrected_Measured_Pose_D2');
    from_(MC, 'From Velocity D1', [900 y0+240 1050 y0+256], 'Velocity_QDrone1');
    from_(MC, 'From Velocity D2', [900 y0+262 1050 y0+278], 'Velocity_QDrone2');
    % switch de modo MPC: mismo estilo que Command Enable, naranja; InitFcn -> entrada inferior (MPC)
    switch_modo(MC, 'MPC Mode', 'PID Backup', '0', 'MPC', '1', [1020 y0+300]);
    L(MC, 'MPC Mode/1', 'STACKING INFO/5');
    L(MC, 'From Corrected Pose D1 S/1', 'STACKING INFO/1');
    L(MC, 'From Corrected Pose D2 S/1', 'STACKING INFO/2');
    L(MC, 'From Velocity D1/1', 'STACKING INFO/3');
    L(MC, 'From Velocity D2/1', 'STACKING INFO/4');
    goto_(MC, 'Goto MPC Extra D1', [1300 y0+200 1450 y0+220], 'QDrone1_MPC_Extra', 'global');
    goto_(MC, 'Goto MPC Extra D2', [1300 y0+235 1450 y0+255], 'QDrone2_MPC_Extra', 'global');
    add_block('simulink/Sinks/Display', [MC '/dz (m)'], 'Position', [1300 y0+270 1360 y0+290], 'BackgroundColor', 'yellow');
    L(MC, 'STACKING INFO/1', 'Goto MPC Extra D1/1');
    L(MC, 'STACKING INFO/2', 'Goto MPC Extra D2/1');
    L(MC, 'STACKING INFO/3', 'dz (m)/1');
    add_block('simulink/Sources/Constant', [MC '/No Guard (MPC)'], 'Position', [1300 y0+312 1320 y0+328], 'Value', '0');
    goto_(MC, 'Goto Guard Active', [1350 y0+310 1470 y0+330], 'Guard_Active', 'local');
    L(MC, 'No Guard (MPC)/1', 'Goto Guard Active/1');
    add_block('simulink/Sinks/Terminator', [MC '/Terminator Guard Context'], 'Position', [885 y0+95 895 y0+105], 'ShowName', 'off');
    L(MC, 'MISSION SUPERVISOR/3', 'Terminator Guard Context/1');   % sin guard en MPC
    src1 = 'MISSION SUPERVISOR/1'; src2 = 'MISSION SUPERVISOR/2';
end
% Rate limiter por norma y Gotos de consigna. Cada fuente se alinea con su
% puerto (líneas rectas, sin cruces): reloj propio por limitador, Triggers y
% Mission Info (Mission Info(38:39) = el dron se aparta de un dron en falla
% -> límite xy P.falla_rate_xy). Las consignas llegan con un quiebre en x = xq.
RL1 = [MC '/SETPOINT RATE LIMITER QDRONE 1'];
RL2 = [MC '/SETPOINT RATE LIMITER QDRONE 2'];
add_block([lib '/SETPOINT RATE LIMITER'], RL1, 'Position', [1320 y0+18 1470 y0+88]);
add_block([lib '/SETPOINT RATE LIMITER'], RL2, 'Position', [1320 y0+112 1470 y0+182]);
set_param(RL2, 'dron', '2');
nombres_rl = {'SETPOINT RATE LIMITER QDRONE 1', 'SETPOINT RATE LIMITER QDRONE 2'};
for i = 1:2
    R = [MC '/' nombres_rl{i}];
    yr = @(k) puerto_y(R, 'Inport', k);
    if i == 1, nc = 'Clock Rate Limiter'; else, nc = 'Clock Rate Limiter 2'; end
    add_block('simulink/Sources/Digital Clock', [MC '/' nc], 'Position', [1240 yr(1)-10 1280 yr(1)+10], ...
        'SampleTime', 'qc_get_step_size', 'ShowName', 'off');
    from_(MC, sprintf('From Triggers QDrone%d RL', i), [1190 yr(3)-7 1295 yr(3)+7], sprintf('Triggers_QDrone%d', i));
    from_(MC, sprintf('From Mission Info RL%d', i), [1190 yr(4)-7 1295 yr(4)+7], 'Mission_Info');
    L(MC, [nc '/1'], [nombres_rl{i} '/1']);
    L(MC, sprintf('From Triggers QDrone%d RL/1', i), [nombres_rl{i} '/3']);
    L(MC, sprintf('From Mission Info RL%d/1', i), [nombres_rl{i} '/4']);
end
if es_mpc, xq = 1150; else, xq = 1205; end
linea_quiebre(MC, src1, 'SETPOINT RATE LIMITER QDRONE 1/2', xq);
linea_quiebre(MC, src2, 'SETPOINT RATE LIMITER QDRONE 2/2', xq);
goto_(MC, 'Goto Desired Pose Mission D1', [1490 puerto_y(RL1, 'Outport', 1)-10 1655 puerto_y(RL1, 'Outport', 1)+10], 'Desired_Pose_Mission_QDrone1', 'local');
goto_(MC, 'Goto Desired Pose D2', [1490 puerto_y(RL2, 'Outport', 1)-10 1655 puerto_y(RL2, 'Outport', 1)+10], 'Desired_Pose_QDrone2', 'local');
L(MC, 'SETPOINT RATE LIMITER QDRONE 1/1', 'Goto Desired Pose Mission D1/1');
L(MC, 'SETPOINT RATE LIMITER QDRONE 2/1', 'Goto Desired Pose D2/1');

% --- Fila inferior: OPERATOR SAFETY LAYER (DUAL) + OPERATOR COMMAND MERGE
% Cada fuente se alinea con su puerto (líneas rectas).
yb = y0 + 340;
OSL = [MC '/OPERATOR SAFETY LAYER'];
add_block([lib '/OPERATOR SAFETY LAYER DUAL'], OSL, 'Position', [900 yb 1080 yb+360]);
yi = @(k) puerto_y(OSL, 'Inport', k);
yo = @(k) puerto_y(OSL, 'Outport', k);
add_block('simulink/Sources/Digital Clock', [MC '/Clock Operator'], 'Position', [830 yi(1)-10 870 yi(1)+10], ...
    'SampleTime', 'qc_get_step_size', 'ShowName', 'off');
from_(MC, 'From Joystick Signals', [710 yi(2)-10 870 yi(2)+10], 'Joystick_Signals');
switch_modo(MC, 'Command Source', 'Manual (Equipo2)', '1', 'Mission (2 QDrones)', '0', [840 yi(5)-18]);
set_param([MC '/Command Source'], 'NamePlacement', 'alternate');   % nombre arriba (abajo va el Flight State)
% Flight State de tierra con 1 paso de retardo (el Flight Plan depende de los
% disparos que calcula este bloque: sin el retardo habría lazo algebraico)
from_(MC, 'From Flight Plan D1 O', [630 yi(6)-22 780 yi(6)-6], 'Fligth_Plan');
from_(MC, 'From Flight Plan D2 O', [630 yi(6)+6 780 yi(6)+22], 'Fligth_Plan_D2');
add_block('simulink/Signal Routing/Mux', [MC '/Mux Flight State O'], 'Position', [800 yi(6)-20 805 yi(6)+20], ...
    'Inputs', '2', 'ShowName', 'off');
add_block('simulink/Discrete/Unit Delay', [MC '/Flight State z-1'], 'Position', [830 yi(6)-15 860 yi(6)+15], ...
    'SampleTime', 'qc_get_step_size', 'InitialCondition', '[0; 0]', 'ShowName', 'off');
from_(MC, 'From Corrected Pose D1 O', [710 yi(7)-10 870 yi(7)+10], 'Corrected_Measured_Pose');
from_(MC, 'From Corrected Pose D2 O', [710 yi(8)-10 870 yi(8)+10], 'Corrected_Measured_Pose_D2');
from_(MC, 'From Mission Info O', [710 yi(9)-10 870 yi(9)+10], 'Mission_Info');
L(MC, 'Clock Operator/1', 'OPERATOR SAFETY LAYER/1');
L(MC, 'From Joystick Signals/1', 'OPERATOR SAFETY LAYER/2');
L(MC, 'MISSION SUPERVISOR/6', 'OPERATOR SAFETY LAYER/3');
L(MC, 'MISSION SUPERVISOR/7', 'OPERATOR SAFETY LAYER/4');
L(MC, 'Command Source/1', 'OPERATOR SAFETY LAYER/5');
L(MC, 'From Flight Plan D1 O/1', 'Mux Flight State O/1');
L(MC, 'From Flight Plan D2 O/1', 'Mux Flight State O/2');
L(MC, 'Mux Flight State O/1', 'Flight State z-1/1');
L(MC, 'Flight State z-1/1', 'OPERATOR SAFETY LAYER/6');
L(MC, 'From Corrected Pose D1 O/1', 'OPERATOR SAFETY LAYER/7');
L(MC, 'From Corrected Pose D2 O/1', 'OPERATOR SAFETY LAYER/8');
L(MC, 'From Mission Info O/1', 'OPERATOR SAFETY LAYER/9');
goto_(MC, 'Goto Triggers QDrone1', [1110 yo(1)-10 1250 yo(1)+10], 'Triggers_QDrone1', 'local');
goto_(MC, 'Goto Triggers QDrone2', [1110 yo(2)-10 1250 yo(2)+10], 'Triggers_QDrone2', 'local');
goto_(MC, 'Goto Command Source', [1110 yo(3)-10 1290 yo(3)+10], 'Command_Source_Effective', 'local');
L(MC, 'OPERATOR SAFETY LAYER/1', 'Goto Triggers QDrone1/1');
L(MC, 'OPERATOR SAFETY LAYER/2', 'Goto Triggers QDrone2/1');
L(MC, 'OPERATOR SAFETY LAYER/3', 'Goto Command Source/1');
% Operator Request -> (1 paso de retardo) -> OPERATOR COMMAND MERGE -> MISSION SUPERVISOR
y4 = yo(4);
add_block('simulink/Discrete/Unit Delay', [MC '/Operator Request z-1'], 'Position', [1110 y4-15 1140 y4+15], ...
    'SampleTime', 'qc_get_step_size', 'InitialCondition', '[0; 0]', 'ShowName', 'off');
goto_(MC, 'Goto Operator Request', [1110 y4+30 1250 y4+50], 'Operator_Request', 'local');
MG = [MC '/OPERATOR COMMAND MERGE'];
add_block([lib '/OPERATOR COMMAND MERGE'], MG, 'Position', [1300 y4-52 1440 y4+18]);
from_(MC, 'From Python Commands', [1150 puerto_y(MG, 'Inport', 1)-9 1285 puerto_y(MG, 'Inport', 1)+9], 'Python_Commands');
goto_(MC, 'Goto Python Commands Merged', [1470 puerto_y(MG, 'Outport', 1)-10 1655 puerto_y(MG, 'Outport', 1)+10], ...
    'Python_Commands_Merged', 'local');
L(MC, 'OPERATOR SAFETY LAYER/4', 'Operator Request z-1/1');
L(MC, 'OPERATOR SAFETY LAYER/4', 'Goto Operator Request/1');
L(MC, 'Operator Request z-1/1', 'OPERATOR COMMAND MERGE/2');
L(MC, 'From Python Commands/1', 'OPERATOR COMMAND MERGE/1');
L(MC, 'OPERATOR COMMAND MERGE/1', 'Goto Python Commands Merged/1');
nota(MC, [120 yb+20 600 yb+150], sprintf([ ...
    'OPERATOR SAFETY LAYER (modo MISIÓN):\n' ...
    '- Arm/Disarm y Takeoff/Land arriba = PERMISO (despega el START de la UI).\n' ...
    '- Takeoff/Land abajo en vuelo = LAND ALL ordenado del supervisor\n' ...
    '  (se inyecta en OPERATOR COMMAND MERGE; R primero, E se aparta).\n' ...
    '- Respaldo crudo (alerta 8) si el supervisor no lo acepta en 2 s o a los\n' ...
    '  90 s; nunca suelta a un dron que tiene al otro debajo a < d_min_h.\n' ...
    '- Emergency Stop = corte inmediato. Command Source solo cambia en\n' ...
    '  tierra con Arm y Takeoff abajo.']));

% --- MPC LINK CHECK (solo MPC): a la derecha del OPERATOR SAFETY LAYER
if es_mpc
    area9_enlace(MC, lib, yb);
end

% --- Fila 3: MISSION END CHECK
ye = yb + 385;
add_block('simulink/Sources/Digital Clock', [MC '/Clock End Check'], 'Position', [1250 ye+2 1290 ye+22], 'SampleTime', 'qc_get_step_size', 'ShowName', 'off');
from_(MC, 'From Flight Plan D1', [1130 ye+32 1260 ye+48], 'Fligth_Plan');
from_(MC, 'From Flight Plan D2', [1130 ye+54 1260 ye+70], 'Fligth_Plan_D2');
add_block('simulink/Signal Routing/Mux', [MC '/Mux Flight State'], 'Position', [1280 ye+30 1285 ye+72], 'Inputs', '2', 'ShowName', 'off');
from_(MC, 'From Corrected Pose D1 E', [1130 ye+82 1290 ye+98], 'Corrected_Measured_Pose');
from_(MC, 'From Corrected Pose D2 E', [1130 ye+106 1290 ye+122], 'Corrected_Measured_Pose_D2');
add_block([lib '/MISSION END CHECK'], [MC '/MISSION END CHECK'], 'Position', [1320 ye+0 1470 ye+130]);
goto_(MC, 'Goto Stop Allowed', [1500 ye+20 1650 ye+40], 'Stop_Allowed', 'global');
add_block('simulink/Sinks/Display', [MC '/QDrone Finished'], 'Position', [1500 ye+85 1550 ye+115], 'BackgroundColor', 'yellow');
L(MC, 'From Flight Plan D1/1', 'Mux Flight State/1');
L(MC, 'From Flight Plan D2/1', 'Mux Flight State/2');
L(MC, 'Clock End Check/1', 'MISSION END CHECK/1');
L(MC, 'Mux Flight State/1', 'MISSION END CHECK/2');
L(MC, 'From Corrected Pose D1 E/1', 'MISSION END CHECK/3');
L(MC, 'From Corrected Pose D2 E/1', 'MISSION END CHECK/4');
L(MC, 'MISSION END CHECK/1', 'Goto Stop Allowed/1');
L(MC, 'MISSION END CHECK/2', 'QDrone Finished/1');

nota(MC, [120 ye+20 900 ye+95], sprintf([ ...
    'Flujo: OptiTrack (2 RB) -> ROLE & HOME ASSIGNMENT -> MISSION SUPERVISOR -> %s\n' ...
    '-> SETPOINT RATE LIMITER (por norma) -> Desired Pose de cada dron. Los comandos de Python (START, PAUSE, RESUME,\n' ...
    'ABORT, LAND ALL y objetivos) llegan por el PYTHON BRIDGE (raíz, :18700) y pasan por OPERATOR COMMAND MERGE.\n' ...
    'Arm/Takeoff/E-stop: switches del área 1 + OPERATOR SAFETY LAYER. Command Source: 0 = MISIÓN 2 QDrones (por defecto),\n' ...
    '1 = MANUAL Equipo2 (solo QDrone 1, Waypoint Trajectories original).'], ...
    ternario(es_mpc, 'STACKING INFO (MPC, sin guard)', 'SEPARATION GUARD (PID)')));
end

function area9_enlace(MC, lib, yb)
% MPC LINK CHECK: valida la respuesta de 8 doubles de cada dron (etiqueta
% y(8) = 2, FSM 1..9) y entrega la FSM a bordo al MISSION SUPERVISOR
% (Goto Onboard_Status). Espacio libre a la derecha de los Goto del
% OPERATOR SAFETY LAYER; cada fuente alineada con su puerto.
LC = [MC '/MPC LINK CHECK'];
add_block([lib '/MPC LINK CHECK'], LC, 'Position', [1372 yb+15 1542 yb+195]);
set_param(LC, 'version', '2');
yl = @(k) puerto_y(LC, 'Inport', k);
add_block('simulink/Sources/Digital Clock', [MC '/Clock Link Check'], 'Position', [1322 yl(1)-9 1362 yl(1)+9], ...
    'SampleTime', 'qc_get_step_size', 'ShowName', 'off');
L(MC, 'Clock Link Check/1', 'MPC LINK CHECK/1');
fuentes = {'From Telemetry D1 L', 'QDrone1_Telemetry'; 'From Telemetry D2 L', 'QDrone2_Telemetry'; ...
           'From Loopback D1 L', 'Communication_Loopback_Time'; 'From Loopback D2 L', 'Communication_Loopback_Time_D2'; ...
           'From Communication Status D1 L', 'Communication_Status'; 'From Communication Status D2 L', 'Communication_Status_D2'};
for k = 1:size(fuentes, 1)
    y = yl(k + 1);
    from_(MC, fuentes{k,1}, [1262 y-8 1362 y+8], fuentes{k,2});
    L(MC, [fuentes{k,1} '/1'], sprintf('MPC LINK CHECK/%d', k + 1));
end
% 1 paso de retardo: la respuesta del dron depende (vía Communication) de los
% disparos que salen del supervisor; sin el retardo habría lazo con los
% Triggered Subsystem de Communication Issue (orden ambiguo)
yo = puerto_y(LC, 'Outport', 1);
add_block('simulink/Discrete/Unit Delay', [MC '/Onboard Status z-1'], 'Position', [1550 yo-12 1572 yo+12], ...
    'SampleTime', 'qc_get_step_size', 'InitialCondition', '[0; 0; 0; 0]', 'ShowName', 'off');
goto_(MC, 'Goto Onboard Status', [1580 yo-10 1655 yo+10], 'Onboard_Status', 'local');
L(MC, 'MPC LINK CHECK/1', 'Onboard Status z-1/1');
L(MC, 'Onboard Status z-1/1', 'Goto Onboard Status/1');
add_block('simulink/Sinks/Display', [MC '/Onboard Status'], 'Position', [1575 yo+40 1645 yo+80], 'BackgroundColor', 'yellow');
L(MC, 'Onboard Status z-1/1', 'Onboard Status/1');
h = nota(MC, [1310 yb+205 1655 yb+250], sprintf([ ...
    'MPC LINK CHECK: [fsm1 fsm2 cod1 cod2]. FSM a bordo 7/8/9 o enlace perdido en vuelo\n' ...
    '-> alerta 9: el sano se aparta >= d_min_h a su altura, regresa y aterriza (DONE).\n' ...
    'Sin y(8) = 2 o FSM fuera de 1..9 > 0.5 s conectado -> alerta 10 (no disponible).']), 'Arial', 8);
h.ForegroundColor = 'red';
end

function y = puerto_y(blk, tipo, k)
% Coordenada y del puerto k ('Inport' / 'Outport') de un bloque.
ph = get_param(blk, 'PortHandles');
p = get_param(ph.(tipo)(k), 'Position');
y = p(2);
end

function switch_modo(MC, nombre, n_arriba, v_arriba, n_abajo, v_abajo, xy)
% Copia el estilo de "Command Enable" (constantes Custom Mode / Home Mode
% con el nombre a la izquierda) en naranja, como Trajectory Enable.
% Su InitFcn (copiado) deja el switch en la entrada inferior (sw = 0),
% igual que los switches originales.
x = xy(1); y = xy(2);
add_block([MC '/Custom Mode'], [MC '/' n_arriba], 'Position', [x-50 y x-30 y+16], 'Value', v_arriba);
add_block([MC '/Home Mode'], [MC '/' n_abajo], 'Position', [x-50 y+20 x-30 y+36], 'Value', v_abajo);
add_block([MC '/Command Enable'], [MC '/' nombre], 'Position', [x y x+30 y+36], 'BackgroundColor', 'orange');
L(MC, [n_arriba '/1'], [nombre '/1']);
L(MC, [n_abajo '/1'], [nombre '/2']);
end

function area10(MC, lib, es_mpc, nver)
% 10. Python Bridge Telemetry & Blackbox
y0 = 2230;   % debajo del área 9
from_(MC, 'From Corrected Pose D1 V', [125 y0+40 285 y0+60], 'Corrected_Measured_Pose');
from_(MC, 'From Corrected Pose D2 V', [125 y0+110 285 y0+130], 'Corrected_Measured_Pose_D2');
add_block('simulink/Sources/Digital Clock', [MC '/Telemetry Clock'], 'Position', [190 y0+75 230 y0+95], 'SampleTime', 'qc_get_step_size');
add_block([lib '/VELOCITY ESTIMATE'], [MC '/VELOCITY ESTIMATE QDRONE 1'], 'Position', [330 y0+30 470 y0+70]);
add_block([lib '/VELOCITY ESTIMATE'], [MC '/VELOCITY ESTIMATE QDRONE 2'], 'Position', [330 y0+100 470 y0+140]);
L(MC, 'Telemetry Clock/1', 'VELOCITY ESTIMATE QDRONE 1/1');
L(MC, 'From Corrected Pose D1 V/1', 'VELOCITY ESTIMATE QDRONE 1/2');
L(MC, 'Telemetry Clock/1', 'VELOCITY ESTIMATE QDRONE 2/1');
L(MC, 'From Corrected Pose D2 V/1', 'VELOCITY ESTIMATE QDRONE 2/2');
goto_(MC, 'Goto Velocity D1', [520 y0+20 650 y0+40], 'Velocity_QDrone1', 'local');
goto_(MC, 'Goto Velocity D2', [520 y0+150 650 y0+170], 'Velocity_QDrone2', 'local');
L(MC, 'VELOCITY ESTIMATE QDRONE 1/1', 'Goto Velocity D1/1');
L(MC, 'VELOCITY ESTIMATE QDRONE 2/1', 'Goto Velocity D2/1');

% Estado de cada dron para la telemetría (Drone Status). Cada fuente se
% alinea con su entrada del Mux (líneas rectas, sin cruces).
if ~es_mpc
    entradas = '[1 1 2 1 1 1 1 2]';
else
    entradas = '[1 1 1 1 1 1 1 1 2]';
end
MX = [MC '/Mux Drone Status'];
add_block('simulink/Signal Routing/Mux', MX, 'Position', [600 y0+180 605 y0+345], 'Inputs', entradas, 'ShowName', 'off');
ym = @(k) puerto_y(MX, 'Inport', k);
if ~es_mpc
    from_(MC, 'From Flight Plan D1 T', [380 ym(1)-7 570 ym(1)+7], 'Fligth_Plan');
    from_(MC, 'From Flight Plan D2 T', [380 ym(2)-7 570 ym(2)+7], 'Fligth_Plan_D2');
    add_block('simulink/Sources/Constant', [MC '/Battery N//A'], 'Position', [500 ym(3)-7 570 ym(3)+7], 'Value', '[NaN NaN]');
    L(MC, 'From Flight Plan D1 T/1', 'Mux Drone Status/1');
    L(MC, 'From Flight Plan D2 T/1', 'Mux Drone Status/2');
    L(MC, 'Battery N//A/1', 'Mux Drone Status/3');
    k0 = 4;
else
    % telemetría a bordo (8 doubles de cada dron): [2] = FSM, [3] = batería (V)
    sel = {'FSM 1', 'QDrone1_Telemetry', '2'; 'FSM 2', 'QDrone2_Telemetry', '2'; ...
           'Battery 1', 'QDrone1_Telemetry', '3'; 'Battery 2', 'QDrone2_Telemetry', '3'};
    for k = 1:4
        y = ym(k);
        from_(MC, ['From ' sel{k,1} ' Telemetry'], [340 y-7 500 y+7], sel{k,2});
        add_block('simulink/Signal Routing/Selector', [MC '/' sel{k,1}], 'Position', [530 y-7 560 y+7], ...
            'IndexOptions', 'Index vector (dialog)', 'Indices', sel{k,3}, 'InputPortWidth', '8', 'ShowName', 'off');
        L(MC, ['From ' sel{k,1} ' Telemetry/1'], [sel{k,1} '/1']);
        L(MC, [sel{k,1} '/1'], sprintf('Mux Drone Status/%d', k));
    end
    k0 = 5;
end
fuentes = {'From Communication Status D1', 'Communication_Status'; ...
           'From Communication Status D2 T', 'Communication_Status_D2'; ...
           'From Loopback D1', 'Communication_Loopback_Time'; ...
           'From Loopback D2', 'Communication_Loopback_Time_D2'; ...
           'From Operator Request', 'Operator_Request'};   % [n_peticiones alerta] del OPERATOR SAFETY LAYER
for k = 1:size(fuentes, 1)
    y = ym(k0 + k - 1);
    from_(MC, fuentes{k,1}, [380 y-7 570 y+7], fuentes{k,2});
    L(MC, [fuentes{k,1} '/1'], sprintf('Mux Drone Status/%d', k0 + k - 1));
end
nota(MC, [110 y0+180 330 y0+245], sprintf([ ...
    'Drone Status [%d] = [fsm1 fsm2 bat1 bat2\ncomm_issue1 comm_issue2 loopback1\nloopback2 n_peticiones alerta_operador]\n%s'], ...
    10, ternario(es_mpc, '(fsm/bat: telemetría a bordo, §3.2)', '(fsm = Flight State de tierra; bat = NaN)')));

% TELEMETRY PACK
TP = [MC '/TELEMETRY PACK'];
add_block([lib '/TELEMETRY PACK'], TP, 'Position', [880 y0+30 1060 y0+290]);
set_param(TP, 'version', num2str(nver));
from_(MC, 'From Mission Info', [700 y0+60 840 y0+76], 'Mission_Info');
from_(MC, 'From Guard Active', [700 y0+84 840 y0+100], 'Guard_Active');
from_(MC, 'From Corrected Pose D1 T', [700 y0+108 840 y0+124], 'Corrected_Measured_Pose');
from_(MC, 'From Corrected Pose D2 T', [700 y0+132 840 y0+148], 'Corrected_Measured_Pose_D2');
from_(MC, 'From Desired Pose D1 T', [700 y0+156 840 y0+172], 'Desired_Pose');
from_(MC, 'From Desired Pose D2 T', [700 y0+180 840 y0+196], 'Desired_Pose_QDrone2');
from_(MC, 'From Velocity D1 T', [700 y0+204 840 y0+220], 'Velocity_QDrone1');
from_(MC, 'From Velocity D2 T', [700 y0+228 840 y0+244], 'Velocity_QDrone2');
L(MC, 'Telemetry Clock/1', 'TELEMETRY PACK/1');
L(MC, 'From Mission Info/1', 'TELEMETRY PACK/2');
L(MC, 'From Guard Active/1', 'TELEMETRY PACK/3');
L(MC, 'From Corrected Pose D1 T/1', 'TELEMETRY PACK/4');
L(MC, 'From Corrected Pose D2 T/1', 'TELEMETRY PACK/5');
L(MC, 'From Desired Pose D1 T/1', 'TELEMETRY PACK/6');
L(MC, 'From Desired Pose D2 T/1', 'TELEMETRY PACK/7');
L(MC, 'From Velocity D1 T/1', 'TELEMETRY PACK/8');
L(MC, 'From Velocity D2 T/1', 'TELEMETRY PACK/9');
L(MC, 'Mux Drone Status/1', 'TELEMETRY PACK/10');
goto_(MC, 'Goto Python Telemetry', [1120 y0+140 1280 y0+160], 'Python_Telemetry', 'global');
L(MC, 'TELEMETRY PACK/1', 'Goto Python Telemetry/1');
% Blackbox de la misión (además del log original del dron 1)
TH = [MC '/To Host File Dual'];
add_block([MC '/To Host File'], TH, 'Position', [1400 y0+120 1480 y0+196]);
set_param(TH, 'file_name', 'saved_data/%m/log_%{regexprep(char(datetime),'' '',''_'')}_MissionDual.mat', ...
    'variable_name', 'mission_dual_data', 'decimation', '20');
L(MC, 'TELEMETRY PACK/1', 'To Host File Dual/1');
nota(MC, [1120 y0+30 1640 y0+110], sprintf([ ...
    'Telemetry [64] -> Goto global Python_Telemetry -> PYTHON BRIDGE (raíz) -> estación Python :18700.\n' ...
    'Mismo vector al log mission_dual_data (cada 20 muestras = 50 Hz): fila 1 = t del log,\n' ...
    'filas 2-65 = los 64 campos de ARQUITECTURA §3.3 (55 = alerta: operador 5-8 o supervisor 0-4, 9, 10;\n' ...
    '56-60 = un objetivo por mensaje, rotando: [id clase x y estado]).']));
end

% #########################################################################
%  VERIFICACIÓN, CAPTURAS, EXPORTACIÓN
% #########################################################################
function v = verificar(archivo)
[~, mdl] = fileparts(archivo);
if ~bdIsLoaded(mdl), load_system(archivo); end
v = struct('modelo', mdl, 'update_ok', false, 'mensaje', '', 'tiempo', 0);
t0 = tic;
try
    set_param(mdl, 'SimulationCommand', 'update');
    v.update_ok = true;
    v.mensaje = 'Update Diagram OK';
catch e
    v.mensaje = mensaje_completo(e);
end
v.tiempo = toc(t0);
fprintf('Verificación %s: %s (%.1f s)\n', mdl, v.mensaje, v.tiempo);
% El Update Diagram ejecuta el InitFcn, que puede escribir rutas en el
% bloque OptiTrack; se guarda para dejar el archivo limpio.
if strcmp(get_param(mdl, 'Dirty'), 'on')
    save_system(mdl);
end
end

function m = mensaje_completo(e)
m = e.message;
for k = 1:numel(e.cause)
    m = [m newline '  causa: ' mensaje_completo(e.cause{k})]; %#ok<AGROW>
end
end

function capturas(raiz, orig, archivos)
dcap = fullfile(raiz, 'capturas');
if ~exist(dcap, 'dir'), mkdir(dcap); end
w = warning('off', 'all');
[~, mo] = fileparts(orig);
load_system(orig);
print(['-s' mo], '-dpng', '-r110', fullfile(dcap, 'original_raiz.png'));
print(['-s' mo '/MISSION CONTROL QDRONE POSITION CONTROL'], '-dpng', '-r90', fullfile(dcap, 'original_mission_control.png'));
close_system(mo, 0);
for k = 1:numel(archivos)
    [~, m] = fileparts(archivos{k});
    if ~bdIsLoaded(m), load_system(archivos{k}); end
    v = lower(m(end-2:end));
    print(['-s' m], '-dpng', '-r110', fullfile(dcap, sprintf('dual_%s_raiz.png', v)));
    print(['-s' m '/MISSION CONTROL QDRONE POSITION CONTROL'], '-dpng', '-r90', fullfile(dcap, sprintf('dual_%s_mission_control.png', v)));
    print(['-s' m '/PYTHON BRIDGE'], '-dpng', '-r110', fullfile(dcap, sprintf('dual_%s_python_bridge.png', v)));
    print(['-s' m '/Communication QDrone 2'], '-dpng', '-r110', fullfile(dcap, sprintf('dual_%s_communication_qdrone2.png', v)));
    print(['-s' m '/MISSION CONTROL QDRONE POSITION CONTROL/Subsystem'], '-dpng', '-r110', fullfile(dcap, sprintf('dual_%s_flight_plan.png', v)));
end
print('-sQD2_Mission_Lib', '-dpng', '-r110', fullfile(dcap, 'libreria_QD2_Mission_Lib.png'));
warning(w);
fprintf('Capturas en %s\n', dcap);
end

function res = exportar(raiz, archivos)
versiones = {'R2022a', 'R2022b', 'R2023a', 'R2023b', 'R2025a'};
res = struct('version', {}, 'archivo', {}, 'ok', {}, 'mensaje', {});
for i = 1:numel(versiones)
    dv = fullfile(raiz, 'versiones', versiones{i});
    if ~exist(dv, 'dir'), mkdir(dv); end
    for k = 1:numel(archivos)
        [~, m] = fileparts(archivos{k});
        destino = fullfile(dv, [m '.slx']);
        r = struct('version', versiones{i}, 'archivo', destino, 'ok', false, 'mensaje', '');
        try
            if exist(destino, 'file'), delete(destino); end
            if strcmp(versiones{i}, 'R2025a')
                copyfile(archivos{k}, destino, 'f');      % copia nativa
            else
                if ~bdIsLoaded(m), load_system(archivos{k}); end
                save_system(m, destino, 'ExportToVersion', versiones{i});
            end
            r.ok = exist(destino, 'file') == 2;
            r.mensaje = 'OK';
        catch e
            r.mensaje = e.message;
        end
        fprintf('Exportar %s %s: %s\n', versiones{i}, m, r.mensaje);
        res(end+1) = r; %#ok<AGROW>
    end
end
end

% #########################################################################
%  UTILIDADES
% #########################################################################
function preparar_rutas(raiz)
% Artefactos de compilación (slprj, .slxc) fuera del repositorio
dc = fullfile(tempdir, 'qd2_slcache');
if ~exist(dc, 'dir'), mkdir(dc); end
Simulink.fileGenControl('set', 'CacheFolder', dc, 'CodeGenFolder', dc);
addpath(fullfile(raiz, 'lib'));
addpath(raiz);
if isempty(which('qc_get_step_size'))
    q = 'C:\Program Files\Quanser\QUARC\quarc';
    if exist(q, 'dir'), addpath(q); end
end
end

function cerrar(nombres)
for k = 1:numel(nombres)
    if bdIsLoaded(nombres{k}), close_system(nombres{k}, 0); end
end
end

function n = esc(n)
n = strrep(n, '/', '//');
end

function L(sys, a, b)
add_line(sys, a, b, 'autorouting', 'smart');
end

function linea_quiebre(sys, a, b, xq)
% Línea 'bloque/puerto' -> 'bloque/puerto' con un solo quiebre vertical en
% x = xq (sin autorouting: así no cruza los From alineados del destino).
ta = regexp(a, '^(.*)/(\d+)$', 'tokens', 'once');
tb = regexp(b, '^(.*)/(\d+)$', 'tokens', 'once');
pha = get_param([sys '/' ta{1}], 'PortHandles');
phb = get_param([sys '/' tb{1}], 'PortHandles');
po = pha.Outport(str2double(ta{2}));
pi_ = phb.Inport(str2double(tb{2}));
pa = get_param(po, 'Position');
pb = get_param(pi_, 'Position');
h = add_line(sys, po, pi_);
if abs(pa(2) - pb(2)) > 0.5
    set_param(h, 'Points', [pa; xq pa(2); xq pb(2); pb]);
end
end

function from_(sys, nombre, pos, tag, orient)
if nargin < 5, orient = 'right'; end
add_block('simulink/Signal Routing/From', [sys '/' nombre], 'Position', pos, 'GotoTag', tag, ...
    'ShowName', 'off', 'Orientation', orient);
end

function goto_(sys, nombre, pos, tag, vis, orient)
if nargin < 6, orient = 'right'; end
add_block('simulink/Signal Routing/Goto', [sys '/' nombre], 'Position', pos, 'GotoTag', tag, ...
    'TagVisibility', vis, 'ShowName', 'off', 'Orientation', orient);
end

function area_(sys, pos, titulo)
h = add_block('built-in/Area', [sys '/' titulo], 'Position', pos);
gris = '[0.952941, 0.952941, 0.952941]';
set_param(h, 'BackgroundColor', gris, 'ForegroundColor', gris, 'FontSize', 12);
end

function h = nota(sys, pos, texto, fuente, tam)
h = Simulink.Annotation([sys '/' strrep(strtok(texto, newline), '/', '//')]);
h.Text = texto;
h.Position = pos;
h.HorizontalAlignment = 'left';
if nargin >= 4
    h.FontName = fuente;
    h.FontSize = tam;
end
end

function borrar_linea_a(blk, puerto)
% Borra solo la rama de línea que llega a la entrada 'puerto' de 'blk'.
lh = get_param(blk, 'LineHandles');
delete_line(lh.Inport(puerto));
end

function r = ternario(c, a, b)
if c, r = a; else, r = b; end
end

function s = preload_fcn()
s = sprintf([ ...
'%% Rutas: simulink_qdrone2 (qd2_cargar_config), lib (qd2_*.m) y la carpeta\n' ...
'%% del modelo (QD2_Mission_Lib.slx de la misma versión de MATLAB).\n' ...
'qd2_d = fileparts(get_param(bdroot, ''FileName'')); qd2_r = qd2_d;\n' ...
'while ~isempty(qd2_r) && ~exist(fullfile(qd2_r, ''qd2_cargar_config.m''), ''file'')\n' ...
'    qd2_p = fileparts(qd2_r); if strcmp(qd2_p, qd2_r), qd2_r = ''''; else, qd2_r = qd2_p; end\n' ...
'end\n' ...
'if ~isempty(qd2_r), addpath(fullfile(qd2_r, ''lib'')); addpath(qd2_r); end\n' ...
'if exist(fullfile(qd2_d, ''QD2_Mission_Lib.slx''), ''file''), addpath(qd2_d); end\n' ...
'clear qd2_d qd2_r qd2_p']);
end

function t = texto_raiz(es_mpc)
if es_mpc
    v = sprintf(['VERSIÓN MPC: cada dron recibe 24 doubles (16 originales + [x y z vx vy vz] del otro dron,\n' ...
        'mpc_mode, 2 = etiqueta) y responde 8 (timestamp, FSM a bordo, batería, estado del MPC, [T roll pitch], 2).\n' ...
        'MPC LINK CHECK: FSM a bordo 7/8/9 en vuelo -> alerta 9 (el sano se aparta y aterriza); sin etiqueta -> 10.\n' ...
        'Se permite apilarse (dz >= 0.8 m); no hay SEPARATION GUARD.']);
else
    v = sprintf(['VERSIÓN PID: mismo paquete de 16 doubles del Equipo2 para cada dron (binario a bordo sin\n' ...
        'cambios). SEPARATION GUARD: nunca uno encima del otro (distancia horizontal >= 0.9 m).']);
end
t = sprintf([ ...
'DOS QDRONES (agregado al modelo del Equipo2)\n\n' ...
'Communication QDrone 1: Stream Server :18001 (argumento -URI_Host, igual que el original).\n' ...
'Communication QDrone 2: Stream Server :18002 (argumento -URI_Host2).\n' ...
'PYTHON BRIDGE: Stream Server :18700 (estación Python, ARQUITECTURA §3.3).\n' ...
'Las señales del dron 2 y de Python cruzan con Goto/From globales\n' ...
'(QDrone2_Data_In, QDrone2_Communication_Issue, QDrone2_Timestamp_Received,\n' ...
'Python_Telemetry, Python_Commands) para no mover ningún bloque original.\n\n' ...
'%s\n\n' ...
'Configuración: config/estacion.json -> qd2_cargar_config (InitFcn): rigid bodies,\n' ...
'bias de altura, archivos .cal/.motive, puertos y parámetros de misión.\n\n' ...
'Orden de arranque: este modelo (Monitor & Tune) -> estación Python -> modelo de cada dron\n' ...
'(-URI_Host tcpip://<IP_PC>:18001 y :18002). Arm/Disarm y Takeoff/Land en 1 = PERMISO;\n' ...
'el START de la UI hace despegar. Takeoff/Land a 0 en vuelo = LAND ALL ordenado del\n' ...
'supervisor (R aterriza primero, E se aparta; respaldo crudo si no lo acepta, sin soltar\n' ...
'nunca a un dron que tiene al otro debajo). Emergency Stop = corte inmediato.\n' ...
'El modelo solo se detiene solo cuando los DOS drones están en el suelo.'], v);
end

function t = texto_grupos(es_mpc)
if es_mpc
    g = 'STACKING INFO (MPC)';
else
    g = 'SEPARATION GUARD (PID)';
end
t = sprintf([ ...
'Grupos agregados (2 QDrones):\n\n' ...
'7. Localization Data Server - QDrone 2: rigid body 2 de OptiTrack, Measurement Conditioning y paquete al dron 2.\n' ...
'8. Communication, Signal Check & Flight Plan - QDrone 2: copias de los grupos 3 y 5 y del Flight Plan.\n' ...
'9. Mission Supervision: ROLE & HOME ASSIGNMENT, MISSION SUPERVISOR, %s,\n' ...
'   SETPOINT RATE LIMITER (por norma), OPERATOR SAFETY LAYER + OPERATOR COMMAND MERGE y MISSION END CHECK (paro seguro).\n' ...
'10. Python Bridge Telemetry & Blackbox: velocidades, TELEMETRY PACK (64 doubles) y log de la misión.'], g);
end
