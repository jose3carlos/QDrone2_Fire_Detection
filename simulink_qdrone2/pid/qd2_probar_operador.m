function R = qd2_probar_operador()
%QD2_PROBAR_OPERADOR Pruebas unitarias de la lógica de seguridad del operador.
%   R = qd2_probar_operador()
%   Extrae el código de los MATLAB Function de lib/QD2_Mission_Lib.slx tal
%   como está (OPERATOR SAFETY LAYER DUAL, OPERATOR SAFETY LAYER, MISSION
%   END CHECK, OPERATOR COMMAND MERGE) a una carpeta temporal y lo ejecuta
%   como funciones MATLAB con entradas sintéticas: modo MANUAL con 1 s
%   arm->takeoff, cambio de Command Source en vuelo, Takeoff/Land abajo ->
%   LAND ALL, respaldo crudo que no suelta a E encima de R, E-stop, paro solo
%   con los dos en el suelo y la inyección de LAND ALL con Python caído
%   (con qd2_mission_step real). Imprime OK/FALLA por caso.
raiz = fileparts(fileparts(mfilename('fullpath')));
addpath(raiz); addpath(fullfile(raiz,'lib'));
d = fullfile(tempdir, 'qd2_probar_operador'); if ~exist(d,'dir'), mkdir(d); end
load_system(fullfile(raiz,'lib','QD2_Mission_Lib.slx'));
rt = sfroot;
extraer(rt, 'OPERATOR SAFETY LAYER DUAL', 'u_osl', d);
extraer(rt, 'OPERATOR SAFETY LAYER', 'u_osl_simple', d);
extraer(rt, 'MISSION END CHECK', 'u_fin', d);
extraer(rt, 'OPERATOR COMMAND MERGE', 'u_merge', d);
addpath(d); rehash;
P.d_min_h = 0.9;
dt = 0.001;
pose = @(x,y,z,trk) [x;y;z;0;0;0;trk];
nok = 0; nfail = 0;
    function chk(nombre, cond)
        if cond, nok = nok + 1; fprintf('  OK    %s\n', nombre); else, nfail = nfail + 1; fprintf('  FALLA %s\n', nombre); end
    end
% ---- a) MANUAL: takeoff >= 1 s después de armar (switches arriba desde t = 0)
clear u_osl
D1 = pose(0,0,0.07,1); D2 = pose(1,0,0.07,1); info = zeros(32,1);
tr = [];
for t = 0:dt:1.5
    [t1,~,m] = u_osl(t, [1 1 0 0], [0;0;0], [0;0;0], 1, [0 0], D1, D2, info, P); tr(end+1,:) = [t t1 m]; %#ok<AGROW>
end
chk('manual: arm en t=0', tr(1,2) == 1);
chk('manual: sin takeoff antes de 1 s', all(tr(tr(:,1) < 0.999, 3) == 0));
chk('manual: takeoff en t = 1 s', tr(end,3) == 1 && abs(tr(find(tr(:,3) > 0.5,1),1) - 1.0) < 2e-3);
clear u_osl_simple
tr = [];
for t = 0:dt:1.5
    [t1,~,m] = u_osl_simple(t, [1 1 0 0], [0;0;0], [0;0;0], 1); tr(end+1,:) = [t t1 m]; %#ok<AGROW>
end
chk('versión simple (simulación), manual: takeoff solo tras 1 s', all(tr(tr(:,1) < 0.999, 3) == 0) && tr(end,3) == 1);

% ---- b) cambio de modo en vuelo: queda pendiente; nunca arm+takeoff solos
clear u_osl
c1 = [1;1;0]; fs = [2 2]; D1 = pose(0,0,1,1); D2 = pose(1,0,1,1); sel = 0;
lg = [];
for t = 0:dt:20
    if t >= 5, sel = 1; end                 % operador cambia Command Source en vuelo
    if t >= 10                              % misión DONE: el supervisor desarma, drones en tierra
        c1 = [0;0;0]; D1 = pose(0,0,0.0,1); D2 = pose(1,0,0.0,1); fs = [3 3];
    end
    sw = [1 1 0 0];
    if t >= 15, sw = [0 0 0 0]; end         % operador baja Arm y Takeoff
    [t1,t2,m] = u_osl(t, sw, c1, c1, sel, fs, D1, D2, info, P);
    lg(end+1,:) = [t t1 t2 m]; %#ok<AGROW>
end
chk('modo no cambia en vuelo (5-10 s)', all(lg(lg(:,1) >= 5 & lg(:,1) < 10, 10) == 0));
chk('modo no cambia con Flight State 3 (tras volar) aunque estén en tierra y con switches abajo', all(lg(:,10) == 0));
chk('tras DONE nunca vuelve a armar ni despegar solo', all(lg(lg(:,1) >= 10.01, 2) == 0) && all(lg(lg(:,1) >= 10.01, 3) == 0));
clear u_osl
D1 = pose(0,0,0.07,1); D2 = pose(1,0,0.07,1); lg = [];
for t = 0:dt:6
    sw = [1 1 0 0]; if t >= 2 && t < 4, sw = [0 0 0 0]; end
    [t1,~,m] = u_osl(t, sw, [0;0;0], [0;0;0], 1*(t >= 1), [0 0], D1, D2, info, P);
    lg(end+1,:) = [t t1 m]; %#ok<AGROW>
end
chk('nunca voló, selector a MANUAL con switches arriba: no cambia', all(lg(lg(:,1) >= 1 & lg(:,1) < 2, 6) == 0));
chk('con switches abajo y en tierra: cambia a MANUAL', lg(find(lg(:,1) >= 2.01,1), 6) == 1);
i4 = find(lg(:,1) >= 4, 1);
chk('al subir los switches en MANUAL: arm inmediato, takeoff 1 s después', lg(i4,2) == 1 && lg(i4,3) == 0 && ...
    lg(find(lg(:,1) >= 5.0, 1), 3) == 1);

% ---- c) Takeoff/Land abajo en vuelo: LAND ALL, el takeoff sigue al supervisor
clear u_osl
D1 = pose(0,0,1.5,1); D2 = pose(0.3,0.2,0.5,1); c = [1;1;0]; info(1) = 5; lg = [];
for t = 0:dt:6
    sw = [1 1 0 0]; if t >= 3, sw = [1 0 0 0]; end
    if t >= 3.002, info(1) = 7; end        % el supervisor aceptó (LANDING)
    [t1,t2,~,rq] = u_osl(t, sw, c, c, 0, [2 2], D1, D2, info, P);
    lg(end+1,:) = [t t1 t2 rq']; %#ok<AGROW>
end
chk('takeoff en vuelo antes de bajar el switch', lg(find(lg(:,1) >= 2.9,1), 3) == 1);
chk('bajar Takeoff/Land NO suelta el takeoff (sigue al supervisor)', all(lg(lg(:,1) >= 3, 3) == 1) && all(lg(lg(:,1) >= 3, 7) == 1));
chk('pide 1 LAND ALL (contador = 1) y alerta 6', lg(end, 10) == 1 && lg(end, 11) == 6);

% ---- d) respaldo crudo: el supervisor no acepta en 2 s; E encima de R a 0.36 m
clear u_osl
info(1) = 5; lg = [];
for t = 0:dt:6
    sw = [1 1 0 0]; if t >= 2, sw = [1 0 0 0]; end   % takeoff efectivo en t = 1 s; switch abajo en t = 2 s
    [t1,t2,~,rq] = u_osl(t, sw, c, c, 0, [2 2], D1, D2, info, P);   % D1 = E (z 1.5) sobre D2 = R (z 0.5)
    lg(end+1,:) = [t t1 t2 rq']; %#ok<AGROW>
end
chk('respaldo: antes de 2 s los dos siguen con takeoff', all(lg(lg(:,1) >= 1.01 & lg(:,1) < 3.99, 3) == 1) && all(lg(lg(:,1) >= 1.01 & lg(:,1) < 3.99, 7) == 1));
chk('respaldo: R (abajo) suelta el takeoff a los 2 s', lg(end, 7) == 0 && abs(lg(find(lg(:,1) > 1.5 & lg(:,7) == 0, 1), 1) - 4.0) < 2e-3);
chk('respaldo: E (con R debajo a 0.36 m) NO suelta el takeoff', lg(end, 3) == 1);
chk('respaldo: alerta 8', lg(end, 11) == 8);
clear u_osl
D2b = pose(1.2,0,0.5,1); lg = [];
for t = 0:dt:5
    sw = [1 1 0 0]; if t >= 2, sw = [1 0 0 0]; end
    [t1,t2] = u_osl(t, sw, c, c, 0, [2 2], D1, D2b, info, P); lg(end+1,:) = [t t1 t2]; %#ok<AGROW>
end
chk('respaldo: separados 1.2 m -> los dos sueltan', lg(end,3) == 0 && lg(end,7) == 0);

% ---- e) E-stop: corte inmediato + LAND ALL + alerta 5
clear u_osl
lg = [];
for t = 0:dt:2
    sw = [1 1 0 0]; if t >= 1, sw = [1 1 1 0]; end
    [t1,t2,~,rq] = u_osl(t, sw, c, c, 0, [2 2], D1, D2, info, P); lg(end+1,:) = [t t1 t2 rq']; %#ok<AGROW>
end
i1 = find(lg(:,1) >= 1, 1);
chk('E-stop inmediato en los dos', lg(i1, 4) == 1 && lg(i1, 8) == 1);
chk('E-stop pide LAND ALL y alerta 5', lg(end,10) == 1 && lg(end,11) == 5);

% ---- f) MISSION END CHECK
clear u_fin
ok = zeros(1, 20001); tt = 0:0.01:200;
for k = 1:numel(tt)
    fsm = [2 2]; if tt(k) >= 10, fsm = [9 3]; end
    ok(k) = u_fin(tt(k), fsm, pose(0,0,1.0,1), pose(1,0,0.0,1));   % dron 1 rastreado EN EL AIRE con Flight State 9
end
chk('END CHECK: nunca permite el paro con un dron rastreado en el aire (190 s en Flight State 9)', ~any(ok));
clear u_fin
tt = 0:0.01:100; ok = zeros(size(tt));
for k = 1:numel(tt)
    fsm = [2 2]; if tt(k) >= 10, fsm = [3 3]; end
    ok(k) = u_fin(tt(k), fsm, pose(0,0,0.0,0), pose(1,0,0.0,1));   % dron 1 sin rastreo, dron 2 en el suelo
end
chk('END CHECK: sin rastreo 60 s seguidos -> permite (t = 70 s)', abs(tt(find(ok,1)) - 70) < 0.02);
clear u_fin
tt = 0:0.01:30; ok = zeros(size(tt));
for k = 1:numel(tt)
    fsm = [2 2]; if tt(k) >= 10, fsm = [3 3]; end
    ok(k) = u_fin(tt(k), fsm, pose(0,0,0.02,1), pose(1,0,0.03,1));
end
chk('END CHECK: los dos en el suelo 3 s -> permite (t = 13 s)', abs(tt(find(ok,1)) - 13) < 0.02);

% ---- g) OPERATOR COMMAND MERGE + qd2_mission_step real
p = qd2_params();
s = qd2_mission_init(p);
clear u_merge
in.pose1 = [-1 0.3 0.07 0]; in.pose2 = [0.9 -0.3 0.07 0]; in.tracked1 = 1; in.tracked2 = 1; in.version = 1;
py = zeros(32,1); n = 0; est = [];
for t = 0:0.01:25
    py(1) = floor(min(t, 12)/0.05) + 1;                % Python se cae en t = 12 (heartbeat congelado)
    if t >= 5, py(2) = 1; py(3) = 1; end              % START de Python (seq 1)
    if t >= 15, n = 1; end                             % operador baja Takeoff/Land
    in.t = t; in.py = u_merge(py, [n; 6]);
    [s, ~, ~, tel] = qd2_mission_step(s, in, p);
    est(end+1,:) = [t tel.mission_state tel.cmd_seq_ack]; %#ok<AGROW>
end
e14 = est(find(est(:,1) >= 14.9,1), 2);
e15 = est(find(est(:,1) >= 15.05,1), 2);
fprintf('     estados: %s; antes de la petición = %d, después = %d, cmd_seq_ack final = %g\n', ...
    mat2str(unique(est(:,2))'), e14, e15, est(end,3));
chk('MERGE: START de Python aceptado (estado 3)', any(est(:,2) == 3));
chk('MERGE: LAND ALL del operador aceptado con Python caído', e14 ~= e15 && (e15 == 7 || e15 == 2));
chk('MERGE: cmd_seq_ack vuelve a la secuencia real de Python (1)', est(end,3) == 1);
i15 = find(est(:,1) >= 15.05, 1);
chk('MERGE: el START viejo de Python NO se re-ejecuta (no vuelve a TAKEOFF)', ~any(est(i15:end, 2) == 3));

fprintf('UNIT: %d OK, %d FALLAS\n', nok, nfail);
R = struct('ok', nok, 'fallas', nfail);
rmpath(d);
end

function extraer(rt, blk, nombre, d)
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', ['QD2_Mission_Lib/' blk '/MATLAB Function']);
s = ch.Script;
s = regexprep(s, '= fcn\(', ['= ' nombre '('], 'once');
f = fopen(fullfile(d, [nombre '.m']), 'w', 'n', 'UTF-8'); fwrite(f, s, 'char'); fclose(f);
end
