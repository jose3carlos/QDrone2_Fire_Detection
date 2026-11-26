function generar_referencia_rastreo(salida)
%GENERAR_REFERENCIA_RASTREO Trazas de referencia de qd2_mission_step (MATLAB) con pérdida de rastreo.
%   generar_referencia_rastreo()          escribe referencia_rastreo_matlab.json junto a este archivo
%   generar_referencia_rastreo(salida)    en otra ruta
%
%   Lee escenarios_rastreo.json (misma carpeta), corre la FUNCIÓN PURA qd2_mission_step
%   (simulink_qdrone2/lib) a 50 Hz con una planta IDÉNTICA a Drone.step de web/app/mock_sim.py
%   (consigna -> velocidad 1.6*err, |vxy| <= 0.55, |vz| <= 0.5, primer orden tau = 0.35 s; takeoff = 0
%   -> baja a z = 0 en la xy de su consigna) y guarda, por escenario, los cambios de estado, alerta,
%   takeoff, falla y fase de la reacción, más las posiciones cada 0.1 s desde t0 - 1 s. Sin guard ni stack a bordo:
%   las consignas de la misión van directo a la planta (igual que el mock fuera de MISSION). START se
%   manda 0.5 s después de entrar a ASSIGN_ROLES.
%   Además evalúa qd2_params_from_config con varios valores de mision.t_perdida_aterrizar_s
%   (casos_config) para comparar la validación con config.mision_t_perdida_aterrizar de la estación,
%   y de mision.t_caido_s (casos_config_caido, config.mision_t_caido).
%   Dron caído (dual PID): 'apagados' = [dron t1] (t1 relativo a t0): desde t1 ese dron baja en su sitio
%   (consigna [x y 0] donde se apagó, sin takeoff) y no vuelve a despegar; sigue rastreado. Es el
%   MockMission.apagar_dron del mock; lo compara web/tests/test_dron_caido.py.
%
%   Lo compara web/tests/test_rastreo_perdida.py (test_mock_igual_que_matlab_*). Volver a correrlo si
%   cambia la lógica de pérdida de rastreo de qd2_mission_step:
%     matlab -batch "cd web/tests/referencia_rastreo; generar_referencia_rastreo"

aqui = fileparts(mfilename('fullpath'));
raiz = fileparts(fileparts(fileparts(aqui)));
addpath(fullfile(raiz, 'simulink_qdrone2', 'lib'));
if nargin < 1 || isempty(salida)
    salida = fullfile(aqui, 'referencia_rastreo_matlab.json');
end
E = jsondecode(fileread(fullfile(aqui, 'escenarios_rastreo.json')));
dt = E.dt;
out = struct();
out.generado_con = sprintf('MATLAB %s, qd2_mission_step de %s', version, ...
    fullfile('simulink_qdrone2', 'lib'));
out.dt = dt;
res = cell(numel(E.escenarios), 1);
for i = 1:numel(E.escenarios)
    if iscell(E.escenarios)
        esc = E.escenarios{i};
    else
        esc = E.escenarios(i);
    end
    res{i} = correr(esc, E.base, dt);
    fprintf('%-24s estados %s\n', esc.nombre, mat2str(res{i}.estados));
end
out.escenarios = res;
% validación de mision.t_perdida_aterrizar_s
casos = E.casos_config;
cc = cell(numel(casos), 1);
for i = 1:numel(casos)
    txt = casos{i};
    cfg = jsondecode(['{"mision": {"t_perdida_aterrizar_s": ' txt '}}']);
    [p, aviso] = qd2_params_from_config(cfg);
    cc{i} = struct('json', txt, 'valor', p.t_perdida_aterrizar_s, 'aviso', ~isempty(aviso));
    fprintf('t_perdida_aterrizar_s = %-12s -> %.2f %s\n', txt, p.t_perdida_aterrizar_s, aviso);
end
out.casos_config = cc;
% validación de mision.t_caido_s
casos = E.casos_config_caido;
cc = cell(numel(casos), 1);
for i = 1:numel(casos)
    txt = casos{i};
    cfg = jsondecode(['{"mision": {"t_caido_s": ' txt '}}']);
    [p, aviso] = qd2_params_from_config(cfg);
    cc{i} = struct('json', txt, 'valor', p.t_caido_s, 'aviso', ~isempty(aviso));
    fprintf('t_caido_s = %-12s -> %.2f %s\n', txt, p.t_caido_s, aviso);
end
out.casos_config_caido = cc;
fid = fopen(salida, 'w');
fwrite(fid, jsonencode(out), 'char');
fclose(fid);
fprintf('escrito %s\n', salida);
end

% =========================================================================
function r = correr(esc, base, dt)
cfg = base;
f = fieldnames(esc.mision);
for k = 1:numel(f)
    cfg.mision.(f{k}) = esc.mision.(f{k});
end
p = qd2_params_from_config(cfg);
s = qd2_mission_init(p);
H = esc.homes;
pos = [H(1,1) H(1,2) 0; H(2,1) H(2,2) 0];
vel = zeros(2, 3);
vuela = [false false];
perd = esc.perdidas;
if iscell(perd)
    perd = cell2mat(cellfun(@(c) reshape(nulo_inf(c), 1, 3), perd, 'UniformOutput', false));
end
if size(perd, 2) ~= 3
    perd = reshape(perd, [], 3);
end
perd(isnan(perd(:,3)), 3) = inf;
apag = zeros(0, 2);               % [dron t1]: apagado a bordo desde t0 + t1 (dron caído)
if isfield(esc, 'apagados') && ~isempty(esc.apagados)
    apag = esc.apagados;
end
xy_ap = nan(2, 2);                % xy donde se apagó cada dron
cmds = esc.comandos;
nombres = {'START', 'ABORT', 'LAND_ALL', 'PAUSE', 'RESUME'};
t = 0;
t0 = NaN;
t_start = NaN;       % START 0.5 s después de entrar a ASSIGN_ROLES (con roles ya evaluados)
hb = 0; cmd = 0; seq = 0; kc = 1;
N = 100000;
ev = zeros(0, 3);    % [t campo valor] (campo: 1 estado 2 alerta 3 takeoff1 4 takeoff2 5 falla1 6 falla2 7 fase)
mu = zeros(0, 7);    % [t x1 y1 z1 x2 y2 z2]
prev = nan(1, 7);
for n = 1:N
    t = t + dt;
    trk = [1 1];
    if ~isnan(t_start) && seq == 0 && t >= t_start
        seq = 1;
        cmd = 1;
    end
    if ~isnan(t0)
        for j = 1:size(perd, 1)
            if t >= t0 + perd(j,2) && t < t0 + perd(j,3)
                trk(perd(j,1)) = 0;
            end
        end
        if kc <= size(cmds, 1)
            tc = cmd_t(cmds, kc);
            if t >= t0 + tc
                seq = seq + 1;
                cmd = find(strcmp(nombres, cmd_nombre(cmds, kc)));
                kc = kc + 1;
            end
        end
    end
    hb = hb + 1;
    py = zeros(32, 1);
    py(1) = hb; py(2) = cmd; py(3) = seq;
    in = struct();
    in.t = t;
    in.pose1 = [pos(1,:) 0];
    in.pose2 = [pos(2,:) 0];
    if ~trk(1), in.pose1 = nan(1, 4); end
    if ~trk(2), in.pose2 = nan(1, 4); end
    in.tracked1 = trk(1);
    in.tracked2 = trk(2);
    in.py = py;
    in.version = 1;
    [s, ~, ~, tel] = qd2_mission_step(s, in, p);
    sp = [tel.sp1(1:3); tel.sp2(1:3)];
    for d = 1:2
        ap = false;
        for j = 1:size(apag, 1)
            if apag(j,1) == d && ~isnan(t0) && t >= t0 + apag(j,2)
                ap = true;
            end
        end
        if ap
            % apagado a bordo: baja en su sitio sin obedecer (= MockMission._aplicar_apagados)
            if isnan(xy_ap(d,1)), xy_ap(d,:) = pos(d,1:2); end
            sp(d,:) = [xy_ap(d,:) 0];
            vuela(d) = false;
            [pos(d,:), vel(d,:)] = planta(pos(d,:), vel(d,:), sp(d,:), vuela(d), dt);
            continue;
        end
        if tel.takeoff(d) > 0.5
            vuela(d) = true;
        else
            sp(d,3) = 0;
        end
        [pos(d,:), vel(d,:)] = planta(pos(d,:), vel(d,:), sp(d,:), vuela(d), dt);
        if tel.takeoff(d) < 0.5 && vuela(d) && pos(d,3) < 0.05
            vuela(d) = false;
        end
    end
    val = [tel.mission_state tel.alerta tel.takeoff(1) tel.takeoff(2) tel.falla(1) tel.falla(2) tel.falla_fase];
    for c = 1:7
        if ~isequal(val(c), prev(c))
            ev(end+1, :) = [t c val(c)]; %#ok<AGROW>
        end
    end
    prev = val;
    if isnan(t_start) && tel.mission_state == 2
        t_start = t + 0.5;
    end
    if isnan(t0) && tel.mission_state == esc.disparo.estado
        t0 = t + esc.disparo.retraso_s;
    end
    if ~isnan(t0) && mod(round((t - t0)/dt), 5) == 0 && t >= t0 - 1
        mu(end+1, :) = [t pos(1,:) pos(2,:)]; %#ok<AGROW>
    end
    if ~isnan(t0) && t >= t0 + esc.t_fin
        break;
    end
end
r = struct();
r.nombre = esc.nombre;
r.t0 = t0;
r.eventos = ev;
r.muestras = round(mu, 5);
st = ev(ev(:,2) == 1, 3);
r.estados = st';
end

function v = nulo_inf(c)
if iscell(c)
    v = zeros(1, numel(c));
    for k = 1:numel(c)
        if isempty(c{k}), v(k) = NaN; else, v(k) = c{k}; end
    end
else
    v = c;
end
end

function tc = cmd_t(cmds, k)
if iscell(cmds)
    c = cmds{k};
    if iscell(c), tc = c{1}; else, tc = c(1); end
else
    tc = cmds(k, 1);
end
end

function nm = cmd_nombre(cmds, k)
if iscell(cmds)
    c = cmds{k};
    if iscell(c), nm = c{2}; else, nm = c(2); end
else
    nm = cmds{k, 2};
end
end

function [p, v] = planta(p, v, sp, vuela, dt)
% = Drone.step de web/app/mock_sim.py (vmax_xy 0.55, vmax_z 0.5, tau 0.35)
if ~vuela && sp(3) <= 0 && p(3) <= 0.01
    v = [0 0 0];
    p(3) = 0;
    return;
end
err = sp - p;
vd = 1.6*err;
n = hypot(vd(1), vd(2));
if n > 0.55
    vd(1:2) = vd(1:2)*0.55/n;
end
vd(3) = max(-0.5, min(0.5, vd(3)));
dv = (vd - v)*min(1, dt/0.35);
v = v + dv;
p = p + v*dt;
if p(3) < 0
    p(3) = 0;
    v(3) = 0;
end
end
