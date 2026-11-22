function R = qd2_correr_sim(version, op)
%QD2_CORRER_SIM Corre QD2_Sim_Dual sin interfaz (más rápido que tiempo real).
%   R = qd2_correr_sim(1)      % versión PID
%   R = qd2_correr_sim(2)      % versión MPC
%   R = qd2_correr_sim(v, op)  con opciones:
%     op.fuente   1 = estación simulada (por defecto), 2 = estación Python :18700
%     op.pacing   true = tiempo real (Simulation Pacing), false = lo más rápido posible (defecto)
%     op.t_final  s (defecto 400; la simulación se detiene sola 3 s después de DONE)
%     op.override struct con campos de QD2_SIM a cambiar (homes, objetivos, trk_off...)
%     op.puerto   puerto del PYTHON BRIDGE ([] = el de config/estacion.json, 18700).
%                 En pruebas sin la UI usa uno de prueba (p. ej. 18751) para no
%                 chocar con la prueba E2E (equivale a override.puerto).
%     op.dibujar  true = figura con trayectorias y estados
%
%   Devuelve métricas: misión completada (DONE) y su tiempo, objetivos
%   detectados/visitados, distancia horizontal mínima con los dos en el aire
%   (lo que importa en la versión PID) y error de z del dron de abajo
%   mientras están apilados (lo que importa en la versión MPC).

if nargin < 1, version = 1; end
if nargin < 2, op = struct(); end
d = struct('fuente', 1, 'pacing', false, 't_final', 400, 'override', struct(), 'dibujar', false, 'puerto', []);
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(op, f{i}), op.(f{i}) = d.(f{i}); end
end
dir_sim = fileparts(mfilename('fullpath'));
raiz = fileparts(dir_sim);
addpath(fullfile(raiz, 'lib'));
addpath(dir_sim);
cache = fullfile(tempdir, 'qd2_cache_simulink');
if ~exist(cache, 'dir'), mkdir(cache); end
Simulink.fileGenControl('set', 'CacheFolder', cache, 'CodeGenFolder', cache);
% QD2_Mission_Lib la regenera construir_modelos.m; si justo no está, se espera
f_lib = fullfile(raiz, 'lib', 'QD2_Mission_Lib.slx');
for k = 1:60
    if exist(f_lib, 'file'), break; end
    pause(2);
end
mdl = 'QD2_Sim_Dual';
load_system(fullfile(dir_sim, [mdl '.slx']));
b_ver = [mdl '/' sprintf('VERSION\n(1 = PID, 2 = MPC)')];
b_src = [mdl '/' sprintf('COMMAND SOURCE\n(1 = sim station, 2 = Python :18700)')];
% valores guardados en el archivo: se restauran al terminar (el modelo no
% queda "sucio" y no aparece el aviso de cambios sin guardar al cerrarlo)
orig = {get_param(b_ver, 'Value'), get_param(b_src, 'Value'), get_param(mdl, 'EnablePacing'), ...
    get_param(mdl, 'StopTime'), get_param(mdl, 'Dirty')};
limpiar = onCleanup(@() restaurar(mdl, b_ver, b_src, orig)); %#ok<NASGU>
set_param(b_ver, 'Value', num2str(version));
set_param(b_src, 'Value', num2str(op.fuente));
if op.pacing
    set_param(mdl, 'EnablePacing', 'on', 'PacingRate', 1);
else
    set_param(mdl, 'EnablePacing', 'off');
end
set_param(mdl, 'StopTime', num2str(op.t_final));
if ~isempty(op.puerto)
    op.override.puerto = op.puerto;
end
if isempty(fieldnames(op.override))
    evalin('base', 'clear QD2_SIM_OVERRIDE');
else
    assignin('base', 'QD2_SIM_OVERRIDE', op.override);
end
t0 = tic;
out = sim(mdl);
R.version = version;
R.t_reloj = toc(t0);
evalin('base', 'clear QD2_SIM_OVERRIDE');
T = filas(out.get('qd2_tel'));
X1 = filas(out.get('qd2_x1'));
X2 = filas(out.get('qd2_x2'));
S1 = filas(out.get('qd2_st1'));
S2 = filas(out.get('qd2_st2'));
n = min([size(T,1) size(X1,1) size(X2,1)]);
T = T(1:n,:); X1 = X1(1:n,:); X2 = X2(1:n,:); S1 = S1(1:n,:); S2 = S2(1:n,:);
t = T(:,1);
est = T(:,2);
R.t_sim = t(end);
R.factor_tiempo_real = R.t_sim/R.t_reloj;
c = [true; diff(est) ~= 0];
R.estados = est(c)';
R.completada = any(est == 8);
R.t_done = NaN;
if R.completada, R.t_done = t(find(est == 8, 1)); end
R.t_start = NaN;
i3 = find(est == 3, 1);
if ~isempty(i3), R.t_start = t(i3); end
R.explorer_id = T(end,4);
R.wp = [max(T(:,6)) T(end,7)];
R.explorer_done = max(T(:,14));
R.n_conocidos = max(T(:,9));
R.n_visitados = max(T(:,10));
R.visited_mask = max(T(:,51));
R.alerta_max = max(T(:,55));
R.guard_activo_frac = mean(T(est == 5, 11));
R.fsm_final = [S1(end,1) S2(end,1)];
R.bat_final = [S1(end,2) S2(end,2)];
R.estado_mpc = unique([S1(:,3); S2(:,3)])';
f1 = S1(:,1); f2 = S2(:,1);
R.fsm_seq1 = f1([true; diff(f1) ~= 0])';
R.fsm_seq2 = f2([true; diff(f2) ~= 0])';
R.comm_issue = [max(S1(:,5)) max(S2(:,5))];
aire = X1(:,3) > 0.2 & X2(:,3) > 0.2;
dh = hypot(X1(:,1) - X2(:,1), X1(:,2) - X2(:,2));
R.t_aire = sum(aire)*median(diff(t));
R.dh_min_aire = NaN;
R.dh_min_consignas = NaN;
if any(aire)
    R.dh_min_aire = min(dh(aire));
    dsp = hypot(T(:,21) - T(:,39), T(:,22) - T(:,40));
    R.dh_min_consignas = min(dsp(aire));
end
% apilamiento: R (respondedor) es el de abajo
rid = T(end,5);
if rid == 2
    zl = X2(:,3); zl_ref = T(:,41); zh = X1(:,3);
else
    zl = X1(:,3); zl_ref = T(:,23); zh = X2(:,3);
end
apil = aire & dh < 0.3 & (zh - zl) > 0.5 & est == 5;
R.t_apilado = sum(apil)*median(diff(t));
R.err_z_bajo_apilado_max = NaN;
R.err_z_bajo_apilado_rms = NaN;
if any(apil)
    e = zl(apil) - zl_ref(apil);
    R.err_z_bajo_apilado_max = max(abs(e));
    R.err_z_bajo_apilado_rms = sqrt(mean(e.^2));
end
mis = est == 5 & aire;
R.err_z_R_mision_max = NaN;
if any(mis)
    R.err_z_R_mision_max = max(abs(zl(mis) - zl_ref(mis)));
end
R.alertas = unique(T(:,55))';
% Falla a bordo (estado 7 de un stack a bordo): reacción de la misión
R.falla = struct('dron', 0);
k7 = find(S1(:,1) == 7 | S2(:,1) == 7, 1);
if ~isempty(k7)
    pz = qd2_params();
    F.dron = 1 + double(S2(k7,1) == 7 && S1(k7,1) ~= 7);
    F.t = t(k7);
    F.dh = dh(k7);
    F.dz = abs(X1(k7,3) - X2(k7,3));
    k9 = find(T(:,55) == 9 & (1:n)' >= k7, 1);
    F.t_alerta = NaN;
    if ~isempty(k9), F.t_alerta = t(k9) - F.t; end
    F.dh_1s = dh(min(k7 + round(1/median(diff(t))), n));
    F.dh_2s = dh(min(k7 + round(2/median(diff(t))), n));
    ks = find(dh(k7:end) >= pz.d_min_h, 1);
    F.t_sep = NaN;
    if ~isempty(ks), F.t_sep = t(k7 + ks - 1) - F.t; end
    if F.dron == 1, zf = X1(:,3); zh = X2(:,3); else, zf = X2(:,3); zh = X1(:,3); end
    c = (k7:n)';
    c = c(abs(zf(c) - zh(c)) < 0.4 & zf(c) > 0.05);
    F.dh_al_cruzar = min([inf; dh(c)]);
    v = (k7:n)';
    if ~isempty(ks)
        v = v(v >= k7 + ks - 1 & zf(v) > 0.1 & zh(v) > 0.1);
        F.dh_min_tras_separar = min([inf; dh(v)]);
    else
        F.dh_min_tras_separar = NaN;
    end
    e2 = est(k7:end);
    F.estados = e2([true; diff(e2) ~= 0])';
    F.z_final = [X1(end,3) X2(end,3)];
    R.falla = F;
    fprintf(['[sim %s] FALLA a bordo del dron %d en t=%.2f s (dh %.3f m, dz %.2f m) -> alerta 9 a los %.2f s; dh %.3f m a 1 s, %.3f m a 2 s; ' ...
        '>= d_min_h a los %.2f s; dh cuando el que cae llega a |dz| < 0.4 m: %.3f m; dh min ya separados %.3f m; estados %s; z final %s\n'], ...
        nombres_v(version), F.dron, F.t, F.dh, F.dz, F.t_alerta, F.dh_1s, F.dh_2s, F.t_sep, F.dh_al_cruzar, ...
        F.dh_min_tras_separar, mat2str(F.estados), mat2str(F.z_final, 3));
end
R.datos = struct('t', t, 'tel', T, 'x1', X1, 'x2', X2, 'st1', S1, 'st2', S2);
nombres = {'PID', 'MPC'};
fprintf(['[sim %s] %.1f s simulados en %.1f s de reloj (x%.1f) | estados %s | DONE=%d en %.1f s | ' ...
    'waypoints %d/%d | objetivos visitados %d de %d (mask %d) | alerta max %d\n'], ...
    nombres{version}, R.t_sim, R.t_reloj, R.factor_tiempo_real, mat2str(R.estados), R.completada, R.t_done, ...
    R.wp(1), R.wp(2), R.n_visitados, R.n_conocidos, R.visited_mask, R.alerta_max);
fprintf(['[sim %s] dh min en el aire %.3f m (consignas %.3f m) | guard activo %.0f %% de MISSION | ' ...
    'apilados %.1f s, error z del de abajo max %.4f m rms %.4f m | error z R en MISSION max %.4f m | FSM final %s\n'], ...
    nombres{version}, R.dh_min_aire, R.dh_min_consignas, 100*R.guard_activo_frac, R.t_apilado, ...
    R.err_z_bajo_apilado_max, R.err_z_bajo_apilado_rms, R.err_z_R_mision_max, mat2str(R.fsm_final));
fprintf('[sim %s] FSM a bordo dron 1 %s | dron 2 %s | comm issue %s | alertas y(55) %s\n', nombres{version}, ...
    mat2str(R.fsm_seq1), mat2str(R.fsm_seq2), mat2str(R.comm_issue), mat2str(R.alertas));
if op.dibujar
    dibujar(R);
end
end

function s = nombres_v(version)
if version > 1.5, s = 'MPC'; else, s = 'PID'; end
end

function restaurar(mdl, b_ver, b_src, orig)
if ~bdIsLoaded(mdl), return; end
set_param(b_ver, 'Value', orig{1});
set_param(b_src, 'Value', orig{2});
set_param(mdl, 'EnablePacing', orig{3});
set_param(mdl, 'StopTime', orig{4});
if strcmp(orig{5}, 'off')
    set_param(mdl, 'Dirty', 'off');
end
end

function A = filas(A)
if ndims(A) == 3
    A = permute(A, [3 1 2]);
end
end

function dibujar(R)
D = R.datos;
figure('Name', sprintf('QD2_Sim_Dual versión %d', R.version));
subplot(2,2,1);
plot(D.x1(:,1), D.x1(:,2), D.x2(:,1), D.x2(:,2)); axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)'); legend('dron 1', 'dron 2'); title('Trayectorias xy');
subplot(2,2,2);
plot(D.t, D.x1(:,3), D.t, D.x2(:,3)); grid on; xlabel('t (s)'); ylabel('z (m)'); title('Altura');
subplot(2,2,3);
plot(D.t, D.tel(:,2), D.t, D.tel(:,10)); grid on; xlabel('t (s)'); legend('estado misión', 'visitados');
subplot(2,2,4);
plot(D.t, D.tel(:,12), D.t, D.tel(:,13)); grid on; xlabel('t (s)'); legend('dh (m)', 'dz (m)');
end
