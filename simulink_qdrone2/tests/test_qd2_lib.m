function tests = test_qd2_lib
%TEST_QD2_LIB Pruebas unitarias y de lazo cerrado de simulink_qdrone2/lib.
%   Ejecutar:  results = runtests('test_qd2_lib');   (o correr_pruebas)
%   Cada prueba imprime las métricas medidas (distancias, errores, tiempos).
tests = functiontests(localfunctions);
end

function setupOnce(tc)
raiz = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(raiz, 'lib'));
addpath(fullfile(raiz, 'tests'));
tc.TestData.p = qd2_params();
end

% =========================================================================
%  Roles, waypoints, planta y downwash
% =========================================================================
function test_roles(tc)
[e, r, h1, h2, ok] = qd2_assign_roles([0.5 0.2 0 0], [-0.7 0.3 0 0], 1, 1);
tc.verifyEqual([e r ok], [2 1 1]);                 % atrás (menor x) = explorador
tc.verifyEqual(h1, [0.5 0.2]);
tc.verifyEqual(h2, [-0.7 0.3]);
[e, r, ~, ~, ok] = qd2_assign_roles([-1 0], [1 0], 1, 1);
tc.verifyEqual([e r ok], [1 2 1]);
[e, r, ~, ~, ok] = qd2_assign_roles([-1 0], [1 0], 1, 0);
tc.verifyEqual([e r ok], [0 0 0]);                 % sin rastreo no hay roles
% en la misión: exige rastreo estable 1 s antes de ASSIGN_ROLES
p = tc.TestData.p;
s = qd2_mission_init(p);
in = entrada_basica(0, [1 0 0 0], [-1 0.5 0 0]);
t = 0; trk = 1;
estados = [];
while t < 4
    t = t + 0.01;
    in.t = t;
    in.tracked2 = trk;
    if t > 1.5 && t < 1.6, trk = 0; else, trk = 1; end
    [s, ~, ~, tel] = qd2_mission_step(s, in, p);
    estados(end+1) = tel.mission_state; %#ok<AGROW>
end
t_asig = 0.01*find(estados == 2, 1);
tc.verifyGreaterThan(t_asig, 2.55);                % 1.6 s + 1 s estable
tc.verifyEqual(tel.explorer_id, 2);                % el dron 2 está atrás
tc.verifyEqual(tel.responder_id, 1);
fprintf('[roles] ASSIGN_ROLES a t=%.2f s (rastreo perdido 1.5-1.6 s), E=%d R=%d\n', t_asig, tel.explorer_id, tel.responder_id);
end

function test_waypoints(tc)
p = tc.TestData.p;
[wp, n] = qd2_track_waypoints(p.vertices, 0.25, 1.5, p.n_vertices);
tc.verifyEqual(n, 35);                             % 1 + 10 + 14 + 10
tc.verifyEqual(size(wp), [128 4]);
tc.verifyEqual(wp(1,1:2), p.vertices(1,:));
tc.verifyEqual(wp(n,1:2), p.vertices(4,:), 'AbsTol', 1e-12);
d = sqrt(sum(diff(wp(1:n,1:2)).^2, 2));
tc.verifyLessThanOrEqual(max(d), 0.25 + 1e-9);
tc.verifyGreaterThan(min(d), 0.2);
tc.verifyEqual(wp(1:n,3), 1.5*ones(n,1));
tc.verifyEqual(wp(n+1:end,:), repmat(wp(n,:), 128-n, 1));
for k = 1:4                                        % los vértices son waypoints
    tc.verifyLessThan(min(sqrt(sum((wp(1:n,1:2) - p.vertices(k,:)).^2, 2))), 1e-9);
end
% sin n_vertices: filas NaN se ignoran
V = nan(16,2); V(1:4,:) = p.vertices(1:4,:);
[~, n2] = qd2_track_waypoints(V, 0.25, 1.5);
tc.verifyEqual(n2, 35);
% polilínea larga: se agranda el espaciado para caber en 128
V = zeros(16,2);
for k = 1:16, V(k,:) = [-1.4 + 2.8*mod(k,2), -1.4 + 2.8*(k-1)/15]; end
[wp3, n3] = qd2_track_waypoints(V, 0.05, 1.0);
tc.verifyLessThanOrEqual(n3, 128);
tc.verifyEqual(wp3(n3,1:2), V(16,:), 'AbsTol', 1e-12);
fprintf('[waypoints] N por defecto: %d wp, paso max %.3f m; polilinea larga: %d wp\n', n, max(d), n3);
end

function test_planta_downwash(tc)
p = tc.TestData.p;
[F, sg] = qd2_downwash([0 0 0.5], [0 0 1.5], p.dw);
tc.verifyEqual(F(3), -p.dw.F0*(1-exp(-10))/2, 'RelTol', 1e-12);
tc.verifyGreaterThan(-F(3)/(p.masa*p.g), 0.15);    % ~20 % del peso a 1 m
tc.verifyLessThan(-F(3)/(p.masa*p.g), 0.25);
F2 = qd2_downwash([0.25 0 0.5], [0 0 1.5], p.dw);
tc.verifyEqual(F2(3)/F(3), exp(-0.5), 'RelTol', 1e-9);    % gaussiana sigma 0.25
F3 = qd2_downwash([0 0 1.5], [0 0 0.5], p.dw);
tc.verifyEqual(F3, zeros(3,1));                    % el de arriba no recibe
tc.verifyGreaterThan(sg(3), 0);
% planta: hover con trim, pitch>0 -> +x, roll>0 -> -y
x = [0;0;1;0;0;0;0;0;0;p.T_trim];
x1 = qd2_quad_plant(x, [p.T_trim;0;0;0], [0;0;0], 2.0, p.planta);
tc.verifyEqual(x1(3), 1, 'AbsTol', 1e-9);
x2 = qd2_quad_plant(x, [p.T_trim;0;0.1;0], [0;0;0], 0.5, p.planta);
x3 = qd2_quad_plant(x, [p.T_trim;0.1;0;0], [0;0;0], 0.5, p.planta);
tc.verifyGreaterThan(x2(4), 0.1);
tc.verifyLessThan(x3(5), -0.1);
% actitud de primer orden con tau_att
x4 = qd2_quad_plant(x, [p.T_trim;0;0.2;0], [0;0;0], p.planta.tau_att, p.planta);
tc.verifyEqual(x4(8), 0.2*(1-exp(-1)), 'RelTol', 0.02);
fprintf('[downwash] dz=1.0 r=0: %.2f N (%.0f%% del peso); r=0.25: %.2f N\n', F(3), -100*F(3)/(p.masa*p.g), F2(3));
end

function test_params_config(tc)
p = qd2_params_from_config();                 % config/estacion.json del repo
cfg = jsondecode(fileread(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'config', 'estacion.json')));
tc.verifyEqual(p.z_E, cfg.mision.z_explorador);
tc.verifyEqual(p.d_min_h, cfg.mision.d_min_h);
nv = size(cfg.pista.vertices, 1);
tc.verifyEqual(p.n_vertices, nv);
tc.verifyEqual(p.vertices(1:nv,:), cfg.pista.vertices);
tc.verifyEqual(size(p.vertices), [16 2]);
tc.verifyEqual(p.T_trim, 1.504*9.81, 'AbsTol', 1e-12);
fprintf('[config] estacion.json -> z_E=%.2f z_R=%.2f d_min_h=%.2f, %d vertices\n', p.z_E, p.z_R, p.d_min_h, p.n_vertices);
end

% =========================================================================
%  Supervisor: comandos, deduplicación
% =========================================================================
function test_dedupe_objetivos(tc)
p = tc.TestData.p;
s = qd2_mission_init(p);
in = entrada_basica(0, [-1 0 0 0], [1 0 0 0]);
py = zeros(32,1);
ranuras = [1 0 0.50 0.50 0.9;     % objetivo nuevo
           2 1 0.60 0.55 0.8;     % a 0.11 m del 1 -> duplicado
           3 1 0.90 0.50 0.7];    % a 0.40 m del 1 -> distinto
for j = 1:3, py(5+5*(j-1):9+5*(j-1)) = ranuras(j,:); end
py(1) = 1;
in.py = py; in.t = 0.1;
[s, ~, ~, tel] = qd2_mission_step(s, in, p);
tc.verifyEqual(tel.n_targets_known, 2);
tc.verifyEqual(s.n_alias, 1);
tc.verifyEqual(s.alias_id(1), 2);
% el mismo id se actualiza (no se duplica)
py(5:9) = [1 0 0.52 0.49 0.95]; py(10:19) = 0; py(1) = 2;
in.py = py; in.t = 0.2;
[s, ~, ~, tel] = qd2_mission_step(s, in, p);
tc.verifyEqual(tel.n_targets_known, 2);
tc.verifyEqual([s.obj_x(1) s.obj_y(1)], [0.52 0.49]);
% hasta 32 objetivos, sin desbordar
for k = 1:40
    py(5:9) = [100+k 0 -1.3 + 0.3*mod(k,9), -1.3 + 0.3*floor(k/9), 0.9];
    py(1) = py(1) + 1; in.py = py; in.t = in.t + 0.05;
    [s, ~, ~, tel] = qd2_mission_step(s, in, p);
end
tc.verifyLessThanOrEqual(tel.n_targets_known, 32);
fprintf('[dedupe] 3 ids publicados -> %d objetivos unicos (id 2 = duplicado del 1); tras 40 mas: %d (max 32)\n', 2, tel.n_targets_known);
end

function test_comandos_y_ack(tc)
p = tc.TestData.p;
s = qd2_mission_init(p);
in = entrada_basica(0, [-1 0 0 0], [1 0 0 0]);
py = zeros(32,1);
py(1) = 1; py(2) = 1; py(3) = 7;     % START "viejo" que la estación repite al conectar
t = 0;
for k = 1:300
    t = t + 0.01; in.t = t; py(1) = k; in.py = py;
    [s, ~, ~, tel] = qd2_mission_step(s, in, p);
end
tc.verifyEqual(tel.mission_state, 2);      % no despega con el comando viejo
tc.verifyEqual(tel.cmd_seq_ack, 7);
py(3) = 8;                                  % START nuevo
in.py = py; in.t = t + 0.01;
[s, ~, ~, tel] = qd2_mission_step(s, in, p);
tc.verifyEqual(tel.mission_state, 3);
tc.verifyEqual(tel.cmd_seq_ack, 8);
tc.verifyEqual(tel.arm, [1 1]);
py(2) = 4; py(3) = 9;                       % PAUSE en TAKEOFF: se ignora pero se confirma
in.py = py; in.t = in.t + 0.01;
[s, ~, ~, tel] = qd2_mission_step(s, in, p); %#ok<ASGLU>
tc.verifyEqual(tel.mission_state, 3);
tc.verifyEqual(tel.cmd_seq_ack, 9);
fprintf('[comandos] START viejo ignorado (ack=7), START nuevo -> TAKEOFF (ack=8), PAUSE en TAKEOFF ignorado (ack=9)\n');
end

% =========================================================================
%  SEPARATION GUARD (unitario)
% =========================================================================
function test_guard_unitario(tc)
p = tc.TestData.p;
D = p.d_min_h;
% a) R marcando en (0,1.2): E (en -1.2,1.2) no avanza hacia él
gs = qd2_guard_init();
pE = [-1.2 1.2 1.5]; pR = [0 1.2 0.5];
spE = [-0.95 1.2 1.5 0]; spR = [0 1.2 0.5 0];
[spEo, spRo, act, gs] = qd2_separation_guard(pE, pR, spE, spR, p, gs, [1 0 D 0]); %#ok<ASGLU>
tc.verifyEqual(spEo(1:2), pE(1:2));         % E espera
tc.verifyEqual(spRo, spR);                  % R sigue marcando
tc.verifyEqual(act, 1);
% b) R libre (sin marcar) en el camino de E: R se evade, E espera hasta que se aleje
gs = qd2_guard_init();
pR = [0 1.2 0.5];
for k = 1:200
    [spEo, spRo, act, gs] = qd2_separation_guard(pE, pR, spE, spR, p, gs, [0 0 D 0]);
    dv = spRo(1:2) - pR(1:2);               % R se mueve <= 5 cm por paso
    if norm(dv) > 0.05, dv = 0.05*dv/norm(dv); end
    pR(1:2) = pR(1:2) + dv;
    tc.verifyGreaterThanOrEqual(norm(spEo(1:2) - spRo(1:2)), D - 1e-9);
end
tc.verifyGreaterThanOrEqual(qd2_dist_punto_segmento(pR, pE, spE), D + p.guard_margen);
tc.verifyEqual(spEo, spE);                  % E ya avanza
% c) el camino recto de R cruza a E: R espera en un punto seguro
gs = qd2_guard_init();
pE = [0 0 1.5]; spE = [0.1 0 1.5 0];
pR = [-1.3 -1.3 0.5]; spR = [1.3 1.3 0.5 0];
[~, spRo, act] = qd2_separation_guard(pE, pR, spE, spR, p, gs, [0 0 D 0]);
tc.verifyEqual(spRo(1:2), pR(1:2));
tc.verifyEqual(act, 1);
% d) prioridad invertida (E terminó): E cede a R
gs = qd2_guard_init();
pE = [0 1.0 1.5]; spE = [0 1.0 1.5 0];
pR = [0 -0.3 0.5]; spR = [0 1.0 0.5 0];
[spEo, spRo] = qd2_separation_guard(pE, pR, spE, spR, p, gs, [0 1 D 0]);
tc.verifyEqual(spRo(1:2), pR(1:2));         % R espera (E encima de su objetivo)...
tc.verifyGreaterThan(norm(spEo(1:2) - pE(1:2)), 0.1);   % ...y E se quita
fprintf('[guard] marcar->E espera; R libre en el camino->evade a %.2f m; cruce->R retenido; prioridad invertida OK\n', ...
    qd2_dist_punto_segmento(pR, pE, spE));
end

% =========================================================================
%  MPC (unitario)
% =========================================================================
function test_mpc_unitario(tc)
p = tc.TestData.p;
m = p.mpc;
N = m.N;
rng(7);
% exactitud de FISTA contra quadprog (si está disponible)
hay_qp = ~isempty(which('quadprog')) && license('test', 'Optimization_Toolbox');
err_max = 0;
for k = 1:20
    x0 = [2*randn; randn; 0.2*randn];
    xe = [x0(1); 0; 0; x0(2); 0; 0; 0; x0(3); 0];
    ms = qd2_mpc_init(p);
    u = qd2_mpc_position(xe, [0;0;0;0], nan(6,1), p, ms);
    [~, ms] = qd2_mpc_position(xe, [0;0;0;0], nan(6,1), p, qd2_mpc_init(p));
    tc.verifyLessThanOrEqual(abs(u(3)), m.att_max + 1e-12);
    if hay_qp
        f = m.Fxy*x0;
        Uq = quadprog(m.Hxy, f, [], [], [], [], -m.att_max*ones(N,1), m.att_max*ones(N,1), [], ...
            optimoptions('quadprog', 'Display', 'off', 'OptimalityTolerance', 1e-12, 'ConstraintTolerance', 1e-12));
        err_max = max(err_max, max(abs(ms.Ux - Uq)));
    end
end
if hay_qp
    tc.verifyLessThan(err_max, 1e-4);
end
% cotas de empuje y status con entrada inválida
ms = qd2_mpc_init(p);
u = qd2_mpc_position([0;0;0;0;0;0;0;0;0], [0;0;2;0], nan(6,1), p, ms);
tc.verifyLessThanOrEqual(u(1), m.T_max + 1e-9);
u = qd2_mpc_position([0;0;3;0;0;0;0;0;0], [0;0;0;0], nan(6,1), p, ms);
tc.verifyGreaterThanOrEqual(u(1), m.T_min - 1e-9);
u = qd2_mpc_position(nan(9,1), [0;0;0;0], nan(6,1), p, ms);
tc.verifyEqual(u(5), 2);
tc.verifyEqual(u(1:4)', [p.T_trim 0 0 0]);
% tiempo por paso (MATLAB interpretado en esta laptop; en C será menor)
ms = qd2_mpc_init(p);
x = [0;0;1;0;0;0;0;0;0;p.T_trim];
nt = 2000; tt = zeros(nt,1); st = zeros(nt,1);
for k = 1:nt
    ref = [0.8*sign(sin(k*0.02*0.5)); 0.5; 1; 0];
    ti = tic;
    [u, ms] = qd2_mpc_position([x(1:6); x(7:9)], ref, [0.3; 0; 2; 0; 0; 0], p, ms);
    tt(k) = toc(ti);
    st(k) = u(5);
    x = qd2_quad_plant(x, u(1:4), [0;0;0], m.Ts, p.planta);
end
tc.verifyEqual(max(st), 0);
fprintf('[mpc] N=%d, %d iter FISTA x 3 ejes; |U_fista-U_quadprog|max=%.2e rad (%s)\n', N, m.n_iter, err_max, mat2str(hay_qp));
fprintf('[mpc] tiempo/paso (MATLAB): media %.3f ms, p99 %.3f ms, max %.3f ms; status max %d\n', ...
    1e3*mean(tt(10:end)), 1e3*percentil(tt(10:end), 99), 1e3*max(tt(10:end)), max(st));
end

function test_mpc_downwash(tc)
% Vuelo apilado: E cruza y se queda 6 s encima de R (dz = 1.0 m y 0.8 m).
% El downwash real es 30 % mayor que el modelo del feedforward.
res = zeros(2, 4);
cfg = [1 1; 0 1; 1 0; 0 0];
zs = [1.5 1.3];
for a = 1:2
    for c = 1:4
        M = qd2_sim_apilado(struct('usar_ff', cfg(c,1), 'usar_obs', cfg(c,2), 'z_alto', zs(a)));
        tc.verifyTrue(M.estable);
        res(a, c) = M.err_max;
    end
    Mp = qd2_sim_apilado(struct('modo_bajo', 1, 'z_alto', zs(a)));
    fprintf('[mpc downwash] dz=%.1f m, F_dw min %.2f N: err_max z(R) ff+obs %.3f | solo obs %.3f | solo ff %.3f | sin nada %.3f | PID original %.3f m\n', ...
        zs(a) - 0.5, M.F_min, res(a,1), res(a,2), res(a,3), res(a,4), Mp.err_max);
    tc.verifyLessThan(res(a,1), 0.15);
    tc.verifyGreaterThan(res(a,4), 2*res(a,1));     % claramente peor sin ff ni observador
end
end

function test_mpc_trim_real(tc)
% En el vuelo real el empuje medio en FLIGHT fue 13.80 N (no 14.754 N):
% equivale a una masa de ~1.407 kg (ver qd2_identificar_actitud). El
% observador de perturbación debe absorber ese error de trim.
p = tc.TestData.p;
e = zeros(1,2);
for c = 1:2
    pp = p; pp.mpc.usar_obs = 2 - c;          % 1: con observador, 0: sin
    pl = p.planta; pl.masa = 1.407;
    ms = qd2_mpc_init(pp);
    x = [0;0;1;0;0;0;0;0;0;p.T_trim];
    for k = 1:750                              % 15 s
        [u, ms] = qd2_mpc_position([x(1:6); x(7:9)], [0;0;1;0], nan(6,1), pp, ms);
        x = qd2_quad_plant(x, u(1:4), [0;0;0], pp.mpc.Ts, pl);
    end
    e(c) = x(3) - 1;
end
tc.verifyLessThan(abs(e(1)), 0.005);
fprintf('[mpc trim] planta 1.407 kg vs modelo 1.504 kg: error z final con observador %.4f m, sin observador %.4f m\n', e(1), e(2));
end

% =========================================================================
%  Misión completa en lazo cerrado
% =========================================================================
function test_mision_pid(tc)
p = tc.TestData.p;
op = struct('version', 1, 'objetivos', [0 1.2; 0.5 0.5; -0.4 -1.2], ...
    'duplicados', [0.1 1.25], 'inyectar', [20 -0.8 -1.2], 't_final', 400);
L = qd2_sim_lazo_cerrado(op);
tc.verifyEqual(L.estado(end), 8);                      % DONE
n_unicos = L.n_obj(end);
tc.verifyEqual(n_unicos, 4);                           % 3 vistos + 1 inyectado, duplicado fuera
tc.verifyEqual(L.n_vis(end), n_unicos);
tc.verifyEqual(L.mask(end), 2^size(L.publicados,1) - 1);   % incluye el id duplicado
tc.verifyEqual(L.s_final.wp_idx, L.s_final.n_wp);
tc.verifyGreaterThanOrEqual(L.min_dh_aire, p.d_min_h - 0.05);
sp_min = min(L.sp_dist(L.aire));
tc.verifyGreaterThanOrEqual(sp_min, p.d_min_h - 1e-6);
% fase MISSION: E sobre un objetivo publicado -> R esperó
fprintf('[mision PID] fin t=%.1f s; objetivos %d/%d visitados; mask=%d\n', L.t(end), L.n_vis(end), n_unicos, L.mask(end));
fprintf('[mision PID] min dist horizontal medida (ambos en el aire) = %.3f m (d_min_h=%.2f); min dist consignas = %.3f m; min dist consignas tras rate limiter = %.3f m; guard activo %.0f%% del tiempo\n', ...
    L.min_dh_aire, p.d_min_h, sp_min, min(L.sp_rl_dist(L.aire)), 100*mean(L.guard));
end

function test_mision_mpc(tc)
p = tc.TestData.p;
% objetivo inyectado por delante de E sobre la N: R llega, marca y se queda
% ahí (libre) mientras E pasa por encima -> vuelo apilado con downwash.
op = struct('version', 2, 'objetivos', [0.5 0.5; -0.4 -1.2; 0.6 -1.2], ...
    'inyectar', [15 0.96 1.2], 't_final', 400);
L = qd2_sim_lazo_cerrado(op);
tc.verifyEqual(L.estado(end), 8);
tc.verifyEqual(L.n_vis(end), L.n_obj(end));
tc.verifyEqual(L.n_obj(end), 4);
ir = 3 - L.explorer_id(end);
zR = squeeze(L.pos(3,:,ir));
apil = L.aire & L.dh < 0.5 & L.estado == 5;           % dentro del chorro (2 sigma)
tc.verifyGreaterThan(sum(apil)*0.02, 1.0);             % hubo vuelo apilado
tc.verifyLessThan(min(L.F_dw(ir,:)), -1.5);            % y R sintió el downwash
eR = max(abs(zR(apil) - p.z_R));
tc.verifyLessThan(eR, 0.15);
tc.verifyEqual(max(L.mpc_status(:)), 0);
tc.verifyTrue(all(isfinite(L.pos(:))));
dzmin = min(abs(L.dz(apil)));
fprintf('[mision MPC] fin t=%.1f s; %d/%d visitados; apilados (dh<0.5 m) %.1f s (dz min %.2f m); err z(R) apilado max %.3f m; F_dw min %.2f N\n', ...
    L.t(end), L.n_vis(end), L.n_obj(end), sum(apil)*0.02, dzmin, eR, min(L.F_dw(:)));
fprintf('[mision MPC] FISTA: residuo max %.1e, status max %d; tiempo/paso MATLAB media %.3f ms, p99 %.3f ms\n', ...
    max(L.mpc_res(:)), max(L.mpc_status(:)), 1e3*mean(L.tiempo_mpc), 1e3*percentil(L.tiempo_mpc, 99));
end

function test_mision_pid_aleatorio(tc)
% 12 escenarios aleatorios (homes, objetivos sobre la N y uno inyectado)
p = tc.TestData.p;
[wp, n] = qd2_track_waypoints(p.vertices, 0.25, 1.5, p.n_vertices);
dmin = inf; tfin = zeros(1,12);
for sd = 1:12
    rng(100 + sd);
    while true
        h1 = [-1.3 + 0.9*rand, -1.3 + 2.6*rand];
        h2 = [0.2 + 1.1*rand, -1.3 + 2.6*rand];
        if norm(h1 - h2) > 1.25, break; end
    end
    nt = 2 + randi(4);
    obj = wp(randi([3 n-2], nt, 1), 1:2) + 0.05*randn(nt, 2);
    op = struct('version', 1, 'home1', h1, 'home2', h2, 'objetivos', obj, 't_final', 500, ...
        'seed', sd, 'inyectar', [15 + 20*rand, wp(randi([5 n-5]), 1:2)]);
    L = qd2_sim_lazo_cerrado(op);
    tc.verifyEqual(L.estado(end), 8, sprintf('semilla %d no terminó', sd));
    tc.verifyEqual(L.n_vis(end), L.n_obj(end));
    tc.verifyGreaterThanOrEqual(L.min_dh_aire, p.d_min_h - 0.05);
    dmin = min(dmin, L.min_dh_aire);
    tfin(sd) = L.t(end);
end
fprintf('[mision PID aleatoria] 12 escenarios: todos DONE; min dist horizontal medida %.3f m; duración %.0f-%.0f s\n', dmin, min(tfin), max(tfin));
end

function test_heartbeat_pausa(tc)
op = struct('version', 1, 'objetivos', [0 1.2; 0.5 0.5], 'hb_off', [40 46], ...
    'cmds', [3 1; 50 5], 't_final', 400);
L = qd2_sim_lazo_cerrado(op);
k10 = find(L.estado == 10, 1);
tc.verifyNotEmpty(k10);
t_p = L.t(k10);
tc.verifyGreaterThan(t_p, 40 + 1.0 - 0.06);           % >1 s sin heartbeat
tc.verifyLessThan(t_p, 40 + 1.2);
kr = find(L.t > 50.05 & L.estado == 5, 1);
tc.verifyNotEmpty(kr);                                 % RESUME vuelve a MISSION
% durante la pausa ambos quedan en su sitio
v = L.t >= t_p + 2 & L.t < 50;
mov = 0;
for i = 1:2
    P = L.pos(1:2, v, i);
    mov = max(mov, max(sqrt(sum((P - P(:,1)).^2, 1))));
end
tc.verifyLessThan(mov, 0.1);
tc.verifyEqual(L.estado(end), 8);
fprintf('[heartbeat] heartbeat cortado en t=40 s -> PAUSED en t=%.2f s; deriva en pausa %.3f m; RESUME t=50 -> MISSION t=%.2f; fin DONE t=%.1f\n', ...
    t_p, mov, L.t(kr), L.t(end));
end

function test_abort_a_home(tc)
for v = 1:2
    op = struct('version', v, 'objetivos', [0 1.2; 0.5 0.5], ...
        'cmds', [3 1; 45 2; 85 3], 't_final', 200, 'home1', [-1.0 0.2], 'home2', [0.9 -0.6]);
    L = qd2_sim_lazo_cerrado(op);
    k9 = find(L.estado == 9, 1);
    tc.verifyNotEmpty(k9);
    tc.verifyEqual(L.t(k9), 45, 'AbsTol', 0.05);
    p = L.p;
    s = L.s_final;
    if s.explorer_id == 2, zs = [p.z_R p.z_E]; else, zs = [p.z_E p.z_R]; end
    k = find(L.t >= 84.9, 1);
    H = [op.home1; op.home2];
    err = zeros(1,2);
    for i = 1:2
        err(i) = norm(L.pos(1:2, k, i)' - H(i,:));
        tc.verifyLessThan(err(i), 0.15);
        tc.verifyEqual(L.pos(3, k, i), zs(i), 'AbsTol', 0.15);
    end
    tc.verifyEqual(L.estado(end), 8);
    tc.verifyLessThan(max(abs(squeeze(L.pos(3, end, :)))), 0.06);
    if v == 1
        tc.verifyGreaterThanOrEqual(L.min_dh_aire, p.d_min_h - 0.05);
    end
    fprintf('[abort v%d] ABORT t=45 -> ABORT; a t=85 error a home %.3f / %.3f m; LAND_ALL -> DONE t=%.1f; min dh aire %.3f m\n', ...
        v, err(1), err(2), L.t(end), L.min_dh_aire);
end
end

% =========================================================================
%  Compatibilidad Simulink / codegen
% =========================================================================
function test_simulink_codegen(tc)
R = qd2_prueba_simulink(60);
tc.verifyTrue(R.ok);
tc.verifyTrue(any(R.estados == 5));
tc.verifyGreaterThanOrEqual(R.dh_min, 0.9 - 0.05);
tc.verifyEqual(R.pos3(end,:), [0.3 1.3 1.0], 'AbsTol', 0.05);   % qd2_onboard_step (PID y MPC)
tc.verifyEqual(max(R.fsm3), 5);
tc.verifyGreaterThanOrEqual(max(R.est2), 3);          % misión versión MPC también avanza
tc.verifyTrue(all(isfinite(R.act)));                  % guard con 5 args / ctx de 4 sin NaN
fprintf('[simulink] modelo con 8 bloques MATLAB Function compilado y simulado (60 s en %.1f s); estados %s (version 2: %s); dh min %.3f m; residuo FISTA max %.1e; dron 3 (qd2_onboard_step) final %s\n', ...
    R.t_sim_reloj, mat2str(R.estados), mat2str(R.estados2), R.dh_min, max(R.res), mat2str(R.pos3(end,:), 3));
end

% =========================================================================
%  Seguridad (agregadas tras la revisión)
% =========================================================================
function test_abort_antes_de_despegar(tc)
% ABORT / LAND_ALL en TAKEOFF sin nadie en el aire: nunca se da takeoff.
p = tc.TestData.p;
for cmdp = [2 3]
    % a) antes de takeoff (0.5 s tras START): desarmar y volver a ASSIGN_ROLES
    s = qd2_mission_init(p);
    in = entrada_basica(0, [-1 0 0 0], [1 0 0 0]);
    py = zeros(32,1); t = 0; tk = 0;
    for k = 1:1200
        t = t + 0.01; in.t = t; py(1) = k;
        if k == 400, py(2) = 1; py(3) = 1; end
        if k == 450, py(2) = cmdp; py(3) = 2; end
        if k == 1000, py(2) = 1; py(3) = 3; end          % START otra vez
        in.py = py;
        [s, ~, ~, tel] = qd2_mission_step(s, in, p);
        if k > 450 && k < 1000, tk = max(tk, max([tel.takeoff tel.arm])); end
        if k == 999, tc.verifyEqual(tel.mission_state, 2); end
    end
    tc.verifyEqual(tk, 0);
    tc.verifyEqual(tel.mission_state, 3);                 % se puede reintentar
    % b) takeoff ya dado pero en tierra (2.5 s tras START): ambos bajan takeoff
    s = qd2_mission_init(p);
    py = zeros(32,1); t = 0; tk = 0;
    for k = 1:1000
        t = t + 0.01; in.t = t; py(1) = k;
        if k == 400, py(2) = 1; py(3) = 1; end
        if k == 650, py(2) = cmdp; py(3) = 2; end
        in.py = py;
        [s, ~, ~, tel] = qd2_mission_step(s, in, p);
        if k > 650, tk = max(tk, max(tel.takeoff)); end
    end
    tc.verifyEqual(tk, 0);
    tc.verifyEqual(tel.mission_state, 8);
end
% c) lazo cerrado con el stack a bordo emulado: ABORT antes de takeoff,
%    ABORT durante la rampa de empuje y ABORT recién despegados
L1 = qd2_sim_lazo_cerrado(struct('cmds', [3 1; 3.5 2], 't_final', 25, 'objetivos', zeros(0,2)));
z1 = max(max(squeeze(L1.pos(3,:,:))));
tc.verifyLessThan(z1, 0.02);
tc.verifyEqual(L1.estado(end), 2);
tc.verifyEqual(max(max(L1.takeoff(:, L1.t > 3.5))), 0);
L2 = qd2_sim_lazo_cerrado(struct('cmds', [3 1; 7 2], 't_final', 40, 'objetivos', zeros(0,2)));
z2 = max(max(squeeze(L2.pos(3,:,:))));
tc.verifyLessThan(z2, 0.05);
tc.verifyEqual(L2.estado(end), 8);
tc.verifyEqual(max(max(L2.takeoff(:, L2.t > 7.01))), 0);
% primer instante con un dron a > 0.3 m
L0 = qd2_sim_lazo_cerrado(struct('cmds', [3 1], 't_final', 25, 'objetivos', zeros(0,2), 'parar_en_done', false));
zmax = max(squeeze(L0.pos(3,:,:)), [], 2)';
t_lev = L0.t(find(zmax > 0.3, 1));
L3 = qd2_sim_lazo_cerrado(struct('cmds', [3 1; t_lev 2; t_lev + 25 3], 't_final', 120, 'objetivos', zeros(0,2)));
tc.verifyEqual(L3.estado(find(L3.t > t_lev + 0.05, 1)), 9);   % en el aire -> ABORT normal
tc.verifyEqual(L3.estado(end), 8);
tc.verifyGreaterThanOrEqual(L3.min_dh_aire, p.d_min_h - 0.05);
fprintf(['[abort en tierra] ABORT/LAND_ALL 0.5 s tras START -> ASSIGN_ROLES, takeoff nunca 1; 2.5 s tras START -> DONE sin takeoff.\n' ...
    '[abort en tierra] lazo cerrado: ABORT t=3.5 -> z max %.3f m, estado %d; ABORT t=7 (rampa) -> z max %.3f m, estado %d; ' ...
    'ABORT t=%.2f (ya despegado) -> ABORT, LAND_ALL -> DONE t=%.1f\n'], z1, L1.estado(end), z2, L2.estado(end), t_lev, L3.t(end));
end

function test_homes_cerca(tc)
% Homes a menos de la separación mínima: START rechazado (alerta 1).
p = tc.TestData.p;
casos = [0.45 1; 1.00 1; 1.10 1; 0.45 2; 0.90 2];   % [distancia version]
esperado = [2 2 3 2 3];
for c = 1:size(casos, 1)
    s = qd2_mission_init(p);
    in = entrada_basica(0, [-casos(c,1)/2 0 0 0], [casos(c,1)/2 0.0 0 0]);
    in.version = casos(c,2);
    py = zeros(32,1); t = 0;
    for k = 1:450
        t = t + 0.01; in.t = t; py(1) = k;
        if k == 400, py(2) = 1; py(3) = 1; end
        in.py = py;
        [s, ~, ~, tel] = qd2_mission_step(s, in, p);
    end
    tc.verifyEqual(tel.mission_state, esperado(c), sprintf('caso %d', c));
    tc.verifyEqual(tel.alerta, double(esperado(c) == 2));
    tc.verifyEqual(tel.cmd_seq_ack, 1);
    y = qd2_telemetry_pack(t, tel, in.version, 0, in.pose1, in.pose2, tel.sp1, tel.sp2, [0 0 0], [0 0 0], [1 1 1 1 16 16 1 1]);
    tc.verifyEqual(y(55), tel.alerta);
end
L = qd2_sim_lazo_cerrado(struct('version', 1, 'home1', [-0.2 0], 'home2', [0.25 0], 't_final', 30, 'objetivos', zeros(0,2)));
zc = max(max(squeeze(L.pos(3,:,:))));
tc.verifyLessThan(zc, 0.02);
tc.verifyEqual(L.estado(end), 2);
tc.verifyEqual(L.alerta(end), 1);
fprintf('[homes cerca] PID: 0.45/1.00 m rechazado, 1.10 m aceptado (minimo %.2f m); MPC: 0.45 m rechazado, 0.90 m aceptado (minimo %.2f m); lazo cerrado 0.45 m: z max %.3f m, estado %d, alerta %d (y(55))\n', ...
    p.d_min_h + p.guard_margen, p.d_home_min_mpc, zc, L.estado(end), L.alerta(end));
end

function test_overrides_acotados(tc)
p = tc.TestData.p;
s = qd2_mission_init(p);
in = entrada_basica(0.1, [-1 0 0 0], [1 0 0 0]);
py = zeros(32,1); py(1) = 1; py(23) = 0.2; py(22) = 1e6; in.py = py;
[s, ~, ~, tel] = qd2_mission_step(s, in, p);
tc.verifyEqual(tel.d_min_h, p.d_min_h);              % no se puede reducir
tc.verifyEqual(tel.t_mark, p.t_mark_lim(2));
tc.verifyEqual(tel.ctx_guard(3), p.d_min_h);
py(23) = 5; py(22) = 0.01; py(1) = 2; in.py = py; in.t = 0.2;
[s, ~, ~, tel] = qd2_mission_step(s, in, p);
tc.verifyEqual(tel.d_min_h, p.d_min_h_max);
tc.verifyEqual(tel.t_mark, p.t_mark_lim(1));
py(23) = 1.2; py(21) = 1.5; py(20) = 1.0; py(1) = 3; in.py = py; in.t = 0.3; in.version = 2;
[s, ~, ~, tel] = qd2_mission_step(s, in, p);
tc.verifyEqual(tel.d_min_h, 1.2);
tc.verifyEqual(tel.z_R, p.z_max - p.dz_min, 'AbsTol', 1e-12);
tc.verifyGreaterThanOrEqual(tel.z_E - tel.z_R, p.dz_min - 1e-12);
% el guard tampoco acepta un d_min_h menor
[~, spRo] = qd2_separation_guard([0 0 1.5], [0.35 0 0.5], [0 0 1.5 0], [0.3 0 0.5 0], p, qd2_guard_init(), [0 0 0.2 0 0 0]);
tc.verifyGreaterThanOrEqual(norm(spRo(1:2)), p.d_min_h);
% después de START los overrides quedan fijos
s = qd2_mission_init(p);
in = entrada_basica(0, [-1 0 0 0], [1 0 0 0]);
py = zeros(32,1); t = 0;
for k = 1:600
    t = t + 0.01; in.t = t; py(1) = k;
    if k == 400, py(2) = 1; py(3) = 1; end
    if k == 500, py(23) = 1.4; py(21) = 1.0; end
    in.py = py;
    [s, ~, ~, tel] = qd2_mission_step(s, in, p);
end
tc.verifyEqual(tel.d_min_h, p.d_min_h);
tc.verifyEqual(tel.z_R, p.z_R);
fprintf('[overrides] d_min_h 0.2 -> %.2f, 5 -> %.2f; t_mark 1e6 -> %.1f, 0.01 -> %.1f; MPC z_R 1.5 -> %.2f (z_E %.2f); cambios tras START ignorados\n', ...
    p.d_min_h, p.d_min_h_max, p.t_mark_lim(2), p.t_mark_lim(1), p.z_max - p.dz_min, p.z_max);
end

function test_perdida_rastreo(tc)
p = tc.TestData.p;
% guard: con pose NaN nunca devuelve NaN
gs = qd2_guard_init();
[a1, a2] = qd2_separation_guard([-1 0 1.5], [NaN NaN NaN], [0 0 1.5 0], [1 0 0.5 0], p, gs, [0 0 0.9 0 0 0]);
tc.verifyTrue(all(isfinite([a1 a2])));
[~, ~, ~, gs] = qd2_separation_guard([-1 0 1.5], [1 0 0.5], [-1 0 1.5 0], [1 0 0.5 0], p, gs, [0 0 0.9 0 NaN NaN]);
[a1, a2] = qd2_separation_guard([-1 0 1.5], [NaN NaN NaN], [0 0 1.5 0], [NaN NaN 0.5 0], p, gs, [0 0 0.9 0]);
tc.verifyTrue(all(isfinite([a1 a2])));
tc.verifyGreaterThanOrEqual(norm(a1(1:2) - a2(1:2)), p.d_min_h);   % separación con la última pose de R
% antes de dar takeoff: vuelve a WAIT_TRACKING y desarma
s = qd2_mission_init(p);
in = entrada_basica(0, [-1 0 0 0], [1 0 0 0]);
py = zeros(32,1); t = 0;
for k = 1:460
    t = t + 0.01; in.t = t; py(1) = k;
    if k == 400, py(2) = 1; py(3) = 1; end
    if k >= 420, in.tracked2 = 0; in.pose2 = nan(4,1); end
    in.py = py;
    [s, spE, spR, tel] = qd2_mission_step(s, in, p);
end
tc.verifyEqual(tel.mission_state, 1);
tc.verifyEqual([tel.arm tel.takeoff], [0 0 0 0]);
tc.verifyTrue(all(isfinite([spE spR])));
% en MISSION (lazo cerrado): dron 2 perdido 40-45 s; RESUME a 42 s (aún
% perdido) se ignora; RESUME a 50 s vuelve a MISSION y la misión termina
op = struct('version', 1, 'objetivos', [0 1.2; 0.5 0.5], 'trk_off', [2 40 45], ...
    'cmds', [3 1; 42 5; 50 5], 't_final', 400);
L = qd2_sim_lazo_cerrado(op);
k10 = find(L.estado == 10, 1);
tc.verifyNotEmpty(k10);
t_p = L.t(k10);
tc.verifyGreaterThan(t_p, 40 + p.t_perdida_trk - 0.03);
tc.verifyLessThan(t_p, 40 + p.t_perdida_trk + 0.1);
tc.verifyEqual(L.alerta(k10), 2);
tc.verifyTrue(all(L.estado(L.t > 42 & L.t < 50) == 10));   % RESUME sin rastreo ignorado
kr = find(L.t > 50.01 & L.estado == 5, 1);
tc.verifyNotEmpty(kr);
tc.verifyTrue(all(isfinite(L.sp(:))));
% el dron rastreado (1) queda en hover donde estaba al pausar
v = L.t >= t_p & L.t < 50;
P = L.pos(1:2, v, 1);
mov = max(sqrt(sum((P - P(:,1)).^2, 1)));
tc.verifyLessThan(mov, 0.1);
tc.verifyEqual(L.estado(end), 8);
tc.verifyGreaterThanOrEqual(L.min_dh_aire, p.d_min_h - 0.05);
fprintf('[rastreo] guard con NaN -> salidas finitas; perdida antes de takeoff -> WAIT_TRACKING desarmado; en MISSION: perdida t=40 -> PAUSED t=%.2f (alerta 2), RESUME a 42 s ignorado, RESUME a 50 s -> MISSION t=%.2f; deriva del dron rastreado %.3f m; DONE t=%.1f; min dh %.3f m\n', ...
    t_p, L.t(kr), mov, L.t(end), L.min_dh_aire);
end

function test_timeouts(tc)
% Despegue y aterrizaje que no se completan (simulación cinemática simple).
p = tc.TestData.p;
% a) nadie despega -> cancelar en tierra -> DONE, alerta 3
K = sim_cinematica(p, struct('z_tope', [0 0], 'cmds', [4 1], 't_final', 60));
tc.verifyEqual(K.estado(end), 8);
tc.verifyEqual(K.alerta(end), 3);
t_c = K.t(find(K.estado == 7, 1));
% b) R no llega a su altura (tope 0.3 m) -> ABORT (hover en home), alerta 3
K2 = sim_cinematica(p, struct('z_tope', [3 0.3], 'cmds', [4 1], 't_final', 60));
tc.verifyEqual(K2.estado(end), 9);
tc.verifyEqual(K2.alerta(end), 3);
% c) pivot alto: R nunca se ve por debajo de 0.12 m -> timeout de LANDING, DONE, alerta 4
K3 = sim_cinematica(p, struct('z_piso', [0 0.12], 'cmds', [4 1; 30 3], 't_final', 150));
tc.verifyEqual(K3.estado(end), 8);
tc.verifyEqual(K3.alerta(end), 4);
t_l = K3.t(find(K3.estado == 8, 1)) - 30;
% d) sin problemas: LAND_ALL -> DONE sin alerta
K4 = sim_cinematica(p, struct('cmds', [4 1; 30 3], 't_final', 150));
tc.verifyEqual(K4.estado(end), 8);
tc.verifyEqual(K4.alerta(end), 0);
fprintf('[timeouts] nadie despega -> cancelado a t=%.1f s (takeoff a t=6 s) y DONE; R sin altura -> ABORT (alerta 3); pivot alto -> DONE %.1f s tras LAND_ALL (alerta 4); normal -> DONE %.1f s tras LAND_ALL\n', ...
    t_c, t_l, K4.t(find(K4.estado == 8, 1)) - 30);
end

function test_mision_pid_rl_por_eje(tc)
% Informativo: misión PID con el Rate Limiter2 ORIGINAL (por eje). El
% guard supone caminos rectos; por eje los movimientos diagonales hacen
% una "L". Se reporta la distancia mínima (no se garantiza d_min_h).
p = tc.TestData.p;
[wp, n] = qd2_track_waypoints(p.vertices, 0.25, 1.5, p.n_vertices);
dm = zeros(1,6);
for sd = 1:6
    rng(100 + sd);
    while true
        h1 = [-1.3 + 0.9*rand, -1.3 + 2.6*rand];
        h2 = [0.2 + 1.1*rand, -1.3 + 2.6*rand];
        if norm(h1 - h2) > 1.25, break; end
    end
    nt = 2 + randi(4);
    obj = wp(randi([3 n-2], nt, 1), 1:2) + 0.05*randn(nt, 2);
    L = qd2_sim_lazo_cerrado(struct('version', 1, 'home1', h1, 'home2', h2, 'objetivos', obj, 't_final', 500, ...
        'seed', sd, 'inyectar', [15 + 20*rand, wp(randi([5 n-5]), 1:2)], 'rl_norma', 0));
    tc.verifyEqual(L.estado(end), 8);
    dm(sd) = L.min_dh_aire;
end
tc.verifyGreaterThan(min(dm), 0.6);                   % sin contacto físico (dron de ~0.49 m)
fprintf('[rate limiter por eje] 6 escenarios: min dist horizontal por escenario %s m (d_min_h=%.2f; con limitador por norma el minimo fue >= 1.12 m)\n', ...
    mat2str(dm, 3), p.d_min_h);
end

% =========================================================================
%  Falla a bordo (versión MPC): alertas 9 y 10, reacción del dron sano
% =========================================================================
function test_enlace_mpc(tc)
% qd2_enlace_mpc: valida la respuesta de 8 doubles del dron MPC en tierra.
p = tc.TestData.p;
ok = [0.5 5 16 0 14 0 0 2];                 % [ts fsm bat est T roll pitch etiqueta]
[e, tm] = qd2_enlace_mpc([-1 -1], 1, ok, ok, [0.01 0.01], [0 0], p, 2);
tc.verifyEqual(e, [5 5 0 0]);
tc.verifyEqual(tm, [-1 -1 1 1]);                 % [t_mal1 t_mal2 visto1 visto2]
% versión PID: nada que validar
e = qd2_enlace_mpc([-1 -1], 1, ok, ok, [0.01 0.01], [0 0], p, 1);
tc.verifyEqual(e, [0 0 0 0]);
% binario PID leído en unidades de 8 doubles: todo son ecos de timestamp
pid = 10.0 + 0.001*(0:7);
% sin etiqueta, FSM 10, FSM 2.5 o FSM 0 -> inválida; 0.5 s seguidos conectado -> protocolo incorrecto
malos = {pid, [0.5 10 16 0 14 0 0 2], [0.5 2.5 16 0 14 0 0 2], [0.5 0 16 0 14 0 0 2], [0.5 5 16 0 14 0 0 0]};
for c = 1:numel(malos)
    tm = [-1 -1]; t_flag = NaN;
    for k = 0:100
        t = 2 + 0.01*k;
        [e, tm] = qd2_enlace_mpc(tm, t, malos{c}, ok, [0.01 0.01], [0 0], p, 2);
        tc.verifyEqual(e(1), 0);                       % FSM desconocida
        tc.verifyEqual(e(2), 5);
        tc.verifyEqual(e(4), 0);
        if e(3) > 0.5 && isnan(t_flag), t_flag = t - 2; end
    end
    tc.verifyEqual(t_flag, p.t_proto_mal + 0.01, 'AbsTol', 1e-9, sprintf('caso %d', c));
end
% respuesta inválida pero el dron NO está conectado (sin eco reciente, o
% Communication Status = 1): no es protocolo incorrecto
tm = [-1 -1];
for k = 0:200
    [e, tm] = qd2_enlace_mpc(tm, 2 + 0.01*k, [0 NaN(1,7)], pid, [2 + 0.01*k, 0.01], [0 1], p, 2);
    tc.verifyEqual(e, [0 0 0 0]);
end
% una trama buena reinicia el conteo
tm = [-1 -1];
for k = 0:100
    y = pid; if k == 40, y = ok; end
    [e, tm] = qd2_enlace_mpc(tm, 2 + 0.01*k, y, ok, [0.01 0.01], [0 0], p, 2);
    if k == 60, tc.verifyEqual(e(3), 0); end
end
tc.verifyEqual(e(3), 1);
% enlace perdido DESPUÉS de haber estado conectado (revisión: antes se
% reportaba la última FSM retenida, p. ej. 5, y tierra nunca reaccionaba):
% respuesta retenida (Stream Server "most recent"), loopback creciendo y
% luego Communication Status = 1 -> FSM desconocida (0) y código 2
tm = [-1 -1];
[e, tm] = qd2_enlace_mpc(tm, 10, ok, ok, [0.01 0.01], [0 0], p, 2);
tc.verifyEqual(e, [5 5 0 0]);
t_cod2 = NaN;
for t = 10.1:0.1:13
    [e, tm] = qd2_enlace_mpc(tm, t, ok, ok, [t - 10, 0.01], [double(t > 12) 0], p, 2);
    if t - 10 < p.t_proto_mal - 1e-9
        tc.verifyEqual(e, [5 5 0 0]);
    else
        tc.verifyEqual(e, [0 5 2 0]);
        if isnan(t_cod2), t_cod2 = t - 10; end
    end
end
% el enlace vuelve -> FSM otra vez y código 0
[e, tm] = qd2_enlace_mpc(tm, 13.1, ok, ok, [0.01 0.01], [0 0], p, 2);
tc.verifyEqual(e, [5 5 0 0]);
% nunca conectado (dron aún sin lanzar): no es enlace perdido
tm = [-1 -1];
for t = 0:0.1:3
    [e, tm] = qd2_enlace_mpc(tm, t, [0 NaN(1,7)], [0 NaN(1,7)], [t t], [1 1], p, 2);
    tc.verifyEqual(e, [0 0 0 0]);
end
fprintf(['[enlace MPC] respuesta válida -> FSM; sin etiqueta y(8)=2 / FSM fuera de 1..9 conectado -> protocolo incorrecto a los %.2f s; ' ...
    'desconectado sin contacto previo -> sin alerta; enlace perdido tras contacto -> FSM 0 y código 2 a los %.2f s de eco viejo\n'], ...
    p.t_proto_mal + 0.01, t_cod2);
end

function test_falla_antes_de_start(tc)
% Antes de START: protocolo incorrecto -> START rechazado (alerta 10). Un
% dron DESARMADO con FSM 7/8/9 NO es alerta (ARQUITECTURA §3.2: un 9 tras
% aterrizar es el fin normal); con START, ya armado en TAKEOFF se detecta y
% se cancela el despegue sin dar takeoff (alerta 9 mientras siga en 7/8/9).
% Sin los campos opcionales, qd2_mission_step se comporta igual que antes.
p = tc.TestData.p;
for f9 = [7 8 9]
    s = qd2_mission_init(p);
    in = entrada_basica(0, [-1 0 0 0], [1 0 0 0]);
    in.version = 2; in.fsm_a_bordo = [1 f9]; in.proto_mal = [0 0];
    py = zeros(32,1); t = 0; tk = 0; arm_max = 0; al_pre = 0;
    for k = 1:700
        t = t + 0.01; in.t = t; py(1) = k;
        if k == 400, py(2) = 1; py(3) = 1; end
        in.py = py;
        [s, ~, ~, tel] = qd2_mission_step(s, in, p);
        if k < 400, al_pre = max(al_pre, tel.alerta); end
        tk = max(tk, max(tel.takeoff));
        arm_max = max(arm_max, max(tel.arm));
    end
    tc.verifyEqual(al_pre, 0, sprintf('FSM %d desarmado antes de START no es alerta', f9));
    tc.verifyEqual(tk, 0);                               % takeoff nunca se dio
    tc.verifyEqual(tel.mission_state, 2);                % despegue cancelado
    tc.verifyEqual([tel.arm tel.takeoff], [0 0 0 0]);
    tc.verifyEqual(tel.alerta, 9);                       % se queda mientras siga en 7/8/9
    % el dron se relanza (FSM 1) -> la alerta se limpia y START despega normal
    in.fsm_a_bordo = [1 1];
    for k = 701:1200
        t = t + 0.01; in.t = t; py(1) = k;
        if k == 750, py(2) = 1; py(3) = 2; end
        in.py = py;
        [s, ~, ~, tel] = qd2_mission_step(s, in, p);
    end
    tc.verifyEqual(tel.alerta, 0);
    tc.verifyEqual(tel.mission_state, 3);
    tc.verifyEqual(tel.takeoff, [1 1]);
end
casos = {[1 1], [0 1], 10; [1 1], [0 0], 0};
for c = 1:size(casos, 1)
    s = qd2_mission_init(p);
    in = entrada_basica(0, [-1 0 0 0], [1 0 0 0]);
    in.version = 2;
    in.fsm_a_bordo = casos{c,1};
    in.proto_mal = casos{c,2};
    py = zeros(32,1); t = 0;
    for k = 1:450
        t = t + 0.01; in.t = t; py(1) = k;
        if k == 400, py(2) = 1; py(3) = 1; end
        in.py = py;
        [s, ~, ~, tel] = qd2_mission_step(s, in, p);
    end
    tc.verifyEqual(tel.alerta, casos{c,3});
    tc.verifyEqual(tel.cmd_seq_ack, 1);
    if casos{c,3} > 0
        tc.verifyEqual(tel.mission_state, 2);             % START rechazado
        tc.verifyEqual([tel.arm tel.takeoff], [0 0 0 0]);
        % el dron vuelve a estar disponible -> la alerta se limpia y START se acepta
        in.fsm_a_bordo = [1 1]; in.proto_mal = [0 0];
        for k = 451:520
            t = t + 0.01; in.t = t; py(1) = k;
            if k == 500, py(2) = 1; py(3) = 2; end
            in.py = py;
            [s, ~, ~, tel] = qd2_mission_step(s, in, p);
        end
        tc.verifyEqual(tel.alerta, 0);
    end
    tc.verifyEqual(tel.mission_state, 3);
end
% falla durante TAKEOFF antes de dar takeoff (solo armados): se desarma y
% vuelve a ASSIGN_ROLES; takeoff nunca se da
s = qd2_mission_init(p);
in = entrada_basica(0, [-1 0 0 0], [1 0 0 0]);
in.version = 2; in.fsm_a_bordo = [1 1]; in.proto_mal = [0 0];
py = zeros(32,1); t = 0; tk = 0;
for k = 1:700
    t = t + 0.01; in.t = t; py(1) = k;
    if k == 400, py(2) = 1; py(3) = 1; end
    if k == 450, in.fsm_a_bordo = [2 8]; end         % E-stop a bordo del dron 2 armado
    in.py = py;
    [s, ~, ~, tel] = qd2_mission_step(s, in, p);
    if k > 450, tk = max(tk, max([tel.takeoff tel.arm])); end
end
tc.verifyEqual(tk, 0);
tc.verifyEqual(tel.mission_state, 2);
tc.verifyEqual(tel.alerta, 9);
% compatibilidad: sin los campos opcionales = con FSM desconocida (0)
s1 = qd2_mission_init(p); s2 = s1;
in1 = entrada_basica(0, [-1 0 0 0], [1 0 0 0]); in1.version = 2;
in2 = in1; in2.fsm_a_bordo = [0 0]; in2.proto_mal = [0 0];
py = zeros(32,1); dif = 0;
for k = 1:900
    t = 0.01*k; in1.t = t; in2.t = t; py(1) = k;
    if k == 400, py(2) = 1; py(3) = 1; end
    z = min(max(0, (k - 650)*0.01), 1.5);
    in1.pose1 = [-1 0 z 0]; in1.pose2 = [1 0 min(z, 0.5) 0];
    in2.pose1 = in1.pose1; in2.pose2 = in1.pose2;
    in1.py = py; in2.py = py;
    [s1, a1, b1, t1] = qd2_mission_step(s1, in1, p);
    [s2, a2, b2, t2] = qd2_mission_step(s2, in2, p);
    dif = max([dif abs(a1 - a2) abs(b1 - b2) abs(t1.mission_state - t2.mission_state) abs(t1.takeoff - t2.takeoff)]);
end
tc.verifyEqual(dif, 0);
tc.verifyGreaterThanOrEqual(t1.mission_state, 4);
fprintf('[falla antes de START] FSM 7/8/9 desarmado -> sin alerta; con START, armado en TAKEOFF -> cancelado sin takeoff (alerta 9 hasta relanzarlo); protocolo incorrecto -> START rechazado (alerta 10); FSM 8 armado en TAKEOFF -> ASSIGN_ROLES sin takeoff; sin campos = FSM desconocida (diferencia %g)\n', dif);
end

function test_falla_R_apilado(tc)
% MPC: R falla (stack a bordo -> Error, cae) justo DEBAJO de E. E se aparta
% en horizontal a su altura, regresa a su home y aterriza; DONE.
p = tc.TestData.p;
L = qd2_sim_lazo_cerrado(struct('version', 2, 'objetivos', [0.5 0.5; -0.4 -1.2; 0.6 -1.2], ...
    'inyectar', [15 0.96 1.2], 't_final', 300, 'falla_apilado', 2));
M = metricas_falla(tc, L, p);
tc.verifyEqual(L.dron_falla, 3 - L.explorer_id(end));            % falló R
h = 3 - L.dron_falla;
H = [L.op.home1; L.op.home2];
tc.verifyLessThan(norm(L.pos(1:2,end,h)' - H(h,:)), 0.15);       % E aterrizó en su home
tc.verifyEqual(M.z_sano_fase1, p.z_E*[1 1], 'AbsTol', 0.06);     % se apartó a SU altura
fprintf(['[falla R apilado] t falla %.2f s (dh %.3f m, dz %.2f m) -> alerta 9 a los %.2f s; E se aparta: dh %.3f m a 1 s, %.3f m a 2 s, >= d_min_h a los %.2f s; ' ...
    'z de E en fase 1 [%.3f %.3f] m; DONE %.1f s tras la falla; takeoff de R re-subido %d veces\n'], ...
    L.t_falla, M.dh0, M.dz0, M.t_alerta, M.dh_1s, M.dh_2s, M.t_sep, M.z_sano_fase1, M.t_done, M.resubidas);
end

function test_falla_E_apilado(tc)
% MPC: E falla ENCIMA de R (cae sobre él). R sale de debajo de inmediato
% (mismo paso, límite xy p.falla_rate_xy), a su altura; luego home y
% aterriza. Se compara con el límite normal de 1/3 m/s.
p = tc.TestData.p;
op = struct('version', 2, 'objetivos', [0.5 0.5; -0.4 -1.2; 0.6 -1.2], ...
    'inyectar', [15 0.96 1.2], 't_final', 300, 'falla_apilado', 1);
L = qd2_sim_lazo_cerrado(op);
M = metricas_falla(tc, L, p);
tc.verifyEqual(L.dron_falla, L.explorer_id(end));                % falló E
h = 3 - L.dron_falla;
k0 = find(L.t >= L.t_falla, 1);
% R recibe la orden de apartarse en <= 2 pasos (la FSM a bordo llega con 1 paso)
tc.verifyLessThanOrEqual(M.t_escape, 2*0.02 + 1e-9);
tc.verifyGreaterThan(norm(L.sp_mision(1:2,k0+2,h) - L.sp_mision(1:2,k0-1,h)), p.d_min_h);
tc.verifyEqual(M.z_sano_fase1, p.z_R*[1 1], 'AbsTol', 0.06);
% cuando E (cayendo) llega a la altura de R, R ya está a >= d_min_h en
% horizontal (p.falla_rate_xy = 1.5 m/s)
tc.verifyGreaterThanOrEqual(M.dh_al_cruzar, p.d_min_h);
op.rate_escape = p.pid.rate_xy;                                  % comparación: límite normal
L2 = qd2_sim_lazo_cerrado(op);
M2 = metricas_falla(tc, L2, p);
fprintf(['[falla E apilado] t falla %.2f s (dh %.3f m, dz %.2f m) -> alerta 9 a los %.2f s; R recibe la orden a los %.2f s; dh %.3f m a 1 s, %.3f m a 2 s, >= d_min_h a los %.2f s; ' ...
    'dh cuando E cae a |dz| < 0.4 m: %.3f m (con el límite normal de 1/3 m/s: %.3f m); z de R en fase 1 [%.3f %.3f] m; DONE %.1f s tras la falla; takeoff de E re-subido %d veces\n'], ...
    L.t_falla, M.dh0, M.dz0, M.t_alerta, M.t_escape, M.dh_1s, M.dh_2s, M.t_sep, M.dh_al_cruzar, M2.dh_al_cruzar, ...
    M.z_sano_fase1, M.t_done, M.resubidas);
end

function test_falla_protocolo(tc)
% MPC: el dron 2 deja de mandar la etiqueta y(8) = 2 en MISSION (t = 40 s):
% a los 0.5 s, alerta 10 y se le trata como no disponible (el sano se
% aparta, regresa y aterriza; al dron 2 se le baja takeoff y nunca sube).
p = tc.TestData.p;
L = qd2_sim_lazo_cerrado(struct('version', 2, 'objetivos', [0 1.2; 0.5 0.5], ...
    'proto_mal', [2 40 400], 't_final', 300));
tc.verifyEqual(L.estado(end), 8);
k10 = find(L.alerta == 10, 1);
tc.verifyNotEmpty(k10);
tc.verifyEqual(L.t(k10), 40 + p.t_proto_mal + 0.02, 'AbsTol', 0.021);
tc.verifyEqual(max(L.falla(2,:)), 10);
tc.verifyEqual(max(L.falla(1,:)), 0);
tc.verifyEqual(sum(diff(L.takeoff(2, k10:end)) > 0.5), 0);
tc.verifyLessThan(max(abs(squeeze(L.pos(3,end,:)))), 0.06);
ks = k10 - 1 + find(L.dh(k10:end) >= p.d_min_h, 1);             % ya separados
tc.verifyNotEmpty(ks);
aire = L.aire & L.t >= L.t(ks);
dmin = min([inf L.dh(aire)]);
tc.verifyGreaterThanOrEqual(dmin, p.d_min_h - 0.05);
fprintf('[falla protocolo] etiqueta perdida t=40 s -> alerta 10 a los %.2f s (dh %.3f m); estados %s; separados a los %.2f s; DONE t=%.1f s; dh min con los dos en el aire ya separados %.3f m\n', ...
    L.t(k10), L.dh(k10), mat2str(unique(L.estado(k10:end))), L.t(ks), L.t(end), dmin);
end

function test_objetivos_y56(tc)
% y(56:60): un objetivo por mensaje, rotando [id clase x y estado].
p = tc.TestData.p;
s = qd2_mission_init(p);
in = entrada_basica(0, [-1 0 0 0], [1 0 0 0]);
py = zeros(32,1); py(1) = 1; py(4) = 3;
py(5:19) = [4 0 0.1 0.2 0.9, 7 1 -0.5 0.6 0.8, 9 1 0.7 -0.7 0.8]';
in.py = py;
ids = zeros(1, 6);
for k = 1:6
    in.t = 0.02*k;
    [s, ~, ~, tel] = qd2_mission_step(s, in, p);
    y = qd2_telemetry_pack(in.t, tel, 1, 0, in.pose1, in.pose2, tel.sp1, tel.sp2, [0 0 0], [0 0 0], [1 1 1 1 16 16 1 1]);
    ids(k) = y(56);
    j = find([4 7 9] == y(56));
    tc.verifyNotEmpty(j);
    tc.verifyEqual(y(57:60)', [py(6 + 5*(j-1)) py(7 + 5*(j-1)) py(8 + 5*(j-1)) 1], 'AbsTol', 1e-12);
end
tc.verifyEqual(sort(unique(ids)), [4 7 9]);
s0 = qd2_mission_init(p);
[~, ~, ~, tel] = qd2_mission_step(s0, entrada_basica(0, [-1 0 0 0], [1 0 0 0]), p);
y = qd2_telemetry_pack(0, tel, 1, 0, [-1 0 0 0], [1 0 0 0], tel.sp1, tel.sp2, [0 0 0], [0 0 0], [1 1 1 1 16 16 1 1]);
tc.verifyEqual(y(56:60)', [0 0 0 0 0]);
% ids fuera de 1..32 (visited_mask) se ignoran: nunca salen en y(56)
s = qd2_mission_init(p);
in = entrada_basica(0, [-1 0 0 0], [1 0 0 0]);
py = zeros(32,1); py(1) = 1; py(4) = 3;
py(5:19) = [33 0 0.1 0.2 0.9, 40 1 -0.5 0.6 0.8, 32 1 0.7 -0.7 0.8]';
in.py = py;
ids2 = zeros(1, 4);
for k = 1:4
    in.t = 0.02*k;
    [s, ~, ~, tel] = qd2_mission_step(s, in, p);
    y = qd2_telemetry_pack(in.t, tel, 1, 0, in.pose1, in.pose2, tel.sp1, tel.sp2, [0 0 0], [0 0 0], [1 1 1 1 16 16 1 1]);
    ids2(k) = y(56);
end
tc.verifyEqual(tel.n_targets_known, 1);
tc.verifyEqual(unique(ids2), 32);
fprintf('[y(56:60)] ids enviados en 6 mensajes: %s; con ids 33/40/32 publicados solo sale %s\n', mat2str(ids), mat2str(unique(ids2)));
end

function test_actitud_y61(tc)
% y(61:63) = [roll pitch válido] del EXPLORADOR (rad); y(64) reservado (0).
% att = [roll1 pitch1 roll2 pitch2] va por dron; qd2_telemetry_pack elige
% el del explorador con el mismo criterio que la pose (y(o+3:o+6)).
p = tc.TestData.p;
att = [0.05 -0.08 -0.03 0.12];
aux = [1 1 1 1 16 16 1 1];
s0 = qd2_mission_init(p);
[~, ~, ~, tel] = qd2_mission_step(s0, entrada_basica(0, [-1 0 0 0], [1 0 0 0]), p);
pack = @(tel, aux, varargin) qd2_telemetry_pack(0, tel, 1, 0, [-1 0 1.5 0.3], [1 0 1.5 -0.2], ...
    tel.sp1, tel.sp2, [0 0 0], [0 0 0], aux, varargin{:});
% explorador 1 y 2: toma la actitud del dron correcto
tel.explorer_id = 1; tel.responder_id = 2;
y = pack(tel, aux, att);
tc.verifyEqual(y(61:64)', [0.05 -0.08 1 0]);
tc.verifyEqual(y(20), 0.3);                         % yaw del dron 1 (y(o+6)): misma pose
tel.explorer_id = 2; tel.responder_id = 1;
y = pack(tel, aux, att);
tc.verifyEqual(y(61:64)', [-0.03 0.12 1 0]);
tc.verifyEqual(y(38), -0.2);                        % yaw del dron 2
% explorador no rastreado -> 0 0 0 (aunque el otro dron sí esté rastreado)
y = pack(tel, [1 0 1 1 16 16 1 1], att);
tc.verifyEqual(y(61:64)', [0 0 0 0]);
% ángulos no finitos (pérdida de rastreo en Corrected Pose Data) -> 0 0 0
y = pack(tel, aux, [0.05 -0.08 NaN 0.12]);
tc.verifyEqual(y(61:64)', [0 0 0 0]);
y = pack(tel, aux, [0.05 -0.08 0.03 Inf]);
tc.verifyEqual(y(61:64)', [0 0 0 0]);
y = pack(tel, aux, [NaN NaN 0.03 -0.04]);           % NaN solo en el responder: no importa
tc.verifyEqual(y(61:64)', [0.03 -0.04 1 0]);
% sin explorador (roles sin asignar) -> 0 0 0
tel.explorer_id = 0; tel.responder_id = 0;
y = pack(tel, aux, att);
tc.verifyEqual(y(61:64)', [0 0 0 0]);
% llamadores viejos (11 argumentos) -> 0 0 0, y el resto del vector no cambia
tel.explorer_id = 1; tel.responder_id = 2;
y11 = pack(tel, aux);
y12 = pack(tel, aux, att);
tc.verifyEqual(y11(61:64)', [0 0 0 0]);
tc.verifyEqual(y12(1:60), y11(1:60));
% con la misión: tras 1 s de rastreo estable asigna roles -> sale la actitud del explorador
s = qd2_mission_init(p);
in = entrada_basica(0, [1 0 0 0], [-1 0.5 0 0]);    % el dron 2 está atrás -> explorador 2
for k = 1:300
    in.t = 0.01*k;
    [s, ~, ~, tel] = qd2_mission_step(s, in, p);
end
tc.verifyEqual(tel.explorer_id, 2);
y = qd2_telemetry_pack(in.t, tel, 1, 0, in.pose1, in.pose2, tel.sp1, tel.sp2, [0 0 0], [0 0 0], aux, att);
ie = tel.explorer_id;
tc.verifyEqual(y(61:63)', [att(2*ie-1) att(2*ie) 1]);
fprintf('[y(61:63)] explorador %d -> roll %.3f pitch %.3f válido %d; sin rastreo / NaN / sin explorador / 11 args -> 0 0 0\n', ...
    ie, y(61), y(62), y(63));
end

function test_falla_landing_apilado(tc)
% Revisión (defecto alto): LAND_ALL dado con los drones APILADOS (MPC) y
% falla 0.2 s después, con la misión en LANDING paso 1. Antes R ya bajaba
% en su sitio (debajo de E) y no había reacción: distancia 3D mínima
% 0.378 m. Ahora R espera a su altura hasta que E se aparta >= d_min_h, y
% si uno falla en ese momento hay reacción completa (el sano se aparta).
p = tc.TestData.p;
op = struct('version', 2, 'objetivos', [0.5 0.5; -0.4 -1.2; 0.6 -1.2], ...
    'inyectar', [15 0.96 1.2], 't_final', 300);
L0 = qd2_sim_lazo_cerrado(op);
z0 = squeeze(L0.pos(3,:,:));
apil = L0.estado == 5 & L0.dh < 0.15 & abs(L0.dz) > 0.5 & min(z0, [], 2)' > 0.2;
ka = find(apil, 1);
tc.assertNotEmpty(ka);
tA = L0.t(ka) + 1.0;                       % 1 s apilados
eid = L0.explorer_id(end); rid = 3 - eid;
% (a) LAND_ALL apilado sin falla: R no baja mientras E esté encima
opA = op; opA.cmds = [3 1; tA 3];
LA = qd2_sim_lazo_cerrado(opA);
tc.verifyEqual(LA.estado(end), 8);
kA = find(LA.t >= tA, 1);
zR = squeeze(LA.pos(3,:,rid)); zE = squeeze(LA.pos(3,:,eid));
bajo = (kA:numel(LA.t));
bajo = bajo(zR(bajo) < p.z_R - 0.1 & zR(bajo) > 0.05 & zE(bajo) > zR(bajo) + 0.3);
dh_R_baja = min([inf LA.dh(bajo)]);
tc.verifyGreaterThanOrEqual(dh_R_baja, p.d_min_h - 0.05);
tc.verifyLessThan(max(abs(squeeze(LA.pos(3,end,:)))), 0.06);
res = zeros(2, 4);
for caso = 1:2
    fd = eid; if caso == 2, fd = rid; end
    h = 3 - fd;
    op2 = opA; op2.falla_fsm = [fd tA + 0.2];
    L = qd2_sim_lazo_cerrado(op2);
    k0 = find(L.t >= L.t_falla, 1);
    tc.verifyEqual(L.estado(k0), 7);                     % la falla llega en LANDING
    tc.verifyEqual(max(L.alerta(k0:end)), 9);
    tc.verifyGreaterThanOrEqual(max(L.falla_fase(k0:end)), 1);   % hay reacción
    tc.verifyEqual(L.estado(end), 8);
    tc.verifyEqual(sum(diff(L.takeoff(fd, k0:end)) > 0.5), 0);   % nunca re-sube
    tc.verifyEqual(L.takeoff(fd, end), 0);
    tc.verifyLessThan(max(abs(squeeze(L.pos(3,end,:)))), 0.06);
    % el sano se aparta a SU altura
    zh = squeeze(L.pos(3,:,h)); zf = squeeze(L.pos(3,:,fd));
    f1 = L.falla_fase == 1;
    z_rol = [p.z_E p.z_R];
    tc.verifyEqual([min(zh(f1)) max(zh(f1))], z_rol(1 + double(h == rid))*[1 1], 'AbsTol', 0.06);
    d3 = sqrt(L.dh(k0:end).^2 + L.dz(k0:end).^2);
    c = k0:numel(L.t);
    c = c(abs(zf(c) - zh(c)) < 0.4 & zf(c) > 0.05 & zh(c) > 0.05);
    dh_cr = min([inf L.dh(c)]);
    tc.verifyGreaterThanOrEqual(dh_cr, p.d_min_h - 0.05);
    ks = find(L.dh(k0:end) >= p.d_min_h, 1);
    tc.verifyNotEmpty(ks);
    res(caso,:) = [min(d3), dh_cr, L.t(k0 + ks - 1) - L.t_falla, L.t(end) - L.t_falla];
end
fprintf(['[falla en LANDING apilado] apilados t=%.2f, LAND_ALL t=%.2f: sin falla, dh min mientras R baja con E encima %.3f m. ' ...
    'E falla (encima): dist 3D min %.3f m, dh al cruzar %.3f m, >= d_min_h a los %.2f s, DONE %.1f s después. ' ...
    'R falla (debajo): dist 3D min %.3f m, dh al cruzar %.3f m, >= d_min_h a los %.2f s, DONE %.1f s después\n'], ...
    L0.t(ka), tA, dh_R_baja, res(1,:), res(2,:));
end

function test_falla_enlace_perdido(tc)
% Revisión (defecto medio): enlace caído en LOS DOS sentidos en vuelo. El
% dron se queda con el último paquete y a los 0.5 s su FSM pasa a 7, pero
% tierra no lo ve (retiene la última respuesta, FSM 5). Ahora el MPC LINK
% CHECK da código 2 (enlace perdido) y el supervisor lo trata como falla
% (alerta 9): el sano se aparta, regresa y aterriza; DONE.
p = tc.TestData.p;
% (a) apilados: cae el de ARRIBA (E) sin enlace
op = struct('version', 2, 'objetivos', [0.5 0.5; -0.4 -1.2; 0.6 -1.2], ...
    'inyectar', [15 0.96 1.2], 't_final', 300);
L0 = qd2_sim_lazo_cerrado(op);
z0 = squeeze(L0.pos(3,:,:));
apil = L0.estado == 5 & L0.dh < 0.15 & abs(L0.dz) > 0.5 & min(z0, [], 2)' > 0.2;
ka = find(apil, 1);
tc.assertNotEmpty(ka);
tA = L0.t(ka) + 0.5;
eid = L0.explorer_id(end);
op.enlace_off = [eid tA 400];
L = qd2_sim_lazo_cerrado(op);
k9 = find(L.alerta == 9, 1);
tc.verifyNotEmpty(k9);
t_al = L.t(k9) - tA;
tc.verifyLessThanOrEqual(t_al, p.t_proto_mal + 0.05);   % eco viejo 0.5 s -> código 2
tc.verifyEqual(max(L.falla(eid,:)), 9);
tc.verifyEqual(L.estado(end), 8);
h = 3 - eid;
zf = squeeze(L.pos(3,:,eid)); zh = squeeze(L.pos(3,:,h));
c = k9:numel(L.t);
c = c(abs(zf(c) - zh(c)) < 0.4 & zf(c) > 0.05 & zh(c) > 0.05);
dh_cr = min([inf L.dh(c)]);
tc.verifyGreaterThanOrEqual(dh_cr, p.d_min_h - 0.05);
tc.verifyLessThan(max(abs(squeeze(L.pos(3,end,:)))), 0.06);
% (b) sin los campos nuevos (antes de la corrección) tierra no reaccionaba:
% se reproduce con la FSM retenida -> nunca alerta 9
tm = [-1 -1 0 0];
y = [10 5 16 0 0 0 0 2];
[~, tm] = qd2_enlace_mpc(tm, 10, y, y, [0 0], [0 0], p, 2);
[e, ~] = qd2_enlace_mpc(tm, 11, y, y, [1 0], [1 0], p, 2);
tc.verifyEqual(e(1), 0);
tc.verifyEqual(e(3), 2);
fprintf(['[falla enlace perdido] E apilado encima sin enlace desde t=%.2f: FSM a bordo 7 en t=%.2f, alerta 9 a los %.2f s; ' ...
    'dh cuando E cae a la altura de R %.3f m; DONE t=%.1f\n'], tA, L.t_falla, t_al, dh_cr, L.t(end));
end

function M = metricas_falla(tc, L, p)
% Métricas comunes de las pruebas de falla a bordo (lazo cerrado).
tc.verifyFalse(isnan(L.t_falla));
tc.verifyEqual(L.estado(end), 8);                                % termina en DONE
f = L.dron_falla; h = 3 - f;
k0 = find(L.t >= L.t_falla, 1);
M.dh0 = L.dh(k0);
M.dz0 = abs(L.dz(k0));
tc.verifyLessThan(M.dh0, 0.15);                                  % estaban apilados
ka = find(L.alerta == 9, 1);
tc.verifyNotEmpty(ka);
M.t_alerta = L.t(ka) - L.t_falla;
tc.verifyLessThanOrEqual(M.t_alerta, 0.05);
M.t_escape = L.t(find(L.escape(h,:) > 0.5, 1)) - L.t_falla;
M.dh_1s = L.dh(min(k0 + 50, end));
M.dh_2s = L.dh(min(k0 + 100, end));
ks = find(L.dh(k0:end) >= p.d_min_h, 1);
tc.verifyNotEmpty(ks);
M.t_sep = L.t(k0 + ks - 1) - L.t_falla;
f1 = L.falla_fase == 1;
zh = squeeze(L.pos(3,:,h));
M.z_sano_fase1 = [min(zh(f1)) max(zh(f1))];
% el sano nunca vuelve a acercarse a < d_min_h una vez separado (hasta que
% el de la falla está en el suelo)
zf = squeeze(L.pos(3,:,f));
v = (k0 + ks - 1):numel(L.t);
v = v(zf(v) > 0.1 & zh(v) > 0.1);
tc.verifyGreaterThanOrEqual(min([inf L.dh(v)]), p.d_min_h - 0.05);
% cuando el que cae llega a la altura del sano (|dz| < 0.4 m)
c = k0:numel(L.t);
c = c(abs(zf(c) - zh(c)) < 0.4 & zf(c) > 0.05);
M.dh_al_cruzar = min([inf L.dh(c)]);
% al dron en falla nunca se le vuelve a subir takeoff
M.resubidas = sum(diff(L.takeoff(f, k0:end)) > 0.5);
tc.verifyEqual(M.resubidas, 0);
tc.verifyEqual(L.takeoff(f, end), 0);
tc.verifyEqual(max(L.falla(f,:)), 9);
tc.verifyEqual(max(L.falla(h,:)), 0);
M.t_done = L.t(find(L.estado == 8, 1)) - L.t_falla;
tc.verifyLessThan(max(abs(squeeze(L.pos(3,end,:)))), 0.06);    % los dos en el suelo
tc.verifyTrue(all(isfinite(L.sp(:))));
end

function K = sim_cinematica(p, op)
% Drones cinemáticos: siguen la consigna a 0.5 m/s; suben solo 3 s después
% de takeoff=1 (rampa) y bajan cuando takeoff=0. z_tope / z_piso por dron.
d = struct('z_tope', [3 3], 'z_piso', [0 0], 'cmds', [4 1], 't_final', 60);
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(op, f{i}), op.(f{i}) = d.(f{i}); end
end
s = qd2_mission_init(p);
P = [-1 0 0; 1 -0.3 0];
dt = 0.02; n = round(op.t_final/dt);
t_to = [-1 -1]; seq = 0; cmd = 0; hecho = false(size(op.cmds,1),1);
K.t = (1:n)*dt; K.estado = zeros(1,n); K.alerta = zeros(1,n);
for k = 1:n
    t = k*dt;
    for c = 1:size(op.cmds,1)
        if ~hecho(c) && t >= op.cmds(c,1), hecho(c) = true; seq = seq + 1; cmd = op.cmds(c,2); end
    end
    py = zeros(32,1); py(1) = k; py(2) = cmd; py(3) = seq;
    in.t = t; in.pose1 = [P(1,:) 0]; in.pose2 = [P(2,:) 0];
    in.tracked1 = 1; in.tracked2 = 1; in.py = py; in.version = 1;
    [s, ~, ~, tel] = qd2_mission_step(s, in, p);
    sp = [tel.sp1; tel.sp2];
    for i = 1:2
        if tel.takeoff(i) > 0.5
            if t_to(i) < 0, t_to(i) = t; end
        else
            t_to(i) = -1;
        end
        objetivo = sp(i,:);
        if t_to(i) < 0 || t - t_to(i) < 3
            objetivo(3) = 0;
            if t_to(i) >= 0, objetivo(1:2) = P(i,1:2); end
        end
        dv = objetivo(1:3) - P(i,:);
        if norm(dv) > 0.5*dt, dv = 0.5*dt*dv/norm(dv); end
        P(i,:) = P(i,:) + dv;
        P(i,3) = min(max(P(i,3), op.z_piso(i)), op.z_tope(i));
    end
    K.estado(k) = tel.mission_state;
    K.alerta(k) = tel.alerta;
end
end

% =========================================================================
function v = percentil(x, q)
% Percentil simple (sin Statistics Toolbox).
x = sort(x(:));
v = x(max(1, min(numel(x), ceil(q/100*numel(x)))));
end

function in = entrada_basica(t, pose1, pose2)
in.t = t;
in.pose1 = pose1(:);
in.pose2 = pose2(:);
in.tracked1 = 1;
in.tracked2 = 1;
in.py = zeros(32,1);
in.version = 1;
end
