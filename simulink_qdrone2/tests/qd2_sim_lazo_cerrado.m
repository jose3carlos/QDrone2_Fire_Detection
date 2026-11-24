function L = qd2_sim_lazo_cerrado(op)
%QD2_SIM_LAZO_CERRADO Simulación SIL (MATLAB puro) de la misión con 2 drones.
%   L = qd2_sim_lazo_cerrado(op) encadena, por paso de dt:
%   poses medidas (con ruido) -> qd2_mission_step -> qd2_separation_guard
%   (solo PID) -> Rate Limiter del MissionCtrl original -> qd2_onboard_step
%   (PID o MPC) -> downwash entre drones + ruido -> qd2_quad_plant.
%   También simula la estación Python (heartbeat, comandos con cmd_seq y
%   ranuras rotativas de objetivos) y una "visión" que publica un objetivo
%   cuando el explorador pasa a menos de op.radio_camara de él.
%
%   Campos de op (todos opcionales, ver valores por defecto abajo):
%     version (1 PID / 2 MPC), home1, home2 ([x y]), objetivos (Kx2),
%     duplicados (Kx2, publicados con id nuevo junto al objetivo k=1..),
%     t_final, dt, cmds (Mx2 [t cmd]), hb_off ([t1 t2]), seed,
%     dw_factor (downwash real / modelo), usar_ff, usar_obs, ruido_pos,
%     inyectar (Kx3 [t x y]: objetivo publicado en t sin pasar por visión),
%     auto_land, parar_en_done, radio_camara,
%     rl_norma (1 = rate limiter xy por norma, recomendado; 0 = por eje
%     como el Rate Limiter2 original),
%     trk_off (Kx3 [dron t1 t2]: ese dron sin rastreo y con pose NaN en
%     [t1, t2)), py_extra (Mx2 [índice valor]: ranuras fijas de Python,
%     p. ej. overrides 20-23).
%   Falla a bordo (versión MPC, ver qd2_mission_step):
%     a_bordo (1 = tierra recibe la FSM a bordo y el chequeo de protocolo
%     de qd2_enlace_mpc, como QD2_MissionCtrl_Dual_MPC; 0 = sin esos campos,
%     como la versión PID; -1 = automático: 1 solo si version = 2),
%     falla_fsm (Kx2 [dron t]: el stack a bordo de ese dron entra a Error
%     (estado 7: actitud nivelada y rampa de empuje a -1.75 N/s hasta
%     7.38 N, luego 9 con motores apagados), como el stack original),
%     falla_apilado (0 nada; 1 = falla el EXPLORADOR, 2 = falla el
%     RESPONDEDOR la primera vez que llevan t_apilado_falla s apilados en
%     MISSION: dh < 0.15 m y dz > 0.5 m),
%     proto_mal (Kx3 [dron t1 t2]: ese dron responde sin la etiqueta
%     y(8) = 2 en [t1, t2)),
%     rate_escape (límite xy del dron que se aparta; por defecto
%     p.falla_rate_xy),
%     enlace_off (Kx3 [dron t1 t2]: enlace caído en LOS DOS sentidos en
%     [t1, t2): el dron se queda con el último paquete y a los 0.5 s pasa
%     a Error (7) como la "Detect Communication Loss" original; tierra
%     retiene su última respuesta y el loopback crece).
%   Modo de misión / parámetros:
%     p (struct de parámetros; [] = qd2_params(); p. ej. con p.modo_mision = 1
%     y p.dron_solo para el modo solo explorador),
%     z_ausente (m; NaN = nada. Si no es NaN, la z MEDIDA del dron ausente
%     (3 - p.dron_solo en solo explorador; si no, el dron 2) se fuerza a ese
%     valor mientras esté rastreado, p. ej. un dron sobre una mesa).
%   L : struct con series de tiempo y métricas. Además de lo de siempre:
%     L.y64 / L.y5 / L.y12 (qd2_telemetry_pack de cada paso),
%     L.responder_id, L.queue.

if nargin < 1, op = struct(); end
d = struct('version', 1, 'home1', [-1.0 0.0], 'home2', [1.0 -0.3], ...
    'objetivos', [0 1.2; 0.6 -0.6; -0.6 -1.2], 'duplicados', zeros(0,2), ...
    't_final', 200, 'dt', 0.02, 'cmds', [3 1], 'hb_off', [-1 -1], ...
    'seed', 1, 'dw_factor', 1.3, 'usar_ff', 1, 'usar_obs', 1, ...
    'ruido_pos', 0.002, 'radio_camara', 0.5, 'parar_en_done', true, ...
    'auto_land', 1, 'inyectar', zeros(0,3), 'rl_norma', 1, ...
    'trk_off', zeros(0,3), 'py_extra', zeros(0,2), ...
    'a_bordo', -1, 'falla_fsm', zeros(0,2), 'falla_apilado', 0, 't_apilado_falla', 1.0, ...
    'proto_mal', zeros(0,3), 'rate_escape', -1, 'enlace_off', zeros(0,3), ...
    'p', [], 'z_ausente', NaN);
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(op, f{i}), op.(f{i}) = d.(f{i}); end
end
if isempty(op.p)
    p = qd2_params();
else
    p = op.p;
end
rng(op.seed);
if op.a_bordo < 0
    op.a_bordo = double(op.version == 2);
end
if op.rate_escape < 0
    op.rate_escape = p.falla_rate_xy;
end
p.auto_land = op.auto_land;
p.mpc.usar_ff = op.usar_ff;
p.mpc.usar_obs = op.usar_obs;
dw_real = p.dw;
dw_real.F0 = p.dw.F0*op.dw_factor;
dt = op.dt;
n = round(op.t_final/dt);

% estados de planta [x y z vx vy vz roll pitch yaw T]
X = zeros(10, 2);
X(1:2,1) = op.home1(:); X(1:2,2) = op.home2(:);
ob = [qd2_onboard_init(p), qd2_onboard_init(p)];
s = qd2_mission_init(p);
gs = qd2_guard_init();
sp_rl = [X(1,1) X(2,1) 0 0; X(1,2) X(2,2) 0 0];   % salida del rate limiter
rate = [p.pid.rate_xy p.pid.rate_xy p.pid.rate_z p.pid.rate_yaw];

% estación Python simulada
hb = 0; seq = 0; cmd = 0;
nT = size(op.objetivos, 1);
obs = zeros(nT, 1);
pub = zeros(0, 5);         % [id cls x y conf] publicados
pub_dup = false(size(op.duplicados, 1), 1);
idn = 0;
rot = 0;
cmd_hecho = false(size(op.cmds, 1), 1);

% registro
L.t = (1:n)*dt;
L.pos = nan(3, n, 2); L.sp = nan(4, n, 2); L.sp_mision = nan(4, n, 2);
L.estado = nan(1, n); L.guard = nan(1, n); L.dh = nan(1, n); L.dz = nan(1, n);
L.fsm = nan(2, n); L.mpc_status = nan(2, n); L.mpc_res = nan(2, n);
L.wp_idx = nan(1, n); L.n_vis = nan(1, n); L.n_obj = nan(1, n);
L.mask = nan(1, n); L.ack = nan(1, n); L.F_dw = nan(2, n); L.d_hat = nan(2, n);
L.r_marking = nan(1, n); L.explorer_id = nan(1, n); L.tiempo_mpc = zeros(0,1);
L.alerta = nan(1, n); L.arm = nan(2, n); L.takeoff = nan(2, n);
L.falla_fase = nan(1, n); L.escape = nan(2, n); L.falla = nan(2, n);
L.y64 = nan(1, n); L.y5 = nan(1, n); L.y12 = nan(1, n); L.responder_id = nan(1, n); L.queue = nan(1, n);
% dron ausente (z_ausente): el del modo solo explorador, o el 2
i_aus = 2;
if p.modo_mision > 0.5 && (p.dron_solo == 1 || p.dron_solo == 2)
    i_aus = 3 - p.dron_solo;
end
ultimo = n;
% falla a bordo
T7 = [0 0];                 % empuje de la rampa de emergencia (estado 7)
u_prev = zeros(4, 2);
t_mal = [-1 -1 0 0];        % estado de qd2_enlace_mpc
y_ult = [0 NaN(1,7); 0 NaN(1,7)];   % última respuesta recibida en tierra
rx = zeros(2, 6);           % último paquete recibido a bordo [arm takeoff x y z yaw]
t_sin = [-1 -1];            % desde cuándo cada dron no recibe paquetes
L.t_falla = NaN;            % instante de la falla inyectada
L.dron_falla = 0;
t_apil = -1;

for k = 1:n
    t = k*dt;
    % ---------------- estación Python -------------------------------------
    if ~(t >= op.hb_off(1) && t < op.hb_off(2)) && mod(k, round(0.05/dt)) == 0
        hb = hb + 1;
    end
    for c = 1:size(op.cmds, 1)
        if ~cmd_hecho(c) && t >= op.cmds(c,1)
            cmd_hecho(c) = true;
            seq = seq + 1;
            cmd = op.cmds(c,2);
        end
    end
    % visión: el explorador ve objetivos bajo él
    if s.explorer_id > 0
        ie = s.explorer_id;
        if X(3,ie) > 1.0
            for j = 1:nT
                if norm(X(1:2,ie)' - op.objetivos(j,:)) < op.radio_camara
                    obs(j) = obs(j) + 1;
                    if obs(j) == 5
                        idn = idn + 1;
                        pub(end+1,:) = [idn mod(j,2) op.objetivos(j,:) + 0.02*randn(1,2) 0.9]; %#ok<AGROW>
                        % duplicados asociados a este objetivo
                        for q = 1:size(op.duplicados, 1)
                            if ~pub_dup(q) && norm(op.duplicados(q,:) - op.objetivos(j,:)) < 0.3
                                pub_dup(q) = true;
                                idn = idn + 1;
                                pub(end+1,:) = [idn mod(j,2) op.duplicados(q,:) 0.7]; %#ok<AGROW>
                            end
                        end
                    end
                end
            end
        end
    end
    for j = 1:size(op.inyectar, 1)
        if abs(t - op.inyectar(j,1)) < dt/2
            idn = idn + 1;
            pub(end+1,:) = [idn 1 op.inyectar(j,2:3) 0.8]; %#ok<AGROW>
        end
    end
    py = zeros(32, 1);
    py(1) = hb; py(2) = cmd; py(3) = seq; py(4) = size(pub, 1);
    for j = 1:size(op.py_extra, 1)
        py(op.py_extra(j,1)) = op.py_extra(j,2);
    end
    if ~isempty(pub) && mod(k, round(0.05/dt)) == 0
        rot = rot + 1;
    end
    for j = 1:3
        if ~isempty(pub)
            r = mod(rot*3 + j - 1, size(pub, 1)) + 1;
            py(5 + 5*(j-1) : 9 + 5*(j-1)) = pub(r, :);
        end
    end

    % ---------------- tierra: misión + guard -------------------------------
    pm = X(1:3,:) + op.ruido_pos*randn(3,2);
    in.t = t;
    in.pose1 = [pm(:,1); X(9,1)];
    in.pose2 = [pm(:,2); X(9,2)];
    in.tracked1 = 1; in.tracked2 = 1;
    for j = 1:size(op.trk_off, 1)
        if t >= op.trk_off(j,2) && t < op.trk_off(j,3)
            if op.trk_off(j,1) == 1
                in.tracked1 = 0; in.pose1 = nan(4,1);
            else
                in.tracked2 = 0; in.pose2 = nan(4,1);
            end
        end
    end
    if ~isnan(op.z_ausente)
        if i_aus == 1 && in.tracked1 > 0.5
            in.pose1(3) = op.z_ausente;
        elseif i_aus == 2 && in.tracked2 > 0.5
            in.pose2(3) = op.z_ausente;
        end
    end
    in.py = py;
    in.version = op.version;
    if op.a_bordo > 0.5
        % respuesta de 8 doubles simulada [ts fsm bat est T roll pitch etiqueta]
        tg = [2 2];
        for j = 1:size(op.proto_mal, 1)
            if t >= op.proto_mal(j,2) && t < op.proto_mal(j,3)
                tg(op.proto_mal(j,1)) = 0;
            end
        end
        lb = [0 0];
        for i = 1:2
            if enlace_caido(op.enlace_off, i, t)
                lb(i) = t - y_ult(i,1);           % se retiene la última respuesta
            else
                y_ult(i,:) = [t ob(i).fsm 16 0 0 0 0 tg(i)];
            end
        end
        [ab, t_mal] = qd2_enlace_mpc(t_mal, t, y_ult(1,:), y_ult(2,:), lb, [0 0], p, 2);
        in.fsm_a_bordo = ab(1:2);
        in.proto_mal = ab(3:4);
    end
    [s, spE, spR, tel] = qd2_mission_step(s, in, p);
    if s.explorer_id == 2
        pE = in.pose2; pR = in.pose1;
    else
        pE = in.pose1; pR = in.pose2;
    end
    ga = 0;
    if op.version == 1
        [spE, spR, ga, gs] = qd2_separation_guard(pE, pR, spE, spR, p, gs, tel.ctx_guard);
    end
    if s.explorer_id == 2
        sp = [spR; spE];
    else
        sp = [spE; spR];
    end
    for i = 1:2
        rt = rate;
        if tel.escape(i) > 0.5
            rt(1:2) = op.rate_escape;      % se aparta del dron en falla
        end
        sp_rl(i,:) = qd2_rate_limit(sp_rl(i,:), sp(i,:), rt, dt, op.rl_norma);
    end

    % ---------------- fallas a bordo inyectadas ------------------------------
    for j = 1:size(op.falla_fsm, 1)
        i = op.falla_fsm(j,1);
        if t >= op.falla_fsm(j,2) && ob(i).fsm <= 6 && isnan(L.t_falla)
            ob(i).fsm = 7; T7(i) = u_prev(1,i);
            L.t_falla = t; L.dron_falla = i;
        end
    end
    if op.falla_apilado > 0 && isnan(L.t_falla)
        apil = tel.mission_state == 5 && norm(X(1:2,1) - X(1:2,2)) < 0.15 && ...
            abs(X(3,1) - X(3,2)) > 0.5 && min(X(3,:)) > 0.2;
        if apil
            if t_apil < 0, t_apil = t; end
            if t - t_apil >= op.t_apilado_falla
                if op.falla_apilado == 1, i = s.explorer_id; else, i = s.responder_id; end
                ob(i).fsm = 7; T7(i) = u_prev(1,i);
                L.t_falla = t; L.dron_falla = i;
            end
        else
            t_apil = -1;
        end
    end

    % enlace caído en los dos sentidos: el dron se queda con el último
    % paquete y a los 0.5 s pasa a Error (7)
    for i = 1:2
        if enlace_caido(op.enlace_off, i, t)
            if t_sin(i) < 0, t_sin(i) = t; end
            if t - t_sin(i) >= 0.5 && ob(i).fsm <= 6
                ob(i).fsm = 7; T7(i) = u_prev(1,i);
                if isnan(L.t_falla), L.t_falla = t; L.dron_falla = i; end
            end
        else
            t_sin(i) = -1;
            rx(i,:) = [tel.arm(i) tel.takeoff(i) sp_rl(i,:)];
        end
    end

    % ---------------- a bordo + planta -------------------------------------
    for i = 1:2
        o = 3 - i;
        xe = [X(1:3,i) + op.ruido_pos*randn(3,1); X(4:6,i) + 0.01*randn(3,1); X(7:9,i)];
        other = [X(1:3,o); X(4:6,o)];
        mod_mpc = op.version;
        ti = tic;
        if ob(i).fsm == 7
            % Error -> EMERGENCY STOP del stack original: actitud nivelada,
            % sin control de posición, empuje baja a 1.75 N/s; a 7.38 N -> 9
            T7(i) = max(T7(i) - 1.75*dt, 0);
            u = [T7(i); 0; 0; 0];
            if T7(i) <= 7.38
                ob(i).fsm = 9;
                u = zeros(4, 1);
            end
        else
            [u, ob(i)] = qd2_onboard_step(ob(i), rx(i,1), rx(i,2), rx(i,3:6), xe, other, p, dt, mod_mpc);
        end
        u_prev(:,i) = u;
        if op.version == 2 && ob(i).fsm >= 4 && ob(i).fsm <= 6
            L.tiempo_mpc(end+1,1) = toc(ti);
        end
        [Fdw, sg] = qd2_downwash(X(1:3,i), X(1:3,o), dw_real);
        Fd = Fdw + sg.*randn(3,1);
        X(:,i) = qd2_quad_plant(X(:,i), u, Fd, dt, p.planta);
        L.F_dw(i,k) = Fdw(3);
        L.fsm(i,k) = ob(i).fsm;
        L.mpc_status(i,k) = ob(i).status;
        L.mpc_res(i,k) = ob(i).res;
        L.d_hat(i,k) = ob(i).mst.d_hat;
    end

    % ---------------- registro ---------------------------------------------
    L.pos(:,k,1) = X(1:3,1); L.pos(:,k,2) = X(1:3,2);
    L.sp(:,k,1) = sp_rl(1,:)'; L.sp(:,k,2) = sp_rl(2,:)';
    L.sp_mision(:,k,1) = sp(1,:)'; L.sp_mision(:,k,2) = sp(2,:)';
    L.estado(k) = tel.mission_state; L.guard(k) = ga;
    L.dh(k) = norm(X(1:2,1) - X(1:2,2)); L.dz(k) = X(3,1) - X(3,2);
    L.wp_idx(k) = tel.explorer_wp_idx; L.n_vis(k) = tel.n_targets_visited;
    L.n_obj(k) = tel.n_targets_known; L.mask(k) = tel.visited_mask;
    L.ack(k) = tel.cmd_seq_ack; L.r_marking(k) = tel.r_marking;
    L.explorer_id(k) = tel.explorer_id; L.alerta(k) = tel.alerta;
    L.arm(:,k) = tel.arm(:); L.takeoff(:,k) = tel.takeoff(:);
    L.falla_fase(k) = tel.falla_fase; L.escape(:,k) = tel.escape(:); L.falla(:,k) = tel.falla(:);
    L.sp_dist(k) = norm(sp(1,1:2) - sp(2,1:2)); %#ok<AGROW>
    yt = qd2_telemetry_pack(t, tel, op.version, ga, in.pose1, in.pose2, sp_rl(1,:), sp_rl(2,:), ...
        X(4:6,1), X(4:6,2), [in.tracked1 in.tracked2 ob(1).fsm ob(2).fsm NaN NaN 1 1]);
    L.y64(k) = yt(64); L.y5(k) = yt(5); L.y12(k) = yt(12);
    L.responder_id(k) = tel.responder_id; L.queue(k) = tel.queue_length;
    L.sp_rl_dist(k) = norm(sp_rl(1,1:2) - sp_rl(2,1:2)); %#ok<AGROW>
    if op.parar_en_done && tel.mission_state == 8 && t > 5
        ultimo = k;
        break;
    end
end
% recortar
campos = {'t','estado','guard','dh','dz','wp_idx','n_vis','n_obj','mask','ack','r_marking','explorer_id','sp_dist','sp_rl_dist','alerta','arm','takeoff', ...
    'falla_fase','escape','falla','y64','y5','y12','responder_id','queue'};
for i = 1:numel(campos)
    L.(campos{i}) = L.(campos{i})(:, 1:ultimo);
end
L.pos = L.pos(:, 1:ultimo, :); L.sp = L.sp(:, 1:ultimo, :); L.sp_mision = L.sp_mision(:, 1:ultimo, :);
L.fsm = L.fsm(:, 1:ultimo); L.mpc_status = L.mpc_status(:, 1:ultimo);
L.mpc_res = L.mpc_res(:, 1:ultimo); L.F_dw = L.F_dw(:, 1:ultimo); L.d_hat = L.d_hat(:, 1:ultimo);
L.s_final = s;
L.tel_final = tel;
L.publicados = pub;
L.p = p;
L.op = op;
% métricas: ambos en el aire (z > 0.2 m)
aire = squeeze(L.pos(3,:,1)) > 0.2 & squeeze(L.pos(3,:,2)) > 0.2;
L.aire = aire;
if any(aire)
    L.min_dh_aire = min(L.dh(aire));
else
    L.min_dh_aire = nan;
end
end

function c = enlace_caido(eo, i, t)
c = false;
for j = 1:size(eo, 1)
    if eo(j,1) == i && t >= eo(j,2) && t < eo(j,3)
        c = true;
    end
end
end
