function R = sim3d_correr(opc)
%SIM3D_CORRER Motor del simulador 3D en tiempo real de 2 QDrone 2.
%   R = sim3d_correr(opc)     (opc: ver sim3d_opciones / qdrone2_sim3d.m)
%
%   Cadena por paso de opc.dt (igual a simulink_qdrone2/tests/qd2_sim_lazo_cerrado.m
%   y a los modelos de tierra):
%     estación (interna o Python por TCP) -> 32 doubles
%     poses OptiTrack (con ruido) -> qd2_mission_step
%     -> qd2_separation_guard (solo PID) -> limitador por NORMA (qd2_rate_limit(...,1))
%     -> qd2_onboard_step (PID original o MPC) -> downwash + ruido -> qd2_quad_plant
%     detector simulado de la RealSense del explorador (clusters de la estación)
%     -> telemetría de 64 doubles (qd2_telemetry_pack) por TCP :18700
%
%   R: struct con el registro (R.log), métricas (R.met) y opciones usadas.

if nargin < 1, opc = struct(); end
opc = sim3d_opciones(opc);
aqui = fileparts(mfilename('fullpath'));
raiz = fileparts(aqui);
lib = fullfile(raiz, 'simulink_qdrone2', 'lib');
if isempty(which('qd2_mission_step'))
    addpath(lib);
end
C = sim3d_config(raiz);
% uso de la actitud en la geolocalización del detector interno ([] = config/estacion.json, vision.*)
if ~isempty(opc.usar_actitud), C.vis.usar_actitud = logical(opc.usar_actitud); end
if ~isempty(opc.inclinacion_max_deg)
    C.vis.incl_max_deg = double(opc.inclinacion_max_deg);
    if ~(isfinite(C.vis.incl_max_deg) && C.vis.incl_max_deg > 0), C.vis.incl_max_deg = []; end
end
sb_ = deg2rad(opc.sesgo_montaje_deg);
R_montaje = rot_zyx(0, sb_(2), sb_(1));   % error del soporte (solo la cámara real; la estación no lo conoce)
p = C.p;
if ~isempty(opc.auto_land), p.auto_land = double(opc.auto_land); end
version = 1 + double(strcmpi(char(opc.version), 'MPC'));
dt = opc.dt;
% Flujo de números aleatorios PROPIO para sensores, planta y detector: la corrida da lo
% mismo con figura o sin ella (la figura usa el flujo global para cosas cosméticas).
rs = RandStream('mt19937ar', 'Seed', opc.semilla_ruido);
reloj_hb = tic;          % reloj de pared para el heartbeat de Python (ver sim3d_estacion)
esc = sim3d_escenario(opc, C);
log_msg(opc, 'Simulador 3D QDrone 2 · versión %s · escenario "%s" (%d objetos)', ...
    ternario(version == 1, 'PID', 'MPC'), esc.nombre, numel(esc.obj));

% ---------------- estado de la simulación ---------------------------------
X = zeros(10, 2);
X(1:2, 1) = esc.home(1,:).'; X(1:2, 2) = esc.home(2,:).';
X(9, :) = esc.yaw0;
ob = [qd2_onboard_init(p), qd2_onboard_init(p)];
fp = [0 0];              % Flight State de TIERRA (Flight Plan del Equipo2) por dron: o+2 en la versión PID
s = qd2_mission_init(p);
gs = qd2_guard_init();
sp_rl = [X(1,1) X(2,1) 0 X(9,1); X(1,2) X(2,2) 0 X(9,2)];
rate = [p.pid.rate_xy p.pid.rate_xy p.pid.rate_z p.pid.rate_yaw];
bat = [16.6 16.5];
est = sim3d_estacion('init', [], opc);
det = sim3d_detector('init', [], C, opc);
det.rs = rs;
objetivos_det = zeros(0, 5);
in_ult = [];             % última entrada de la misión (poses medidas y tracked) para la telemetría
n_tel = 0;               % mensajes de telemetría enviados (rota las ranuras 56-60)
aviso_vision_web = false;
cada_cam = max(1, round(1/(opc.hz_camara*dt)));
cada_tel = max(1, round(1/(opc.hz_telemetria*dt)));
cmd_prog = opc.comandos;
if ~isempty(opc.auto_start)
    cmd_prog = [cmd_prog; opc.auto_start 1];
end
cmd_prog_hecho = false(size(cmd_prog, 1), 1);
t_en_home = -1;
land_auto_hecho = false;
pt = [];
if opc.usar_tcp
    puerto = C.puerto;
    if ~isempty(opc.puerto), puerto = opc.puerto; end
    try
        pt = sim3d_puente('abrir', [], opc.host_tcp, puerto, opc.backend_tcp);
        log_msg(opc, 'Puente §3.3 escuchando en tcpip://%s:%d (backend %s)', opc.host_tcp, puerto, pt.backend);
    catch err
        warning('sim3d:tcp', 'No se pudo abrir el puerto %d: %s. Sigo sin TCP.', puerto, err.message);
        pt = [];
    end
end
limpiar = onCleanup(@() sim3d_puente('cerrar_todo', [])); %#ok<NASGU>

% ---------------- registro -------------------------------------------------
n_max = ceil(opc.t_final/dt) + 1;
Lg.t = nan(1, n_max);
Lg.pos = nan(3, n_max, 2); Lg.sp = nan(4, n_max, 2); Lg.att = nan(3, n_max, 2);
Lg.estado = nan(1, n_max); Lg.guard = nan(1, n_max); Lg.dh = nan(1, n_max); Lg.dz = nan(1, n_max);
Lg.arm = nan(2, n_max); Lg.takeoff = nan(2, n_max); Lg.fsm = nan(2, n_max);
Lg.wp_idx = nan(1, n_max); Lg.n_wp = nan(1, n_max); Lg.n_obj = nan(1, n_max); Lg.n_vis = nan(1, n_max);
Lg.marking = nan(1, n_max); Lg.r_marking = nan(1, n_max); Lg.explorer_done = nan(1, n_max);
Lg.alerta = nan(1, n_max); Lg.hb_ok = nan(1, n_max); Lg.modo_python = nan(1, n_max);
Lg.explorer_id = nan(1, n_max); Lg.t_mark_acum = nan(1, n_max); Lg.t_en_estado = nan(1, n_max);
Lg.t_dwell = nan(1, n_max);
Lg.F_dw = nan(2, n_max); Lg.ack = nan(1, n_max); Lg.det_cuadros = nan(1, n_max);
Lg.tel_att = nan(3, n_max);   % y(61:63) de la telemetría [roll pitch válido] del explorador
k = 0;
t = 0;
y64 = zeros(64, 1);
tel = [];
ga = 0;
dh_min = inf; dz_min_apil = inf;
estados_vistos = false(1, 11);
estados_cap = [];
paquetes_pend = zeros(32, 0);
n_estela = max(2, round(opc.t_estela/0.1));
estela = {nan(n_estela, 3), nan(n_estela, 3)};
t_estela = -inf;

% ---------------- figura ---------------------------------------------------
G = [];
con_figura = opc.mostrar || ~isempty(opc.capturas) || ~isempty(opc.capturas_estados);
if con_figura
    G = sim3d_escena('crear', [], C, esc, opc, raiz);
    log_msg(opc, 'Figura lista (mallas de objetos: %s).', G.origen_mallas);
end
carpeta_cap = opc.carpeta_capturas;
if ~isempty(carpeta_cap) && ~isfolder(carpeta_cap) && ~startsWith(carpeta_cap, filesep) && ~contains(carpeta_cap, ':')
    carpeta_cap = fullfile(aqui, carpeta_cap);
end
[capturas_pend, orden_cap] = sort(opc.capturas(:).');
vistas_cap = opc.capturas_vistas(:).';
if numel(vistas_cap) == numel(capturas_pend)
    vistas_cap = vistas_cap(orden_cap);
else
    vistas_cap = zeros(size(capturas_pend));        % 0 = no cambiar la vista
end
archivos_cap = {};

% ---------------- bucle principal -------------------------------------------
vel = opc.velocidad;
paso_inicial();          % primer paso para tener tel
reloj = tic;
if con_figura && opc.mostrar && isgraphics(G.fig)
    dibujar();           % el primer cuadro tarda (inicializa la gráfica): antes de arrancar el reloj
    drawnow;
end
reloj = tic;
t_ref_sim = 0; t_ref_reloj = 0;
t_render = -inf;
n_atrasos = 0;
n_cuadros = 0;
salir = false;
t_cuadro_max = 0;
cuadros_lentos = zeros(0, 3);     % [t_sim ms estado] de cuadros de más de 150 ms (diagnóstico)
while ~salir
    % ---- cuántos pasos tocan
    if opc.tiempo_real
        if con_figura && isgraphics(G.fig)
            v2 = getappdata(G.fig, 'vel');
            if v2 ~= vel
                t_ref_sim = t; t_ref_reloj = toc(reloj); vel = v2;
            end
        end
        t_obj = t_ref_sim + (toc(reloj) - t_ref_reloj)*vel;
        if t_obj - t > 1.0*max(vel, 1)        % más de 1 s atrasado (la PC no da): se re-sincroniza
            n_atrasos = n_atrasos + 1;
            t_ref_sim = t; t_ref_reloj = toc(reloj);
            t_obj = t + dt;
        end
        n_pasos = floor((t_obj - t)/dt + 1e-9);
        if n_pasos <= 0
            pause(0.002);
            if con_figura && ~opc.mostrar, continue; end
        end
    else
        if opc.mostrar
            n_pasos = max(1, round(vel*8));
        else
            n_pasos = 1;
        end
    end
    % ---- entradas: TCP y figura
    if ~isempty(pt)
        [pt, P] = sim3d_puente('leer', pt);
        paquetes_pend = [paquetes_pend P]; %#ok<AGROW>
    end
    cmds_fig = [];
    if con_figura && isgraphics(G.fig)
        cmds_fig = getappdata(G.fig, 'cola_cmd');
        setappdata(G.fig, 'cola_cmd', []);
        if getappdata(G.fig, 'salir'), salir = true; end
        if isappdata(G.fig, 'vista_pedida')
            G = sim3d_escena('vista', G, getappdata(G.fig, 'vista_pedida'));
            rmappdata(G.fig, 'vista_pedida');
        end
    elseif con_figura && opc.mostrar
        salir = true;      % se cerró la ventana
    end
    t_iter0 = toc(reloj);
    for q = 1:n_pasos
        paso(cmds_fig);
        cmds_fig = [];
        if t >= opc.t_final - 1e-9 || fin_por_done()
            salir = true;
            break;
        end
        % Ráfagas cortas: a x16 (o cuando la PC se atrasa) una ráfaga podría durar más de
        % t_hb_python de reloj sin leer el TCP y la misión vería caerse el heartbeat de Python.
        % Los pasos que falten se hacen en la siguiente vuelta (después de leer el TCP y la figura).
        if toc(reloj) - t_iter0 > 0.04
            break;
        end
    end
    % ---- gráficos
    if con_figura && isgraphics(G.fig)
        if opc.mostrar && toc(reloj) - t_render >= 1/opc.fps
            t_render = toc(reloj);
            dibujar();
            drawnow;
            n_cuadros = n_cuadros + 1;
            dtc = toc(reloj) - t_render;
            t_cuadro_max = max(t_cuadro_max, dtc);
            if dtc > 0.15, cuadros_lentos(end+1, :) = [t 1000*dtc s.estado]; end %#ok<AGROW>
        end
        revisar_capturas();
    end
end
cerrar_puente();
t_total = toc(reloj);

% ---------------- resultados ------------------------------------------------
campos = fieldnames(Lg);
for ic = 1:numel(campos)      % (no usar i: las funciones anidadas la comparten)
    v = Lg.(campos{ic});
    if ndims(v) == 3
        Lg.(campos{ic}) = v(:, 1:k, :);
    else
        Lg.(campos{ic}) = v(:, 1:k);
    end
end
R.log = Lg;
R.opc = opc;
R.esc = esc;
R.version = version;
R.p = p;
R.s_final = s;
R.tel_final = tel;
R.objetivos_detector = sim3d_detector('objetivos', det);
R.det = det;
R.est = est;
R.met = metricas();
R.capturas = archivos_cap;
if ~isempty(pt), R.tcp = rmfield(pt, intersect(fieldnames(pt), {'srv', 'cli', 'bufrx', 'pend'})); else, R.tcp = []; end
R.figura = [];
if con_figura && isgraphics(G.fig)
    if opc.mostrar
        R.figura = G.fig;
    else
        close(G.fig);        % figura invisible (solo capturas): se libera
    end
end
log_msg(opc, ['Fin: t = %.1f s simulados en %.1f s de reloj (factor %.2f) · estado %d · ' ...
    'objetivos %d (visitados %d) de %d reales · dh mín. en vuelo %s · re-sincronizaciones %d'], ...
    t, t_total, t/max(t_total, 1e-6), s.estado, R.met.n_objetivos, R.met.n_visitados, numel(esc.obj), ...
    num2str(R.met.dh_min_aire, '%.3f'), n_atrasos);
R.met.t_reloj = t_total;
R.met.n_atrasos = n_atrasos;
R.met.fps_medio = n_cuadros/max(t_total, 1e-6);
R.met.cuadro_max_ms = 1000*t_cuadro_max;
R.met.cuadros_lentos = cuadros_lentos;
if opc.graficas_finales && k > 1
    guardar = opc.guardar_graficas;
    if isempty(guardar), guardar = ~opc.mostrar; end
    R.graficas = sim3d_graficas(R, guardar, carpeta_cap);
end

% =========================================================================
%                     funciones anidadas (comparten estado)
% =========================================================================
    function paso_inicial()
        py0 = zeros(32, 1);
        in = entradas(0, py0);
        in_ult = in;
        [s, ~, ~, tel] = qd2_mission_step(s, in, p);
    end

    function in = entradas(tt, py)
        pm = X(1:3, :) + opc.ruido_pos*randn(rs, 3, 2);
        in.t = tt;
        in.pose1 = [pm(:,1); X(9,1)];
        in.pose2 = [pm(:,2); X(9,2)];
        % actitud medida [roll1 pitch1 roll2 pitch2] (rad), como Corrected Pose Data(4:5):
        % solo la usa la telemetría (y(61:63)); sin ruido para no mover el generador aleatorio
        in.att = [X(7,1) X(8,1) X(7,2) X(8,2)];
        in.tracked1 = 1; in.tracked2 = 1;
        for j = 1:size(opc.perdida_rastreo, 1)
            pr = opc.perdida_rastreo(j, :);
            if tt >= pr(2) && tt < pr(3)
                if pr(1) == 1
                    in.tracked1 = 0; in.pose1 = nan(4, 1); in.att(1:2) = NaN;
                else
                    in.tracked2 = 0; in.pose2 = nan(4, 1); in.att(3:4) = NaN;
                end
            end
        end
        in.py = py;
        in.version = version;
        if version == 2
            % Como tierra MPC (Onboard Status, 1 paso de retardo): la FSM a bordo que reporta
            % cada dron. qd2_mission_step la usa para la falla a bordo (alerta 9).
            in.fsm_a_bordo = [ob(1).fsm ob(2).fsm];
        end
    end

    function paso(cmds_fig)
        t = t + dt;
        k = k + 1;
        % ---- comandos programados / automáticos de la estación interna
        cmds = cmds_fig(:).';
        for j = 1:size(cmd_prog, 1)
            if ~cmd_prog_hecho(j) && t >= cmd_prog(j, 1)
                cmd_prog_hecho(j) = true;
                cmds(end+1) = cmd_prog(j, 2); %#ok<AGROW>
            end
        end
        if ~isempty(opc.auto_land_all) && ~land_auto_hecho && ~est.modo_python && p.auto_land < 0.5
            if s.estado == 6 && all(s.ret_paso == 3)
                if t_en_home < 0, t_en_home = t; end
                if t - t_en_home >= opc.auto_land_all
                    cmds(end+1) = 3;
                    land_auto_hecho = true;
                    log_msg(opc, 't = %.1f s: LAND_ALL automático (estación interna)', t);
                end
            else
                t_en_home = -1;
            end
        end
        % ---- estación -> 32 doubles
        [est, py] = sim3d_estacion('paso', est, t, paquetes_pend, cmds, objetivos_det, opc, toc(reloj_hb));
        if est.modo_python && ~opc.use_vision_python && ~aviso_vision_web
            aviso_vision_web = true;
            log_msg(opc, ['t = %.1f s: estación Python conectada con use_vision_python = false. La UI web ' ...
                'actual NO dibuja los objetivos del detector del simulador (solo los de su VisionService); ' ...
                'para una corrida coherente con la UI usa use_vision_python = true y escenario = ''web'' ' ...
                '(README, sección "Con la UI web").'], t);
        end
        paquetes_pend = zeros(32, 0);
        % ---- tierra: misión + guard + limitador
        in = entradas(t, py);
        in_ult = in;
        estado_prev = s.estado;
        [s, spE, spR, tel] = qd2_mission_step(s, in, p);
        est = sim3d_estacion('ack', est, tel.cmd_seq_ack);
        if s.estado ~= estado_prev
            log_msg(opc, 't = %6.2f s: %s -> %s', t, nombre_estado(estado_prev), nombre_estado(s.estado));
        end
        if s.explorer_id == 2
            pE = in.pose2; pR = in.pose1;
        else
            pE = in.pose1; pR = in.pose2;
        end
        ga = 0;
        if version == 1
            [spE, spR, ga, gs] = qd2_separation_guard(pE, pR, spE, spR, p, gs, tel.ctx_guard);
        end
        if s.explorer_id == 2
            sp = [spR; spE];
        else
            sp = [spE; spR];
        end
        for i = 1:2
            sp_rl(i, :) = qd2_rate_limit(sp_rl(i, :), sp(i, :), rate, dt, 1);
        end
        % ---- a bordo + planta + downwash
        F_dw = [0 0];
        for i = 1:2
            o = 3 - i;
            xe = [X(1:3, i) + opc.ruido_pos*randn(rs, 3, 1); X(4:6, i) + 0.01*randn(rs, 3, 1); X(7:9, i)];
            other = [X(1:3, o); X(4:6, o)];
            [u, ob(i)] = qd2_onboard_step(ob(i), tel.arm(i), tel.takeoff(i), sp_rl(i, :), xe, other, p, dt, version);
            fp(i) = flight_plan(fp(i), tel.arm(i), tel.takeoff(i));
            [Fdw, sg] = qd2_downwash(X(1:3, i), X(1:3, o), p.dw);
            X(:, i) = qd2_quad_plant(X(:, i), u, Fdw + sg.*randn(rs, 3, 1), dt, p.planta);
            F_dw(i) = Fdw(3);
            if ob(i).fsm >= 3 && ob(i).fsm <= 6
                bat(i) = max(bat(i) - dt*(0.0022 + 0.00004*X(10, i)), 13.2);
            end
        end
        % ---- detector de la cámara del explorador
        % Mismo filtro que _vision_gate de la estación (web/app/server.py): solo en MISSION o
        % PAUSED, con el explorador rastreado y por encima de web.z_min_deteccion (pose MEDIDA).
        if mod(k, cada_cam) == 0 && s.explorer_id > 0
            ie = s.explorer_id;
            if ie == 1
                pm = in.pose1; trk_e = in.tracked1;
            else
                pm = in.pose2; trk_e = in.tracked2;
            end
            if trk_e > 0.5 && any(s.estado == C.vis.estados) && pm(3) >= C.vis.z_min
                % roll/pitch que recibe la estación en y(61:62) (NaN = y(63) = 0, modelo viejo)
                att_e = [NaN NaN];
                if opc.actitud_en_tel, att_e = in.att(2*ie - 1:2*ie); end
                det = sim3d_detector('cuadro', det, C, esc, X(1:9, ie), [pm(:); att_e(:)], t, opc);
                objetivos_det = sim3d_detector('objetivos', det);
            else
                det.ultimas = zeros(0, 4);
                det.ult_act = [];
            end
        end
        % ---- métricas de separación
        dh = norm(X(1:2, 1) - X(1:2, 2));
        dz = abs(X(3, 1) - X(3, 2));
        if all(X(3, :) > 0.2)
            dh_min = min(dh_min, dh);
            if dh < 0.3, dz_min_apil = min(dz_min_apil, dz); end
        end
        estados_vistos(s.estado + 1) = true;
        % ---- telemetría §3.3
        if mod(k, cada_tel) == 0
            if ~isempty(pt), n_tel = n_tel + 1; end
            y64 = telemetria();
            Lg.tel_att(:, k) = y64(61:63);   % y(61:63) también sin puente (probar_sim3d)
            if ~isempty(pt), pt = sim3d_puente('enviar', pt, y64); end
        end
        % ---- estelas (cada 0.1 s)
        if t - t_estela >= 0.1
            t_estela = t;
            for i = 1:2
                e = estela{i};
                e = [e(2:end, :); X(1:3, i).'];
                if X(3, i) < 0.03, e(end, :) = NaN; end
                estela{i} = e;
            end
        end
        % ---- registro
        Lg.t(k) = t;
        Lg.pos(:, k, 1) = X(1:3, 1); Lg.pos(:, k, 2) = X(1:3, 2);
        Lg.att(:, k, 1) = X(7:9, 1); Lg.att(:, k, 2) = X(7:9, 2);
        Lg.sp(:, k, 1) = sp_rl(1, :).'; Lg.sp(:, k, 2) = sp_rl(2, :).';
        Lg.estado(k) = s.estado; Lg.guard(k) = ga; Lg.dh(k) = dh; Lg.dz(k) = X(3, 1) - X(3, 2);
        Lg.arm(:, k) = tel.arm(:); Lg.takeoff(:, k) = tel.takeoff(:);
        Lg.fsm(:, k) = [ob(1).fsm; ob(2).fsm];
        Lg.wp_idx(k) = s.wp_idx; Lg.n_wp(k) = s.n_wp;
        Lg.n_obj(k) = tel.n_targets_known; Lg.n_vis(k) = tel.n_targets_visited;
        Lg.marking(k) = tel.marking_progress; Lg.r_marking(k) = tel.r_marking;
        Lg.explorer_done(k) = tel.explorer_done; Lg.alerta(k) = tel.alerta;
        Lg.hb_ok(k) = tel.hb_ok; Lg.modo_python(k) = est.modo_python;
        Lg.explorer_id(k) = s.explorer_id;
        Lg.t_mark_acum(k) = s.t_marca;
        Lg.t_en_estado(k) = t - s.t_entrada;
        Lg.t_dwell(k) = s.t_dwell;
        Lg.F_dw(:, k) = F_dw(:);
        Lg.ack(k) = tel.cmd_seq_ack;
        Lg.det_cuadros(k) = det.n_cuadros;
    end

    function y = telemetria()
        % §3.3: o+1 tracked y o+3..o+6 = pose MEDIDA (la de OptiTrack que usa la misión):
        % con pérdida de rastreo va tracked = 0 y la pose en NaN (la estación la descarta).
        pose1 = in_ult.pose1; pose2 = in_ult.pose2;
        % o+2 según la versión (ARQUITECTURA §3.3): PID = Flight State de TIERRA (0 Idle, 1 Armed,
        % 2 Takeoff, 3 Land, 9 MS Stop), porque el binario PID no reporta su FSM; MPC = FSM a bordo (1..9).
        if version == 1
            fsm_tel = fp;
        else
            fsm_tel = [ob(1).fsm ob(2).fsm];
        end
        aux = [in_ult.tracked1 in_ult.tracked2 fsm_tel(1) fsm_tel(2) bat(1) bat(2) 1 1];
        % 61-63: roll, pitch y válido del explorador (actitud real de la planta, la misma
        % convención R = Rz(yaw)Ry(pitch)Rx(roll) que OptiTrack); 0 0 0 sin rastreo.
        % opc.actitud_en_tel = false: como un llamador viejo (sin att), y(61:63) = 0
        if opc.actitud_en_tel
            y = qd2_telemetry_pack(t, tel, version, ga, pose1, pose2, sp_rl(1, :), sp_rl(2, :), ...
                X(4:6, 1), X(4:6, 2), aux, in_ult.att);
        else
            y = qd2_telemetry_pack(t, tel, version, ga, pose1, pose2, sp_rl(1, :), sp_rl(2, :), ...
                X(4:6, 1), X(4:6, 2), aux);
        end
        if est.py_visto
            y(54) = est.ack_py;      % ack en la numeración de cmd_seq de Python
        end
        % 56-60 (contrato §3.3): un objetivo de la tabla por mensaje, rotando; la estación los
        % une con los suyos (así la UI ve también los del detector del simulador):
        % [id clase x y estado(1 en cola, 2 asignado, 3 visitado)]
        if s.n_obj > 0
            j = mod(n_tel - 1, s.n_obj) + 1;      % contador de mensajes ENVIADOS (no de pasos)
            y(56:60) = [s.obj_id(j); s.obj_cls(j); s.obj_x(j); s.obj_y(j); s.obj_estado(j)];
        end
    end

    function r = fin_por_done()
        r = false;
        if opc.parar_en_done && s.estado == 8
            r = (t - s.t_entrada) >= opc.t_extra_done;
        end
    end

    function dibujar()
        E.t = t; E.vel = vel; E.tiempo_real = opc.tiempo_real;
        E.factor_real = t/max(toc(reloj), 1e-6);
        E.X = X; E.sp = sp_rl; E.fsm = [ob(1).fsm ob(2).fsm]; E.bat = bat;
        E.s = s; E.tel = tel; E.ga = ga; E.p = p; E.esc = esc; E.version = version;
        E.dh = norm(X(1:2, 1) - X(1:2, 2)); E.dz = abs(X(3, 1) - X(3, 2));
        E.dh_min = dh_min;
        E.estela = estela;
        ie = max(s.explorer_id, 1);
        % lo que ve la cámara REAL: actitud de la planta, pivote de Motive y error del soporte
        E.huella = sim3d_camara('huella', C.cam, X([1:3 9 7 8], ie), R_montaje);
        R_wb = rot_zyx(X(9, ie), X(8, ie), X(7, ie));
        piv = [0; 0; C.cam.pivote];           % la malla gira alrededor del pivote (sim3d_escena)
        E.lente = (X(1:3, ie) + piv + R_wb*(G.malla.lente(:) - piv)).';
        E.obs_ult = det.ultimas;
        [E.estado_real, E.err_obj] = cruzar_objetivos();
        E.n_pend = sum(det.cl(:, 6) == 0);
        E.n_falsos = det.n_falsos;
        E.txt_vision = sprintf('%d cuadros, %d observaciones (radio %.2f m, mín. %d obs.)', ...
            det.n_cuadros, det.n_obs, det.radio, det.min_obs);
        % actitud con la que geolocalizó el último cuadro (como la tarjeta de Visión de la UI)
        E.txt_actitud = '';
        if isfield(det, 'ult_act') && ~isempty(det.ult_act)
            a = det.ult_act;
            if a.inclinado
                E.txt_actitud = sprintf('INCLINADO %.1f° > %g°: %d descartadas', rad2deg(a.incl), ...
                    C.vis.incl_max_deg, det.n_desc_incl);
            elseif a.ok
                E.txt_actitud = sprintf('roll %.1f° pitch %.1f° (incl. %.1f°, máx. %.1f°)', rad2deg(a.roll), ...
                    rad2deg(a.pitch), rad2deg(a.incl), rad2deg(det.incl_max));
            else
                E.txt_actitud = 'nivelada (sin actitud o usar_actitud = false)';
            end
        end
        if opc.use_vision_python
            E.txt_vision = [E.txt_vision ' · objetivos de PYTHON'];
        end
        if est.modo_python
            E.txt_estacion = sprintf('PYTHON (hb %s, %d paquetes)', ternario(tel.hb_ok > 0.5, 'ok', 'PERDIDO'), est.n_py);
            if ~opc.use_vision_python
                E.txt_estacion = [E.txt_estacion ' · la UI no ve los objetivos del sim'];
            end
        else
            E.txt_estacion = sprintf('interna%s', ternario(est.py_visto, ' (Python se desconectó)', ''));
        end
        [~, E.txt_tcp] = sim3d_puente('estado', pt);
        G = sim3d_escena('actualizar', G, E);
    end

    function [ereal, err] = cruzar_objetivos()
        % estado de cada objeto real según la tabla de la misión, y error de cada objetivo
        nobj = numel(esc.obj);
        ereal = zeros(1, nobj);
        err = cell(1, max(s.n_obj, 1));
        ox = [esc.obj.x]; oy = [esc.obj.y];
        for kk = 1:s.n_obj
            d = hypot(ox - s.obj_x(kk), oy - s.obj_y(kk));
            [dm, j] = min(d);
            if ~isempty(dm) && dm < 0.35
                ereal(j) = max(ereal(j), s.obj_estado(kk));
                err{kk} = sprintf('%.2f m', dm);
            else
                err{kk} = 'falso';
            end
        end
        % detectado por el detector pero aún no en la misión: estado 1
        % (con use_vision_python el detector interno no publica nada)
        T = objetivos_det;
        if opc.use_vision_python, T = zeros(0, 5); end
        for kk = 1:size(T, 1)
            d = hypot(ox - T(kk, 3), oy - T(kk, 4));
            [dm, j] = min(d);
            if ~isempty(dm) && dm < 0.35 && ereal(j) == 0
                ereal(j) = 1;
            end
        end
    end

    function revisar_capturas()
        hacer = false;
        etiqueta = '';
        while ~isempty(capturas_pend) && t >= capturas_pend(1) - 1e-9
            if vistas_cap(1) > 0
                G = sim3d_escena('vista', G, vistas_cap(1));
                etiqueta = sprintf('t%05.1f_vista%d', t, vistas_cap(1));
            else
                etiqueta = sprintf('t%05.1f', t);
            end
            capturas_pend(1) = [];
            vistas_cap(1) = [];
            hacer = true;
        end
        if ~isempty(opc.capturas_estados) && any(opc.capturas_estados == s.estado) && ~any(estados_cap == s.estado) ...
                && (t - s.t_entrada) >= 1.0
            estados_cap(end+1) = s.estado;
            hacer = true;
            etiqueta = sprintf('t%05.1f_%s', t, nombre_estado(s.estado));
        end
        if hacer
            dibujar();
            drawnow;
            if ~isfolder(carpeta_cap), mkdir(carpeta_cap); end
            f = fullfile(carpeta_cap, sprintf('%s_%s_%s.png', opc.prefijo_capturas, ...
                ternario(version == 1, 'pid', 'mpc'), etiqueta));
            try
                exportgraphics(G.fig, f, 'Resolution', 110, 'BackgroundColor', 'current');
            catch
                print(G.fig, f, '-dpng', '-r110');
            end
            archivos_cap{end+1} = f;
            log_msg(opc, 'Captura: %s', f);
        end
    end

    function M = metricas()
        M.t_sim = t;
        M.estado_final = s.estado;
        M.estados_vistos = find(estados_vistos) - 1;
        M.dh_min_aire = dh_min;
        M.dz_min_apilados = dz_min_apil;
        M.n_objetivos = s.n_obj;
        M.n_visitados = tel.n_targets_visited;
        M.n_reales = numel(esc.obj);
        [er, ~] = cruzar_objetivos();
        M.reales_detectados = sum(er > 0);
        M.reales_visitados = sum(er == 3);
        e = [];
        ox = [esc.obj.x]; oy = [esc.obj.y];
        for kk = 1:s.n_obj
            d = min(hypot(ox - s.obj_x(kk), oy - s.obj_y(kk)));
            if d < 0.35, e(end+1) = d; end %#ok<AGROW>
        end
        M.err_pos = e;
        M.n_falsos_publicados = s.n_obj - numel(e);
        M.det_cuadros = det.n_cuadros;
        M.det_obs = det.n_obs;
        M.det_con_actitud = det.n_con_actitud;      % cuadros geolocalizados con roll/pitch
        M.det_nivelado = det.n_nivelado;            % cuadros geolocalizados nivelados
        M.det_desc_incl = det.n_desc_incl;          % observaciones descartadas por inclinación
        M.det_incl_max_deg = rad2deg(det.incl_max); % inclinación máxima usada al geolocalizar
        M.err_obs = det.err_obs;                    % m, error de cada observación real que entró al cluster
        if ~isempty(pt)
            M.tcp_rx = pt.n_rx; M.tcp_tx = pt.n_tx; M.tcp_conex = pt.n_conex;
        end
        M.paquetes_python = est.n_py;
        M.comandos_python = est.n_cmd_py;
    end

    function cerrar_puente()
        if ~isempty(pt)
            pt = sim3d_puente('cerrar', pt);
        end
    end
end

% =========================================================================
function log_msg(opc, varargin)
if opc.verbose
    fprintf('[sim3d] %s\n', sprintf(varargin{:}));
end
end

function n = nombre_estado(e)
nombres = {'INIT', 'WAIT_TRACKING', 'ASSIGN_ROLES', 'TAKEOFF', 'HOVER_READY', 'MISSION', ...
    'RETURN_HOME', 'LANDING', 'DONE', 'ABORT', 'PAUSED'};
n = nombres{min(max(e, 0), 10) + 1};
end

function st = flight_plan(st, arm, takeoff)
%FLIGHT_PLAN Flight Plan de TIERRA del Equipo2 (QD2_MissionCtrl_NoJoystick_2021a, "Subsystem"):
% 0 Idle, 1 Armed, 2 Takeoff, 3 Land, 9 MS Stop (absorbente). Mismas transiciones que el
% original, con E-stop = 0, joystick_issue = 0 y sin pérdida de enlace (el simulador no la emula).
switch st
    case 0
        if arm > 0.5
            st = 1;
        elseif takeoff > 0.5
            st = 9;
        end
    case 1
        if arm < 0.5
            st = 0;
        elseif takeoff > 0.5
            st = 2;
        end
    case 2
        if takeoff < 0.5
            st = 3;
        elseif arm < 0.5
            st = 9;
        end
    case 3
        if arm < 0.5
            st = 9;
        end
end
end

function r = ternario(c, a, b)
if c, r = a; else, r = b; end
end

function R = rot_zyx(yaw, pitch, roll)
cy = cos(yaw); sy = sin(yaw); cp = cos(pitch); sp = sin(pitch); cr = cos(roll); sr = sin(roll);
R = [cy -sy 0; sy cy 0; 0 0 1]*[cp 0 sp; 0 1 0; -sp 0 cp]*[1 0 0; 0 cr -sr; 0 sr cr];
end
