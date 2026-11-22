function [s, sp_E, sp_R, tel] = qd2_mission_step(s, in, p)
%QD2_MISSION_STEP Supervisor de misión de 2 QDrone 2 (FSM de ARQUITECTURA §3.3).
%   [s, sp_E, sp_R, tel] = qd2_mission_step(s, in, p)
%
%   s   : estado (struct de tamaño fijo) de qd2_mission_init(p).
%   in  : struct de entradas
%         .t        tiempo (s), monótono. La FSM usa diferencias de tiempo,
%                   así que funciona a cualquier tasa (1 kHz, 100 Hz, 50 Hz).
%         .pose1    [x y z yaw] medidos del dron 1 (RSF, z ya corregido)
%         .pose2    [x y z yaw] medidos del dron 2
%         .tracked1 1 si OptiTrack rastrea al dron 1
%         .tracked2 1 si OptiTrack rastrea al dron 2
%         .py       32x1 vector de comandos/detecciones de Python (§3.3)
%         .version  1 = PID, 2 = MPC
%         Opcionales (versión MPC; si faltan = sin información a bordo, el
%         comportamiento es el de antes):
%         .fsm_a_bordo [1x2] FSM a bordo de cada DRON (índice 2 de la
%                   respuesta de 8 doubles, validada con qd2_enlace_mpc);
%                   0 o NaN = desconocida.
%         .proto_mal   [1x2] código del enlace por dron (qd2_enlace_mpc):
%                   0 bien; 1 = el dron responde con un protocolo MPC
%                   incorrecto (sin etiqueta y(8) = 2 o FSM fuera de 1..9)
%                   desde hace > p.t_proto_mal s estando conectado;
%                   2 = enlace perdido después de haber estado conectado.
%   p   : qd2_params()
%
%   Falla a bordo (solo con las entradas opcionales): un dron armado que
%   reporta FSM 7 u 8, o 9 con takeoff = 1 (o en TAKEOFF armado), o que
%   pierde el enlace con takeoff = 1 (a bordo pasa a 7 a los 0.5 s), o con
%   protocolo incorrecto sostenido, es un dron EN FALLA (alerta 9; 10 si
%   solo es el protocolo). En vuelo (también en LANDING mientras R espera
%   a que E se aparte, ver paso_aterrizaje) el
%   dron sano se aparta primero en horizontal >= d_min_h A SU ALTURA (con
%   límite de velocidad p.falla_rate_xy, tel.escape), luego regresa a su
%   home (o aterriza donde quedó si su home está bajo el dron en falla) y
%   aterriza; la misión termina en DONE. Al dron en falla nunca se le vuelve
%   a dar takeoff (solo puede bajar). Antes de START, un dron con
%   protocolo incorrecto bloquea START (alerta 10). Un dron DESARMADO con
%   FSM 7/8/9 no es alerta (ARQUITECTURA §3.2: un 9 tras aterrizar es el fin
%   normal); si se da START, en TAKEOFF ya armado se detecta, se cancela el
%   despegue sin dar takeoff y queda la alerta 9 mientras siga en 7/8/9.
%
%   sp_E, sp_R : 1x4 [x y z yaw] consignas por ROL (antes del guard).
%   tel        : struct de salida (ver al final del archivo) con los campos
%                de telemetría §3.3, disparos por DRON (arm, takeoff) y las
%                consignas por DRON (sp1, sp2), más el contexto del guard.
%
%   Estados: 0 INIT, 1 WAIT_TRACKING, 2 ASSIGN_ROLES, 3 TAKEOFF,
%   4 HOVER_READY, 5 MISSION, 6 RETURN_HOME, 7 LANDING, 8 DONE, 9 ABORT,
%   10 PAUSED.
%   Comandos (py(2), se aplican cuando cambia py(3) = cmd_seq):
%   1 START (en ASSIGN_ROLES), 2 ABORT (regresar a home y hover),
%   3 LAND_ALL, 4 PAUSE, 5 RESUME.
%#codegen

% ======================= Entradas y parámetros efectivos ==================
t = in.t;
if s.primera > 0.5
    s.t_prev = t;
    s.t_entrada = t;
    s.primera = 0;
end
dt = max(t - s.t_prev, 0);
s.t_prev = t;

py = zeros(32,1);
py(:) = in.py(1:32);
py(~isfinite(py)) = 0;
version = in.version;

pose1 = [in.pose1(1) in.pose1(2) in.pose1(3) in.pose1(4)];
pose2 = [in.pose2(1) in.pose2(2) in.pose2(3) in.pose2(4)];
trk1 = in.tracked1 > 0.5;
trk2 = in.tracked2 > 0.5;
% Rastreo válido = OptiTrack lo rastrea Y la pose es finita. Si la pose no
% es finita se usa la última pose finita (nunca entra NaN a la FSM).
fin1 = all(isfinite(pose1));
fin2 = all(isfinite(pose2));
val1 = trk1 && fin1;
val2 = trk2 && fin2;
if fin1, s.pose_ok1 = pose1; else, pose1 = s.pose_ok1; end
if fin2, s.pose_ok2 = pose2; else, pose2 = s.pose_ok2; end
if val1 || s.estado <= 2, s.t_trk(1) = t; end
if val2 || s.estado <= 2, s.t_trk(2) = t; end
perdido = (t - s.t_trk(1) > p.t_perdida_trk) || (t - s.t_trk(2) > p.t_perdida_trk);

% Overrides de la estación (0 = valor por defecto). Solo se leen antes de
% START (estados 0-2); después quedan fijos durante todo el vuelo.
% Acotados: d_min_h solo puede AUMENTAR (hasta d_min_h_max), t_mark en
% t_mark_lim, alturas en [0.3, z_max] y, en la versión MPC,
% z_R <= z_max - dz_min y z_E >= z_R + dz_min.
if s.estado <= 2
    s.z_E = p.z_E;
    if py(20) > 0, s.z_E = min(max(py(20), 0.3), p.z_max); end
    s.z_R = p.z_R;
    if py(21) > 0, s.z_R = min(max(py(21), 0.3), p.z_max); end
    s.t_mark = p.t_mark;
    if py(22) > 0, s.t_mark = min(max(py(22), p.t_mark_lim(1)), p.t_mark_lim(2)); end
    s.d_min_h = p.d_min_h;
    if py(23) > 0, s.d_min_h = min(max(py(23), p.d_min_h), max(p.d_min_h_max, p.d_min_h)); end
    if version > 1.5
        % Versión MPC: garantizar la separación vertical para apilarse.
        s.z_R = min(s.z_R, p.z_max - p.dz_min);
        if (s.z_E - s.z_R) < p.dz_min
            s.z_E = min(s.z_R + p.dz_min, p.z_max);
        end
    end
end
Dm = s.d_min_h + p.guard_margen;     % separación de planeación

% poses por rol
if s.explorer_id == 2
    pE = pose2; pR = pose1; hE = s.home2; hR = s.home1;
else
    pE = pose1; pR = pose2; hE = s.home1; hR = s.home2;
end

% ======================= Heartbeat ========================================
if py(1) ~= s.hb_last
    s.hb_last = py(1);
    s.t_hb = t;
    if s.hb_visto < 0.5
        % primer contacto: no re-ejecutar un comando viejo de la estación
        s.hb_visto = 1;
        s.cmd_seq_ack = py(3);
    end
end
hb_ok = (s.hb_visto > 0.5) && ((t - s.t_hb) <= p.hb_timeout);

% ======================= Tabla de objetivos ===============================
s = actualizar_objetivos(s, py, p);

% ======================= Estado a bordo (opcional, versión MPC) ===========
% Por DRON. fsm_err = FSM a bordo 7 (Error), 8 (E-stop) o 9 (motores
% apagados); proto = protocolo MPC incorrecto sostenido; sin_enl = enlace
% perdido tras haber estado conectado (qd2_enlace_mpc: código 1 / 2).
fsm_ab = [0 0];
proto = [0 0];
sin_enl = [0 0];
if isfield(in, 'fsm_a_bordo')
    for i = 1:2
        v = in.fsm_a_bordo(i);
        if isfinite(v)
            fsm_ab(i) = round(v);
        end
    end
end
if isfield(in, 'proto_mal')
    for i = 1:2
        v = in.proto_mal(i);
        proto(i) = double(v > 0.5 && v < 1.5);
        sin_enl(i) = double(v > 1.5);
    end
end
fsm_err = [0 0];
for i = 1:2
    fsm_err(i) = double(fsm_ab(i) >= 7 && fsm_ab(i) <= 9);
end
% START se rechaza solo por protocolo incorrecto (alerta 10). Un dron con
% FSM 7/8/9 desarmado se detecta en TAKEOFF ya armado (antes de takeoff).
no_disp = (proto(1) + proto(2)) > 0;

% ======================= Comandos =========================================
cmd = 0;
if hb_ok && py(3) ~= s.cmd_seq_ack
    cmd = round(py(2));
    s.cmd_seq_ack = py(3);
end
est = s.estado;
% En el aire: estados de vuelo, o TAKEOFF solo si ya se dio takeoff y
% algún dron ya despegó (z > z_despegado).
despegado = max(pE(3), pR(3)) > p.z_despegado;
aire = (est == 4) || (est == 5) || (est == 6) || (est == 9) || (est == 10) || ...
    (est == 3 && s.takeoff(1) > 0.5 && despegado);
if s.falla_fase > 0.5
    % Reacción a una falla a bordo en curso: solo se atiende LAND_ALL
    % (aterrizar en cuanto el dron sano esté apartado). START, ABORT,
    % PAUSE y RESUME se confirman pero no se ejecutan.
    if cmd == 3
        s.falla_land = 1;
    end
elseif (cmd == 2 || cmd == 3) && est == 3 && ~aire
    % ABORT / LAND_ALL antes de despegar: nunca se da un takeoff nuevo.
    s = cancelar_despegue(s, pE, pR, t);
elseif cmd == 1 && est == 2 && s.roles_ok > 0.5 && s.home_ok > 0.5 && ~no_disp
    s.alerta = 0;
    s.falla = [0 0];
    % START: fijar roles/home, generar la N y despegar
    [wp, nwp] = qd2_track_waypoints(p.vertices, p.wp_spacing, s.z_E, p.n_vertices);
    if s.explorer_id == 2
        hE = s.home2; hR = s.home1; pE = pose2; pR = pose1;
    else
        hE = s.home1; hR = s.home2; pE = pose1; pR = pose2;
    end
    if nwp > 1 && norm(wp(nwp,1:2) - hE) < norm(wp(1,1:2) - hE)
        wp2 = wp;
        for k = 1:nwp
            wp2(k,:) = wp(nwp + 1 - k, :);
        end
        wp = wp2;
    end
    s.wp = wp;
    s.n_wp = nwp;
    s.wp_idx = 1;
    s.t_dwell = 0;
    s.explorer_done = 0;
    s.r_fase = 0;
    s.r_obj = 0;
    s.r_idle_xy = hR;
    s = entrar(s, 3, t);
elseif cmd == 2 && aire
    s = planear_regreso(s, pE, pR, hE, hR, Dm, p);
    s = entrar(s, 9, t);
elseif cmd == 3 && aire
    s = planear_aterrizaje(s, pE, pR, hE, Dm, p, t);
    s = entrar(s, 7, t);
elseif cmd == 4 && (est == 5 || est == 6)
    s.estado_prev = est;
    s = pausar(s, pE, pR);
    s.pausa_por_hb = 0;
    s = entrar(s, 10, t);
elseif cmd == 5 && est == 10 && ~perdido
    % RESUME (solo con ambos drones rastreados)
    s.pausa_por_hb = 0;
    s.pausa_por_trk = 0;
    if s.alerta == 2, s.alerta = 0; end
    if s.estado_prev == 6 || s.estado_prev == 9
        s = planear_regreso(s, pE, pR, hE, hR, Dm, p);
    end
    if s.estado_prev == 3
        s.t_despegue = t;            % reinicia el timeout de despegue
    end
    s = entrar(s, s.estado_prev, t);
end

% ======================= Falla a bordo ====================================
% Un dron armado que reporta FSM 7/8 (o 9 con takeoff = 1, o cualquiera de
% las tres en TAKEOFF antes de dar takeoff), que pierde el enlace debiendo
% volar (a bordo: FSM 7 a los 0.5 s), o con protocolo MPC incorrecto
% sostenido, está EN FALLA (se recuerda por dron en s.falla).
if s.falla_fase < 0.5
    est = s.estado;
    nueva = [0 0];
    for i = 1:2
        r = rol_de(s, i);
        armado_i = s.arm(r) > 0.5;
        takeoff_i = s.takeoff(r) > 0.5;
        if armado_i && (fsm_ab(i) == 7 || fsm_ab(i) == 8)
            nueva(i) = 9;
        elseif fsm_ab(i) == 9 && (takeoff_i || (est == 3 && armado_i))
            nueva(i) = 9;
        elseif sin_enl(i) > 0.5 && (takeoff_i || (est == 3 && armado_i))
            nueva(i) = 9;
        elseif armado_i && proto(i) > 0.5
            nueva(i) = 10;
        end
    end
    if nueva(1) + nueva(2) > 0
        despegado = max(pE(3), pR(3)) > p.z_despegado;
        en_aire = (est == 4) || (est == 5) || (est == 6) || (est == 9) || (est == 10) || ...
            (est == 3 && s.takeoff(1) > 0.5 && despegado);
        if est == 3 && ~en_aire
            % nadie en el aire: cancelar el despegue (nunca un takeoff nuevo)
            s = cancelar_despegue(s, pE, pR, t);
            s = marcar_falla(s, nueva);
        elseif en_aire || (est == 7 && s.land_paso == 1 && s.land_r_baja < 0.5)
            % en vuelo, o en LANDING con R todavía esperando a su altura a
            % que E se aparte (p. ej. LAND_ALL dado con los drones apilados
            % en la versión MPC): reacción completa (el sano se aparta)
            s = iniciar_falla(s, nueva, pose1, pose2, Dm, p, t);
        elseif est == 7 && s.land_paso ~= 4
            % LANDING con R ya bajando: E ya está a >= d_min_h en horizontal
            % del punto de R (condición para que R baje). Se avisa y al dron
            % en falla se le baja takeoff (paso_aterrizaje).
            s = marcar_falla(s, nueva);
        end
    end
end

% ======================= Pérdida de rastreo ===============================
% En el aire, si algún dron lleva > t_perdida_trk sin rastreo válido, los
% dos quedan en hover en su sitio (PAUSED). El dron perdido entra a
% emergencia a bordo; el otro no se mueve hacia una pose fantasma. Se sale
% con RESUME (cuando vuelva el rastreo), ABORT o LAND_ALL. Durante la
% reacción a una falla a bordo no se pausa (la reacción ya es el modo seguro).
if perdido && s.falla_fase < 0.5
    est = s.estado;
    if est == 3 && s.takeoff(1) < 0.5
        s.arm = [0 0]; s.takeoff = [0 0];
        s.t_ok_desde = -1;
        s = entrar(s, 1, t);
    elseif est == 3 || est == 4 || est == 5 || est == 6 || est == 9
        s.estado_prev = est;
        s = pausar(s, pE, pR);
        s.pausa_por_hb = 0;
        s.pausa_por_trk = 1;
        s.alerta = 2;
        s = entrar(s, 10, t);
    end
end

% ======================= Máquina de estados ===============================
sp_E = s.sp_E;
sp_R = s.sp_R;
marking_progress = 0;
r_marking = 0;
prioridad_R = 0;
solo_respaldo = 1;
escape = [0 0];   % por DRON: 1 = se aparta del dron en falla (límite p.falla_rate_xy)

if s.falla_fase > 0.5
    % Reacción a una falla a bordo (estado 9 mientras el sano se aparta y
    % regresa; 7 mientras aterrizan; luego 8 DONE).
    [s, sp_E, sp_R, escape] = paso_falla(s, pose1, pose2, t, p, Dm);
else
switch s.estado
    case 0  % INIT
        s.arm = [0 0]; s.takeoff = [0 0];
        sp_E = [pE(1) pE(2) 0 0];
        sp_R = [pR(1) pR(2) 0 0];
        if t - s.t_entrada >= p.t_init
            s = entrar(s, 1, t);
        end

    case 1  % WAIT_TRACKING
        s.arm = [0 0]; s.takeoff = [0 0];
        sp_E = [pE(1) pE(2) 0 0];
        sp_R = [pR(1) pR(2) 0 0];
        if val1 && val2
            if s.t_ok_desde < 0
                s.t_ok_desde = t;
            end
            if t - s.t_ok_desde >= p.t_tracking_estable
                s = entrar(s, 2, t);
            end
        else
            s.t_ok_desde = -1;
        end

    case 2  % ASSIGN_ROLES (se reevalúa hasta START)
        s.arm = [0 0]; s.takeoff = [0 0];
        [eid, rid, h1, h2, ok] = qd2_assign_roles(pose1, pose2, double(val1), double(val2));
        if ok > 0.5
            s.explorer_id = eid;
            s.responder_id = rid;
            s.home1 = h1;
            s.home2 = h2;
            s.roles_ok = 1;
            % Separación de los home: con los drones demasiado cerca se
            % rechaza START (alerta 1) porque despegarían uno junto al otro.
            if version > 1.5
                d_home_min = p.d_home_min_mpc;
            else
                d_home_min = Dm;
            end
            if norm(h1 - h2) >= d_home_min
                s.home_ok = 1;
                if s.alerta == 1, s.alerta = 0; end
            else
                s.home_ok = 0;
                s.alerta = 1;
            end
        else
            s.roles_ok = 0;
            s.home_ok = 0;
            s.t_ok_desde = -1;
            s = entrar(s, 1, t);
        end
        if s.explorer_id == 2
            pE = pose2; pR = pose1;
        else
            pE = pose1; pR = pose2;
        end
        sp_E = [pE(1) pE(2) 0 0];
        sp_R = [pR(1) pR(2) 0 0];

    case 3  % TAKEOFF
        s.arm = [1 1];
        % takeoff se da t_arm_a_despegue después de armar y queda fijo
        % (un RESUME desde PAUSED no lo vuelve a bajar)
        if s.takeoff(1) < 0.5 && t - s.t_entrada >= p.t_arm_a_despegue
            s.takeoff = [1 1];
            s.t_despegue = t;
        end
        sp_E = [hE(1) hE(2) s.z_E 0];
        sp_R = [hR(1) hR(2) s.z_R 0];
        if s.takeoff(1) > 0.5 && abs(pE(3) - s.z_E) <= p.tol_z && abs(pR(3) - s.z_R) <= p.tol_z
            s = entrar(s, 4, t);
        elseif s.takeoff(1) > 0.5 && (t - s.t_despegue) >= p.t_takeoff_max
            % no llegaron a la altura: si alguno despegó, ABORT (hover en
            % home hasta LAND_ALL); si nadie despegó, cancelar en tierra
            s.alerta = 3;
            if max(pE(3), pR(3)) > p.z_despegado
                s = planear_regreso(s, pE, pR, hE, hR, Dm, p);
                s = entrar(s, 9, t);
            else
                s = cancelar_despegue(s, pE, pR, t);
                s.alerta = 3;
            end
        end

    case 4  % HOVER_READY
        s.arm = [1 1]; s.takeoff = [1 1];
        sp_E = [hE(1) hE(2) s.z_E 0];
        sp_R = [hR(1) hR(2) s.z_R 0];
        if t - s.t_entrada >= p.t_hover_ready
            s = entrar(s, 5, t);
        end

    case 5  % MISSION
        s.arm = [1 1]; s.takeoff = [1 1];
        solo_respaldo = 0;
        % ---- Explorador: recorre la N densificada (tolerancia + dwell)
        if s.explorer_done < 0.5
            k = min(max(s.wp_idx, 1), max(s.n_wp, 1));
            w = s.wp(k, :);
            sp_E = [w(1) w(2) s.z_E 0];
            dentro = norm(pE(1:2) - w(1:2)) <= p.tol_xy && abs(pE(3) - s.z_E) <= p.tol_z;
            if dentro
                s.t_dwell = s.t_dwell + dt;
            else
                s.t_dwell = 0;
            end
            if s.t_dwell >= p.t_dwell_wp
                s.t_dwell = 0;
                if s.wp_idx < s.n_wp
                    s.wp_idx = s.wp_idx + 1;
                else
                    s.explorer_done = 1;
                    s.t_e_done = t;
                end
            end
        else
            % terminó la N: regresa a su home (a su altura) y cede prioridad
            sp_E = [hE(1) hE(2) s.z_E 0];
            prioridad_R = 1;
        end
        % ---- Respondedor: cola de objetivos, el más cercano primero
        if s.r_fase == 0
            kbest = 0;
            dbest = inf;
            for k = 1:32
                if k <= s.n_obj && s.obj_estado(k) == 1
                    d = norm([s.obj_x(k) s.obj_y(k)] - pR(1:2));
                    if d < dbest
                        dbest = d;
                        kbest = k;
                    end
                end
            end
            if kbest > 0
                s.r_obj = kbest;
                s.obj_estado(kbest) = 2;
                s.r_fase = 1;
                s.t_marca = 0;
            end
        end
        if s.r_fase >= 1 && s.r_obj >= 1 && s.r_obj <= 32
            k = s.r_obj;
            tg = recortar_xy([s.obj_x(k) s.obj_y(k)], p);
            sp_R = [tg(1) tg(2) s.z_R 0];
            exy = norm(pR(1:2) - tg);
            ez = abs(pR(3) - s.z_R);
            if s.r_fase == 1
                if exy <= p.tol_xy && ez <= p.tol_z
                    s.r_fase = 2;
                end
            end
            if s.r_fase == 2
                r_marking = 1;
                if exy <= p.tol_mark_factor*p.tol_xy && ez <= p.tol_mark_factor*p.tol_z
                    s.t_marca = s.t_marca + dt;
                end
                marking_progress = min(s.t_marca/max(s.t_mark, 1e-6), 1);
                if s.t_marca >= s.t_mark
                    s.obj_estado(k) = 3;
                    s.r_fase = 0;
                    s.r_obj = 0;
                    s.r_idle_xy = tg;
                    r_marking = 0;
                    marking_progress = 1;
                end
            end
        else
            s.r_fase = 0;
            s.r_obj = 0;
            sp_R = [s.r_idle_xy(1) s.r_idle_xy(2) s.z_R 0];
        end
        % ---- Fin de misión
        cola = 0;
        for k = 1:32
            if k <= s.n_obj && s.obj_estado(k) == 1
                cola = cola + 1;
            end
        end
        if s.explorer_done > 0.5 && (t - s.t_e_done) >= p.t_fin_gracia && cola == 0 && s.r_fase == 0
            s = planear_regreso(s, pE, pR, hE, hR, Dm, p);
            s = entrar(s, 6, t);
        elseif ~hb_ok
            s.estado_prev = 5;
            s = pausar(s, pE, pR);
            s.pausa_por_hb = 1;
            s = entrar(s, 10, t);
            sp_E = s.hold_E;
            sp_R = s.hold_R;
        end

    case {6, 9}  % RETURN_HOME / ABORT
        s.arm = [1 1]; s.takeoff = [1 1];
        [s, sp_E, sp_R, listos] = paso_regreso(s, pE, pR, hE, hR, t, p);
        if listos && s.estado == 6 && p.auto_land > 0.5
            s = planear_aterrizaje(s, pE, pR, hE, Dm, p, t);
            s = entrar(s, 7, t);
        end

    case 7  % LANDING (R primero, luego E)
        s.arm = [1 1];
        [s, sp_E, sp_R] = paso_aterrizaje(s, pE, pR, t, p);
        if s.land_paso == 3
            s = entrar(s, 8, t);
        end

    case 8  % DONE
        s.arm = [0 0]; s.takeoff = [0 0];
        sp_E = [s.land_xy(1,1) s.land_xy(1,2) 0 0];
        sp_R = [s.land_xy(2,1) s.land_xy(2,2) 0 0];

    case 10 % PAUSED: ambos en hover en su sitio
        s.arm = [1 1]; s.takeoff = [1 1];
        sp_E = s.hold_E;
        sp_R = s.hold_R;
        if s.estado_prev == 5 && s.r_fase == 2
            r_marking = 1;
            marking_progress = min(s.t_marca/max(s.t_mark, 1e-6), 1);
        end

    otherwise
        s = entrar(s, 0, t);
end
end

% Antes de START un dron con protocolo MPC incorrecto no está disponible:
% START se rechaza y se avisa (10). Un dron desarmado con FSM 7/8/9 no es
% alerta (ARQUITECTURA §3.2); la alerta 9 que dejó un despegue cancelado
% (FSM 7/8/9 en TAKEOFF armado) se mantiene mientras ese dron siga en 7/8/9.
if s.estado <= 2
    if proto(1) + proto(2) > 0
        s.alerta = 10;
    elseif s.alerta == 10
        s.alerta = 0;
    elseif s.alerta == 9 && fsm_err(1) + fsm_err(2) == 0
        s.alerta = 0;
    end
end

% recorte a la arena y techo (sin NaN: se conserva la consigna anterior)
sp_E = recortar_sp(sp_E, s.sp_E, p);
sp_R = recortar_sp(sp_R, s.sp_R, p);
s.sp_E = sp_E;
s.sp_R = sp_R;

% ======================= Telemetría =======================================
n_vis = 0; cola = 0;
for k = 1:32
    if k <= s.n_obj
        if s.obj_estado(k) == 3
            n_vis = n_vis + 1;
        elseif s.obj_estado(k) == 1
            cola = cola + 1;
        end
    end
end
tid = 0;
if s.r_obj >= 1 && s.r_obj <= 32
    tid = s.obj_id(s.r_obj);
end

tel.mission_state = s.estado;
tel.explorer_id = s.explorer_id;
tel.responder_id = s.responder_id;
tel.explorer_wp_idx = s.wp_idx;
tel.explorer_wp_total = s.n_wp;
tel.responder_target_id = tid;
tel.n_targets_known = s.n_obj;
tel.n_targets_visited = n_vis;
tel.explorer_done = s.explorer_done;
tel.visited_mask = mascara_visitados(s);
tel.marking_progress = marking_progress;
tel.queue_length = cola;
tel.cmd_seq_ack = s.cmd_seq_ack;
tel.home1 = s.home1;
tel.home2 = s.home2;
tel.r_marking = r_marking;
tel.hb_ok = double(hb_ok);
tel.pausa_por_hb = s.pausa_por_hb;
tel.z_E = s.z_E;
tel.z_R = s.z_R;
tel.d_min_h = s.d_min_h;
tel.t_mark = s.t_mark;
tel.alerta = s.alerta;             % 0 nada, 1 homes cerca, 2 pérdida de rastreo,
                                   % 3 timeout de despegue, 4 timeout de aterrizaje,
                                   % 9 falla a bordo, 10 protocolo MPC incorrecto
tel.home_ok = s.home_ok;
tel.pausa_por_trk = s.pausa_por_trk;
% Falla a bordo (por DRON): 0 nada, 9 FSM 7/8/9, 10 protocolo MPC incorrecto
tel.falla = s.falla;
tel.falla_fase = s.falla_fase;     % 0 ninguna, 1 apartarse, 2 regresar, 3 aterriza el sano,
                                   % 4 espera al dron en falla en el suelo
tel.escape = escape;               % por DRON: 1 = límite xy p.falla_rate_xy (se aparta)
% y(56:60): un objetivo de la tabla por periodo p.t_rot_obj, rotando:
% [id clase x y estado] (estado 1 pendiente, 2 asignado, 3 visitado)
obj_rot = [0 0 0 0 0];
if s.n_obj > 0
    j = mod(floor(t/p.t_rot_obj + 1e-6), s.n_obj) + 1;
    j = min(max(j, 1), 32);
    obj_rot = [s.obj_id(j) s.obj_cls(j) s.obj_x(j) s.obj_y(j) s.obj_estado(j)];
end
tel.obj_rot = obj_rot;
% contexto para qd2_separation_guard:
% [r_marking, prioridad_R, d_min_h, solo_respaldo, look_x, look_y]
% look = punto de la N p.guard_look_wp waypoints adelante de E (para que
% R no se refugie justo hacia donde va E).
look = [sp_E(1) sp_E(2)];
if s.estado == 5 && s.explorer_done < 0.5 && s.n_wp > 0
    kl = min(s.wp_idx + p.guard_look_wp, s.n_wp);
    look = recortar_xy(s.wp(kl, 1:2), p);
end
tel.ctx_guard = [r_marking, prioridad_R, s.d_min_h, solo_respaldo, look(1), look(2)];
% por dron
if s.explorer_id == 2
    tel.role1 = 2; tel.role2 = 1;
    tel.sp1 = sp_R; tel.sp2 = sp_E;
    tel.arm = [s.arm(2) s.arm(1)];
    tel.takeoff = [s.takeoff(2) s.takeoff(1)];
elseif s.explorer_id == 1
    tel.role1 = 1; tel.role2 = 2;
    tel.sp1 = sp_E; tel.sp2 = sp_R;
    tel.arm = s.arm;
    tel.takeoff = s.takeoff;
else
    tel.role1 = 0; tel.role2 = 0;
    tel.sp1 = sp_E; tel.sp2 = sp_R;
    tel.arm = s.arm;
    tel.takeoff = s.takeoff;
end
tel.estop = [0 0];
end

% =========================================================================
function s = entrar(s, nuevo, t)
s.estado = nuevo;
s.t_entrada = t;
end

function s = pausar(s, pE, pR)
s.hold_E = [pE(1) pE(2) s.sp_E(3) 0];
s.hold_R = [pR(1) pR(2) s.sp_R(3) 0];
if s.sp_E(3) <= 0
    s.hold_E(3) = s.z_E;
end
if s.sp_R(3) <= 0
    s.hold_R(3) = s.z_R;
end
end

function xy = recortar_xy(xy, p)
lo = p.xy_lim(1) + p.xy_margen;
hi = p.xy_lim(2) - p.xy_margen;
xy = min(max(xy, lo), hi);
end

function s = cancelar_despegue(s, pE, pR, t)
% ABORT / LAND_ALL en TAKEOFF sin ningún dron en el aire.
% - takeoff aún no dado: desarmar y volver a ASSIGN_ROLES (se puede dar
%   START otra vez). A bordo: ARMED -> IDLE, sin error.
% - takeoff ya dado (rampa de empuje a bordo): bajar takeoff de los dos a
%   la vez en su sitio (LANDING paso 4), con arm = 1 hasta que estén en el
%   suelo. A bordo: en TAKEOFF_THROTTLE soltar trigger_2 es un Error
%   (rampa de emergencia en tierra, el modelo a bordo termina); en
%   TAKEOFF_HEIGHT pasa a AUTOLAND.
if s.takeoff(1) < 0.5
    s.arm = [0 0];
    s.takeoff = [0 0];
    s = entrar(s, 2, t);
else
    s.takeoff = [0 0];
    s.land_paso = 4;
    s.t_suelo = [-1 -1];
    s.t_land_paso = t;
    s.land_xy = [pE(1) pE(2); pR(1) pR(2)];
    s.land_via_E = [pE(1) pE(2)];
    s.land_usa_via_E = 0;
    s = entrar(s, 7, t);
end
end

function sp = recortar_sp(sp, prev, p)
for i = 1:4
    if ~isfinite(sp(i))
        if isfinite(prev(i))
            sp(i) = prev(i);
        else
            sp(i) = 0;
        end
    end
end
lo = p.xy_lim(1) + p.xy_margen;
hi = p.xy_lim(2) - p.xy_margen;
sp(1) = min(max(sp(1), lo), hi);
sp(2) = min(max(sp(2), lo), hi);
sp(3) = min(max(sp(3), 0), p.z_max);
end

function s = actualizar_objetivos(s, py, p)
% Ranuras 5-19: [id, clase, x, y, confianza] x 3. Se actualiza por id.
for j = 1:3
    o = 4 + 5*(j-1);
    id = round(py(o+1));
    cls = py(o+2);
    x = py(o+3);
    y = py(o+4);
    cf = py(o+5);
    % ids válidos: 1..32 (visited_mask, y(56) de la telemetría)
    if id < 1 || id > 32 || ~isfinite(x) || ~isfinite(y)
        continue;
    end
    % ¿id ya conocido?
    k_id = 0;
    for k = 1:32
        if k <= s.n_obj && s.obj_id(k) == id
            k_id = k;
        end
    end
    if k_id > 0
        % refinar posición mientras no esté visitado ni marcándose
        marcando = (s.r_fase == 2) && (s.r_obj == k_id);
        if s.obj_estado(k_id) <= 2 && ~marcando
            s.obj_cls(k_id) = cls;
            s.obj_x(k_id) = x;
            s.obj_y(k_id) = y;
            s.obj_conf(k_id) = cf;
        end
        continue;
    end
    % ¿id ya registrado como duplicado?
    es_alias = 0;
    for k = 1:32
        if k <= s.n_alias && s.alias_id(k) == id
            es_alias = 1;
        end
    end
    if es_alias
        continue;
    end
    % ¿cerca de un objetivo existente? -> duplicado
    k_cerca = 0;
    dmin = inf;
    for k = 1:32
        if k <= s.n_obj
            d = norm([s.obj_x(k) s.obj_y(k)] - [x y]);
            if d < dmin
                dmin = d;
                k_cerca = k;
            end
        end
    end
    if k_cerca > 0 && dmin < p.radio_cluster
        if s.n_alias < 32
            s.n_alias = s.n_alias + 1;
            s.alias_id(s.n_alias) = id;
            s.alias_de(s.n_alias) = k_cerca;
        end
        continue;
    end
    if s.n_obj < 32
        s.n_obj = s.n_obj + 1;
        k = s.n_obj;
        s.obj_id(k) = id;
        s.obj_cls(k) = cls;
        s.obj_x(k) = x;
        s.obj_y(k) = y;
        s.obj_conf(k) = cf;
        s.obj_estado(k) = 1;
    end
end
end

function mask = mascara_visitados(s)
% bit (id-1) = objetivo con id de Python "id" visitado (ids 1..32),
% incluidos los ids duplicados de un objetivo visitado.
bits = zeros(32,1);
for k = 1:32
    if k <= s.n_obj && s.obj_estado(k) == 3
        id = s.obj_id(k);
        if id >= 1 && id <= 32
            bits(id) = 1;
        end
    end
end
for k = 1:32
    if k <= s.n_alias
        m = s.alias_de(k);
        if m >= 1 && m <= 32 && s.obj_estado(m) == 3
            id = s.alias_id(k);
            if id >= 1 && id <= 32
                bits(id) = 1;
            end
        end
    end
end
mask = 0;
for k = 1:32
    if bits(k) > 0
        mask = mask + 2^(k-1);
    end
end
end

function s = planear_regreso(s, pE, pR, hE, hR, Dm, p)
% Secuencia de regreso que nunca cruza trayectorias (separación r = Dm+0.05):
%  - segmentos rectos a >= r: ambos a la vez;
%  - si no, va primero el dron cuyo home NO queda bloqueado por la posición
%    de espera del otro (preferentemente R), con desvío alrededor del otro;
%    el segundo sale cuando el primero llegó, con desvío alrededor del home
%    del primero (la ruta se calcula en ese momento);
%  - si ambos homes están bloqueados: R se estaciona en un punto libre,
%    E regresa, y luego R.
r = Dm + 0.05;
s.ret_r = r;
s.ret_t_llegada = [-1 -1];
s.ret_espera_xy = [pE(1) pE(2); pR(1) pR(2)];
s.ret_via = [hE; hR];
dseg = qd2_dist_segmentos(pE(1:2), hE, pR(1:2), hR);
if dseg >= r
    s.ret_orden = 0;
    s.ret_paso = [2 2];
    return;
end
if norm(pE(1:2) - hR) >= r
    [viaR, nR, okR] = qd2_ruta_desvio(pR(1:2), hR, pE(1:2), r, p);
    if okR
        s.ret_orden = 1;
        s.ret_via(2,:) = viaR;
        s.ret_paso = [0, 2 - nR];
        return;
    end
end
if norm(pR(1:2) - hE) >= r
    [viaE, nE, okE] = qd2_ruta_desvio(pE(1:2), hE, pR(1:2), r, p);
    if okE
        s.ret_orden = 2;
        s.ret_via(1,:) = viaE;
        s.ret_paso = [2 - nE, 0];
        return;
    end
end
% estacionamiento para R: libre de pE, hE y hR, camino recto sin acercarse a E
lo = p.xy_lim(1) + p.xy_margen;
hi = p.xy_lim(2) - p.xy_margen;
mejor = inf;
c_ok = [0 0];
dR = norm(pR(1:2) - pE(1:2));
for i = 1:9
    for j = 1:9
        c = [lo + (hi - lo)*(i-1)/8, lo + (hi - lo)*(j-1)/8];
        if norm(c - pE(1:2)) >= r && norm(c - hE) >= r && ...
                qd2_dist_punto_segmento(c, pE(1:2), hE) >= r && ...
                qd2_dist_punto_segmento(pE(1:2), pR(1:2), c) >= min(r, dR) - 1e-9
            dc = norm(c - pR(1:2));
            if qd2_dist_punto_segmento(hE, c, hR) < r
                dc = dc + 2;            % preferir que luego R llegue directo
            end
            if dc < mejor
                mejor = dc;
                c_ok = c;
            end
        end
    end
end
if isfinite(mejor)
    s.ret_orden = 3;
    s.ret_via(2,:) = c_ok;
    s.ret_paso = [0 5];
    return;
end
% sin ruta limpia: simultáneo (el SEPARATION GUARD hace de respaldo)
s.ret_orden = 0;
s.ret_paso = [2 2];
end

function [s, sp_E, sp_R, listos] = paso_regreso(s, pE, pR, hE, hR, t, p)
% Avanza la secuencia de regreso.
% paso: 0 espera, 1 hacia via, 2 hacia home, 3 en home, 5 hacia
% estacionamiento, 6 estacionado.
P = [pE; pR];
H = [hE; hR];
z = [s.z_E s.z_R];
sp = zeros(2, 4);
for i = 1:2
    paso = s.ret_paso(i);
    if paso == 0
        sp(i,:) = [s.ret_espera_xy(i,1) s.ret_espera_xy(i,2) z(i) 0];
    elseif paso == 1 || paso == 5 || paso == 6
        sp(i,:) = [s.ret_via(i,1) s.ret_via(i,2) z(i) 0];
        if norm(P(i,1:2) - s.ret_via(i,:)) <= 1.5*p.tol_xy
            if paso == 1
                s.ret_paso(i) = 2;
            else
                s.ret_paso(i) = 6;
            end
        end
    else
        sp(i,:) = [H(i,1) H(i,2) z(i) 0];
        en_home = norm(P(i,1:2) - H(i,:)) <= p.tol_xy && abs(P(i,3) - z(i)) <= p.tol_z;
        if paso == 2
            if en_home
                if s.ret_t_llegada(i) < 0
                    s.ret_t_llegada(i) = t;
                end
                if t - s.ret_t_llegada(i) >= p.t_asentar
                    s.ret_paso(i) = 3;
                end
            else
                s.ret_t_llegada(i) = -1;
            end
        end
    end
end
% arranque del segundo dron (ruta calculada con la posición actual del primero)
% i = el que espera, o = el que ya llegó / se estacionó
for i = 1:2
    o = 3 - i;
    arranca = false;
    if s.ret_paso(i) == 0
        if (s.ret_orden == 1 && i == 1) || (s.ret_orden == 2 && i == 2)
            arranca = s.ret_paso(o) == 3;
        elseif s.ret_orden == 3 && i == 1
            arranca = s.ret_paso(o) == 6;
        end
    elseif s.ret_paso(i) == 6 && s.ret_orden == 3 && i == 2
        arranca = s.ret_paso(o) == 3;
    end
    if arranca
        [via, nv, ok] = qd2_ruta_desvio(P(i,1:2), H(i,:), P(o,1:2), s.ret_r, p);
        if nv > 0 && ok > 0
            s.ret_via(i,:) = via;
            s.ret_paso(i) = 1;
        else
            s.ret_paso(i) = 2;
        end
        s.ret_t_llegada(i) = -1;
    end
end
sp_E = sp(1,:);
sp_R = sp(2,:);
listos = (s.ret_paso(1) == 3) && (s.ret_paso(2) == 3);
end

function s = planear_aterrizaje(s, pE, pR, hE, Dm, p, t)
% R (abajo) aterriza en su sitio; E aterriza en su sitio si queda a >= Dm
% del punto de R, si no en su home, si no en un punto libre cercano.
r = Dm + 0.05;
s.land_paso = 1;
s.t_land_paso = t;
s.t_suelo = [-1 -1];
s.land_r_baja = 0;
xyR = [pR(1) pR(2)];
s.land_xy(2,:) = xyR;
if norm(pE(1:2) - xyR) >= r
    xyE = [pE(1) pE(2)];
elseif norm(hE - xyR) >= r
    xyE = hE;
else
    xyE = [pE(1) pE(2)];
    mejor = inf;
    lo = p.xy_lim(1) + p.xy_margen;
    hi = p.xy_lim(2) - p.xy_margen;
    for k = 1:16
        ang = 2*pi*(k-1)/16;
        c = min(max(xyR + (r + 0.1)*[cos(ang) sin(ang)], lo), hi);
        if norm(c - xyR) >= r && norm(c - pE(1:2)) < mejor
            mejor = norm(c - pE(1:2));
            xyE = c;
        end
    end
end
s.land_xy(1,:) = xyE;
[via, nv, ok] = qd2_ruta_desvio(pE(1:2), xyE, xyR, r, p);
s.land_via_E = via;
s.land_usa_via_E = double(nv > 0 && ok > 0);
end

function [s, sp_E, sp_R] = paso_aterrizaje(s, pE, pR, t, p)
% Pasos: 1 baja R (E va a su punto a su altura), 2 baja E, 3 listo,
% 4 ambos bajan a la vez en su sitio (despegue cancelado en tierra).
% Paso 1: R NO baja mientras E esté a < d_min_h en horizontal de su punto
% (con LAND_ALL dado con los drones apilados, versión MPC, E todavía está
% encima): R espera a su altura (takeoff = 1) hasta que E se aparta; así,
% si E falla en ese momento, R sigue en vuelo y puede salir de debajo
% (reacción a la falla). Un dron en falla nunca recibe takeoff aquí.
% Cada paso tiene un timeout p.t_land_max (alerta 4) para no quedarse
% esperando un "suelo" que OptiTrack no reporta (pivot alto, rastreo perdido).
xyE = s.land_xy(1,:);
xyR = s.land_xy(2,:);
% detección de suelo
if pR(3) <= p.z_aterrizado
    if s.t_suelo(2) < 0, s.t_suelo(2) = t; end
else
    s.t_suelo(2) = -1;
end
if pE(3) <= p.z_aterrizado
    if s.t_suelo(1) < 0, s.t_suelo(1) = t; end
else
    s.t_suelo(1) = -1;
end
r_en_suelo = s.t_suelo(2) >= 0 && (t - s.t_suelo(2)) >= p.t_aterrizado;
e_en_suelo = s.t_suelo(1) >= 0 && (t - s.t_suelo(1)) >= p.t_aterrizado;
vencido = (t - s.t_land_paso) >= p.t_land_max;
sp_R = [xyR(1) xyR(2) 0 0];
fE = s.falla(rol_de(s, 1)) > 0;     % dron del ROL E en falla (rol_de es su propia inversa)
fR = s.falla(rol_de(s, 2)) > 0;
if s.land_paso == 4
    s.takeoff = [0 0];
    sp_E = [xyE(1) xyE(2) 0 0];
    if (r_en_suelo && e_en_suelo) || vencido
        if ~(r_en_suelo && e_en_suelo), s.alerta = 4; end
        s.land_paso = 3;
    end
    return;
end
% E se traslada a su punto de aterrizaje (a su altura) mientras R baja
if s.land_usa_via_E > 0.5
    sp_E = [s.land_via_E(1) s.land_via_E(2) s.z_E 0];
    if norm(pE(1:2) - s.land_via_E) <= 1.5*p.tol_xy
        s.land_usa_via_E = 0;
    end
else
    sp_E = [xyE(1) xyE(2) s.z_E 0];
end
if s.land_paso == 1 && s.land_r_baja < 0.5
    if norm(pE(1:2) - xyR) >= s.d_min_h || fR || fE
        s.land_r_baja = 1;
    end
end
if s.land_paso == 1 && s.land_r_baja < 0.5
    s.takeoff(2) = 1;        % R espera a su altura (E todavía cerca en horizontal)
    sp_R = [xyR(1) xyR(2) s.z_R 0];
else
    s.takeoff(2) = 0;        % autoland a bordo de R
end
% (E en falla ya no va a su punto: no se le espera)
e_en_punto = (s.land_usa_via_E < 0.5 && norm(pE(1:2) - xyE) <= p.tol_xy) || fE;
if s.land_paso == 1
    if fE
        s.takeoff(1) = 0;    % E en falla: nunca takeoff (baja en su sitio)
        sp_E = [pE(1) pE(2) 0 0];
    else
        s.takeoff(1) = 1;
    end
    % Timeout: si R no se detecta en el suelo, E baja igual una vez en su
    % punto (a >= d_min_h + margen del punto de R). Último recurso (2x):
    % E baja aunque no haya llegado a su punto.
    if ((r_en_suelo || vencido) && e_en_punto) || (t - s.t_land_paso) >= 2*p.t_land_max
        if ~r_en_suelo || ~e_en_punto, s.alerta = 4; end
        s.land_paso = 2;
        s.t_land_paso = t;
        s.land_usa_via_E = 0;
    end
end
if s.land_paso == 2
    s.takeoff(1) = 0;        % autoland a bordo de E
    sp_E = [xyE(1) xyE(2) 0 0];
    if e_en_suelo || (t - s.t_land_paso) >= p.t_land_max
        if ~e_en_suelo, s.alerta = 4; end
        s.land_paso = 3;
    end
end
end

% =========================================================================
%  Falla a bordo
% =========================================================================
function r = rol_de(s, i)
% Índice de ROL (1 = E, 2 = R) del DRON i (con roles sin asignar, el mismo).
if s.explorer_id == 2
    r = 3 - i;
else
    r = i;
end
end

function s = marcar_falla(s, nueva)
% Recuerda la falla por dron y pone la alerta (9 manda sobre 10).
for i = 1:2
    s.falla(i) = max(s.falla(i), nueva(i));
end
if s.falla(1) == 9 || s.falla(2) == 9
    s.alerta = 9;
else
    s.alerta = 10;
end
end

function s = iniciar_falla(s, nueva, pose1, pose2, Dm, p, t)
% Falla con algún dron en el aire. Si uno sigue sano, empieza la reacción
% (fase 1: apartarse). Si los dos están en falla, aterrizaje ordenado normal
% (R abajo primero, E a >= d_min_h): es lo más seguro si aún obedecen.
s = marcar_falla(s, nueva);
P = [pose1; pose2];
if s.explorer_id == 2
    pE = pose2; pR = pose1; hE = s.home2;
else
    pE = pose1; pR = pose2; hE = s.home1;
end
if s.falla(1) > 0 && s.falla(2) > 0
    s = planear_aterrizaje(s, pE, pR, hE, Dm, p, t);
    s = entrar(s, 7, t);
    return;
end
if s.falla(1) > 0
    f = 1;
else
    f = 2;
end
h = 3 - f;
rf = rol_de(s, f);
rh = rol_de(s, h);
s.falla_sano = h;
s.falla_fase = 1;
s.falla_land = 0;
s.t_falla_fase = t;
s.t_falla_llego = -1;
s.t_falla_suelo = -1;
% dron en falla: su consigna se congela donde estaba (x, y y su altura de
% consigna). Su takeoff no se toca hasta que el sano esté apartado.
z_sp = [s.sp_E(3) s.sp_R(3)];
zf = z_sp(rf);
if zf <= 0
    zf = P(f,3);
end
s.falla_hold = [P(f,1) P(f,2) zf];
% dron sano: se aparta a SU altura
z_rol = [s.z_E s.z_R];
s.falla_z = z_rol(rh);
if h == 1
    hh = s.home1;
else
    hh = s.home2;
end
s.falla_sep = punto_separacion(P(h,1:2), P(f,1:2), hh, Dm + 0.05, p);
s = entrar(s, 9, t);
end

function c_ok = punto_separacion(ph, pf, hh, r, p)
% Punto a >= r (en horizontal) del dron en falla pf, lo más cerca posible
% del dron sano ph (se aparta en línea recta, alejándose de pf) y dentro de
% la arena. Si ya está a >= r, se queda donde está.
lo = p.xy_lim(1) + p.xy_margen;
hi = p.xy_lim(2) - p.xy_margen;
ph = [ph(1) ph(2)];
pf = [pf(1) pf(2)];
hh = [hh(1) hh(2)];
c_ok = min(max(ph, lo), hi);
if norm(c_ok - pf) >= r
    return;
end
mejor = inf;
d = norm(ph - pf);
for k = 0:32
    if k == 0
        if d < 1e-3
            continue;
        end
        u = (ph - pf)/d;             % dirección radial: alejarse en recta
    else
        ang = 2*pi*(k-1)/32;
        u = [cos(ang) sin(ang)];
    end
    c = min(max(pf + r*u, lo), hi);
    if norm(c - pf) >= r - 1e-6
        costo = norm(c - ph) + 0.05*norm(c - hh);
        if costo < mejor
            mejor = costo;
            c_ok = c;
        end
    end
end
if ~isfinite(mejor)
    % (no pasa en una arena de 3 x 3 m con r ~ 1.1 m) la esquina más lejana
    dmax = -1;
    esquinas = [lo lo; lo hi; hi lo; hi hi];
    for k = 1:4
        if norm(esquinas(k,:) - pf) > dmax
            dmax = norm(esquinas(k,:) - pf);
            c_ok = esquinas(k,:);
        end
    end
end
end

function [s, sp_E, sp_R, esc] = paso_falla(s, pose1, pose2, t, p, Dm)
% Reacción a la falla a bordo de un dron (el otro está sano).
%  1 APARTARSE: el sano va en recta, a su altura, a un punto a >= d_min_h +
%    margen del dron en falla (si estaban apilados sale de debajo/encima de
%    inmediato, con límite xy p.falla_rate_xy). El dron en falla conserva su
%    consigna.
%  2 REGRESAR: takeoff del dron en falla = 0 (si aún vuela, autoland en su
%    sitio: ya no tiene a nadie debajo). El sano va a su home (con desvío
%    alrededor del dron en falla) o, si su home queda a < d_min_h + margen
%    del dron en falla, se queda donde está.
%  3 ATERRIZA EL SANO (takeoff = 0, autoland).
%  4 Espera a que el dron en falla esté en el suelo (o t_land_max) -> DONE.
% LAND_ALL durante la reacción: el sano aterriza en cuanto está apartado.
P = [pose1; pose2];
h = s.falla_sano;
f = 3 - h;
rh = rol_de(s, h);
rf = rol_de(s, f);
ph = P(h,:);
pf = P(f,:);
r = Dm + 0.05;
esc = [0 0];
sp = zeros(2, 4);                     % por DRON
sp(f,:) = [s.falla_hold(1) s.falla_hold(2) s.falla_hold(3) 0];
if s.falla_fase >= 2
    sp(f,3) = 0;
end
if h == 1
    hh = s.home1;
else
    hh = s.home2;
end
separado = norm(ph(1:2) - pf(1:2)) >= s.d_min_h;
if s.falla_fase == 1
    % si el dron en falla se desplazó (cae sin control de posición), se
    % recalcula el punto de separación
    if norm(s.falla_sep - pf(1:2)) < r - 0.05
        s.falla_sep = punto_separacion(ph(1:2), pf(1:2), hh, r, p);
    end
    sp(h,:) = [s.falla_sep(1) s.falla_sep(2) s.falla_z 0];
    esc(h) = 1;
    llego = norm(ph(1:2) - s.falla_sep) <= 1.5*p.tol_xy;
    if (separado && llego) || (t - s.t_falla_fase) >= p.t_falla_sep_max
        s.takeoff(rf) = 0;            % el de la falla: autoland en su sitio (nunca sube)
        sp(f,3) = 0;
        if s.falla_land > 0.5 && separado
            land = [ph(1) ph(2)];
        elseif norm(hh - pf(1:2)) >= r
            land = [hh(1) hh(2)];
        else
            land = [ph(1) ph(2)];
        end
        s.falla_land_xy = land;
        [via, nv, ok] = qd2_ruta_desvio(ph(1:2), land, pf(1:2), r, p);
        s.falla_via = [via(1) via(2)];
        s.falla_usa_via = double(nv > 0 && ok > 0);
        s.falla_fase = 2;
        s.t_falla_fase = t;
        s.t_falla_llego = -1;
        if s.falla_land > 0.5 && separado
            s.falla_usa_via = 0;
            s.falla_fase = 3;
            s = entrar(s, 7, t);
        end
    end
end
if s.falla_fase == 2
    if s.falla_land > 0.5 && separado
        s.falla_land_xy = [ph(1) ph(2)];
        s.falla_usa_via = 0;
        s.falla_fase = 3;
        s.t_falla_fase = t;
        s = entrar(s, 7, t);
    else
        if s.falla_usa_via > 0.5
            sp(h,:) = [s.falla_via(1) s.falla_via(2) s.falla_z 0];
            if norm(ph(1:2) - s.falla_via) <= 1.5*p.tol_xy
                s.falla_usa_via = 0;
            end
        else
            sp(h,:) = [s.falla_land_xy(1) s.falla_land_xy(2) s.falla_z 0];
        end
        en_punto = s.falla_usa_via < 0.5 && norm(ph(1:2) - s.falla_land_xy) <= p.tol_xy;
        if en_punto
            if s.t_falla_llego < 0
                s.t_falla_llego = t;
            end
        else
            s.t_falla_llego = -1;
        end
        if (en_punto && (t - s.t_falla_llego) >= p.t_asentar) || (t - s.t_falla_fase) >= 2*p.t_land_max
            s.falla_fase = 3;
            s.t_falla_fase = t;
            s = entrar(s, 7, t);
        end
    end
end
if s.falla_fase == 3
    s.takeoff(rh) = 0;                % autoland a bordo del sano
    sp(h,:) = [s.falla_land_xy(1) s.falla_land_xy(2) 0 0];
    if ph(3) <= p.z_aterrizado
        if s.t_falla_suelo < 0
            s.t_falla_suelo = t;
        end
    else
        s.t_falla_suelo = -1;
    end
    if (s.t_falla_suelo >= 0 && (t - s.t_falla_suelo) >= p.t_aterrizado) || (t - s.t_falla_fase) >= p.t_land_max
        s.falla_fase = 4;
        s.t_falla_fase = t;
        s.t_falla_suelo = -1;
    end
elseif s.falla_fase == 4
    sp(h,:) = [s.falla_land_xy(1) s.falla_land_xy(2) 0 0];
    if pf(3) <= p.z_aterrizado
        if s.t_falla_suelo < 0
            s.t_falla_suelo = t;
        end
    else
        s.t_falla_suelo = -1;
    end
    if (s.t_falla_suelo >= 0 && (t - s.t_falla_suelo) >= p.t_aterrizado) || (t - s.t_falla_fase) >= p.t_land_max
        % DONE: land_xy por rol (fila 1 = E, fila 2 = R) para el caso 8
        s.land_xy(rh,:) = s.falla_land_xy;
        s.land_xy(rf,:) = [s.falla_hold(1) s.falla_hold(2)];
        s.arm = [0 0];
        s.takeoff = [0 0];
        s.falla_fase = 0;
        s = entrar(s, 8, t);
    end
end
if s.falla_fase == 1 || s.falla_fase == 2
    if s.falla_land > 0.5
        s.estado = 7;                 % LAND_ALL aceptado (la capa del operador lo ve)
    else
        s.estado = 9;
    end
end
if rh == 1
    sp_E = sp(h,:);
    sp_R = sp(f,:);
else
    sp_E = sp(f,:);
    sp_R = sp(h,:);
end
end
