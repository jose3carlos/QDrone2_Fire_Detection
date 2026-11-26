function s = qd2_mission_init(p)
%QD2_MISSION_INIT Estado inicial (struct de tamaño fijo) del supervisor.
%   s = qd2_mission_init(p) se usa una sola vez (persistent en el bloque
%   MATLAB Function o al iniciar el simulador). Ver qd2_mission_step.
%#codegen

MAXO = 32;
MAXWP = 128;

s.estado      = 0;    % mission_state (§3.3): 0 INIT ... 10 PAUSED
s.estado_prev = 5;    % estado al que vuelve RESUME
s.t_entrada   = 0;    % tiempo de entrada al estado actual
s.t_prev      = 0;
s.primera     = 1;
s.t_ok_desde  = -1;   % inicio de rastreo continuo de ambos drones

% Roles y home
s.explorer_id  = 0;
s.responder_id = 0;
s.home1 = [0 0];
s.home2 = [0 0];
s.roles_ok = 0;

% Explorador
s.wp      = zeros(MAXWP, 4);
s.n_wp    = 0;
s.wp_idx  = 1;
s.t_dwell = 0;
s.explorer_done = 0;
s.t_e_done = 0;

% Tabla de objetivos (índice = orden de llegada; id = el de Python)
s.obj_id     = zeros(MAXO, 1);
s.obj_cls    = zeros(MAXO, 1);
s.obj_x      = zeros(MAXO, 1);
s.obj_y      = zeros(MAXO, 1);
s.obj_conf   = zeros(MAXO, 1);
s.obj_estado = zeros(MAXO, 1);   % 0 vacío, 1 en cola, 2 asignado, 3 visitado
s.n_obj      = 0;
% ids duplicados (a menos de radio_cluster de otro objetivo)
s.alias_id = zeros(MAXO, 1);
s.alias_de = zeros(MAXO, 1);     % índice de la tabla al que equivale
s.n_alias  = 0;

% Respondedor
s.r_obj   = 0;        % índice de la tabla del objetivo actual (0 = ninguno)
s.r_fase  = 0;        % 0 libre, 1 viajando, 2 marcando
s.t_marca = 0;        % tiempo acumulado de marca (s)
s.r_idle_xy = [0 0];  % dónde espera el respondedor libre

% Comunicación con la estación Python
s.hb_last  = 0;
s.t_hb     = 0;
s.hb_visto = 0;
s.cmd_seq_ack  = 0;
s.pausa_por_hb = 0;

% Consignas mantenidas (PAUSED) [x y z yaw]
s.hold_E = [0 0 0 0];
s.hold_R = [0 0 0 0];

% Disparos por rol [E R]
s.arm     = [0 0];
s.takeoff = [0 0];

% Regreso a home (RETURN_HOME / ABORT): fila 1 = E, fila 2 = R
s.ret_paso = [0 0];              % 0 espera, 1 hacia via, 2 hacia home, 3 en home,
                                 % 5 hacia estacionamiento, 6 estacionado
s.ret_via  = zeros(2, 2);        % punto de desvío (o de estacionamiento)
s.ret_espera_xy = zeros(2, 2);
s.ret_t_llegada = [-1 -1];
s.ret_orden = 0;                 % 0 simultáneo, 1 R primero, 2 E primero,
                                 % 3 R se estaciona, E llega, luego R
s.ret_r = 1.1;                   % separación usada para planear (m)

% Aterrizaje
s.land_paso = 0;                 % 1 baja R (E va a su punto), 2 baja E, 3 listo,
                                 % 4 ambos bajan a la vez (despegue cancelado en tierra)
s.land_xy   = zeros(2, 2);       % fila 1 = E, fila 2 = R
s.land_via_E = [0 0];
s.land_usa_via_E = 0;
s.land_r_baja = 0;               % paso 1: 1 = R ya baja (E está a >= d_min_h en horizontal);
                                 % 0 = R espera a su altura (no aterriza debajo de E)
s.t_suelo   = [-1 -1];           % [E R] desde cuándo están en el suelo

% Rastreo: última pose finita por dron y último instante con rastreo válido
s.pose_ok1 = [0 0 0 0];
s.pose_ok2 = [0 0 0 0];
s.t_trk    = [0 0];
s.pausa_por_trk = 0;

% Seguridad
s.alerta     = 0;                % 0 nada, 1 homes demasiado cerca (START rechazado),
                                 % 2 pérdida de rastreo (en el aire aterriza ese dron a los
                                 % p.t_perdida_aterrizar_s; PAUSED solo si ese valor es
                                 % mayor que p.t_perdida_trk), 3 timeout de despegue,
                                 % 4 timeout de aterrizaje, 9 falla a bordo de un dron,
                                 % 10 protocolo MPC incorrecto (5-8 son del operador)
s.home_ok    = 0;                % 1 si los homes cumplen la separación mínima
s.t_despegue = 0;                % instante en que se dio takeoff = 1
s.t_land_paso = 0;               % instante de entrada al paso actual de LANDING

% Falla a bordo (versión MPC; ver qd2_mission_step y qd2_enlace_mpc; en dual PID, dron caído)
s.falla      = [0 0];            % por DRON: 0 nada, 9 FSM a bordo 7/8/9 (o enlace perdido en vuelo, o caído en dual PID),
                                 % 10 protocolo MPC incorrecto, 2 pérdida de rastreo sostenida en
                                 % el aire (> p.t_perdida_aterrizar_s): aterriza, nunca más takeoff
s.falla_fase = 0;                % reacción: 0 ninguna, 1 apartarse, 2 regresar,
                                 % 3 aterriza el sano, 4 espera al dron en falla en el suelo
s.falla_sano = 0;                % dron sano (1 o 2)
s.falla_sep  = [0 0];            % punto de separación del dron sano
s.falla_hold = [0 0 0];          % [x y z] de consigna del dron en falla (congelada)
s.falla_z    = 0;                % altura del dron sano mientras se aparta y regresa
s.falla_land_xy = [0 0];         % donde aterriza el dron sano
s.falla_via  = [0 0];
s.falla_usa_via = 0;
s.falla_land = 0;                % 1 = LAND_ALL pedido durante la reacción
s.falla_trk  = 0;                % 1 = el dron de la reacción "falló" por pérdida de rastreo
                                 % sostenida (falla = 2): su pose es la última vista, así que el
                                 % sano se aparta con el margen extra p.d_deriva_sin_rastreo
s.t_falla_fase  = 0;
s.t_falla_llego = -1;
s.t_falla_suelo = -1;
% Dron caído (dual PID, ver qd2_mission_step y p.t_caido_s), por DRON
s.caido_alto = [0 0];            % 1 = ya estuvo despegado en este vuelo (latch)
s.t_caido    = [-1 -1];          % desde cuándo está en el piso rastreado (-1 = no)

% Parámetros efectivos (con overrides de la estación)
s.z_E = p.z_E;
s.z_R = p.z_R;
s.t_mark = p.t_mark;
s.d_min_h = p.d_min_h;

% Últimas consignas por rol
s.sp_E = [0 0 0 0];
s.sp_R = [0 0 0 0];
end
