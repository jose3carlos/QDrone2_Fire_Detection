function p = qd2_params()
%QD2_PARAMS Valores por defecto de toda la librería de misión (unidades SI).
%   p = qd2_params() devuelve un struct de tamaño fijo, compatible con
%   codegen, con alturas, separaciones, tolerancias, límites de la arena,
%   ganancias del PID original del Equipo2, planta de simulación, modelo de
%   downwash y matrices precalculadas del MPC.
%
%   Uso en un bloque MATLAB Function (recomendado, se calcula una sola vez):
%       persistent p
%       if isempty(p), p = qd2_params(); end
%
%   Los valores de config/estacion.json se sobreponen con
%   qd2_params_from_config (solo MATLAB, no codegen).
%
%   Ver docs/ARQUITECTURA.md §4.

%#codegen

% ---------------- Constantes físicas -------------------------------------
% Nota codegen: no se lee 'p' mientras se construye (no se permite agregar
% campos a un struct después de leerlo), por eso se usan variables locales.
g    = 9.81;
masa = 1.504;
p.g      = g;               % m/s^2
p.masa   = masa;            % kg (Drone Weight del THROTTLE SWITCHBOX original)
p.T_trim = masa*g;          % N  (= 14.754 N, trim del original)
p.T_max_fisico = 43.16;     % N  (Max Throttle HQ 7 inch = 4 x 10.79 N)

% ---------------- Arena y misión ------------------------------------------
p.xy_lim     = [-1.5 1.5];  % m, arena 3x3 centrada en (0,0)
p.xy_margen  = 0.15;        % m, las consignas se recortan a xy_lim -/+ margen
p.z_E        = 1.5;         % m, altura del EXPLORADOR
p.z_R        = 0.5;         % m, altura del RESPONDEDOR
p.z_max      = 2.0;         % m, techo de consignas (el dron declara "too high" en 2.5)
p.t_mark     = 3.0;         % s, hover del respondedor sobre cada objetivo
p.d_min_h    = 0.9;         % m, separación horizontal mínima (versión PID)
p.dz_min     = 0.8;         % m, separación vertical mínima al apilarse (versión MPC)
p.wp_spacing = 0.25;        % m, espaciado de waypoints de la N
p.tol_xy     = 0.12;        % m, tolerancia horizontal de llegada
p.tol_z      = 0.12;        % m, tolerancia vertical de llegada
p.tol_mark_factor = 1.5;    % la marca sigue contando mientras |err| <= factor*tol
p.auto_land  = 0;           % 1 = aterrizar solo al terminar (si no, esperar LAND_ALL)
% Modo de misión (config/estacion.json -> mision.modo / mision.dron_solo, ver
% qd2_params_from_config). Parámetro NO ajustable: cambiarlo obliga a recompilar.
p.modo_mision = 0;          % 0 = dual (2 drones, explorador + respondedor);
                            % 1 = solo explorador (un dron recorre la N, sin respondedor)
p.dron_solo   = 1;          % dron (1 o 2) que vuela en modo solo explorador (con modo_mision = 1);
                            % el otro nunca recibe arm/takeoff
p.t_dwell_wp = 0.3;         % s, permanencia dentro de tolerancia para avanzar de waypoint
p.t_init     = 1.0;         % s, espera en INIT
p.t_tracking_estable = 1.0; % s, ambos rastreados de forma continua antes de asignar roles
p.hb_timeout = 1.0;         % s, sin heartbeat en MISSION -> PAUSED
p.t_arm_a_despegue = 2.0;   % s, entre arm=1 y takeoff=1
p.t_hover_ready = 2.0;      % s, en HOVER_READY antes de pasar a MISSION
p.t_fin_gracia  = 2.0;      % s, espera tras terminar la N por objetivos tardíos
p.t_asentar     = 1.0;      % s, permanencia en home para darlo por alcanzado
p.z_aterrizado  = 0.08;     % m, por debajo = en el suelo
p.t_aterrizado  = 1.0;      % s, permanencia en el suelo para darlo por aterrizado
p.radio_cluster = 0.25;     % m, objetivos más cercanos que esto son el mismo
p.max_objetivos = 32;       % tamaño fijo de la tabla de objetivos
p.max_wp        = 128;      % tamaño fijo de la tabla de waypoints
p.guard_margen  = 0.15;     % m, margen extra del SEPARATION GUARD
p.n_dir_evasion = 24;       % direcciones candidatas para la evasión del guard
p.guard_look_wp = 8;        % waypoints de anticipación de la ruta de E (8 x 0.25 = 2 m)
% Seguridad (agregado tras la revisión)
p.z_despegado   = 0.15;     % m, un dron con z mayor que esto se considera en el aire
p.t_perdida_trk = 0.2;      % s, sin rastreo válido (tracked=1 y pose finita) en el aire -> PAUSED
                            %    (mismo umbral que la FSM a bordo)
p.t_takeoff_max = 30.0;     % s, desde takeoff=1 sin llegar a la altura -> ABORT (o cancelar si nadie despegó)
p.t_land_max    = 30.0;     % s, por paso de LANDING sin detectar el suelo -> siguiente paso
p.d_home_min_mpc = 0.8;     % m, distancia mínima entre homes para aceptar START en la versión MPC
                            %    (en la PID se exige d_min_h + guard_margen)
p.d_min_h_max   = 1.5;      % m, tope del override de d_min_h (el override solo puede AUMENTARLO)
p.t_mark_lim    = [0.5 30]; % s, rango aceptado para el override de t_mark
% Falla a bordo (versión MPC: la FSM a bordo llega en el índice 2 de la respuesta del dron)
p.t_proto_mal   = 0.5;      % s, respuesta sin etiqueta y(8) = 2 o FSM fuera de 1..9, sostenida
                            %    con el dron conectado -> alerta 10 (dron no disponible)
p.falla_rate_xy = 1.5;      % m/s, límite xy del dron SANO mientras se aparta del dron en falla
                            %    (el normal es pid.rate_xy = 1/3 m/s; el MPC sigue escalones de 0.6 m).
                            %    Con 1.5 m/s, cuando el de ARRIBA cae a la altura del de abajo ya
                            %    están a >= d_min_h en horizontal (lazo cerrado: 0.98 m; con 1.0: 0.85 m)
p.t_falla_sep_max = 8.0;    % s, máximo para apartarse; después sigue con el regreso igual
p.t_rot_obj     = 0.02;     % s, periodo de rotación del objetivo en y(56:60) de la telemetría

% Polilínea de la N invertida (fila = vértice [x y]); tamaño fijo 16x2.
% Valor por defecto = config/estacion.json -> pista.vertices.
p.max_vertices = 16;
V = zeros(16,2);
V(1:4,:) = [-1.2 1.2; 1.2 1.2; -1.2 -1.2; 1.2 -1.2];
p.vertices   = V;
p.n_vertices = 4;

% ---------------- PID original del Equipo2 (COMMANDER ANGLE MODE) ---------
% Ganancias efectivas [x y z yaw] (ver contexto/analisis_dronestack.md §4).
Kp_pid = [pi/6*0.9, pi/6*0.9, 35*0.9, 15*0.5];
p.pid.Kp      = Kp_pid;
p.pid.Ki      = [0.01, 0.01, 6, 0];
p.pid.Kd      = [pi/2*0.4, pi/2*0.4, 28, 4*0.1];
p.pid.int_lim = [0.195, 0.2475, 11.25, 0.0375];
p.pid.sat     = [pi/4, pi/4, 35, 100*pi/180];   % [rad rad N rad/s]
p.pid.T_sat   = [0 58];                          % N, throttle neto
p.pid.rate_xy = 1/3;   % m/s,  Rate Limiter2 del MissionCtrl original
p.pid.rate_z  = 1/6;   % m/s
p.pid.rate_yaw = pi/6; % rad/s

% ---------------- Planta de simulación ------------------------------------
% Actitud de primer orden. tau_att identificada con el vuelo real del
% Equipo2 (saved_data, FSM = 5): comando del COMMANDER -> actitud estimada
% por la IMU. Mejor ajuste: retardo ~0.10 s + lag <= 0.02 s; sin retardo
% el primer orden equivalente da tau = 0.11 s (pitch) / 0.165 s (roll).
% Se usa 0.12 s (ver tests/qd2_identificar_actitud.m).
tau_att = 0.12;
p.planta.masa    = masa;
p.planta.g       = g;
p.planta.tau_att = tau_att; % s
p.planta.tau_T   = 0.03;    % s, retardo de empuje de los motores
p.planta.tau_yaw = 0.05;    % s
p.planta.k_drag  = 0.15;    % N/(m/s), arrastre lineal
p.planta.att_max = pi/4;    % rad
p.planta.T_max   = 43.16;
p.planta.dt_int  = 0.001;   % s, paso interno de integración

% ---------------- Modelo de downwash --------------------------------------
% F_z = -F0 * exp(-r^2/(2 sigma^2)) * (1-exp(-dz/dz_on)) / (1 + dz/l_dec)
% Con los valores por defecto: dz = 1.0 m, r = 0 -> ~2.9 N (~20% del peso).
% SUPUESTO: F0 no sale de una medición ni de una fuente; se eligió para que
% el empuje perdido a 1 m sea ~20 % del peso (orden de magnitud plausible).
% Hay que medirlo en el laboratorio (dron de abajo en hover, el otro encima).
p.dw.F0      = 5.9;     % N (supuesto, ver arriba)
p.dw.sigma   = 0.25;    % m, ancho gaussiano del chorro
p.dw.dz_on   = 0.10;    % m, el efecto aparece con dz > ~0.1 m
p.dw.l_dec   = 1.0;     % m, decaimiento con la separación vertical
p.dw.dz_max  = 3.0;     % m, sin efecto más allá
p.dw.ruido_rel  = 0.30; % desviación del ruido vertical relativa a |F_z|
p.dw.ruido_xy   = 0.15; % desviación del ruido horizontal relativa a |F_z|

% ---------------- MPC a bordo ---------------------------------------------
b.g = g;
b.masa = masa;
b.T_trim = masa*g;
b.planta.tau_att = tau_att;
b.pid.Kp = Kp_pid;
p.mpc = qd2_mpc_build(b);

end
