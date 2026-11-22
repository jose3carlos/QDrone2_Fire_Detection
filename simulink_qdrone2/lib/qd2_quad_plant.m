function x = qd2_quad_plant(x, u, dist, dt, pl)
%QD2_QUAD_PLANT Planta simple de cuadricóptero para simulación (RSF, z arriba).
%   x = qd2_quad_plant(x, u, dist, dt)
%   x = qd2_quad_plant(x, u, dist, dt, pl)
%
%   x    : 10x1 [x y z vx vy vz roll pitch yaw T_real]  (m, m/s, rad, N)
%   u    : 4x1  [T_cmd roll_cmd pitch_cmd yaw_rate_cmd] (N, rad, rad, rad/s)
%          (mismo orden que la salida del COMMANDER original).
%   dist : 3x1 fuerza externa en RSF (N): downwash + ruido + viento.
%   dt   : paso (s). Se integra internamente con pl.dt_int (Euler semi-
%          implícito), así que dt = 0.02 s es válido.
%   pl   : parámetros de planta (por defecto qd2_params().planta).
%
%   Modelo:
%     - Traslación: m*a = T*R(roll,pitch,yaw)*e3 - m*g*e3 - k_drag*v + dist,
%       con R = Rz*Ry*Rx  ->  pitch > 0 acelera +x, roll > 0 acelera -y
%       (consistente con roll = -u_y, pitch = u_x del original).
%     - Actitud de primer orden (tau_att) hacia el comando saturado a att_max
%       (aproxima el lazo de ángulo+velocidad del STABILIZER).
%     - Yaw: la razón de yaw sigue al comando con tau_yaw.
%     - Empuje de primer orden (tau_T), saturado en [0, T_max].
%     - Suelo en z = 0 (sin rebote, fricción total al estar apoyado).
%   El ruido NO se genera aquí: el simulador suma a dist el ruido que
%   devuelve qd2_downwash (sigma) u otro que quiera.
%#codegen

if nargin < 5
    pp = qd2_params();
    pl = pp.planta;
end
m = pl.masa;
g = pl.g;
n_sub = max(1, round(dt/pl.dt_int));
h = dt/n_sub;

T_cmd = min(max(u(1), 0), pl.T_max);
phi_c = min(max(u(2), -pl.att_max), pl.att_max);
th_c  = min(max(u(3), -pl.att_max), pl.att_max);
r_c   = u(4);
if ~all(isfinite(u))
    T_cmd = 0; phi_c = 0; th_c = 0; r_c = 0;
end
F = zeros(3,1);
F(:) = dist(1:3);
F(~isfinite(F)) = 0;

% estado auxiliar de razón de yaw (no se guarda: primer orden sobre yaw)
for k = 1:n_sub
    phi = x(7); th = x(8); psi = x(9); T = x(10);
    % dinámica de actitud y empuje
    x(7)  = phi + h*(phi_c - phi)/pl.tau_att;
    x(8)  = th  + h*(th_c - th)/pl.tau_att;
    x(9)  = psi + h*r_c;
    x(10) = T   + h*(T_cmd - T)/pl.tau_T;
    % traslación
    cphi = cos(phi); sphi = sin(phi);
    cth = cos(th);   sth = sin(th);
    cpsi = cos(psi); spsi = sin(psi);
    e3 = [cpsi*sth*cphi + spsi*sphi; spsi*sth*cphi - cpsi*sphi; cth*cphi];
    a = (T*e3 - pl.k_drag*x(4:6) + F)/m;
    a(3) = a(3) - g;
    v = x(4:6) + h*a;
    pos = x(1:3) + h*v;
    if pos(3) <= 0
        pos(3) = 0;
        if v(3) < 0
            v(3) = 0;
        end
        if T*e3(3) + F(3) < m*g   % apoyado: fricción total
            v(1) = 0; v(2) = 0;
            pos(1) = x(1); pos(2) = x(2);
        end
    end
    x(1:3) = pos;
    x(4:6) = v;
end
end
