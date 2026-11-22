function [u, st] = qd2_pid_position(x_est, ref, st, p, dt)
%QD2_PID_POSITION Réplica del PID de posición a bordo del Equipo2.
%   [u, st] = qd2_pid_position(x_est, ref, st, p, dt)
%
%   Replica COMMANDER (ANGLE MODE) del QD2_DroneStack_PID original para la
%   simulación (y como respaldo mpc_mode = 0):
%     u_i = Kp*e + int(Ki*e) - Kd*rate   (D sobre la velocidad medida),
%     error rotado a horizon frame con el yaw estimado,
%     saturación [pi/4 pi/4 35 N 100 deg/s], thrust = trim + u_z en [0 58] N,
%     roll = -u_y, pitch = u_x, yaw_rate = u_yaw.
%   x_est : 9x1 [x y z vx vy vz roll pitch yaw]
%   ref   : 4x1 [x y z yaw] deseado (ya suavizado)
%   st    : struct con campo int (4x1); st = struct('int', zeros(4,1)) al
%           iniciar (o al pasar a TAKEOFF_HEIGHT, como el original).
%   u     : 4x1 [T roll pitch yaw_rate]
%#codegen

psi = x_est(9);
c = cos(psi); s = sin(psi);
ex = ref(1) - x_est(1);
ey = ref(2) - x_est(2);
e = [c*ex + s*ey; -s*ex + c*ey; ref(3) - x_est(3); qd2_wrap_pi(ref(4) - psi)];
vh = [c*x_est(4) + s*x_est(5); -s*x_est(4) + c*x_est(5); x_est(6); 0];
Kp = p.pid.Kp(:); Ki = p.pid.Ki(:); Kd = p.pid.Kd(:);
lim = p.pid.int_lim(:);
st.int = min(max(st.int + dt*Ki.*e, -lim), lim);
v = Kp.*e + st.int - Kd.*vh;
sat = p.pid.sat(:);
v = min(max(v, -sat), sat);
T = min(max(p.T_trim + v(3), p.pid.T_sat(1)), p.pid.T_sat(2));
u = [T; -v(2); v(1); v(4)];
end
