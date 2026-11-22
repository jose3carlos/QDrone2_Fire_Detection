function [u, mst, info] = qd2_mpc_position(x_est, ref, other, p, mst)
%QD2_MPC_POSITION MPC lineal de posición a bordo (50 Hz), sustituye al PID
%   de posición del COMMANDER (ANGLE MODE). Ver docs/ARQUITECTURA.md §4.
%
%   [u, mst, info] = qd2_mpc_position(x_est, ref, other, p, mst)
%
%   x_est : 9x1 [x y z vx vy vz roll pitch yaw] estimados a bordo (RSF).
%   ref   : 4x1 [x y z yaw] deseados (ya suavizados por el Pose Switchbox).
%   other : 6x1 [x y z vx vy vz] del otro dron (índices 17-22 del mensaje
%           MPC). Si other(3) no es finito no se usa feedforward.
%   p     : qd2_params() (usa p.mpc, p.dw, p.T_trim, p.masa, p.g).
%   mst   : estado persistente, qd2_mpc_init(p) al arrancar.
%
%   u     : 5x1 [T roll pitch yaw_rate status]  (N, rad, rad, rad/s, -)
%           mismo orden que la salida del COMMANDER original + status:
%           0 = ok, 1 = FISTA no alcanzó la tolerancia (se aplica igual la
%           última iteración), 2 = entrada inválida (NaN) -> hover nivelado.
%   info  : struct con residuos por eje, d_hat, a_ff y comando sin saturar.
%
%   Modelo por eje: doble integrador + retardo de primer orden en actitud
%   (tau_att identificado). QP con cotas de caja en roll/pitch (+-att_max) y
%   empuje [T_min, T_max] resuelto con FISTA (iteraciones fijas, reinicio
%   adaptativo, arranque en caliente). En z: observador de perturbación
%   (offset-free) + feedforward de downwash con la pose del otro dron
%   (qd2_downwash con p.dw), propagada con su velocidad sobre el horizonte.
%   Yaw: control P como el original.
%#codegen

m  = p.mpc;
N  = size(m.Hxy, 1);
Ts = m.Ts;
masa = p.masa;

info.res = [0 0 0];
info.d_hat = mst.d_hat;
info.a_ff = 0;
info.T_vert = p.T_trim;

if any(~isfinite(x_est(1:9))) || any(~isfinite(ref(1:4)))
    u = [p.T_trim; 0; 0; 0; 2];
    mst.init = 0;
    return;
end

pos = x_est(1:3);
vel = x_est(4:6);
phi = x_est(7); th = x_est(8); psi = x_est(9);

% ---------------- Observador de perturbación vertical --------------------
if mst.init == 0
    mst.v_hat = vel(3);
    mst.init = 1;
else
    v_pred = mst.v_hat + Ts*(mst.a_cmd_prev + mst.a_ff_prev + mst.d_hat);
    e = vel(3) - v_pred;
    mst.v_hat = v_pred + m.obs_l1*e;
    mst.d_hat = mst.d_hat + m.obs_l2*e;
    mst.d_hat = min(max(mst.d_hat, -m.d_max), m.d_max);
end
if m.usar_obs < 0.5
    mst.d_hat = 0;
end

% ---------------- Ejes horizontales (horizon frame) ----------------------
c = cos(psi); s = sin(psi);
dx = pos(1) - ref(1);
dy = pos(2) - ref(2);
pxh = c*dx + s*dy;
pyh = -s*dx + c*dy;
vxh = c*vel(1) + s*vel(2);
vyh = -s*vel(1) + c*vel(2);
if m.usar_att_medida > 0.5
    thx = th;
    thy = -phi;
else
    thx = mst.att_model(1);
    thy = mst.att_model(2);
end
lb = -m.att_max*ones(N,1);
ub =  m.att_max*ones(N,1);

fx = m.Fxy*[pxh; vxh; thx];
fx(1) = fx(1) - m.Rdxy*mst.ux_prev;
U0 = [mst.Ux(2:N); mst.Ux(N)];
[Ux, rx] = fista(m.Hxy, fx, lb, ub, U0, m.Lxy, m.n_iter);

fy = m.Fxy*[pyh; vyh; thy];
fy(1) = fy(1) - m.Rdxy*mst.uy_prev;
U0 = [mst.Uy(2:N); mst.Uy(N)];
[Uy, ry] = fista(m.Hxy, fy, lb, ub, U0, m.Lxy, m.n_iter);

% ---------------- Eje vertical -------------------------------------------
dk = zeros(N,1);
a_ff0 = 0;
usar_ff = (m.usar_ff > 0.5) && all(isfinite(other(1:6)));
for k = 1:N
    a_ff = 0;
    if usar_ff
        tk = (k-1)*Ts;
        ps = pos + vel*tk;
        po = other(1:3) + other(4:6)*tk;
        F = qd2_downwash(ps, po, p.dw);
        a_ff = F(3)/masa;
    end
    if k == 1
        a_ff0 = a_ff;
    end
    dk(k) = mst.d_hat + a_ff;
end
lbz = (m.T_min - p.T_trim)*ones(N,1) + masa*dk;
ubz = (m.T_max - p.T_trim)*ones(N,1) + masa*dk;
fz = m.Fz*[pos(3) - ref(3); vel(3)];
fz(1) = fz(1) - m.Rdz*mst.uz_prev;
U0 = [mst.Uz(2:N); mst.Uz(N)];
[Uz, rz] = fista(m.Hz, fz, lbz, ubz, U0, m.Lz, m.n_iter);

% ---------------- Comandos -------------------------------------------------
pitch_c = Ux(1);
roll_c  = -Uy(1);
T_vert  = p.T_trim + Uz(1) - masa*dk(1);
cinc = max(cos(phi)*cos(th), 0.5);
T = min(max(T_vert/cinc, m.T_min), m.T_max);
yaw_rate = m.yaw_kp*qd2_wrap_pi(ref(4) - psi);
yaw_rate = min(max(yaw_rate, -m.yaw_rate_max), m.yaw_rate_max);

% memoria
mst.Ux = Ux; mst.Uy = Uy; mst.Uz = Uz;
mst.ux_prev = Ux(1); mst.uy_prev = Uy(1); mst.uz_prev = Uz(1);
mst.a_cmd_prev = T*cinc/masa - p.g;
mst.a_ff_prev = a_ff0;
al = Ts/m.tau_att;
mst.att_model = mst.att_model + al*([Ux(1) Uy(1)] - mst.att_model);

status = 0;
if rx > m.tol_kkt || ry > m.tol_kkt || rz > 10*m.tol_kkt
    status = 1;
end
u = [T; roll_c; pitch_c; yaw_rate; status];
info.res = [rx ry rz];
info.d_hat = mst.d_hat;
info.a_ff = a_ff0;
info.T_vert = T_vert;
end

% =========================================================================
function [U, res] = fista(H, f, lb, ub, U0, L, nit)
% FISTA con proyección en caja, reinicio adaptativo e iteraciones fijas.
U = min(max(U0, lb), ub);
Y = U;
t = 1;
for k = 1:nit
    G = H*Y + f;
    Un = min(max(Y - G/L, lb), ub);
    if (G'*(Un - U)) > 0      % reinicio adaptativo (O'Donoghue & Candes)
        t = 1;
        Y = Un;
    else
        tn = (1 + sqrt(1 + 4*t*t))/2;
        Y = Un + ((t - 1)/tn)*(Un - U);
        t = tn;
    end
    U = Un;
end
G = H*U + f;
res = max(abs(U - min(max(U - G/L, lb), ub)));
end
