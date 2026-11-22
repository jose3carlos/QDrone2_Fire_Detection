function M = qd2_sim_apilado(op)
%QD2_SIM_APILADO Escenario de vuelo apilado (versión MPC) bajo downwash.
%   M = qd2_sim_apilado(op): el dron de abajo (R) mantiene hover en
%   (0, 0, z_bajo); el de arriba (E) cruza por encima a z_alto, se queda
%   op.t_encima s justo encima y sigue. Ambos con qd2_onboard_step.
%   op.modo_bajo: 2 = MPC (por defecto), 1 = PID original.
%   op.usar_ff, op.usar_obs: banderas del MPC del dron de abajo.
%   op.dw_factor: downwash real / modelo (desajuste de modelo).
%   M.err_max : max |z - z_bajo| del dron de abajo mientras E está a
%               menos de 0.6 m en horizontal; M.err_rms igual en RMS.
if nargin < 1, op = struct(); end
d = struct('modo_bajo', 2, 'usar_ff', 1, 'usar_obs', 1, 'dw_factor', 1.3, ...
    'z_bajo', 0.5, 'z_alto', 1.5, 't_encima', 6, 'seed', 3, 'dt', 0.02);
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(op, f{i}), op.(f{i}) = d.(f{i}); end
end
rng(op.seed);
p = qd2_params();
pb = p; pb.mpc.usar_ff = op.usar_ff; pb.mpc.usar_obs = op.usar_obs;
dwr = p.dw; dwr.F0 = p.dw.F0*op.dw_factor;
dt = op.dt;
% ya en hover
X = zeros(10,2);
X(:,1) = [0; 0; op.z_bajo; 0; 0; 0; 0; 0; 0; p.T_trim];
X(:,2) = [-1.3; 0; op.z_alto; 0; 0; 0; 0; 0; 0; p.T_trim];
ob = [qd2_onboard_init(p), qd2_onboard_init(p)];
for i = 1:2
    ob(i).fsm = 5;
    ob(i).mst = qd2_mpc_init(p);
end
% trayectoria del de arriba: -1.3 -> 0 (espera) -> 1.3 con rate limiter 1/3 m/s
T = 10 + 1.3*3 + op.t_encima + 1.3*3 + 8;
n = round(T/dt);
ref2 = [-1.3 0 op.z_alto 0];
rate = [p.pid.rate_xy p.pid.rate_xy p.pid.rate_z p.pid.rate_yaw];
M.t = (1:n)*dt; M.z = zeros(1,n); M.dh = zeros(1,n); M.F = zeros(1,n); M.dhat = zeros(1,n);
t_llega = 10 + 1.3*3;
for k = 1:n
    t = k*dt;
    if t < 10
        g2 = [-1.3 0 op.z_alto 0];
    elseif t < t_llega + op.t_encima
        g2 = [0 0 op.z_alto 0];
    else
        g2 = [1.3 0 op.z_alto 0];
    end
    ref2 = qd2_rate_limit(ref2, g2, rate, dt);
    refs = [0 0 op.z_bajo 0; ref2];
    for i = 1:2
        o = 3 - i;
        xe = [X(1:3,i) + 0.002*randn(3,1); X(4:6,i) + 0.01*randn(3,1); X(7:9,i)];
        if i == 1
            modo = op.modo_bajo; pp = pb;
        else
            modo = 2; pp = p;
        end
        [u, ob(i)] = qd2_onboard_step(ob(i), 1, 1, refs(i,:), xe, [X(1:3,o); X(4:6,o)], pp, dt, modo);
        [F, sg] = qd2_downwash(X(1:3,i), X(1:3,o), dwr);
        X(:,i) = qd2_quad_plant(X(:,i), u, F + sg.*randn(3,1), dt, p.planta);
        if i == 1
            M.F(k) = F(3);
            M.dhat(k) = ob(1).mst.d_hat;
        end
    end
    M.z(k) = X(3,1);
    M.dh(k) = norm(X(1:2,1) - X(1:2,2));
    M.dz(k) = X(3,2) - X(3,1);
end
v = M.dh < 0.6 & M.t > 5;
e = M.z(v) - op.z_bajo;
M.err_max = max(abs(e));
M.err_rms = sqrt(mean(e.^2));
M.F_min = min(M.F);
M.dz_min = min(M.dz(v));
M.estable = all(isfinite(M.z)) && max(abs(M.z - op.z_bajo)) < 0.5;
end
