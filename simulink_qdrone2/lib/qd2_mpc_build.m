function m = qd2_mpc_build(p)
%QD2_MPC_BUILD Precalcula las matrices del MPC lineal a bordo.
%   m = qd2_mpc_build(p) se llama desde qd2_params. Construye, para cada
%   eje, la predicción condensada X = Phi*x0 + Gam*U sobre el horizonte N y
%   el QP  min 1/2 U'HU + f'U  s.a.  lb <= U <= ub  que resuelve FISTA.
%
%   Eje horizontal (x e y en horizon frame, idénticos):
%       estado [p; v; th]  (posición relativa a la referencia, velocidad,
%       ángulo real), entrada th_cmd,
%       p' = v,  v' = g*th,  th' = (th_cmd - th)/tau_att.
%       Para y se usa th = -roll (a_y = -g*roll).
%   Eje vertical:
%       estado [z - z_ref; vz], entrada u = dT + masa*d (empuje "neto" que
%       ya cancela la perturbación estimada d = d_hat + a_ff), por lo que el
%       modelo es un doble integrador z'' = u/masa y las cotas de empuje
%       quedan como cotas de caja dependientes de k.
%   Costo: sum (x-r)'Q(x-r) + R u^2 + Rd (du)^2, costo terminal P de la
%   ecuación de Riccati discreta (iteración fija, compatible con codegen).

%#codegen

% Nota codegen: 'm' solo se escribe (nunca se lee mientras se construye).
Ts   = 0.02;      % s (50 Hz)
N    = 20;        % horizonte (constante: fija los tamaños en codegen)
tau  = p.planta.tau_att;
g    = p.g;
masa = p.masa;
Qxy  = [12 3 0];  % [pos vel ang]   (ajuste: tests/test_qd2_mpc.m)
Rxy  = 4;         % sobre th_cmd (rad)
Rdxy = 20;        % sobre cambios de th_cmd
Qzv  = [60 10];   % [pos vel]
Rz   = 0.05;      % sobre u (N)
Rdz  = 0.2;
w_obs = 6.0;      % rad/s, polo doble del observador de perturbación
lam0  = exp(-w_obs*Ts);
l1    = 2 - 2*lam0;
l2    = (lam0^2 - 1 + l1)/Ts;

% ---------------- Eje horizontal -----------------------------------------
Ac = [0 1 0; 0 0 g; 0 0 -1/tau];
Bc = [0; 0; 1/tau];
[Ad, Bd] = discretizar_zoh(Ac, Bc, Ts);
Q  = diag(Qxy);
P  = riccati_d(Ad, Bd, Q, Rxy);
[Hxy, Fxy] = condensar(Ad, Bd, Q, P, Rxy, Rdxy, N);
Kxy = (Rxy + Bd'*P*Bd) \ (Bd'*P*Ad);   % LQR equivalente (referencia)

% ---------------- Eje vertical -------------------------------------------
Acz = [0 1; 0 0];
Bcz = [0; 1/masa];
[Adz, Bdz] = discretizar_zoh(Acz, Bcz, Ts);
Qz = diag(Qzv);
Pz = riccati_d(Adz, Bdz, Qz, Rz);
[Hz, Fz] = condensar(Adz, Bdz, Qz, Pz, Rz, Rdz, N);
Kz = (Rz + Bdz'*Pz*Bdz) \ (Bdz'*Pz*Adz);

m.Ts      = Ts;
m.N       = N;
m.n_iter  = 40;        % iteraciones FISTA fijas por eje
m.tau_att = tau;       % s, modelo de actitud (identificado)
m.att_max = 0.35;      % rad, cota de roll/pitch
m.T_min   = 5.0;       % N, empuje mínimo permitido
m.T_max   = 30.0;      % N, empuje máximo permitido (< 43.16 físico)
m.yaw_kp  = p.pid.Kp(4);   % P de yaw como el original (7.5 (rad/s)/rad)
m.yaw_rate_max = 100*pi/180;
m.tol_kkt = 1e-3;      % residuo de gradiente proyectado para status = 0
m.Qxy  = Qxy;  m.Rxy = Rxy;  m.Rdxy = Rdxy;
m.Qz   = Qzv;  m.Rz  = Rz;   m.Rdz  = Rdz;
m.w_obs   = w_obs;
m.obs_l1  = l1;
m.obs_l2  = l2;
m.d_max   = 6.0;       % m/s^2, saturación de la perturbación estimada
m.usar_ff = 1;         % feedforward de downwash con la pose del otro dron
m.usar_obs = 1;        % observador de perturbación (offset-free)
m.usar_att_medida = 1; % 1 = usa roll/pitch estimados del dron como estado
m.masa = masa;
m.g    = g;
m.T_trim = p.T_trim;
m.Axy = Ad;  m.Bxy = Bd;  m.Pxy = P;
m.Hxy = Hxy; m.Fxy = Fxy;
m.Lxy = lambda_max(Hxy);
m.Kxy = Kxy;
m.Az = Adz;  m.Bz = Bdz;  m.Pz = Pz;
m.Hz = Hz;   m.Fz = Fz;
m.Lz = lambda_max(Hz);
m.Kz = Kz;
end

% =========================================================================
function [Ad, Bd] = discretizar_zoh(Ac, Bc, Ts)
% Discretización exacta con retenedor de orden cero (exponencial aumentada).
n = size(Ac,1);
M = zeros(n+1, n+1);
M(1:n,1:n) = Ac*Ts;
M(1:n,n+1) = Bc*Ts;
E = expm(M);
Ad = E(1:n,1:n);
Bd = E(1:n,n+1);
end

function P = riccati_d(A, B, Q, R)
% Ecuación algebraica de Riccati discreta por iteración (fija).
P = Q;
for k = 1:3000
    K = (R + B'*P*B) \ (B'*P*A);
    Pn = Q + A'*P*(A - B*K);
    Pn = 0.5*(Pn + Pn');
    if max(abs(Pn(:) - P(:))) < 1e-10*max(1, max(abs(P(:))))
        P = Pn;
        break;
    end
    P = Pn;
end
end

function [H, F] = condensar(A, B, Q, P, R, Rd, N)
% H (NxN) y F (N x n) con  f = F*x0 - Rd*u_prev*e1.
n = size(A,1);
Phi = zeros(n*N, n);
Gam = zeros(n*N, N);
Ak = eye(n);
for k = 1:N
    Ak = A*Ak;
    Phi((k-1)*n+1:k*n, :) = Ak;
    for j = 1:k
        Gam((k-1)*n+1:k*n, j) = A^(k-j)*B;
    end
end
Qb = zeros(n*N, n*N);
for k = 1:N
    if k < N
        Qb((k-1)*n+1:k*n, (k-1)*n+1:k*n) = Q;
    else
        Qb((k-1)*n+1:k*n, (k-1)*n+1:k*n) = P;
    end
end
D = eye(N) - diag(ones(N-1,1), -1);   % diferencias (fila 1: u0 - u_prev)
H = Gam'*Qb*Gam + R*eye(N) + Rd*(D'*D);
H = 0.5*(H + H');
F = Gam'*Qb*Phi;
end

function L = lambda_max(H)
% Valor propio máximo por iteración de potencia (determinista).
n = size(H,1);
v = ones(n,1)/sqrt(n);
L = 0;
for k = 1:300
    w = H*v;
    L = norm(w);
    if L <= 0
        break;
    end
    v = w/L;
end
L = 1.02*L;   % pequeño margen de seguridad para el paso 1/L
end
