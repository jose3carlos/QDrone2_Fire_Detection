function [F, sigma] = qd2_downwash(p_low, p_high, dw)
%QD2_DOWNWASH Fuerza de downwash sobre el dron de abajo.
%   [F, sigma] = qd2_downwash(p_low, p_high)
%   [F, sigma] = qd2_downwash(p_low, p_high, dw)
%
%   p_low  : posición [x y z] del dron que recibe el chorro.
%   p_high : posición [x y z] del otro dron. Si no está por encima de p_low,
%            la fuerza es cero (la función se puede llamar con cualquier par).
%   dw     : struct de parámetros (por defecto qd2_params().dw):
%            F0, sigma, dz_on, l_dec, dz_max, ruido_rel, ruido_xy.
%
%   F     : 3x1 fuerza determinista (N) en RSF; solo F(3) <= 0.
%   sigma : 3x1 desviación estándar (N) del ruido de turbulencia que debe
%           sumar el simulador (la función no genera números aleatorios).
%
%   Modelo: chorro gaussiano en la distancia horizontal r y decaimiento con
%   la separación vertical dz:
%       F_z = -F0 * exp(-r^2/(2 sigma_dw^2)) * (1 - exp(-dz/dz_on)) / (1 + dz/l_dec)
%   Con los valores por defecto, dz = 1.0 m y r = 0 dan ~2.9 N (~20 % del
%   peso del QDrone 2) y dz = 0.8 m da ~3.3 N.
%#codegen

if nargin < 3
    pp = qd2_params();
    dw = pp.dw;
end
F = zeros(3,1);
sigma = zeros(3,1);
dz = p_high(3) - p_low(3);
if ~(dz > 0) || dz > dw.dz_max || any(~isfinite(p_high(1:3))) || any(~isfinite(p_low(1:3)))
    return;
end
dx = p_high(1) - p_low(1);
dy = p_high(2) - p_low(2);
r2 = dx*dx + dy*dy;
fz = dw.F0 * exp(-r2/(2*dw.sigma^2)) * (1 - exp(-dz/dw.dz_on)) / (1 + dz/dw.l_dec);
F(3) = -fz;
sigma(1) = dw.ruido_xy*fz;
sigma(2) = dw.ruido_xy*fz;
sigma(3) = dw.ruido_rel*fz;
end
