function [lo, hi] = qd2_limites_xy(p)
%QD2_LIMITES_XY Límites EFECTIVOS de las consignas xy, por eje.
%   [lo, hi] = qd2_limites_xy(p)   lo = [x_min y_min], hi = [x_max y_max] (m)
%
%   Por defecto (p.env_x / p.env_y = [NaN NaN]) los dos ejes usan
%   p.xy_lim -/+ p.xy_margen (mismo cálculo de siempre: ±1.35 con ±1.5).
%   Con la envolvente de la config (mision.limites_x / mision.limites_y,
%   qd2_params_from_config) ese eje usa p.env_x / p.env_y TAL CUAL: es el
%   límite efectivo y el margen NO se le aplica (p. ej. y en [-1.4 1.2]).
%   Recorte de un punto [x y]:  xy = min(max(xy, lo), hi).
%#codegen

lo0 = p.xy_lim(1) + p.xy_margen;
hi0 = p.xy_lim(2) - p.xy_margen;
lo = [lo0 lo0];
hi = [hi0 hi0];
if isfinite(p.env_x(1)) && isfinite(p.env_x(2)) && p.env_x(1) < p.env_x(2)
    lo(1) = p.env_x(1);
    hi(1) = p.env_x(2);
end
if isfinite(p.env_y(1)) && isfinite(p.env_y(2)) && p.env_y(1) < p.env_y(2)
    lo(2) = p.env_y(1);
    hi(2) = p.env_y(2);
end
end
