function [wp, n] = qd2_track_waypoints(vertices, spacing, z, n_vertices)
%QD2_TRACK_WAYPOINTS Densifica la polilínea de la N en waypoints [x y z yaw].
%   [wp, n] = qd2_track_waypoints(vertices, spacing, z)
%   [wp, n] = qd2_track_waypoints(vertices, spacing, z, n_vertices)
%
%   vertices   : matriz Vx2 [x y] (filas extra con NaN o fuera de
%                n_vertices se ignoran). Tamaño fijo recomendado 16x2.
%   spacing    : espaciado máximo entre waypoints (m).
%   z          : altura de vuelo (m).
%   n_vertices : número de vértices válidos (opcional; por defecto todas las
%                filas finitas consecutivas desde la primera).
%
%   wp : 128x4 de tamaño fijo [x y z yaw]; solo las primeras n filas son
%        válidas (el resto repite el último waypoint). yaw = 0 siempre (el
%        original no envuelve el yaw; conviene mantenerlo cerca de 0).
%   n  : número de waypoints válidos (incluye el primer vértice). Si con el
%        espaciado pedido no caben en 128, el espaciado se agranda solo.
%#codegen

MAXWP = 128;
wp = zeros(MAXWP, 4);
n = 0;

nv_tot = size(vertices, 1);
if nargin < 4
    nv = 0;
    for k = 1:nv_tot
        if all(isfinite(vertices(k,1:2)))
            nv = k;
        else
            break;
        end
    end
else
    nv = min(max(floor(n_vertices), 0), nv_tot);
end
if nv < 1
    return;
end
if ~(spacing > 0)
    spacing = 0.25;
end

% Longitud total para garantizar que quepa en MAXWP
Ltot = 0;
for k = 2:nv
    Ltot = Ltot + norm(vertices(k,1:2) - vertices(k-1,1:2));
end
% Cada segmento aporta ceil(L/spacing) puntos (+1 por el primero); como
% ceil agrega a lo más 1 por segmento, se reserva (nv-1) de holgura.
disponible = MAXWP - 1 - (nv - 1);
if Ltot/spacing > disponible
    spacing = Ltot/disponible;
end

n = 1;
wp(1,:) = [vertices(1,1), vertices(1,2), z, 0];
for k = 2:nv
    a = vertices(k-1,1:2);
    b = vertices(k,1:2);
    L = norm(b - a);
    if L < 1e-9
        continue;
    end
    m = ceil(L/spacing - 1e-9);
    for j = 1:m
        if n < MAXWP
            s = j/m;
            n = n + 1;
            wp(n,:) = [a(1) + s*(b(1)-a(1)), a(2) + s*(b(2)-a(2)), z, 0];
        end
    end
end
for k = n+1:MAXWP
    wp(k,:) = wp(n,:);
end
end
