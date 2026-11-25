function M = sim3d_marco(tubos, ruta, homes, lim)
%SIM3D_MARCO Marco de la arena: piso y retícula sobre los que se dibujan los tubos.
%   M = sim3d_marco(tubos, ruta, homes, lim)  ->  M = [x0 x1 y0 y1] (m)
%
%   Misma regla que web/app/static/marco.js (vista 3D y mapa 2D de la estación):
%   caja que contiene los tubos REALES (cell de polilíneas Nx2, C.tubos / esc.tubos), la ruta de
%   vuelo (pista.vertices, Nx2), los homes de los drones (Kx2, [] si no se conocen) y el
%   rectángulo mision.limites_xy (lim = [lo hi] para x e y, p.xy_lim); se le suma MARGEN por
%   lado y se redondea HACIA AFUERA a múltiplos de PASO (el origen OptiTrack cae siempre en una
%   línea de la retícula). Sin datos: el marco ±1.5 de siempre.
%   Retícula: líneas finas cada 0.25 m, marcadas cada 0.5 m y más marcadas cada 1 m. limites_xy
%   se dibuja aparte como "límite de vuelo" (punteado ámbar), distinto del borde del piso.

MARGEN = 0.25;      % m
PASO = 0.5;         % m
COORD_MAX = 10;     % m; un punto más lejos es basura (como marco.js)

P = zeros(0, 2);
if nargin >= 1 && ~isempty(tubos)
    if ~iscell(tubos), tubos = {tubos}; end
    for i = 1:numel(tubos)
        if size(tubos{i}, 2) == 2, P = [P; double(tubos{i})]; end %#ok<AGROW>
    end
end
if nargin >= 2 && size(ruta, 2) == 2, P = [P; double(ruta)]; end
if nargin >= 3 && size(homes, 2) == 2, P = [P; double(homes)]; end
if nargin >= 4 && numel(lim) == 2 && all(isfinite(lim)) && lim(1) < lim(2) && all(abs(lim) <= COORD_MAX)
    P = [P; lim(1) lim(1); lim(2) lim(2)];
end
P = P(all(isfinite(P), 2) & all(abs(P) <= COORD_MAX, 2), :);
if isempty(P)
    M = [-1.5 1.5 -1.5 1.5];
    return;
end
% 1e-9: que un borde que ya cae en un múltiplo (p. ej. -1.75 - 0.25) no salte un paso por redondeo
abajo = @(v) floor((v - MARGEN)/PASO + 1e-9)*PASO;
arriba = @(v) ceil((v + MARGEN)/PASO - 1e-9)*PASO;
M = [abajo(min(P(:,1))) arriba(max(P(:,1))) abajo(min(P(:,2))) arriba(max(P(:,2)))];
end
