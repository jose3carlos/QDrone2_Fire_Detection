function O = sim3d_mallas_objetos(raiz, usar_stl)
%SIM3D_MALLAS_OBJETOS Mallas de los fuegos y de la toma (válvula) en metros.
%   O = sim3d_mallas_objetos(raiz, usar_stl)
%   Si existen los STL de impresion3d/ (piezas reales del proyecto) se
%   cargan con stlread y se reducen con reducepatch; el resultado se guarda
%   en tempdir para que la siguiente corrida arranque rápido. La caché se
%   invalida sola si cambia la fecha o el tamaño de algún STL. Si no hay STL
%   (o usar_stl = false) se usan figuras simples.
%
%   O.fuego.A/B/C : struct array de piezas {V, F, color}
%   O.toma        : struct array de piezas (válvula armada, tubo a lo largo de +x)
%   O.origen      : 'stl' o 'primitivas'

if nargin < 2, usar_stl = true; end
dir_stl = fullfile(raiz, 'impresion3d', 'stl');
dir_uso = fullfile(raiz, 'impresion3d', 'renders', '_mallas_uso');
rojo = [0.85 0.12 0.08]; amarillo = [1.0 0.80 0.10]; gris = [0.6 0.6 0.62];
O = [];
if usar_stl && exist(fullfile(dir_stl, 'fuego_A_rojo.stl'), 'file')
    cache = fullfile(tempdir, 'qdrone2_sim3d_mallas_v2.mat');
    firma = firma_stl(dir_stl, dir_uso);
    if exist(cache, 'file')
        try
            S = load(cache, 'O', 'firma');
            if isfield(S, 'firma') && strcmp(S.firma, firma)
                O = S.O;
            end
        catch
            O = [];
        end
    end
    if isempty(O)
        try
            O.origen = 'stl';
            for v = 'ABC'
                P = pieza(fullfile(dir_stl, sprintf('fuego_%s_rojo.stl', v)), 2500, rojo, eye(3), [0 0 0]);
                P(2) = pieza(fullfile(dir_stl, sprintf('fuego_%s_amarillo.stl', v)), 1500, amarillo, eye(3), [0 0 0]);
                O.fuego.(v) = P;
            end
            % válvula armada (marco de uso: tubo a lo largo de +y, centro del tubo en z = 13.35 mm).
            % Se gira -90 grados en z para que el tubo quede a lo largo de +x (yaw del tramo).
            Rz = [0 1 0; -1 0 0; 0 0 1];
            T = pieza(fullfile(dir_uso, 'cuerpo.stl'), 2000, rojo, Rz, [0 0 0.004]);
            if exist(fullfile(dir_uso, 'vastago.stl'), 'file')
                T(2) = pieza(fullfile(dir_uso, 'vastago.stl'), 2500, rojo*0.9, Rz, [0 0 0.004]);
                T(3) = pieza(fullfile(dir_uso, 'tuerca.stl'), 600, rojo*0.8, Rz, [0 0 0.004]);
                T(4) = pieza(fullfile(dir_uso, 'zapata_balero.stl'), 600, rojo*0.8, Rz, [0 0 0.004]);
                T(5) = pieza(fullfile(dir_uso, 'balero608.stl'), 400, gris, Rz, [0 0 0.004]);
            end
            O.toma = T;
            save(cache, 'O', 'firma');
        catch err
            warning('sim3d:stl', 'No se pudieron cargar los STL (%s); uso figuras simples.', err.message);
            O = [];
        end
    end
end
if isempty(O)
    O = primitivas(rojo, amarillo, gris);
end
end

% -------------------------------------------------------------------------
function f = firma_stl(dir_stl, dir_uso)
% nombre, tamaño y fecha de todos los STL que se usan (clave de la caché)
L = [dir(fullfile(dir_stl, 'fuego_*.stl')); dir(fullfile(dir_uso, '*.stl'))];
f = '';
for k = 1:numel(L)
    f = [f sprintf('%s|%d|%.6f;', L(k).name, L(k).bytes, L(k).datenum)]; %#ok<AGROW>
end
end

function P = pieza(ruta, n_caras, color, R, d)
T = stlread(ruta);
V = double(T.Points)/1000;              % mm -> m
F = double(T.ConnectivityList);
if size(F, 1) > n_caras
    [F, V] = reducepatch(F, V, n_caras);
end
V = (R*V.').' + d;
P.V = V; P.F = F; P.color = color;
end

function O = primitivas(rojo, amarillo, gris)
O.origen = 'primitivas';
for v = 'ABC'
    h = struct('A', 0.112, 'B', 0.125, 'C', 0.096);
    [V, F] = cono(0.048, 0.055, 7);
    P = struct('V', V, 'F', F, 'color', rojo);
    [V, F] = cono(0.017, h.(v), 12);
    P(2) = struct('V', V, 'F', F, 'color', amarillo);
    O.fuego.(v) = P;
end
% cuerpo en C + volante
[V, F] = caja([0 0 0.06], [0.03 0.074 0.11]);
T = struct('V', V, 'F', F, 'color', rojo);
[V, F] = cono(0.033, 0.004, 20);  V(:,3) = V(:,3) + 0.13;
T(2) = struct('V', V, 'F', F, 'color', rojo*0.9);
[V, F] = caja([0 0 0.012], [0.02 0.02 0.02]);
T(3) = struct('V', V, 'F', F, 'color', gris);
O.toma = T;
end

function [V, F] = cono(r, h, n)
% cono con n lados (base en z = 0)
t = (0:n-1)'*2*pi/n;
V = [r*cos(t) r*sin(t) zeros(n,1); 0 0 h; 0 0 0];
i = (1:n)'; j = [2:n 1]';
F = [i j (n+1)*ones(n,1); j i (n+2)*ones(n,1)];
end

function [V, F] = caja(c, s)
h = s/2;
V = [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1].*h + c;
Q = [1 2 3 4; 5 8 7 6; 1 5 6 2; 2 6 7 3; 3 7 8 4; 4 8 5 1];
F = [Q(:, [1 2 3]); Q(:, [1 3 4])];
end
