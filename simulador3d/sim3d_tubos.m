function varargout = sim3d_tubos(accion, varargin)
%SIM3D_TUBOS Tubos REALES de la pista y distancia a ellos, con las reglas de web/app/geo.py.
%
%   T = sim3d_tubos('leer', pista)        tubos_pista(pista): cell 1xK de polilíneas Nx2 (N >= 2)
%   S = sim3d_tubos('sin_tubo', pista)    tramos_sin_tubo(pista): vector fila ordenado (base 1)
%   P = sim3d_tubos('puntos', val)        _puntos(val): Nx2 con los pares [x y] válidos
%   d = sim3d_tubos('dist', x, y, T)      dist_tubos: distancia horizontal (m) al tubo más cercano;
%                                         [] si no hay tubos (la estación no filtra)
%   d = sim3d_tubos('dist_polilinea', x, y, V)   distancia a la polilínea V ([] con < 2 puntos)
%   [p, dir, L] = sim3d_tubos('punto', T, k, s)  punto a s m del inicio del tubo k (recortado a
%                                         [0, L]), dirección unitaria del tramo y largo del tubo
%
%   pista.vertices es la RUTA DE VUELO (la compila Simulink); pista.tubos (opcional) son los tubos
%   reales en el piso, una polilínea por pieza. Si pista.tubos trae al menos una polilínea válida
%   (>= 2 puntos [x y] numéricos) se usan esas; si no, la ruta partida en pista.tramos_sin_tubo
%   (tramo k = vértice k -> k+1). Simulink no lee nada de esto.
%
%   pista viene de jsondecode (sim3d_leer_json) o de un struct de MATLAB: una lista de polilíneas
%   puede llegar como cell (largos distintos o tipos mezclados) o como arreglo KxNx2 (todas del
%   mismo largo); una polilínea como matriz Nx2 o como cell de puntos. Se aceptan los mismos
%   valores que float()/int() de Python en geo.py: números, true/false y texto numérico.

switch accion
    case 'leer'
        varargout{1} = tubos_pista(varargin{:});
    case 'sin_tubo'
        varargout{1} = tramos_sin_tubo(varargin{:});
    case 'puntos'
        varargout{1} = puntos(varargin{:});
    case 'dist'
        varargout{1} = dist_tubos(varargin{:});
    case 'dist_polilinea'
        varargout{1} = dist_polilinea(varargin{:});
    case 'punto'
        [varargout{1:3}] = punto_en_tubo(varargin{:});
    otherwise
        error('sim3d:tubos', 'Acción desconocida: %s', accion);
end
end

% =========================================================================
function T = tubos_pista(pista)
% geo.tubos_pista
T = {};
if ~isstruct(pista), pista = struct(); end
if isfield(pista, 'tubos')
    for q = lista(pista.tubos)
        P = puntos(q{1});
        if size(P, 1) >= 2
            T{end+1} = P; %#ok<AGROW>
        end
    end
    if ~isempty(T), return; end
end
V = zeros(0, 2);
if isfield(pista, 'vertices'), V = puntos(pista.vertices); end
sin = tramos_sin_tubo(pista);
cur = zeros(0, 2);
for k = 1:size(V, 1) - 1
    if any(sin == k)
        if size(cur, 1) >= 2, T{end+1} = cur; end %#ok<AGROW>
        cur = zeros(0, 2);
        continue;
    end
    if isempty(cur), cur = V(k, :); end
    cur(end+1, :) = V(k + 1, :); %#ok<AGROW>
end
if size(cur, 1) >= 2, T{end+1} = cur; end
end

function L = lista(val)
% elementos de pista.tubos (una "lista" de Python) como cell 1xK; algo que no es lista -> {}
L = {};
if iscell(val)
    L = reshape(val, 1, []);
elseif (isnumeric(val) || islogical(val)) && ndims(val) == 3
    % jsondecode de polilíneas del mismo largo: K x N x 2
    for k = 1:size(val, 1)
        L{end+1} = reshape(val(k, :, :), size(val, 2), size(val, 3)); %#ok<AGROW>
    end
elseif (isnumeric(val) || islogical(val)) && ~isempty(val)
    % Nx2: una lista de PUNTOS, no de polilíneas: cada fila es una lista de números -> inválida
    L = num2cell(double(val), 2).';
    L = cellfun(@(r) num2cell(r), L, 'UniformOutput', false);
end
end

function S = tramos_sin_tubo(pista)
% geo.tramos_sin_tubo: int(k) >= 1 (int() trunca; texto entero vale; null, texto no entero, NaN se ignoran)
S = zeros(1, 0);
if ~isstruct(pista) || ~isfield(pista, 'tramos_sin_tubo'), return; end
v = pista.tramos_sin_tubo;
if isnumeric(v) || islogical(v)
    v = num2cell(double(v(:)).');
elseif ischar(v) || isstring(v)
    v = {char(v)};                     % texto suelto: Python itera sus caracteres
    v = num2cell(v{1});
elseif ~iscell(v)
    return;
end
for i = 1:numel(v)
    a = v{i};
    k = NaN;
    if (isnumeric(a) || islogical(a)) && isscalar(a) && isreal(a)
        k = fix(double(a));
    elseif (ischar(a) || isstring(a)) && ~isempty(regexp(char(a), '^\s*[+-]?\d+\s*$', 'once'))
        k = str2double(char(a));
    end
    if isfinite(k) && k >= 1
        S(end+1) = k; %#ok<AGROW>
    end
end
S = unique(S);
end

function P = puntos(val)
% geo._puntos: [float(p[0]), float(p[1])] de cada elemento; los que fallan se ignoran
P = zeros(0, 2);
if isnumeric(val) || islogical(val)
    if ismatrix(val) && size(val, 2) >= 2 && ~isempty(val)
        P = double(val(:, 1:2));
    end
    return;
end
if ~iscell(val), return; end
for i = 1:numel(val)
    p = val{i};
    if (isnumeric(p) || islogical(p)) && numel(p) >= 2
        P(end+1, :) = double([p(1) p(2)]); %#ok<AGROW>
    elseif iscell(p) && numel(p) >= 2
        a = num(p{1}); b = num(p{2});
        if ~isempty(a) && ~isempty(b)
            P(end+1, :) = [a b]; %#ok<AGROW>
        end
    end
end
end

function a = num(v)
% float(v) de Python para un elemento suelto ([] si falla)
a = [];
if (isnumeric(v) || islogical(v)) && isscalar(v) && isreal(v)
    a = double(v);
elseif ischar(v) || isstring(v)
    a = str2double(char(v));
    if isnan(a) && ~strcmpi(strtrim(char(v)), 'nan'), a = []; end
end
end

function d = dist_tubos(x, y, T)
% geo.dist_tubos
d = [];
for k = 1:numel(T)
    e = dist_polilinea(x, y, T{k});
    if ~isempty(e) && (isempty(d) || e < d)
        d = e;
    end
end
end

function d = dist_polilinea(x, y, V)
% geo.dist_polilinea sin tramos excluidos
d = [];
V = puntos(V);
for k = 1:size(V, 1) - 1
    a = V(k, :); s = V(k + 1, :) - a;
    L = s*s.';
    if L < 1e-12
        t = 0;
    else
        t = min(1, max(0, ((x - a(1))*s(1) + (y - a(2))*s(2))/L));
    end
    e = hypot(a(1) + t*s(1) - x, a(2) + t*s(2) - y);
    if isempty(d) || e < d
        d = e;
    end
end
end

function [p, dir, Ltot] = punto_en_tubo(T, k, s)
% punto a s m del inicio del tubo k a lo largo de la polilínea (para los escenarios)
if k < 1 || k > numel(T) || k ~= fix(k)
    error('sim3d:tubos', 'No hay tubo %g (la pista tiene %d tubos).', k, numel(T));
end
P = T{k};
seg = diff(P, 1, 1);
ls = sqrt(sum(seg.^2, 2));
acum = [0; cumsum(ls)];
Ltot = acum(end);
s = min(max(s, 0), Ltot);
i = find(acum(1:end-1) <= s & ls > 0, 1, 'last');
if isempty(i), i = 1; end
u = (s - acum(i))/max(ls(i), 1e-12);
p = P(i, :) + u*seg(i, :);
dir = seg(i, :)/max(ls(i), 1e-12);
end
