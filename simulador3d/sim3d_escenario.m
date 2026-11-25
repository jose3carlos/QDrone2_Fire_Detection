function esc = sim3d_escenario(opc, C)
%SIM3D_ESCENARIO Objetos (fuegos y tomas) y home de los drones.
%   esc = sim3d_escenario(opc, C)
%   opc.escenario = ruta a un JSON (relativa a simulador3d/), 'aleatorio' o 'web'.
%   'web' = los mismos objetos que dibuja la cámara SINTÉTICA de la UI web
%   (anomalias_mock de web/app/mock_world.py: config web.mock_anomalias, o si
%   no existe, default_anomalies: con pista.tubos válidos, SOBRE los tubos
%   reales (toma, fuego, ... a 35/75 % de cada pieza, o 22/52/83 % con una
%   sola); sin ellos, fuego, toma, fuego a 22/52/83 % de la ruta, a 8 cm). Así,
%   con use_vision_python = true, los objetivos que publica la estación Python
%   corresponden a los objetos que se ven en el simulador.
%
%   Formato del JSON (ver escenarios/escenario_base.json):
%     { "homes": [[x1, y1], [x2, y2]],          (dron 1 y dron 2, m)
%       "yaw0_deg": [0, 0],
%       "objetos": [ {"clase": "fuego", "x": 0.45, "y": 0.35, "yaw_deg": 0, "variante": "A"},
%                    {"clase": "toma",  "x": 0.30, "y": 1.20, "yaw_deg": 0},
%                    {"clase": "toma",  "tubo": 1, "s_m": 1.25},
%                    {"clase": "fuego", "tubo": 2, "s": 0.8, "lado_m": -0.08, "variante": "B"} ],
%       "aleatorio": {"fuegos": 2, "tomas": 2, "semilla": 7} }   (si no hay "objetos")
%
%   Un objeto con "tubo" (k, base 1) se coloca SOBRE el tubo real k de la config (C.tubos =
%   pista.tubos o la ruta partida en pista.tramos_sin_tubo, como geo.tubos_pista) en lugar de
%   x/y: a "s_m" metros de su primer punto (o a la fracción "s" de su largo), desplazado
%   "lado_m" m a la izquierda del tubo (negativo = derecha). Sin "yaw_deg", una toma queda
%   orientada con el tubo. Así el escenario sigue a la pista de config/estacion.json sin copiarla.
%
%   esc.obj : struct array con clase (0 fuego, 1 toma), x, y, yaw, variante
%             ('A','B','C'), h (altura aprox., m) y r (radio aprox., m).
%   esc.home: 2x2 [x y] del dron 1 y del dron 2.  esc.yaw0: 1x2 (rad).
%   esc.V   : ruta de vuelo (pista.vertices, Nx2).  esc.nombre: texto.
%   esc.tubos: tubos reales (cell de polilíneas Nx2, C.tubos); los objetos del modo 'aleatorio'
%             se colocan sobre ellos. Sin tubos en la config (C.tubos vacío: la estación no
%             filtra) es la ruta {esc.V}, solo para dibujar y colocar objetos.

aqui = fileparts(mfilename('fullpath'));
p = C.p;
esc.V = p.vertices(1:p.n_vertices, :);
esc.tubos = {esc.V};
% sin tubos en la config (ni pista.tubos ni pista.vertices): C.tubos = {} como la estación (no
% filtra); para dibujar y colocar objetos se usa la ruta (la N por defecto de C.p)
if isfield(C, 'tubos') && ~isempty(C.tubos), esc.tubos = C.tubos; end
esc.home = [-1.0 0.0; 1.0 -0.3];
esc.yaw0 = [0 0];
esc.nombre = 'aleatorio';
js = struct();
n_f = opc.n_fuegos; n_t = opc.n_tomas; semilla = opc.semilla;

es_web = (ischar(opc.escenario) || isstring(opc.escenario)) && strcmpi(char(opc.escenario), 'web');
if es_web
    js.objetos = objetos_web(C.cfg, esc.V);
    esc.nombre = 'web (cámara sintética de la UI)';
elseif ~(ischar(opc.escenario) || isstring(opc.escenario)) || ~strcmpi(char(opc.escenario), 'aleatorio')
    ruta = char(opc.escenario);
    if ~exist(ruta, 'file')
        ruta = fullfile(aqui, ruta);
    end
    if ~exist(ruta, 'file')
        error('sim3d:escenario', 'No encuentro el escenario "%s".', char(opc.escenario));
    end
    js = jsondecode(fileread(ruta));
    [~, esc.nombre] = fileparts(ruta);
    if isfield(js, 'aleatorio')
        a = js.aleatorio;
        if isfield(a, 'fuegos'), n_f = a.fuegos; end
        if isfield(a, 'tomas'), n_t = a.tomas; end
        if isfield(a, 'semilla') && isempty(opc.semilla), semilla = a.semilla; end
    end
end
if isfield(js, 'homes') && isequal(size(js.homes), [2 2])
    esc.home = double(js.homes);
end
if isfield(js, 'yaw0_deg') && numel(js.yaw0_deg) == 2
    esc.yaw0 = deg2rad(double(js.yaw0_deg(:).'));
end
if ~isempty(opc.homes)
    esc.home = double(opc.homes);
end

proto = struct('clase', 0, 'x', 0, 'y', 0, 'yaw', 0, 'variante', 'A', 'h', 0.11, 'r', 0.05);
obj = proto([]);
if isfield(js, 'objetos') && ~isempty(js.objetos)
    L = js.objetos;
    if ~iscell(L), L = num2cell(L); end   % jsondecode da cell si los objetos tienen campos distintos
    for i = 1:numel(L)
        e = L{i};
        o = proto;
        o.clase = double(strcmpi(e.clase, 'toma') || strcmpi(e.clase, 'valvula'));
        if isfield(e, 'tubo') && ~isempty(e.tubo)
            o = sobre_tubo(o, e, esc.tubos);
        else
            o.x = double(e.x); o.y = double(e.y);
        end
        if isfield(e, 'yaw_deg') && ~isempty(e.yaw_deg), o.yaw = deg2rad(double(e.yaw_deg)); end
        if isfield(e, 'variante') && ~isempty(e.variante), o.variante = upper(char(e.variante)); end
        obj(end+1) = completar(o); %#ok<AGROW>
    end
else
    if isempty(semilla)
        % distinta en cada corrida (flujo aparte: no depende de rng global)
        semilla = randi(RandStream('mt19937ar', 'Seed', 'shuffle'), 1e6);
    end
    esc.nombre = sprintf('aleatorio (semilla %d)', semilla);
    obj = aleatorio(esc.tubos, n_f, n_t, semilla, esc.home, proto);
end
esc.obj = obj;
esc.semilla = semilla;
end

% -------------------------------------------------------------------------
function o = sobre_tubo(o, e, tubos)
% objeto colocado sobre el tubo real e.tubo (ver la cabecera)
k = double(e.tubo);
[~, ~, Lt] = sim3d_tubos('punto', tubos, k, 0);
if isfield(e, 's_m') && ~isempty(e.s_m)
    sm = double(e.s_m);
elseif isfield(e, 's') && ~isempty(e.s)
    sm = double(e.s)*Lt;
else
    sm = 0.5*Lt;
end
[q, d] = sim3d_tubos('punto', tubos, k, sm);
lado = 0;
if isfield(e, 'lado_m') && ~isempty(e.lado_m), lado = double(e.lado_m); end
q = q + lado*[-d(2) d(1)];
o.x = q(1); o.y = q(2);
if o.clase == 1, o.yaw = atan2(d(2), d(1)); end
end

function L = objetos_web(cfg, V)
% Objetos de la cámara sintética de la UI web (web/app/mock_world.py).
L = {};
if isfield(cfg, 'web') && isstruct(cfg.web) && isfield(cfg.web, 'mock_anomalias') ...
        && ~isempty(cfg.web.mock_anomalias)
    A = cfg.web.mock_anomalias;
    if ~iscell(A), A = num2cell(A); end
    for i = 1:numel(A)
        L{end+1} = struct('clase', ternario(double(A{i}.cls) == 1, 'toma', 'fuego'), ...
            'x', double(A{i}.x), 'y', double(A{i}.y), 'yaw_deg', 0, 'variante', 'A'); %#ok<AGROW>
    end
    return;
end
% default_anomalies(vertices, tubos_reales(pista)): con pista.tubos válidos (solo esos; no la ruta
% partida en los huecos), en el eje de cada tubo a 0.35/0.75 de su largo (0.22/0.52/0.83 con una sola
% pieza), clases alternadas empezando por toma ((n + 1) % 2)
T = {};
if isfield(cfg, 'pista') && isstruct(cfg.pista) && isfield(cfg.pista, 'tubos')
    solo_tubos.tubos = cfg.pista.tubos;
    T = sim3d_tubos('leer', solo_tubos);
end
if ~isempty(T)
    fr = [0.22 0.52 0.83];
    if numel(T) >= 2, fr = [0.35 0.75]; end
    for i = 1:numel(T)
        for f = fr
            [q, d] = punto_fraccion(T{i}, f);
            q = round(q*1000)/1000;                    % Python redondea a mm
            es_toma = mod(numel(L) + 1, 2) == 1;
            L{end+1} = struct('clase', ternario(es_toma, 'toma', 'fuego'), 'x', q(1), 'y', q(2), ...
                'yaw_deg', rad2deg(atan2(d(2), d(1))), 'variante', 'A'); %#ok<AGROW>
        end
    end
    return;
end
% default_anomalies(vertices): fracciones 0.22, 0.52, 0.83; clase k % 2 (0 fuego, 1 toma);
% 8 cm a la izquierda del tramo (k par) o a la derecha (k impar)
seg = diff(V, 1, 1);
lseg = sqrt(sum(seg.^2, 2));
acum = [0; cumsum(lseg)];
fr = [0.22 0.52 0.83];
vv = 'AB';
for k = 0:2
    sa = fr(k+1)*acum(end);
    i = find(acum(1:end-1) <= sa, 1, 'last');       % igual que searchsorted(..., 'right') - 1
    i = min(i, numel(lseg));
    a = (sa - acum(i))/lseg(i);
    q = V(i, :) + a*seg(i, :);
    d = seg(i, :)/lseg(i);
    nrm = [-d(2) d(1)];
    lado = 0.08*(1 - 2*mod(k, 2));
    q = round((q + lado*nrm)*1000)/1000;           % Python redondea a mm
    L{end+1} = struct('clase', ternario(mod(k, 2) == 1, 'toma', 'fuego'), 'x', q(1), 'y', q(2), ...
        'yaw_deg', rad2deg(atan2(d(2), d(1))), 'variante', vv(1 + (k == 2))); %#ok<AGROW>
end
end

function [q, d] = punto_fraccion(V, f)
% path_points de mock_world.py: punto a la fracción f del largo de la polilínea V y dirección del tramo
seg = diff(V, 1, 1);
lseg = sqrt(sum(seg.^2, 2));
acum = [0; cumsum(lseg)];
sa = min(max(f, 0), 1)*acum(end);
i = find(acum <= sa, 1, 'last');                % searchsorted(..., 'right') - 1
i = min(i, numel(lseg));
a = 0;
if lseg(i) > 0, a = (sa - acum(i))/lseg(i); end
q = V(i, :) + a*seg(i, :);
d = seg(i, :)/max(lseg(i), eps);
if lseg(i) == 0, d = seg(i, :); end
end

function r = ternario(c, a, b)
if c, r = a; else, r = b; end
end

function o = completar(o)
if o.clase == 0
    alt = struct('A', 0.112, 'B', 0.125, 'C', 0.096);
    if ~isfield(alt, o.variante), o.variante = 'A'; end
    o.h = alt.(o.variante); o.r = 0.05;
else
    o.h = 0.137; o.r = 0.04; o.variante = '-';
end
end

function obj = aleatorio(tubos, n_f, n_t, semilla, home, proto)
% Coloca los objetos sobre los tubos reales (C.tubos; sin pista.tubos ni huecos = la N entera):
% las tomas abrazan el tubo (orientadas con el tramo) y los fuegos quedan junto al tubo (a un
% lado, base de ~96 mm). Los tramos de todas las piezas se recorren uno tras otro.
st = rng;
rng(semilla);
V0 = zeros(0, 2); seg = zeros(0, 2);
for i = 1:numel(tubos)
    P = tubos{i};
    V0 = [V0; P(1:end-1, :)]; %#ok<AGROW>
    seg = [seg; diff(P, 1, 1)]; %#ok<AGROW>
end
keep = sqrt(sum(seg.^2, 2)) > 0;
V0 = V0(keep, :); seg = seg(keep, :);
lseg = sqrt(sum(seg.^2, 2));
acum = [0; cumsum(lseg)];
Ltot = acum(end);
n = n_f + n_t;
clases = [zeros(1, n_f) ones(1, n_t)];
clases = clases(randperm(n));
obj = proto([]);
s_usadas = [];
intentos = 0;
k = 0;
while k < n && intentos < 2000
    intentos = intentos + 1;
    sa = 0.15 + rand*(Ltot - 0.30);
    if any(abs(s_usadas - sa) < 0.45)
        continue;
    end
    j = find(sa >= acum(1:end-1), 1, 'last');
    u = (sa - acum(j))/lseg(j);
    d = seg(j,:)/lseg(j);
    q = V0(j,:) + u*seg(j,:);
    o = proto;
    o.clase = clases(k+1);
    if o.clase == 1
        o.x = q(1); o.y = q(2); o.yaw = atan2(d(2), d(1));
    else
        nrm = [-d(2) d(1)]*sign(rand - 0.5);
        q = q + 0.075*nrm;
        o.x = q(1); o.y = q(2); o.yaw = 2*pi*rand;
        vv = 'ABC';
        o.variante = vv(randi(3));
    end
    % lejos de los home (los drones aterrizan ahí)
    if min(sqrt(sum((home - [o.x o.y]).^2, 2))) < 0.35
        continue;
    end
    k = k + 1;
    s_usadas(end+1) = sa; %#ok<AGROW>
    obj(k) = completar(o);
end
rng(st);
end
