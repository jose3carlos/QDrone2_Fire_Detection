function info = agregar_bloque_coordenadas(archivo, guardar)
%AGREGAR_BLOQUE_COORDENADAS Agrega al modelo de tierra el área "COORDENADAS OBJETOS (x,y)".
%   info = agregar_bloque_coordenadas()
%   info = agregar_bloque_coordenadas(archivo_slx, guardar)
%
%   Rama solo-explorador-coordenadas. Por defecto edita
%   versiones/R2022a/QD2_MissionCtrl_Dual_PID.slx (el modelo de tierra del
%   modo SOLO EXPLORADOR que se compila con QUARC en la PC del laboratorio) y
%   lo guarda en formato R2022a.
%
%   Solo AGREGA, en el nivel superior y debajo de todo lo que ya hay (la
%   posición se calcula con las Position de todos los bloques y notas):
%     - un área rotulada 'COORDENADAS OBJETOS (x,y)' con una nota;
%     - el MATLAB Function 'COORDENADAS OBJETOS (x,y)', que llama
%       [coords_objetos, n_objetos] = qd2_coordenadas_objetivos(py)
%       (lib/qd2_coordenadas_objetivos.m) y arma registro = [n; x(1:32); y(1:32)];
%       su entrada es una RAMA nueva de la línea existente
%       PYTHON BRIDGE/1 (Python Commands [32]) -> Goto Python Commands
%       (la línea original no se corta ni se redirige);
%     - 'Display coords_objetos (x,y)' (32x2: filas 1..n = [x y] en m, marco
%       de OptiTrack; el resto 0) y 'Display n objetos', para verlos en vivo
%       en external mode (Monitor & Tune);
%     - 'To Host File Coordenadas' (quarc_library, igual que los To Host File
%       que ya usa el modelo): saved_data/<modelo>/log_<fecha>_CoordenadasObjetos.mat,
%       variable coords_objetos de 66 x N (cada 100 muestras = 10 Hz):
%       fila 1 = t, fila 2 = n, filas 3-34 = x, filas 35-66 = y.
%   La señal de la salida 1 se llama 'coords_objetos'.
%   Ningún bloque, línea, nota ni parámetro existente se mueve ni se cambia
%   en el diagrama (la línea de Python Cmd solo gana un punto de rama). Ojo:
%   guardado desde una MATLAB más nueva (ExportToVersion), el ARCHIVO cambia
%   además en lo que normaliza la exportación (LibraryVersion de
%   quarc_library, InstanceData de los To Host File, parámetros del
%   configSet, alto de notas, ModelUUID; ver lib/README.md, "Coordenadas de
%   los objetos encontrados"): en el lab, guardarlo una vez en R2022a antes
%   del Build.
%
%   Idempotente: si el bloque 'COORDENADAS OBJETOS (x,y)' ya existe no
%   agrega nada ni guarda (info.agregado = false).
%
%   guardar (true por defecto): guarda el archivo en formato R2022a
%   (save_system ... 'ExportToVersion', 'R2022a' a una carpeta temporal y
%   copia encima; si esta MATLAB ya es R2022a, save_system normal). Con
%   false deja el modelo cargado y sin guardar (para inspeccionarlo).
%
%   Requisitos: MATLAB/Simulink R2022a+ con QUARC instalado (bloque To Host
%   File de quarc_library). Para que el modelo compilado lo tenga hay que
%   RECOMPILAR (Build) QD2_MissionCtrl_Dual_PID en R2022a en la PC del
%   laboratorio.

raiz = fileparts(mfilename('fullpath'));                 % .../simulink_qdrone2
if nargin < 1 || isempty(archivo)
    archivo = fullfile(raiz, 'versiones', 'R2022a', 'QD2_MissionCtrl_Dual_PID.slx');
end
if nargin < 2
    guardar = true;
end
addpath(raiz);
addpath(fullfile(raiz, 'lib'));
if isempty(which('qc_get_step_size')) && exist('C:\Program Files\Quanser\QUARC\quarc', 'dir')
    addpath('C:\Program Files\Quanser\QUARC\quarc');
end

[~, m] = fileparts(archivo);
if bdIsLoaded(m)
    error('agregar_bloque_coordenadas:cargado', ...
        'El modelo %s ya está cargado: ciérralo (close_system(''%s'', 0)) y vuelve a correr.', m, m);
end
load_system(archivo);

NOMBRE = 'COORDENADAS OBJETOS (x,y)';
F = [m '/' NOMBRE];
info = struct('archivo', archivo, 'modelo', m, 'agregado', false, 'guardado', false, ...
    'y0', NaN, 'decimation', NaN, 'bloques', {{}});
if ~isempty(find_system(m, 'SearchDepth', 1, 'Name', NOMBRE))
    fprintf('agregar_bloque_coordenadas: "%s" ya existe en %s; no se agrega nada.\n', NOMBRE, m);
    if guardar
        close_system(m, 0);
    end
    return
end

% ---- línea existente de Python Cmd: PYTHON BRIDGE/1 -> Goto Python Commands
PB = [m '/PYTHON BRIDGE'];
lh = get_param(PB, 'LineHandles');
L = lh.Outport(1);
if L < 0
    error('agregar_bloque_coordenadas:pb', 'La salida 1 de PYTHON BRIDGE no tiene línea.');
end
dst = get_param(L, 'DstBlockHandle');
if numel(dst) ~= 1 || ~strcmp(getfullname(dst), [m '/Goto Python Commands'])
    error('agregar_bloque_coordenadas:linea', ...
        'La salida 1 de PYTHON BRIDGE no va (solo) a Goto Python Commands: revisa el modelo.');
end
pts = get_param(L, 'Points');                 % [-860 810; -920 810] en el modelo del lab
if size(pts, 1) < 2 || pts(1, 2) ~= pts(2, 2)
    error('agregar_bloque_coordenadas:geom', 'El primer tramo de la línea de Python Cmd no es horizontal.');
end

% ---- "hasta abajo": debajo de TODO lo que hay en el nivel superior
b = find_system(m, 'SearchDepth', 1, 'Type', 'Block');
b = b(~strcmp(b, m));
y_max = -inf; x_min = inf;
for k = 1:numel(b)
    p = get_param(b{k}, 'Position');
    y_max = max(y_max, p(4));
    x_min = min(x_min, p(1));
end
an = find_system(m, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'Annotation');
for k = 1:numel(an)
    p = get_param(an(k), 'Position');
    y_max = max(y_max, p(4));
    x_min = min(x_min, p(1));
end
ls = find_system(m, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'Line');
for k = 1:numel(ls)
    p = get_param(ls(k), 'Points');
    y_max = max(y_max, max(p(:, 2)));
end
y0 = 10*ceil((y_max + 60)/10);
info.y0 = y0;

% columnas: rama dentro del primer tramo (a 10 px de la salida del bridge),
% MATLAB Function, Display n / To Host File, Display de la tabla
xs = pts(1, 1) + 10*sign(pts(2, 1) - pts(1, 1));
xf = xs + 25;
xc = xf + 340;                                 % deja lugar al nombre de archivo que muestra el To Host File
xd = xc + 130;
x_izq = 10*floor(x_min/10);                    % borde izq. del área (= el de las notas)
x_der = xd + 230;

% ---- área y nota
add_block('built-in/Area', sprintf('%s/%s  -  SOLO EXPLORADOR (rama solo-explorador-coordenadas)', m, NOMBRE), ...
    'Position', [x_izq y0 x_der y0+600]);
nota = Simulink.Annotation(m, sprintf([ ...
    'Rama de la linea Python Cmd (PYTHON BRIDGE -> Goto Python Commands; no se corta).\n' ...
    'qd2_coordenadas_objetivos(py): tabla por id (max 32, orden de llegada) con la\n' ...
    'ultima (x, y) en m, marco OptiTrack, de cada objeto que publica la estacion\n' ...
    '(tomas y fuegos). Solo lectura: no cambia la mision.\n' ...
    'coords_objetos [32x2]: filas 1..n = [x y]; el resto 0. n_objetos = cuantos.\n\n' ...
    'EN VIVO: Monitor & Tune -> Display coords_objetos (x,y).\n' ...
    'DESPUES DEL VUELO: saved_data/%s/log_<fecha>_CoordenadasObjetos.mat,\n' ...
    'variable coords_objetos (66 x N, 10 Hz): fila 1 = t, fila 2 = n,\n' ...
    'filas 3-34 = x, filas 35-66 = y. Tabla final = ultima columna:\n' ...
    '  d = coords_objetos(:, end); n = d(2); xy = [d(3:2+n) d(35:34+n)]\n\n' ...
    'Se reinicia en cada Start del modelo. Cambiar la lib obliga a recompilar (Build, R2022a).'], m));
nota.Position = [x_izq+20 y0+40 xs-40 y0+300];
nota.HorizontalAlignment = 'left';

% ---- MATLAB Function
yf = y0 + 70;
add_block('simulink/User-Defined Functions/MATLAB Function', F, 'Position', [xf yf xf+180 yf+180]);
codigo = sprintf([ ...
    'function [coords_objetos, n_objetos, registro] = fcn(py)\n' ...
    '%% COORDENADAS OBJETOS (x,y) - modo solo explorador.\n' ...
    '%% py: Python Cmd [32] (rama de PYTHON BRIDGE/1). Ranuras py(5:19) rotando\n' ...
    '%% [id clase x y conf] x 3. Tabla por id en qd2_coordenadas_objetivos (lib).\n' ...
    '%% coords_objetos 32x2: filas 1..n = [x y] (m, OptiTrack); el resto 0.\n' ...
    '%% registro 65x1 = [n; x(1:32); y(1:32)] -> To Host File Coordenadas.\n' ...
    '[coords_objetos, n_objetos] = qd2_coordenadas_objetivos(py);\n' ...
    'registro = [n_objetos; coords_objetos(:, 1); coords_objetos(:, 2)];\n']);
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', F);
ch.Script = codigo;
tam = {'coords_objetos', '[32 2]'; 'n_objetos', '1'; 'registro', '[65 1]'};
for k = 1:size(tam, 1)
    d = ch.find('-isa', 'Stateflow.Data', 'Name', tam{k, 1});
    d.Props.Array.Size = tam{k, 2};
    d.DataType = 'double';
end
pf = get_param(F, 'PortConnectivity');
pin = pf(1).Position;
pout = [pf(2).Position; pf(3).Position; pf(4).Position];

% ---- RAMA de la línea existente: empieza sobre su primer tramo
add_line(m, [xs pts(1, 2); xs pin(2); pin]);

% ---- Display n objetos y To Host File, alineados con las salidas 2 y 3
DN = [m '/Display n objetos'];
add_block('simulink/Sinks/Display', DN, 'Position', [xc pout(2, 2)-10 xc+60 pout(2, 2)+10], 'Format', 'short');
TH = [m '/To Host File Coordenadas'];
load_system('quarc_library');
add_block('quarc_library/Sinks/To Host/To Host File', TH, 'Position', [xc pout(3, 2)-20 xc+80 pout(3, 2)+20]);
dec = 100;                                     % Python Cmd sale a qc_get_step_size (1 kHz) -> 10 Hz
set_param(TH, 'file_name', 'saved_data/%m/log_%{regexprep(char(datetime),'' '',''_'')}_CoordenadasObjetos.mat', ...
    'variable_name', 'coords_objetos', 'decimation', num2str(dec), ...
    'file_format', 'MAT-file v.5', 'bitrate', '2000000');   % como To Host File / To Host File Dual
info.decimation = dec;

% ---- Display de la tabla (32 filas x 2 columnas)
DX = [m '/Display coords_objetos (x,y)'];
add_block('simulink/Sinks/Display', DX, 'Position', [xd y0+40 xd+200 y0+560], 'Format', 'short');

ph = get_param(F, 'PortHandles');
pdx = get_param(DX, 'PortConnectivity');
xm = xc + 100;
l1 = add_line(m, [pout(1, :); xm pout(1, 2); xm pdx(1).Position(2); pdx(1).Position]);
set_param(l1, 'Name', 'coords_objetos');
l2 = add_line(m, ph.Outport(2), get_param(DN, 'PortHandles').Inport(1));
set_param(l2, 'Name', 'n_objetos');
add_line(m, ph.Outport(3), get_param(TH, 'PortHandles').Inport(1));

info.agregado = true;
info.bloques = {F, DX, DN, TH};
fprintf('agregar_bloque_coordenadas: agregado "%s" en %s (y0 = %d, rama en x = %d, To Host File cada %d muestras).\n', ...
    NOMBRE, m, y0, xs, dec);

if guardar
    if strcmp(version('-release'), '2022a')
        save_system(m);
    else
        dtmp = tempname;
        mkdir(dtmp);
        limpiar = onCleanup(@() rmdir(dtmp, 's'));
        destino = fullfile(dtmp, [m '.slx']);
        save_system(m, destino, 'ExportToVersion', 'R2022a');
        close_system(m, 0);
        copyfile(destino, archivo, 'f');
    end
    if bdIsLoaded(m)
        close_system(m, 0);
    end
    info.guardado = true;
    mi = Simulink.MDLInfo(archivo);
    fprintf('agregar_bloque_coordenadas: guardado %s (%s, Simulink %s).\n', archivo, mi.ReleaseName, mi.SimulinkVersion);
end
end
