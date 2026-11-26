function R = qd2_bateria(fuente, varargin)
%QD2_BATERIA Analiza la batería de los logs a bordo del QDrone 2 (PID del
%   Equipo2) y deja resúmenes pequeños para subir a git.
%
%   R = qd2_bateria()
%       Busca todos los log_*_QDrone2.mat bajo simulink_qdrone2 (recursivo:
%       equipo2_original/saved_data, saved_data, ...).
%   R = qd2_bateria(carpeta)
%   R = qd2_bateria({carpeta1, 'C:/.../log_..._QDrone2.mat', ...})
%       Busca solo en esas carpetas (recursivo) o usa esos archivos.
%   R = qd2_bateria(..., 'etiqueta', 'con cámara abajo', 'salida', carpeta,
%                   'figuras', true, 'comparacion', true, 'mostrar', true,
%                   'reetiquetar', false)
%
%   La etiqueta solo se pone en vuelos sin .json previo o sin etiqueta: la
%   que un vuelo ya tenía se conserva (con aviso) salvo 'reetiquetar', true.
%   Un vuelo cuyo resumen no cambió no se reescribe (ni su .json, con su
%   fecha_analisis, ni su .png); comparacion.png solo se rehace si algo
%   cambió. resumen.csv y comparacion.png van en orden cronológico.
%
%   Cada log_<fecha>_QDrone2.mat (variable QDrone_data, 56xN, 1 kHz) se
%   empareja con su log_<fecha>_Commander.mat (commander_data, fila 19 =
%   estado FSM a bordo); el nombre puede diferir en 1 s (19-59-40 vs
%   19-59-41). Sin Commander no hay fases y "en el aire" se aproxima con la
%   corriente de motores > 2 A (queda anotado en el resultado).
%
%   Filas usadas de QDrone_data: 1 = t (s), 24 = batería (V), 25 = corriente
%   de la electrónica (A), 26 = corriente de los motores (A).
%   Estados FSM: 1 IDLE, 2 ARMADO, 3 DESPEGUE, 4 SUBIENDO, 5 VUELO,
%   6 AUTOLAND, 7/8 emergencia, 9 deshabilitado. En el aire = 3..6.
%
%   SOLO MIDE Y REPORTA. No cambia ningún límite ni actúa sobre nada. El
%   corte de batería existente vive a bordo (bloque "Check Battery level":
%   V <= 14.0 V durante >= 0.3 s enclava low_battery -> AUTOLAND en vuelo);
%   aquí solo se informa si en el log se dio esa condición. 15.5 V es la
%   recomendación de docs/CHECKLIST_LABORATORIO.md (no automática).
%
%   Salida (por defecto simulink_qdrone2/bateria/resultados):
%     <nombre_log>.json   resumen por vuelo (fases, totales, serie reducida)
%     <nombre_log>.png    figura del vuelo
%     resumen.csv         una fila por vuelo, rehecho desde todos los .json
%     comparacion.png     todos los vuelos de la carpeta superpuestos
%   Nunca copia los .mat.
%
%   R es un arreglo de structs con lo mismo que los .json.

if nargin < 1, fuente = []; end
aqui = fileparts(mfilename('fullpath'));
raiz = fileparts(aqui);                                % simulink_qdrone2

op = inputParser;
op.addParameter('etiqueta', '', @(x) ischar(x) || isstring(x));
op.addParameter('salida', fullfile(aqui, 'resultados'), @(x) ischar(x) || isstring(x));
op.addParameter('figuras', true, @(x) islogical(x) || isnumeric(x));
op.addParameter('comparacion', true, @(x) islogical(x) || isnumeric(x));
op.addParameter('mostrar', true, @(x) islogical(x) || isnumeric(x));
op.addParameter('reetiquetar', false, @(x) islogical(x) || isnumeric(x));
op.parse(varargin{:});
op = op.Results;
op.etiqueta = char(op.etiqueta); op.salida = char(op.salida);

% --- 1. Archivos ---
if isempty(fuente), fuente = {raiz}; end
if ischar(fuente) || isstring(fuente), fuente = cellstr(fuente); end
logs = {};
for k = 1:numel(fuente)
    f = char(fuente{k});
    if isfolder(f)
        d = dir(fullfile(f, '**', 'log_*_QDrone2.mat'));
        for j = 1:numel(d), logs{end+1} = fullfile(d(j).folder, d(j).name); end %#ok<AGROW>
    elseif isfile(f)
        logs{end+1} = f; %#ok<AGROW>
    else
        warning('qd2_bateria:noExiste', 'No existe: %s', f);
    end
end
[~, nombres] = cellfun(@fileparts, logs, 'UniformOutput', false);
[~, iu] = unique(nombres, 'stable');
if numel(iu) < numel(logs)
    warning('qd2_bateria:repetido', 'Hay logs con el mismo nombre en varias carpetas; se usa el primero.');
end
logs = logs(sort(iu));
if isempty(logs)
    warning('qd2_bateria:sinLogs', 'No se encontró ningún log_*_QDrone2.mat.');
    R = struct([]);
    return
end
if ~isfolder(op.salida), mkdir(op.salida); end

% --- 2. Análisis por vuelo ---
R = struct([]);
n_escritos = 0;
for k = 1:numel(logs)
    if op.mostrar, fprintf('qd2_bateria: %s\n', logs{k}); end
    try
        [r, s] = analizar(logs{k}, op);
    catch e
        warning('qd2_bateria:fallo', 'No se pudo analizar %s: %s', logs{k}, e.message);
        continue
    end
    archivo_json = fullfile(op.salida, [r.log '.json']);
    archivo_png = fullfile(op.salida, [r.log '.png']);
    viejo = [];
    if isfile(archivo_json)
        try
            viejo = jsondecode(leer_utf8(archivo_json));
        catch
        end
    end
    % Etiqueta: la que ya tenía el vuelo se conserva; la nueva solo se pone
    % en vuelos nuevos o sin etiqueta ('reetiquetar', true para cambiarla).
    ant = '';
    if isstruct(viejo) && isfield(viejo, 'etiqueta'), ant = char(viejo.etiqueta); end
    if ~isempty(ant) && (isempty(r.etiqueta) || ~op.reetiquetar)
        if ~isempty(r.etiqueta) && ~strcmp(ant, r.etiqueta)
            warning('qd2_bateria:etiqueta', ['%s ya tenía la etiqueta "%s"; se conserva ' ...
                '(usa ''reetiquetar'', true para cambiarla).'], r.log, ant);
        end
        r.etiqueta = ant;
    end
    % Si nada cambió salvo la fecha de análisis, no se reescribe nada.
    escribir = true;
    if isstruct(viejo) && isfield(viejo, 'fecha_analisis')
        ahora = r.fecha_analisis;
        r.fecha_analisis = char(viejo.fecha_analisis);
        escribir = ~isequal(leer_bytes(archivo_json), unicode2native(jsonencode(r, 'PrettyPrint', true), 'UTF-8'));
        if escribir, r.fecha_analisis = ahora; end
    end
    if escribir
        escribir_json(archivo_json, r);
        n_escritos = n_escritos + 1;
    end
    if op.figuras && (escribir || ~isfile(archivo_png))
        figura_vuelo(archivo_png, r, s);
        n_escritos = n_escritos + 1;
    end
    if op.mostrar, imprimir(r); end
    if isempty(R), R = r; else, R(end+1) = r; end %#ok<AGROW>
end

% --- 3. Resumen acumulado y comparación (desde todos los .json) ---
todos = leer_jsons(op.salida);
escribir_csv(fullfile(op.salida, 'resumen.csv'), todos);
archivo_comp = fullfile(op.salida, 'comparacion.png');
dj = dir(fullfile(op.salida, 'log_*.json')); dc = dir(archivo_comp);
vieja = isempty(dc) || any([dj.datenum] > dc.datenum);   % algún .json más nuevo que la figura
if op.figuras && op.comparacion && numel(todos) >= 1 && vieja
    figura_comparacion(archivo_comp, todos);
end
if op.mostrar
    fprintf('qd2_bateria: %d vuelo(s) analizado(s), %d archivo(s) de vuelo reescrito(s); resultados en %s\n', ...
        numel(R), n_escritos, op.salida);
end
end

% =====================================================================
function [r, s] = analizar(archivo, op)
[carpeta, nombre] = fileparts(archivo);
q = load(archivo, 'QDrone_data');
Q = q.QDrone_data;
if size(Q, 1) < 26
    error('QDrone_data tiene %d filas (se esperan 56; batería en la 24)', size(Q, 1));
end
com = buscar_commander(carpeta, nombre);
F = [];
if ~isempty(com)
    c = load(com, 'commander_data');
    C = c.commander_data;
    n = min(size(Q, 2), size(C, 2));            % mismo arranque del modelo: alineados por muestra
    Q = Q(:, 1:n);
    F = round(C(19, 1:n));
end
t = Q(1, :); V = Q(24, :); Ie = Q(25, :); Im = Q(26, :);
ok = isfinite(t) & isfinite(V) & isfinite(Ie) & isfinite(Im);
t = t(ok); V = V(ok); Ie = Ie(ok); Im = Im(ok);
if ~isempty(F), F = F(ok); end
n = numel(t);
if n < 10, error('log vacío'); end
dt = [0 diff(t)];
fs = 1/median(diff(t));
P = V.*(Ie + Im);
nv = max(1, round(0.3*fs));                     % 0.3 s con el dt real (300 muestras a 1 kHz)
mv = movmean(V, nv);

r = struct();
r.version_script = 1;
r.log = nombre;
r.commander = '';
if ~isempty(com), [~, r.commander] = fileparts(com); end
r.etiqueta = op.etiqueta;
r.fecha_vuelo = fecha_de_nombre(nombre);
r.fecha_analisis = datestr(now, 'yyyy-mm-dd HH:MM:SS'); %#ok<TNOW1,DATST>
r.duracion_registro_s = t(end) - t(1);
r.fs_Hz = fs;

% Fases FSM
nom = containers.Map({1,2,3,4,5,6,7,8,9}, {'IDLE','ARMADO','DESPEGUE','SUBIENDO', ...
    'VUELO','AUTOLAND','EMERGENCIA 7','EMERGENCIA 8','DESHABILITADO'});
fases = {};
if ~isempty(F)
    for e = unique(F(isfinite(F)))
        m = F == e;
        if nom.isKey(e), nn = nom(e); else, nn = sprintf('ESTADO %d', e); end
        fa = struct('estado', e, 'nombre', nn, 'duracion_s', sum(dt(m)), ...
            'V_media', mean(V(m)), 'V_min', min(V(m)), 'V_min_0p3s', min(mv(m)), ...
            'Imot_media_A', mean(Im(m)), 'Ielec_media_A', mean(Ie(m)), ...
            'P_media_W', mean(P(m)), 'Pelec_media_W', mean(V(m).*Ie(m)), ...
            'mAh', sum((Ie(m) + Im(m)).*dt(m))/3.6, ...
            'mAh_motores', sum(Im(m).*dt(m))/3.6, 'mAh_electronica', sum(Ie(m).*dt(m))/3.6);
        fases{end+1} = fa; %#ok<AGROW>
    end
    aire = ismember(F, [3 4 5 6]);
    r.criterio_aire = 'FSM a bordo 3-6 (DESPEGUE a AUTOLAND)';
    i0 = find(F == 2, 1);
    if isempty(i0), i0 = find(aire, 1); end
else
    aire = movmean(Im, nv) > 2;
    r.criterio_aire = 'sin Commander: corriente de motores > 2 A (incluye armado en piso)';
    i0 = find(aire, 1);
end
r.fases = fases;

% Totales
mAh = sum((Ie + Im).*dt)/3.6;
r.total = struct('mAh', mAh, 'Wh', sum(P.*dt)/3600, ...
    'mAh_motores', sum(Im.*dt)/3.6, 'mAh_electronica', sum(Ie.*dt)/3.6, ...
    'pct_electronica', 100*sum(Ie.*dt)/max(eps, sum((Ie + Im).*dt)));

% En el aire
ta = sum(dt(aire));
mAh_aire = sum((Ie(aire) + Im(aire)).*dt(aire))/3.6;
v5 = [];
if ~isempty(F), v5 = F == 5; end
a = struct('t_s', ta, 'mAh', mAh_aire, 'mAh_por_min', NaN, 'V_min_0p3s', NaN, ...
    'V_media', NaN, 'Imot_media_A', NaN, 'P_media_W', NaN, 'P_vuelo_W', NaN, 'Pelec_vuelo_W', NaN);
if ta > 0
    a.mAh_por_min = mAh_aire/(ta/60);
    a.V_min_0p3s = min(mv(aire));
    a.V_media = mean(V(aire));
    a.Imot_media_A = mean(Im(aire));
    a.P_media_W = mean(P(aire));
end
if any(v5)
    a.P_vuelo_W = mean(P(v5));
    a.Pelec_vuelo_W = mean(V(v5).*Ie(v5));
end
r.aire = a;

% Reposo antes (2 s antes de armar) y después (tras aterrizar, con los
% motores ya parados: I motores < 1 A durante al menos 0.5 s). Los logs del
% Equipo2 suelen cortarse ~0.1 s después de aterrizar: entonces V_despues
% queda NaN y solo se reporta el último voltaje del registro (bajo carga).
n2 = round(2*fs);
rep = struct('V_antes', NaN, 'V_despues', NaN, 'caida_V', NaN, 'caida_bajo_carga_V', NaN, ...
    'V_final_registro', V(end), 'Imot_final_registro_A', Im(end));
if ~isempty(i0), rep.V_antes = median(V(max(1, i0 - n2):i0)); end
i1 = find(aire, 1, 'last');
if ~isempty(i1)
    quieto = find(Im(i1 + 1:end) < 1) + i1;
    if numel(quieto) >= round(0.5*fs), rep.V_despues = median(V(quieto)); end
end
rep.caida_V = rep.V_antes - rep.V_despues;
if ~isempty(i0) && ta > 0
    rep.caida_bajo_carga_V = rep.V_antes - a.V_media;   % reposo antes - media en el aire
end
r.reposo = rep;

% Corte EXISTENTE a bordo: solo se informa si se dio la condición en el log
bajo = V <= 14.0;
dur_max = 0; t_primer = NaN;
if any(bajo)
    d = diff([0 bajo 0]);
    ini = find(d == 1); fin = find(d == -1) - 1;
    for j = 1:numel(ini)
        du = t(fin(j)) - t(ini(j)) + median(diff(t));
        dur_max = max(dur_max, du);
        if du >= 0.3 && isnan(t_primer), t_primer = t(ini(j)) - t(1); end
    end
end
r.corte_a_bordo = struct( ...
    'nota', 'Corte existente a bordo (bloque Check Battery level): V <= 14.0 V durante >= 0.3 s; aquí solo se informa', ...
    'umbral_V', 14.0, 'tiempo_s', 0.3, ...
    'condicion_en_log', ~isnan(t_primer), 't_condicion_s', t_primer, ...
    'max_tramo_V_le_14_s', dur_max, ...
    't_media_0p3s_le_14_aire_s', sum(dt(aire & mv <= 14.0)));
r.recomendacion_checklist_V = 15.5;

% Serie reducida (para la comparación entre vuelos sin los .mat)
s = struct('t', t - t(1), 'V', V, 'Ie', Ie, 'Im', Im, 'F', F, 'aire', aire, 'mv', mv, 'fs', fs);
ser = struct('t_aire_s', [], 'V_0p3s', [], 'Imot_A', []);
if ta > 0
    ia = find(aire);
    ta_ac = cumsum(dt(ia));
    paso = max(1, ta/200);                      % 1 s, y <= ~200 puntos (JSON < 20 KB)
    tt = 0:paso:ta_ac(end);
    [ta_u, iu] = unique(ta_ac);
    ser.t_aire_s = redondear(tt, 2);
    ser.V_0p3s = redondear(interp1(ta_u, mv(ia(iu)), tt, 'nearest', 'extrap'), 3);
    ser.Imot_A = redondear(interp1(ta_u, movmean(Im(ia(iu)), nv), tt, 'nearest', 'extrap'), 2);
end
r.serie = ser;
end

% ---------------------------------------------------------------------
function com = buscar_commander(carpeta, nombre)
% Mismo prefijo de fecha, hora con tolerancia de 1 s.
com = '';
tk = regexp(nombre, '^log_(.+)_(\d+)-(\d+)-(\d+)_QDrone2$', 'tokens', 'once');
if isempty(tk), return, end
seg = str2double(tk{2})*3600 + str2double(tk{3})*60 + str2double(tk{4});
d = dir(fullfile(carpeta, ['log_' tk{1} '_*_Commander.mat']));
mejor = inf;
for j = 1:numel(d)
    tc = regexp(d(j).name, '^log_(.+)_(\d+)-(\d+)-(\d+)_Commander\.mat$', 'tokens', 'once');
    if isempty(tc) || ~strcmp(tc{1}, tk{1}), continue, end
    sc = str2double(tc{2})*3600 + str2double(tc{3})*60 + str2double(tc{4});
    if abs(sc - seg) <= 1 && abs(sc - seg) < mejor
        mejor = abs(sc - seg);
        com = fullfile(d(j).folder, d(j).name);
    end
end
end

function f = fecha_de_nombre(nombre)
tk = regexp(nombre, '^log_(.+)_(\d+)-(\d+)-(\d+)_', 'tokens', 'once');
if isempty(tk), f = ''; else, f = sprintf('%s %s:%s:%s', tk{1}, tk{2}, tk{3}, tk{4}); end
end

function y = redondear(x, d)
y = round(x*10^d)/10^d;
end

% ---------------------------------------------------------------------
function escribir_json(archivo, r)
txt = jsonencode(r, 'PrettyPrint', true);
fid = fopen(archivo, 'w', 'n', 'UTF-8');
if fid < 0, error('No se pudo escribir %s', archivo); end
fwrite(fid, unicode2native(txt, 'UTF-8'));
fclose(fid);
end

function b = leer_bytes(archivo)
fid = fopen(archivo, 'r');
if fid < 0, error('No se pudo leer %s', archivo); end
b = fread(fid, inf, '*uint8')';
fclose(fid);
end

function txt = leer_utf8(archivo)
% Siempre como UTF-8 (fileread en R2022a usa la codificación del sistema).
b = leer_bytes(archivo);
if numel(b) >= 3 && isequal(b(1:3), uint8([239 187 191])), b = b(4:end); end
txt = native2unicode(b, 'UTF-8');
end

function todos = leer_jsons(carpeta)
d = dir(fullfile(carpeta, 'log_*.json'));
todos = {};
for j = 1:numel(d)
    try
        x = jsondecode(leer_utf8(fullfile(d(j).folder, d(j).name)));
        if isfield(x, 'log') && isfield(x, 'total') && isfield(x, 'aire')
            todos{end+1} = x; %#ok<AGROW>
        end
    catch e
        warning('qd2_bateria:json', 'No se pudo leer %s: %s', d(j).name, e.message);
    end
end
% Orden cronológico por fecha_vuelo (dir() ordena por nombre: '01-Dec' antes
% que '20-Nov'); los que no tienen fecha van al final, por nombre.
clave = inf(1, numel(todos));
for j = 1:numel(todos)
    try
        clave(j) = datenum(datetime(todos{j}.fecha_vuelo, 'InputFormat', 'dd-MMM-yyyy HH:mm:ss', ...
            'Locale', 'en_US')); %#ok<DATNM>
    catch
    end
end
clave(isnan(clave)) = inf;
[~, i] = sort(clave);
todos = todos(i);
end

function escribir_csv(archivo, todos)
col = {'log','etiqueta','fecha_vuelo','con_fases','duracion_registro_s','t_aire_s', ...
    'mAh_total','Wh_total','mAh_aire','mAh_por_min_aire','pct_electronica', ...
    'P_vuelo_W','V_reposo_antes','V_reposo_despues','V_final_registro','caida_bajo_carga_V', ...
    'V_min_aire_0p3s','condicion_14V_0p3s_en_log','fecha_analisis'};
fid = fopen(archivo, 'w', 'n', 'UTF-8');
if fid < 0, error('No se pudo escribir %s', archivo); end
fwrite(fid, uint8([239 187 191]));              % BOM UTF-8: Excel en Windows lee bien los acentos
fprintf(fid, '%s\n', strjoin(col, ','));
for j = 1:numel(todos)
    x = todos{j};
    v = {texto(x.log), texto(campo(x, 'etiqueta', '')), texto(campo(x, 'fecha_vuelo', '')), ...
        sprintf('%d', ~isempty(campo(x, 'commander', ''))), ...
        num(x.duracion_registro_s, 2), num(x.aire.t_s, 2), num(x.total.mAh, 1), num(x.total.Wh, 2), ...
        num(x.aire.mAh, 1), num(x.aire.mAh_por_min, 1), num(x.total.pct_electronica, 2), ...
        num(x.aire.P_vuelo_W, 1), num(x.reposo.V_antes, 3), num(x.reposo.V_despues, 3), num(campo(x.reposo, 'V_final_registro', NaN), 3), ...
        num(x.reposo.caida_bajo_carga_V, 3), num(x.aire.V_min_0p3s, 3), ...
        sprintf('%d', logical(x.corte_a_bordo.condicion_en_log)), texto(campo(x, 'fecha_analisis', ''))};
    fprintf(fid, '%s\n', strjoin(v, ','));
end
fclose(fid);
end

function v = campo(x, nombre, def)
if isfield(x, nombre) && ~isempty(x.(nombre)), v = x.(nombre); else, v = def; end
end

function s = texto(s)
s = ['"' strrep(char(s), '"', '""') '"'];
end

function s = num(x, d)
if isempty(x) || ~isfinite(x), s = ''; else, s = sprintf(['%.' num2str(d) 'f'], x); end
end

% ---------------------------------------------------------------------
function imprimir(r)
fprintf('  %s%s: registro %.1f s, %s\n', r.log, ...
    ternario(isempty(r.etiqueta), '', [' (' r.etiqueta ')']), r.duracion_registro_s, ...
    ternario(isempty(r.commander), 'SIN Commander (sin fases)', ['FSM de ' r.commander]));
for j = 1:numel(r.fases)
    fa = r.fases{j};
    fprintf('    %-13s %6.1f s | V %5.2f (mín %5.2f; 0.3 s %5.2f) | Imot %5.2f A | Ielec %4.2f A | P %6.1f W | %6.1f mAh\n', ...
        fa.nombre, fa.duracion_s, fa.V_media, fa.V_min, fa.V_min_0p3s, fa.Imot_media_A, ...
        fa.Ielec_media_A, fa.P_media_W, fa.mAh);
end
fprintf('    TOTAL %.1f mAh (%.2f Wh), electrónica %.1f%% | en el aire %.1f s, %.1f mAh, %.1f mAh/min\n', ...
    r.total.mAh, r.total.Wh, r.total.pct_electronica, r.aire.t_s, r.aire.mAh, r.aire.mAh_por_min);
fprintf('    V reposo antes %.2f, después %.2f (último del registro %.2f con %.1f A de motores) | V mín en el aire (0.3 s) %.3f V | V<=14.0 V por 0.3 s en el log: %s\n', ...
    r.reposo.V_antes, r.reposo.V_despues, r.reposo.V_final_registro, r.reposo.Imot_final_registro_A, r.aire.V_min_0p3s, ternario(r.corte_a_bordo.condicion_en_log, 'SÍ', 'no'));
end

function y = ternario(c, a, b)
if c, y = a; else, y = b; end
end

% ---------------------------------------------------------------------
function referencias(ax)
% Solo las líneas EXISTENTES: corte a bordo (automático) y recomendación del checklist.
yline(ax, 14.0, '--', '14.0 V por 0.3 s: aterrizaje automático a bordo (ya existente)', 'Color', [0.85 0.1 0.1], ...
    'LabelHorizontalAlignment', 'left', 'LineWidth', 1, 'HandleVisibility', 'off');
yline(ax, 15.5, '--', '15.5 V: mínimo recomendado (checklist, no automático)', 'Color', [0.9 0.5 0], ...
    'LabelHorizontalAlignment', 'left', 'LineWidth', 1, 'HandleVisibility', 'off');
end

function figura_vuelo(archivo, r, s)
f = figure('Visible', 'off', 'Color', 'w', 'Position', [0 0 1300 900]);
limpia = onCleanup(@() close(f));
tl = tiledlayout(f, 3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
tit = sprintf('Batería QDrone 2 · %s', r.log);
if ~isempty(r.etiqueta), tit = [tit ' · ' r.etiqueta]; end
title(tl, tit, 'FontWeight', 'bold', 'Interpreter', 'none');
paso = max(1, round(s.fs/100));                 % graficar a ~100 Hz
k = 1:paso:numel(s.t);
ia = find(s.aire);
if isempty(ia), t0 = 0; else, t0 = s.t(ia(1)); end
x = s.t(k) - t0;

% Voltaje
ax = nexttile(tl, 1, [1 3]); hold(ax, 'on');
vmin = min([13.5, min(s.mv) - 0.2]); vmax = max([16.9, max(s.mv) + 0.2]);   % picos crudos de ms se recortan
sombrear(ax, s, t0, vmin, vmax);
plot(ax, x, s.V(k), 'Color', [0.55 0.7 0.95]);
plot(ax, x, s.mv(k), 'k', 'LineWidth', 1.2);
referencias(ax);
ylim(ax, [vmin vmax]); grid(ax, 'on'); ylabel(ax, 'V');
xlabel(ax, 't (s) desde el despegue');
title(ax, sprintf('Voltaje (negro = media 0.3 s; azul = en el aire: %s)', r.criterio_aire), 'Interpreter', 'none');

% Corrientes
ax = nexttile(tl, 4, [1 3]); hold(ax, 'on');
nc = max(1, round(0.02*s.fs/paso));
imax = max(5, 1.1*max(movmean(s.Im(k), nc)));
sombrear(ax, s, t0, 0, imax);
h1 = plot(ax, x, movmean(s.Im(k), nc), 'Color', [0.8 0.3 0.1]);
h2 = plot(ax, x, 20*movmean(s.Ie(k), nc), 'Color', [0.1 0.6 0.3]);
grid(ax, 'on'); ylabel(ax, 'A'); ylim(ax, [0 imax]); xlabel(ax, 't (s) desde el despegue');
legend([h1 h2], {'motores (A)', 'electrónica ×20 (A)'}, 'Location', 'northwest');
title(ax, 'Corriente');

% mAh por fase
ax = nexttile(tl, 7, [1 2]);
if isempty(r.fases)
    m = [r.total.mAh_motores r.total.mAh_electronica; 0 0];
    etq = {'registro completo', ''};
    dur = [r.duracion_registro_s NaN];
else
    m = cell2mat(cellfun(@(fa) [fa.mAh_motores fa.mAh_electronica], r.fases(:), 'UniformOutput', false));
    etq = cellfun(@(fa) fa.nombre, r.fases, 'UniformOutput', false);
    dur = cellfun(@(fa) fa.duracion_s, r.fases);
    if size(m, 1) == 1, m = [m; 0 0]; etq{end+1} = ''; dur(end+1) = NaN; end
end
b = bar(ax, m, 'stacked');
if isempty(r.fases), xlim(ax, [0.5 1.5]); end
b(1).FaceColor = [0.8 0.3 0.1]; b(2).FaceColor = [0.1 0.6 0.3];
set(ax, 'XTick', 1:numel(etq), 'XTickLabel', etq);
for j = 1:size(m, 1)
    if isfinite(dur(j))
        text(ax, j, sum(m(j, :)), {sprintf('%.1f mAh', sum(m(j, :))), sprintf('%.1f s', dur(j))}, ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 9);
    end
end
ylim(ax, [0 1.25*max(1, max(sum(m, 2)))]);
grid(ax, 'on'); ylabel(ax, 'mAh');
legend(ax, {'motores', 'electrónica (Jetson, sensores)'}, 'Location', 'northwest');
title(ax, 'Carga gastada por fase (mAh)');

% Resumen en texto
ax = nexttile(tl, 9); axis(ax, 'off');
if isnan(r.reposo.V_despues)
    v_despues = sprintf('V después: n/d (log corta al aterrizar; último %.2f V)', r.reposo.V_final_registro);
else
    v_despues = sprintf('V reposo después: %.2f V', r.reposo.V_despues);
end
lin = {sprintf('Total: %.0f mAh (%.2f Wh)', r.total.mAh, r.total.Wh), ...
    sprintf('Electrónica: %.1f %%', r.total.pct_electronica), ...
    sprintf('En el aire: %.1f s, %.0f mAh', r.aire.t_s, r.aire.mAh), ...
    sprintf('Ritmo en el aire: %.0f mAh/min', r.aire.mAh_por_min), ...
    sprintf('Potencia en VUELO: %.0f W', r.aire.P_vuelo_W), ...
    sprintf('V reposo antes: %.2f V', r.reposo.V_antes), ...
    v_despues, ...
    sprintf('V mín en el aire (0.3 s): %.3f V', r.aire.V_min_0p3s), ...
    sprintf('V <= 14.0 V por 0.3 s en el log: %s', ternario(r.corte_a_bordo.condicion_en_log, 'SÍ', 'no')), ...
    '', 'Solo medición: no cambia ningún límite.'};
text(ax, 0, 1, lin, 'VerticalAlignment', 'top', 'FontSize', 10, 'Interpreter', 'none');
exportgraphics(f, archivo, 'Resolution', 90);
end

function sombrear(ax, s, t0, y0, y1)
% Tramos "en el aire" con patch (xregion es de R2023a).
d = diff([0 s.aire 0]);
ini = find(d == 1); fin = find(d == -1) - 1;
for j = 1:numel(ini)
    xa = s.t(ini(j)) - t0; xb = s.t(fin(j)) - t0;
    patch(ax, [xa xb xb xa], [y0 y0 y1 y1], [0.6 0.85 1], 'FaceAlpha', 0.25, ...
        'EdgeColor', 'none', 'HandleVisibility', 'off');
end
end

function figura_comparacion(archivo, todos)
f = figure('Visible', 'off', 'Color', 'w', 'Position', [0 0 1300 700]);
limpia = onCleanup(@() close(f));
tl = tiledlayout(f, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('Batería QDrone 2 · comparación de %d vuelo(s)', numel(todos)), 'FontWeight', 'bold');
col = lines(max(1, numel(todos)));
etq = cell(1, numel(todos)); ritmo = nan(1, numel(todos));
ax = nexttile(tl, 1, [1 2]); hold(ax, 'on');
for j = 1:numel(todos)
    x = todos{j};
    etq{j} = x.fecha_vuelo;
    if isfield(x, 'etiqueta') && ~isempty(x.etiqueta), etq{j} = [etq{j} ' · ' x.etiqueta]; end
    if isempty(etq{j}), etq{j} = x.log; end
    ritmo(j) = valor(x.aire.mAh_por_min);
    if isfield(x, 'serie') && ~isempty(x.serie.t_aire_s)
        plot(ax, x.serie.t_aire_s, x.serie.V_0p3s, 'Color', col(j, :), 'LineWidth', 1.3, ...
            'DisplayName', etq{j});
    end
end
referencias(ax);
yl = ylim(ax); ylim(ax, [min(13.5, yl(1)) max(16.9, yl(2))]);
grid(ax, 'on'); xlabel(ax, 'tiempo en el aire (s)'); ylabel(ax, 'V (media 0.3 s)');
legend(ax, 'Location', 'southwest', 'Interpreter', 'none');
title(ax, 'Voltaje contra tiempo en el aire');
ax = nexttile(tl, 3);
b = bar(ax, ritmo, 'FaceColor', 'flat');
b.CData = col(1:numel(todos), :);
for j = 1:numel(ritmo)
    if isfinite(ritmo(j))
        text(ax, j, ritmo(j), sprintf('%.0f', ritmo(j)), 'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'bottom', 'FontWeight', 'bold');
    end
end
set(ax, 'XTick', 1:numel(etq), 'XTickLabel', etq, 'TickLabelInterpreter', 'none');
xtickangle(ax, 30);
ylim(ax, [0 1.15*max([1 ritmo(isfinite(ritmo))])]);
grid(ax, 'on'); ylabel(ax, 'mAh/min'); title(ax, 'Consumo en el aire');
exportgraphics(f, archivo, 'Resolution', 90);
end

function v = valor(x)
if isempty(x), v = NaN; else, v = double(x); end
end
