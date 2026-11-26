function probar_qd2_bateria(con_reales)
%PROBAR_QD2_BATERIA Pruebas de qd2_bateria con logs sintéticos pequeños
%   (generados aquí, en tempdir; ~10 s) y, si existen, con los 2 vuelos
%   reales del Equipo2 (equipo2_original/saved_data, ~30 s más).
%
%   probar_qd2_bateria          sintéticos + reales si están
%   probar_qd2_bateria(false)   solo sintéticos
if nargin < 1, con_reales = true; end
aqui = fileparts(mfilename('fullpath'));
addpath(aqui);
base = fullfile(tempdir, sprintf('qd2_bateria_prueba_%d', round(1e6*rand)));
mkdir(base);
limpia = onCleanup(@() rmdir(base, 's'));
n_ok = 0;

% --- 1. Vuelo sintético con Commander 1 s antes en el nombre ---
fs = 200;
[Q, C, esp] = sintetico(fs, 0);
d1 = fullfile(base, 'logs', 'a'); mkdir(d1);
guardar(d1, 'log_01-Jan-2030_10-00-01', '10-00-00', Q, C);
sal = fullfile(base, 'res');
R = qd2_bateria(d1, 'salida', sal, 'etiqueta', 'prueba A', 'mostrar', false);
assert(numel(R) == 1, 'debe analizar 1 vuelo');
assert(strcmp(R.commander, 'log_01-Jan-2030_10-00-00_Commander'), 'emparejar Commander con 1 s de diferencia');
cerca(R.total.mAh, esp.mAh, 0.005, 'mAh total');
cerca(R.aire.t_s, esp.t_aire, 0.01, 'tiempo en el aire');
cerca(R.aire.mAh, esp.mAh_aire, 0.005, 'mAh en el aire');
cerca(R.aire.mAh_por_min, esp.mAh_aire/(esp.t_aire/60), 0.01, 'mAh/min');
cerca(R.total.pct_electronica, esp.pct_elec, 0.02, '% electrónica');
cerca(R.reposo.V_antes, 16.0, 0.001, 'V reposo antes');
cerca(R.reposo.V_despues, 14.9, 0.001, 'V reposo después');
cerca(R.aire.V_min_0p3s, 14.4, 0.003, 'V mín en el aire (0.3 s)');
assert(~R.corte_a_bordo.condicion_en_log, 'sin V<=14.0 no hay condición de corte');
nombres = cellfun(@(f) f.nombre, R.fases, 'UniformOutput', false);
assert(isequal(nombres, {'IDLE','ARMADO','DESPEGUE','SUBIENDO','VUELO','AUTOLAND','DESHABILITADO'}), 'fases');
cerca(R.fases{5}.duracion_s, 40, 0.01, 'duración de VUELO');
j = fullfile(sal, 'log_01-Jan-2030_10-00-01_QDrone2.json');
p = fullfile(sal, 'log_01-Jan-2030_10-00-01_QDrone2.png');
assert(isfile(j) && isfile(p) && isfile(fullfile(sal, 'comparacion.png')), 'salidas');
dj = dir(j); dp = dir(p);
assert(dj.bytes < 20e3, 'JSON < 20 KB (%d)', dj.bytes);
assert(dp.bytes < 300e3, 'PNG < 300 KB (%d)', dp.bytes);
assert(isempty(dir(fullfile(sal, '*.mat'))), 'nunca copia .mat');
n_ok = n_ok + 1; fprintf('OK 1: vuelo sintético con fases (%.1f mAh, JSON %d B, PNG %d B)\n', R.total.mAh, dj.bytes, dp.bytes);

% --- 2. Caídas a 13.9 V: 0.5 s (sí cumple la condición) y 0.1 s (no) ---
[Q2, C2] = sintetico(fs, 0.5);
d2 = fullfile(base, 'logs', 'b'); mkdir(d2);
guardar(d2, 'log_01-Jan-2030_10-00-00', '09-59-59', Q2, C2);      % cambio de minuto
[Q3, C3] = sintetico(fs, 0.1);
guardar(d2, 'log_01-Jan-2030_11-00-00', '11-00-03', Q3, C3);      % Commander a 3 s: no se empareja
R2 = qd2_bateria(d2, 'salida', sal, 'figuras', false, 'mostrar', false);
assert(numel(R2) == 2, 'dos vuelos');
a = R2(strcmp({R2.log}, 'log_01-Jan-2030_10-00-00_QDrone2'));
b = R2(strcmp({R2.log}, 'log_01-Jan-2030_11-00-00_QDrone2'));
assert(strcmp(a.commander, 'log_01-Jan-2030_09-59-59_Commander'), 'emparejar a través del cambio de minuto');
assert(a.corte_a_bordo.condicion_en_log, '0.5 s a 13.9 V: condición de corte presente');
cerca(a.corte_a_bordo.max_tramo_V_le_14_s, 0.5, 0.02, 'tramo <= 14 V');
assert(isempty(b.commander) && isempty(b.fases), 'Commander a 3 s no se usa: sin fases');
assert(~b.corte_a_bordo.condicion_en_log, '0.1 s a 13.9 V: sin condición de corte');
assert(contains(b.criterio_aire, 'sin Commander'), 'criterio de aire sin Commander');
assert(b.aire.t_s > 49 && b.aire.t_s < 52, 'aire aprox. por corriente (%.2f s)', b.aire.t_s);
n_ok = n_ok + 1; fprintf('OK 2: corte existente solo informado (0.5 s sí, 0.1 s no); sin Commander\n');

% --- 3. resumen.csv sin duplicados y etiqueta conservada ---
R3 = qd2_bateria(d1, 'salida', sal, 'figuras', false, 'mostrar', false);
assert(strcmp(R3.etiqueta, 'prueba A'), 'etiqueta conservada al rehacer sin etiqueta');
lin = strsplit(strtrim(fileread(fullfile(sal, 'resumen.csv'))), newline);
assert(numel(lin) == 4, 'resumen.csv: encabezado + 3 vuelos (hay %d líneas)', numel(lin));
assert(sum(contains(lin, 'log_01-Jan-2030_10-00-01_QDrone2')) == 1, 'sin duplicados');
n_ok = n_ok + 1; fprintf('OK 3: resumen.csv con 3 vuelos, sin duplicados\n');

% --- 4. Otra etiqueta no pisa la anterior; sin cambios no se reescribe ---
qd2_bateria(d1, 'salida', sal, 'mostrar', false);   % rehace comparacion.png (el paso 2 fue sin figuras)
pause(1.1);                                      % fecha_analisis y fechas de archivo: resolución de 1 s
b_json = leer(j); t_json = dir(j); t_png = dir(p); t_comp = dir(fullfile(sal, 'comparacion.png'));
w = warning('off', 'qd2_bateria:etiqueta');
R4 = qd2_bateria(d1, 'salida', sal, 'etiqueta', 'otra', 'mostrar', false);
warning(w);
assert(strcmp(R4.etiqueta, 'prueba A'), 'una etiqueta nueva no pisa la que ya tenía el vuelo');
assert(isequal(leer(j), b_json), 'JSON sin cambios (ni fecha_analisis) si el resumen es igual');
t2 = dir(j); assert(t2.datenum == t_json.datenum, 'JSON no se reescribe');
t2 = dir(p); assert(t2.datenum == t_png.datenum, 'PNG no se reescribe');
t2 = dir(fullfile(sal, 'comparacion.png')); assert(t2.datenum == t_comp.datenum, 'comparacion.png no se rehace');
R4 = qd2_bateria(d1, 'salida', sal, 'etiqueta', 'otra', 'reetiquetar', true, 'figuras', false, 'mostrar', false);
assert(strcmp(R4.etiqueta, 'otra'), '''reetiquetar'', true sí cambia la etiqueta');
n_ok = n_ok + 1; fprintf('OK 4: etiqueta previa conservada; sin cambios no se reescribe nada\n');

% --- 5. resumen.csv en orden cronológico y con BOM UTF-8 (acentos en Excel) ---
[Q5, C5] = sintetico(fs, 0);
d5 = fullfile(base, 'logs', 'c'); mkdir(d5);
guardar(d5, 'log_25-Dec-2029_08-00-00', '08-00-00', Q5, C5);   % por nombre iría al final
qd2_bateria(d5, 'salida', sal, 'etiqueta', 'con cámara', 'figuras', false, 'mostrar', false);
b = leer(fullfile(sal, 'resumen.csv'));
assert(isequal(b(1:3), uint8([239 187 191])), 'resumen.csv con BOM UTF-8');
lin = strsplit(strtrim(native2unicode(b(4:end), 'UTF-8')), newline);
assert(numel(lin) == 5, 'resumen.csv: encabezado + 4 vuelos (hay %d líneas)', numel(lin));
orden = {'log_25-Dec-2029_08-00-00', 'log_01-Jan-2030_10-00-00', 'log_01-Jan-2030_10-00-01', 'log_01-Jan-2030_11-00-00'};
for k = 1:numel(orden)
    assert(startsWith(lin{k + 1}, ['"' orden{k}]), 'orden cronológico: fila %d = %s', k, lin{k + 1});
end
assert(contains(lin{2}, '"con cámara"'), 'acentos en UTF-8');
n_ok = n_ok + 1; fprintf('OK 5: resumen.csv cronológico y con BOM\n');

% --- 6. Logs reales del Equipo2 (opcional) ---
b_real = fullfile(fileparts(aqui), 'equipo2_original', 'saved_data', 'QD2_DroneStack_PID_2021a');
if con_reales && isfile(fullfile(b_real, 'log_20-Nov-2026_19-59-41_QDrone2.mat'))
    Rr = qd2_bateria(b_real, 'salida', fullfile(base, 'reales'), 'figuras', false, 'mostrar', false);
    v1 = Rr(strcmp({Rr.log}, 'log_20-Nov-2026_19-59-41_QDrone2'));
    v2 = Rr(strcmp({Rr.log}, 'log_20-Nov-2026_20-05-50_QDrone2'));
    cerca(v1.total.mAh, 916.7, 0.005, 'vuelo 1 mAh');
    cerca(v2.total.mAh, 886.6, 0.005, 'vuelo 2 mAh');
    cerca(v2.aire.V_min_0p3s, 14.235, 0.005, 'vuelo 2 V mín (0.3 s)');
    cerca(v1.aire.P_vuelo_W, 367.4, 0.005, 'vuelo 1 potencia en VUELO');
    assert(~v1.corte_a_bordo.condicion_en_log && ~v2.corte_a_bordo.condicion_en_log, 'sin corte en los reales');
    n_ok = n_ok + 1; fprintf('OK 6: logs reales (%.1f y %.1f mAh; V mín %.3f V)\n', v1.total.mAh, v2.total.mAh, v2.aire.V_min_0p3s);
else
    fprintf('-- 6: sin logs reales del Equipo2 (se omite)\n');
end
fprintf('probar_qd2_bateria: %d pruebas OK\n', n_ok);
end

% ---------------------------------------------------------------------
function [Q, C, esp] = sintetico(fs, caida_s)
% Estados, duración (s), I motores (A), V inicial y final.
tab = {1, 5,  0.1, 16.0, 16.0
       2, 2, 15.0, 15.6, 15.5
       3, 3, 22.0, 15.4, 15.3
       4, 1, 20.0, 15.3, 15.2
       5, 40, 24.0, 15.2, 14.4
       6, 3, 18.0, 14.5, 14.5
       9, 0.5, 5.0, 14.7, 14.7
       1, 2, 0.05, 14.9, 14.9};
F = []; Im = []; V = [];
for k = 1:size(tab, 1)
    m = round(tab{k, 2}*fs);
    F = [F repmat(tab{k, 1}, 1, m)]; %#ok<AGROW>
    Im = [Im repmat(tab{k, 3}, 1, m)]; %#ok<AGROW>
    V = [V linspace(tab{k, 4}, tab{k, 5}, m)]; %#ok<AGROW>
end
N = numel(F);
t = (0:N-1)/fs;
Ie = 0.3 + 0.1*(Im > 1);
if caida_s > 0
    i = find(F == 5, 1) + round(20*fs);
    V(i:i + round(caida_s*fs) - 1) = 13.9;
end
Q = zeros(56, N); Q(1, :) = t; Q(24, :) = V; Q(25, :) = Ie; Q(26, :) = Im;
C = zeros(37, N); C(1, :) = t; C(19, :) = F;
dt = [0 diff(t)];
aire = ismember(F, 3:6);
esp.mAh = sum((Ie + Im).*dt)/3.6;
esp.mAh_aire = sum((Ie(aire) + Im(aire)).*dt(aire))/3.6;
esp.t_aire = 47;
esp.pct_elec = 100*sum(Ie.*dt)/sum((Ie + Im).*dt);
end

function guardar(carpeta, prefijo, hora_com, Q, C)
QDrone_data = Q; commander_data = C; %#ok<NASGU>
save(fullfile(carpeta, [prefijo '_QDrone2.mat']), 'QDrone_data');
tk = regexp(prefijo, '^(log_.+)_\d+-\d+-\d+$', 'tokens', 'once');
save(fullfile(carpeta, [tk{1} '_' hora_com '_Commander.mat']), 'commander_data');
end

function b = leer(archivo)
fid = fopen(archivo, 'r'); b = fread(fid, inf, '*uint8')'; fclose(fid);
end

function cerca(x, y, rel, que)
assert(abs(x - y) <= rel*max(abs(y), 1e-9), '%s: %.4f vs esperado %.4f (tolerancia %.1f%%)', que, x, y, 100*rel);
end
