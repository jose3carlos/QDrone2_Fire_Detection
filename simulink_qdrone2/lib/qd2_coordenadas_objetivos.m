function [xy, n] = qd2_coordenadas_objetivos(py)
%QD2_COORDENADAS_OBJETIVOS Tabla (x, y) de TODOS los objetos que publica Python.
%   [xy, n] = qd2_coordenadas_objetivos(py)
%
%   py  vector 'Python Cmd' de 32 doubles (ARQUITECTURA §3.3) que llega por el
%       PYTHON BRIDGE (:18700). Se leen solo las 3 ranuras que rotan,
%       py(5:19): ranura j (1..3), o = 4 + 5*(j-1):
%       id = py(o+1), clase = py(o+2), x = py(o+3), y = py(o+4), conf = py(o+5).
%   xy  32x2: filas 1..n = [x y] (m, marco de OptiTrack, el mismo que usa la
%       misión) de cada objeto, en el ORDEN EN QUE LLEGÓ su id por primera
%       vez; el resto de las filas = 0.
%   n   número de objetos (ids distintos) vistos hasta ahora (0..32).
%
%   Reglas (las mismas validaciones que actualizar_objetivos de
%   qd2_mission_step, pero SIN deduplicar por cercanía ni dejar de refinar):
%   - una ranura vale si id, x e y son finitos y round(id) está en 1..32
%     (ids de la estación, MAX_TARGETS = 32); ranuras vacías (id 0),
%     NaN/Inf e ids fuera de rango se ignoran;
%   - un id nuevo agrega una fila al final (máximo 32, uno por id);
%   - un id ya conocido REFINA su fila con la última x, y recibidas (la
%     estación publica la posición fusionada más reciente de cada id).
%   La clase y la confianza no se guardan: el bloque solo expone (x, y).
%   Como no une por cercanía, n cuenta ids, no objetos: un alias de la
%   misión (2 ids a < radio_cluster) da 2 filas casi iguales, y si la
%   estación se reinicia SIN recuperar web/datos/objetivos_estacion.json,
%   los objetos ya vistos vuelven con ids nuevos (filas repetidas) o un id
%   que era alias se reutiliza (su fila se sobrescribe con otro objeto).
%
%   Reinicio: la tabla vive en un persistent. NO se borra durante el vuelo
%   (ni por pérdida del heartbeat, ni por PAUSED/DONE). Se reinicia cuando
%   el modelo vuelve a arrancar (Start del modelo de tierra / nueva
%   simulación: el código generado inicializa los persistent al iniciar) y,
%   en MATLAB, con  clear qd2_coordenadas_objetivos.
%
%   Solo lectura: no toca el estado de la misión (qd2_mission_step).
%   Bloque 'COORDENADAS OBJETOS (x,y)' del nivel superior de
%   versiones/R2022a/QD2_MissionCtrl_Dual_PID.slx (agregar_bloque_coordenadas.m).
%#codegen

persistent ids tab nn
if isempty(ids)
    ids = zeros(32, 1);
    tab = zeros(32, 2);
    nn = 0;
end

for j = 1:3
    o = 4 + 5*(j-1);
    if numel(py) < o + 4
        continue;
    end
    idr = py(o+1);
    x = py(o+3);
    y = py(o+4);
    if ~isfinite(idr) || ~isfinite(x) || ~isfinite(y)
        continue;
    end
    id = round(idr);
    if id < 1 || id > 32
        continue;
    end
    % ¿id ya conocido?
    k_id = 0;
    for k = 1:32
        if k <= nn && ids(k) == id
            k_id = k;
        end
    end
    if k_id == 0
        if nn >= 32
            continue;
        end
        nn = nn + 1;
        k_id = nn;
        ids(k_id) = id;
    end
    tab(k_id, 1) = x;
    tab(k_id, 2) = y;
end

xy = tab;
n = nn;
end
