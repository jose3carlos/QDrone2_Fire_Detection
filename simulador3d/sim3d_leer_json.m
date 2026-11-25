function cfg = sim3d_leer_json(texto)
%SIM3D_LEER_JSON jsondecode de estacion.json con las reglas de web/app/config.py.
%   cfg = sim3d_leer_json(texto)
%   jsondecode convierte null y [] en el mismo [] de MATLAB, pero la estación los trata distinto
%   en los umbrales de filtros_vision (vision.inclinacion_max_deg, dist_max_tubo_toma_m,
%   dist_max_tubo_fuego_m): null apaga el filtro y una lista ([] o [0.3], no un número) deja el
%   valor por defecto. Aquí esa lista se cambia por el texto "[]" antes de decodificar, para que
%   sim3d_config la trate como "no numérica" igual que Python (jsondecode da 0.3 para [0.3]).
texto = regexprep(char(texto), ...
    '("(?:inclinacion_max_deg|dist_max_tubo_toma_m|dist_max_tubo_fuego_m)"\s*:\s*)\[[^\[\]]*\]', '$1"[]"');
cfg = jsondecode(texto);
end
