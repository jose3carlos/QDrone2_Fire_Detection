function cfg = sim3d_leer_json(texto)
%SIM3D_LEER_JSON jsondecode de estacion.json con las reglas de web/app/config.py.
%   cfg = sim3d_leer_json(texto)
%   jsondecode convierte null y [] en el mismo [] de MATLAB, pero la estación los trata distinto
%   en vision.inclinacion_max_deg: null apaga el descarte y [] (una lista, no un número) deja el
%   valor por defecto (filtros_vision). Aquí [] se cambia por el texto "[]" antes de decodificar,
%   para que sim3d_config lo trate como "no numérico" igual que Python.
texto = regexprep(char(texto), '("inclinacion_max_deg"\s*:\s*)\[\s*\]', '$1"[]"');
cfg = jsondecode(texto);
end
