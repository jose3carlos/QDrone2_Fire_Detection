%CORRER_PRUEBAS Ejecuta todas las pruebas de la librería y muestra un resumen.
%   Desde MATLAB:  cd simulink_qdrone2/tests; correr_pruebas
%   (tarda ~3-4 min la primera vez por la compilación del modelo Simulink).
raiz_tests = fileparts(mfilename('fullpath'));
addpath(fullfile(fileparts(raiz_tests), 'lib'));
addpath(raiz_tests);
resultados = runtests(fullfile(raiz_tests, 'test_qd2_lib.m'));
disp(table(resultados));
fprintf('\nPasaron %d de %d pruebas.\n', sum([resultados.Passed]), numel(resultados));
