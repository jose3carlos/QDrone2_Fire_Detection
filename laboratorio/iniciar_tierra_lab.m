function iniciar_tierra_lab(modo)
%INICIAR_TIERRA_LAB Abre, compila y arranca el modelo de tierra PID (R2022a) en la PC del laboratorio.
%   iniciar_tierra_lab('solo')  -> QD2_MissionCtrl_Dual_PID     + config/estacion.json (solo explorador)
%   iniciar_tierra_lab('dual')  -> QD2_MissionCtrl_Dual_PID_2D  + config/estacion_dual.json (2 drones)
%   Equivale a: cd versiones\R2022a, open_system, qd2_cargar_config, HARDWARE -> Monitor & Tune
%   (qc_build_model + qc_start_model(mdl, 'external'), qc_start_model.html). Lo llama
%   laboratorio\iniciar_lab.ps1. NO guarda el modelo.
if nargin < 1 || isempty(modo), modo = 'solo'; end
lab = fileparts(mfilename('fullpath'));
raiz = fileparts(lab);
dirv = fullfile(raiz, 'simulink_qdrone2', 'versiones', 'R2022a');
if strcmpi(modo, 'dual')
    mdl = 'QD2_MissionCtrl_Dual_PID_2D';
    json = fullfile(raiz, 'config', 'estacion_dual.json');
else
    mdl = 'QD2_MissionCtrl_Dual_PID';
    json = fullfile(raiz, 'config', 'estacion.json');
end
fprintf('\n=== Modelo de tierra: %s (%s) ===\n', mdl, json);
fprintf('Antes de seguir: Motive cerrado, Command Source = 0, switches Arm / Takeoff-Land / E-Stop ABAJO.\n');
cd(dirv);                                   % el build y external mode usan la carpeta actual
open_system(fullfile(dirv, [mdl '.slx']));
qd2_cargar_config(mdl, json);               % comillas en ExtModeMexArgs y rutas de Motive (en memoria)
args = get_param(mdl, 'ExtModeMexArgs');
if isempty(args) || args(1) ~= ''''
    error('iniciar_tierra_lab:args', 'ExtModeMexArgs sin comillas: %s', args);
end
fprintf('Compilando y descargando (unos 20 s)...\n');
t0 = tic;
qc_build_model(mdl);
fprintf('Build OK en %.0f s. Conectando y arrancando (Monitor & Tune)...\n', toc(t0));
qc_start_model(mdl, 'external');
fprintf(['\nModelo de tierra corriendo. El tiempo T= de Simulink debe avanzar.\n' ...
    'NO guardes el modelo. Para detenerlo: Stop en la pestana HARDWARE (nunca con un dron en el aire).\n']);
end
