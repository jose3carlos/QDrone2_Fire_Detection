%% QDRONE2_SIM3D  Simulador 3D en tiempo real de 2 QDrone 2 (detección de tomas y fuegos)
% Mejora del simulador 2D del compañero (referencias/qdrone_sim_realtime_companero.m).
%
% - 2 QDrone 2 en 3D con malla procedural fiel al manual (442.58 x 490.8 x 135.53 mm):
%   aros de carbono, hélices que giran, Jetson, batería y RealSense al frente.
%   Dron 1 = azul, dron 2 = naranja; la etiqueta muestra el rol (EXPLORADOR / RESPONDEDOR).
% - Arena 3x3 m con retícula y ejes OptiTrack (+x adelante, +y izquierda), la N invertida
%   de config/estacion.json (tubo PVC blanco) y fuegos/tomas (STL de impresion3d/stl).
% - Misión completa con la MISMA librería que Simulink (simulink_qdrone2/lib):
%   qd2_mission_step + qd2_separation_guard (PID) o vuelo apilado con qd2_mpc_position y
%   downwash (MPC); stack a bordo (qd2_onboard_step) y planta (qd2_quad_plant).
% - Cámara del explorador (frustum y huella), detector simulado con las reglas de
%   clusters de la estación (radio 0.25 m, mínimo 3 observaciones).
% - Puente §3.3 en tcpip://127.0.0.1:18700 (64 doubles salen, 32 entran) para que la UI
%   web (web\iniciar_web.bat --fuente sim3d) lo muestre y lo controle.
% - Modo SOLO EXPLORADOR (opc.modo_mision = 'solo_explorador' o mision.modo de la config):
%   un solo QDrone recorre la pista; el otro no recibe arm/takeoff ('fuera': oculto;
%   'piso': en gris "NO USADO"); y(64) de la telemetría = dron que vuela.
%
% Uso: abre este archivo, ajusta la sección de OPCIONES y presiona Run (F5).
% Botones/teclas de la figura: s START · p PAUSE · r RESUME · a ABORT · l LAND_ALL ·
% v cambia la vista · + / - velocidad · q salir.
%
% Para correrlo sin abrir este archivo (por ejemplo en -batch), define antes una
% estructura "opc_sim3d" con los campos que quieras cambiar; se mezclan con estos.
% Requiere MATLAB R2022a o más nuevo; no usa Simulink ni toolboxes.

%% ---------------- OPCIONES (edita aquí) ----------------
opc = struct();
opc.version          = 'PID';    % 'PID' = nunca uno encima del otro (guard) · 'MPC' = apilados + downwash
opc.escenario        = 'escenarios/escenario_base.json';   % o 'aleatorio', o 'web' (objetos de la cámara
                                                           % sintética de la UI: úsalo con use_vision_python = true)
opc.n_fuegos         = 2;        % con 'aleatorio'
opc.n_tomas          = 2;        % con 'aleatorio'
opc.semilla          = [];       % semilla del escenario aleatorio ([] = distinta cada vez)

% Modo de misión ([] = el de config/estacion.json -> mision.modo / mision.dron_solo).
% Solo explorador (un solo QDrone recorre la pista, sin respondedor), p. ej. la demo del lab:
%   opc.modo_mision = 'solo_explorador'; opc.dron_solo = 1;
%   opc.escenario = 'escenarios/escenario_solo_lab.json';
opc.modo_mision      = [];       % [] | 'dual' | 'solo_explorador'
opc.dron_solo        = [];       % [] | 1 | 2
opc.dron_ausente     = 'fuera';  % solo explorador: 'fuera' (el otro dron sin rastreo, oculto) o 'piso' (en gris)
opc.config           = [];       % [] = config/estacion.json; o un struct ya decodificado

opc.tiempo_real      = true;     % false = lo más rápido posible
opc.velocidad        = 1.0;      % 2 = el doble de rápido que el tiempo real (funciona con la UI conectada)
opc.t_final          = 400;      % s máximo

opc.usar_tcp         = true;     % servir el puerto 18700 para la UI web
opc.use_vision_python = false;   % true = los objetivos los publica la estación Python. Con false la UI web
                                 % NO dibuja los objetivos del simulador (ver README, "Con la UI web")
opc.auto_start       = [];       % s para un START automático ([] = espera botón/tecla/UI)
opc.auto_land_all    = 4.0;      % s en home -> LAND_ALL automático (solo estación interna)

opc.mostrar          = true;     % figura visible
opc.estelas          = true;
opc.capturas         = [];       % tiempos (s) en los que se guarda un PNG en capturas/
opc.graficas_finales = true;

%% ---------------- correr ----------------
if exist('opc_sim3d', 'var') && isstruct(opc_sim3d) %#ok<*NODEF>
    campos_ = fieldnames(opc_sim3d);
    for i_ = 1:numel(campos_)
        opc.(campos_{i_}) = opc_sim3d.(campos_{i_});
    end
    clear campos_ i_
end
addpath(fileparts(mfilename('fullpath')));
R = sim3d_correr(opc);
