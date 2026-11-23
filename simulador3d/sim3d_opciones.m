function opc = sim3d_opciones(opc)
%SIM3D_OPCIONES Valores por defecto de las opciones del simulador 3D.
%   opc = sim3d_opciones()        devuelve todas las opciones por defecto.
%   opc = sim3d_opciones(opc)     completa los campos que falten en opc.
%
%   Ver la cabecera de qdrone2_sim3d.m para la descripción de cada opción.

d = struct();
% ---- Control y escenario --------------------------------------------------
d.version        = 'PID';      % 'PID' (nunca apilados, SEPARATION GUARD) o 'MPC' (apilados + downwash)
d.escenario      = 'escenarios/escenario_base.json';  % archivo JSON, 'aleatorio' o 'web' (los mismos objetos que
                                                      % dibuja la cámara sintética de la UI web: web.mock_anomalias)
d.n_fuegos       = 2;          % solo con escenario 'aleatorio'
d.n_tomas        = 2;          % solo con escenario 'aleatorio'
d.semilla        = [];         % semilla del escenario aleatorio ([] = al azar)
d.semilla_ruido  = 1;          % semilla del ruido de sensores y del detector
d.homes          = [];         % [x1 y1; x2 y2] para forzar los home (si no, los del escenario)
% ---- Tiempo ---------------------------------------------------------------
d.t_final        = 400;        % s de simulación como máximo
d.dt             = 0.02;       % s, paso de la simulación (50 Hz, igual que el MPC)
d.tiempo_real    = true;       % true = 1 s simulado = 1 s de reloj (por velocidad)
d.velocidad      = 1.0;        % factor de tiempo real (2 = el doble de rápido)
d.parar_en_done  = true;       % terminar unos segundos después de llegar a DONE
d.t_extra_done   = 3.0;        % s que sigue corriendo después de DONE
% ---- Estación interna (cuando no hay estación Python) ---------------------
d.auto_start     = [];         % s: START automático ([] = esperar botón, tecla o Python)
d.auto_land_all  = 4.0;        % s en home tras RETURN_HOME -> LAND_ALL automático ([] = nunca)
d.auto_land      = [];         % [] = config (mision.auto_land); 0/1 para forzarlo
d.comandos       = zeros(0,2); % [t cmd] comandos programados (1 START, 2 ABORT, 3 LAND_ALL, 4 PAUSE, 5 RESUME)
d.overrides      = [0 0 0 0];  % [z_E z_R t_mark d_min_h] que mandaría la estación (0 = por defecto)
d.perdida_rastreo = zeros(0,3); % [dron t1 t2] dron sin rastreo (pose NaN) en [t1, t2)
d.ruido_pos      = 0.002;      % m, ruido de OptiTrack
% ---- Puente TCP (§3.3) ----------------------------------------------------
d.usar_tcp       = true;       % servir el protocolo de 64/32 doubles para la UI web
d.puerto         = [];         % [] = config/estacion.json -> red.puerto_puente (18700)
d.host_tcp       = '127.0.0.1';% '127.0.0.1' = solo esta PC; '0.0.0.0' = toda la red
d.backend_tcp    = 'auto';     % 'auto' | 'java' | 'tcpserver'
d.hz_telemetria  = 50;         % Hz de la telemetría de 64 doubles
d.actitud_en_tel = true;       % false = emula un modelo de Simulink viejo: y(61:63) = 0 (sin actitud)
d.use_vision_python = false;   % true = los objetivos vienen de la estación Python (ranuras 5-19)
d.t_toma_interna = 3.0;        % s de RELOJ sin paquetes de Python -> la estación interna retoma el heartbeat
d.t_hb_python    = 0.5;        % s de RELOJ: Python cuenta como vivo si mandó un heartbeat nuevo en esta ventana
                               % (se mide en reloj para que x2..x16 o tiempo_real = false no pausen la misión)
% ---- Detector simulado ----------------------------------------------------
d.hz_camara      = 15;         % fps de la cámara del explorador
d.p_deteccion    = 0.85;       % probabilidad de detección con el objeto bien visible
d.ruido_px       = 3.0;        % px, ruido del centro de la caja
d.p_falso        = 0.01;       % probabilidad de un falso positivo por cuadro
d.p_confusion    = 0.02;       % probabilidad de confundir la clase
d.tam_min_px     = 10;         % px, tamaño aparente mínimo para detectar
d.sesgo_montaje_deg = [0.8 -0.6]; % [roll pitch] error del soporte de la cámara que la estación NO conoce
d.usar_actitud   = [];         % [] = config (vision.usar_actitud); false = geolocalizar nivelado, como antes
d.inclinacion_max_deg = [];    % [] = config (vision.inclinacion_max_deg, 15); 0 o Inf = sin descarte
% ---- Gráficos -------------------------------------------------------------
d.mostrar        = true;       % figura visible
d.vista          = 1;          % 1 general, 2 cenital, 3 sigue al explorador, 4 primer plano del dron 1
d.fps            = 30;         % cuadros por segundo de la animación
d.usar_stl       = true;       % cargar los STL de impresion3d/stl (si existen)
d.estelas        = true;       % estela de cada dron
d.t_estela       = 40;         % s de estela
d.capturas       = [];         % s: tiempos en los que se guarda un PNG
d.capturas_vistas = [];       % vista (1-4) para cada captura (mismo tamaño que capturas; 0 = no cambiar)
d.capturas_estados = [];       % estados de misión: PNG al entrar a cada uno (p. ej. [2 4 5 6 8])
d.carpeta_capturas = 'capturas';
d.prefijo_capturas = 'sim3d';
d.tam_figura     = [1500 860]; % px
d.graficas_finales = true;     % gráficas de la corrida al terminar
d.guardar_graficas = [];       % [] = solo si no hay figura visible; true/false para forzarlo
d.verbose        = true;       % mensajes en la consola

if nargin < 1 || isempty(opc)
    opc = d;
    return;
end
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(opc, f{i})
        opc.(f{i}) = d.(f{i});
    end
end
g = fieldnames(opc);
for i = 1:numel(g)
    if ~isfield(d, g{i})
        warning('sim3d:opcion', 'Opción desconocida "%s" (se ignora).', g{i});
    end
end
end
