function S = qd2_sim_parametros(cfg_json)
%QD2_SIM_PARAMETROS Parámetros de QD2_Sim_Dual (se crean en el InitFcn).
%   S = qd2_sim_parametros()  devuelve el struct QD2_SIM con los valores por
%   defecto del escenario simulado. Para cambiar un escenario sin tocar el
%   modelo: antes de sim(), crea QD2_SIM_OVERRIDE en el workspace base con
%   los campos que quieras cambiar (p. ej. QD2_SIM_OVERRIDE.home1 = [-1 0.5]).
%
%   Campos:
%     home1, home2   [x y] inicial de cada dron (m). Separación >= 1.2 m
%                    (con < 1.05 m la versión PID rechaza START, alerta 1).
%     objetivos      [K x 3] = [x y clase] de las anomalías simuladas
%                    (clase 0 fuego, 1 toma). Por defecto sobre la N.
%     ruido_pos      desviación del ruido de OptiTrack (m); ruido_yaw (rad).
%     trk_off        [dron t1 t2]: sin rastreo (NaN) en [t1, t2). [] = nunca.
%                    Como en el stack real, el dron afectado pasa a Error
%                    (estado 7, rampa de empuje) a los 0.2 s y luego a 9.
%     comm_off       [dron t1 t2]: enlace PC->dron caído en [t1, t2) (el dron
%                    se queda con el último paquete). 0.5 s -> estado 7;
%                    1 s armado -> 9 (FLIGHT SAFETY corta los motores).
%     falla_fsm      [dron t]: el stack a bordo de ese dron entra a Error
%                    (estado 7: rampa de empuje y luego 9) en t. [0 -1] = nunca.
%     falla_apilado  0 nada; 1 = falla el dron de ARRIBA, 2 = el de ABAJO,
%                    después de 1 s apilados (dh < 0.15 m, dz > 0.5 m).
%     proto_mal      [dron t1 t2]: la respuesta de 8 doubles de ese dron
%                    llega sin la etiqueta y(8) = 2 en [t1, t2) (alerta 10
%                    a los 0.5 s, versión MPC).
%     resp_off       [dron t1 t2]: la respuesta dron->PC no llega en
%                    [t1, t2): tierra retiene la última y el loopback crece.
%                    Con comm_off en el mismo intervalo = enlace caído en
%                    los dos sentidos (tierra no ve la FSM 7 del dron).
%                    Versión MPC en vuelo -> alerta 9 (enlace perdido).
%     lat_pose       s, latencia de la pose OptiTrack que usa el dron (0.015).
%     pose_onboard   1 = el dron se estima con OptiTrack (ruido, 100 Hz,
%                    latencia) como el real; 0 = estado ideal (comparación).
%     dw_factor      downwash real / modelo del MPC (1.3 = el real es 30 %
%                    más fuerte que el que supone el feedforward).
%     det_*          detector simulado (cámara cenital del EXPLORADOR):
%                    hfov/vfov (grados), z mínima, frames para publicar,
%                    probabilidad de detección por frame, ruido (m).
%     t_start        s, START de la estación simulada (fuente = 1).
%     t_land_espera  s en RETURN_HOME antes de que la estación simulada mande
%                    LAND_ALL (solo si auto_land = false).
%     parar_en_done  1 = detiene la simulación 3 s después de DONE.
%     puerto         puerto del PYTHON BRIDGE (config: red.puerto_puente).
%                    Para pruebas sin la UI usa otro (p. ej. 18751) con
%                    qd2_correr_sim(v, struct('puerto', 18751)): 18700 es el
%                    de la prueba E2E con la estación.
%     semilla        semilla de los bloques Random Number.

raiz = fileparts(fileparts(mfilename('fullpath')));
if nargin < 1 || isempty(cfg_json)
    cfg_json = fullfile(fileparts(raiz), 'config', 'estacion.json');
end
S.home1 = [-1.0 0.0];
S.home2 = [1.0 -0.3];
S.objetivos = [0.0 1.2 0; 0.45 0.45 1; -0.6 -1.2 1];
S.ruido_pos = 0.002;
S.ruido_yaw = 0.002;
S.trk_off = [0 -1 -1];
S.comm_off = [0 -1 -1];
S.falla_fsm = [0 -1];
S.falla_apilado = 0;
S.proto_mal = [0 -1 -1];
S.resp_off = [0 -1 -1];
S.lat_pose = 0.015;
S.pose_onboard = 1;
S.dw_factor = 1.3;
S.det_hfov = 69.4;
S.det_vfov = 42.5;
S.det_z_min = 0.8;
S.det_frames = 3;
S.det_p = 0.85;
S.det_ruido = 0.03;
S.det_offset = [0.20 0.0];
S.t_start = 3.0;
S.t_land_espera = 6.0;
S.parar_en_done = 1;
S.puerto = 18700;
S.semilla = 1;
% Valores de config/estacion.json (única fuente de verdad)
if exist(cfg_json, 'file')
    try
        c = jsondecode(fileread(cfg_json));
        if isfield(c, 'red') && isfield(c.red, 'puerto_puente')
            S.puerto = double(c.red.puerto_puente);
        end
        if isfield(c, 'camara')
            if isfield(c.camara, 'hfov_deg'), S.det_hfov = double(c.camara.hfov_deg); end
            if isfield(c.camara, 'vfov_deg'), S.det_vfov = double(c.camara.vfov_deg); end
            if isfield(c.camara, 'offset_camara_m')
                o = double(c.camara.offset_camara_m(:))';
                S.det_offset = o(1:2);
            end
        end
        if isfield(c, 'vision') && isfield(c.vision, 'min_observaciones')
            S.det_frames = double(c.vision.min_observaciones);
        end
    catch e
        warning('qd2_sim_parametros:json', 'No pude leer %s: %s', cfg_json, e.message);
    end
end
% Sobrescrituras del usuario (workspace base)
try
    ov = evalin('base', 'QD2_SIM_OVERRIDE');
    f = fieldnames(ov);
    for i = 1:numel(f)
        S.(f{i}) = ov.(f{i});
    end
catch
end
if isempty(S.trk_off)
    S.trk_off = [0 -1 -1];
end
if isempty(S.comm_off)
    S.comm_off = [0 -1 -1];
end
if isempty(S.falla_fsm)
    S.falla_fsm = [0 -1];
end
if isempty(S.proto_mal)
    S.proto_mal = [0 -1 -1];
end
if isempty(S.resp_off)
    S.resp_off = [0 -1 -1];
end
% tamaño fijo para los bloques (hasta 16 objetivos simulados)
O = nan(16, 3);
k = min(size(S.objetivos, 1), 16);
O(1:k, :) = S.objetivos(1:k, :);
S.objetivos_tabla = O;
end
