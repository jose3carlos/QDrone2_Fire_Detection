function [est, t_mal] = qd2_enlace_mpc(t_mal, t, tel1, tel2, lb, cs, p, version)
%QD2_ENLACE_MPC Valida en tierra la respuesta de 8 doubles de cada dron MPC.
%   [est, t_mal] = qd2_enlace_mpc(t_mal, t, tel1, tel2, lb, cs, p, version)
%
%   ARQUITECTURA §3.2: el dron MPC responde [eco ts, FSM (1-9), batería,
%   estado MPC, T, roll, pitch, 2]. El índice 8 = 2 es la etiqueta
%   "protocolo MPC v1". Una respuesta es VÁLIDA si y(8) == 2 y la FSM es un
%   entero entre 1 y 9.
%
%   t_mal  : [1x2] estado interno (inicia en [-1 -1]): desde cuándo cada dron
%            responde mal estando conectado.
%   t      : tiempo (s).
%   tel1/2 : respuesta de 8 doubles de cada dron (lo que entrega el Stream
%            Server de Communication QDrone i; sin conexión, [0; NaN(7,1)]).
%   lb     : [1x2] loopback (s) = tiempo - timestamp recibido (Signal Check).
%   cs     : [1x2] Communication Status de tierra (1 = enlace perdido).
%   version: 1 = PID (el dron solo devuelve el eco: no hay nada que validar),
%            2 = MPC.
%
%   Un dron está "conectado" si su eco es reciente (loopback finito y
%   < p.t_proto_mal) y tierra no declara pérdida de enlace.
%   - Conectado y respondiendo mal más de p.t_proto_mal s seguidos ->
%     protocolo incorrecto (alerta 10 en el supervisor: dron no disponible).
%     Ejemplo: un binario PID (eco de 1 double) leído en unidades de 8
%     doubles da y(8) = un timestamp, nunca 2.
%   - Sin conexión (eco viejo o Communication Status = 1) DESPUÉS de haber
%     estado conectado -> enlace perdido. La FSM se reporta como 0
%     (desconocida): con el Stream Server en "most recent" la última
%     respuesta queda retenida y NO dice nada del estado actual del dron.
%     A bordo, la "Detect Communication Loss" lleva la FSM a 7 a los 0.5 s,
%     así que el supervisor trata un dron EN VUELO sin enlace como dron en
%     falla (alerta 9). Antes del primer contacto no se marca nada.
%
%   est = [fsm1 fsm2 cod1 cod2]: FSM a bordo validada (0 = desconocida:
%   sin conexión, respuesta inválida o versión PID) y código del enlace
%   por dron: 0 bien, 1 protocolo incorrecto sostenido, 2 enlace perdido.
%   Es la entrada "Onboard Status [4]" del MISSION SUPERVISOR
%   (qd2_mission_step: in.fsm_a_bordo = est(1:2), in.proto_mal = est(3:4)).
%
%   t_mal (salida) = [t_mal1 t_mal2 visto1 visto2] (1x4). Como entrada se
%   acepta [-1 -1] (estado inicial de versiones anteriores) o [-1 -1 0 0].
%#codegen

st = [-1 -1 0 0];
for k = 1:min(numel(t_mal), 4)
    st(k) = t_mal(k);
end
est = zeros(1, 4);
if version < 1.5
    t_mal = [-1 -1 0 0];
    return;
end
for i = 1:2
    if i == 1
        y = tel1;
    else
        y = tel2;
    end
    f = y(2);
    valida = isfinite(y(8)) && y(8) == 2 && isfinite(f) && f == round(f) && f >= 1 && f <= 9;
    conectado = isfinite(lb(i)) && lb(i) >= -p.t_proto_mal && lb(i) < p.t_proto_mal && ...
        ~(cs(i) > 0.5);
    if conectado
        st(2+i) = 1;                     % ya hubo contacto con este dron
        if valida
            est(i) = f;
        end
    elseif st(2+i) > 0.5
        est(2+i) = 2;                    % enlace perdido después de haber estado conectado
    end
    if conectado && ~valida
        if st(i) < 0
            st(i) = t;
        end
        if t - st(i) > p.t_proto_mal
            est(2+i) = 1;
        end
    else
        st(i) = -1;
    end
end
t_mal = st;
end
