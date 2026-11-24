function y = qd2_telemetry_pack(t, tel, version, guard_active, pose1, pose2, sp1, sp2, vel1, vel2, aux, att)
%QD2_TELEMETRY_PACK Arma el vector de 64 doubles Simulink -> Python (§3.3).
%   y = qd2_telemetry_pack(t, tel, version, guard_active, pose1, pose2,
%                          sp1, sp2, vel1, vel2, aux)
%   y = qd2_telemetry_pack(..., aux, att)
%   tel        : salida de qd2_mission_step.
%   version    : 1 PID, 2 MPC.
%   pose1/2    : [x y z yaw] medidos.  sp1/2: [x y z yaw] enviados al dron
%                (después del guard y del rate limiter).
%   vel1/2     : [vx vy vz].
%   aux        : [tracked1 tracked2 fsm1 fsm2 bat1 bat2 comm1 comm2]
%                (fsm = estado a bordo si se conoce, si no el inferido;
%                 batería NaN si no hay dato).
%   att        : (opcional) [roll1 pitch1 roll2 pitch2] en rad, POR DRON (no
%                por rol), de la misma Corrected Pose Data de OptiTrack que
%                pose1/2 (d(4), d(5)). Convención Tait-Bryan ZYX intrínseca,
%                R = Rz(yaw)*Ry(pitch)*Rx(roll), cuerpo FLU: roll > 0 baja el
%                lado derecho, pitch > 0 baja la nariz. Sin este argumento
%                (llamadores viejos) y(61:63) = 0.
%   y          : 64x1 (índices base 1 como en ARQUITECTURA.md; 55 = alerta;
%                61 = roll y 62 = pitch del EXPLORADOR, 63 = 1 si son válidos,
%                64 = dron_solo: 0 = modo dual (o modelo viejo), 1 o 2 = modo
%                solo explorador con ese dron).
%   Modo solo explorador: se reconoce por tel.explorer_id = 1 o 2 con
%   tel.responder_id = 0 (firma que nunca ocurre en dual: qd2_assign_roles da
%   (0,0), (1,2) o (2,1)). Entonces y(64) = explorer_id e y(12) = y(13) = 0
%   (no hay respondedor). Así sirve con el 'tel' que arman los bloques de
%   tierra desde Mission Info, sin campos nuevos.
%#codegen

y = zeros(64,1);
y(1) = t;
y(2) = tel.mission_state;
y(3) = version;
y(4) = tel.explorer_id;
y(5) = tel.responder_id;
y(6) = tel.explorer_wp_idx;
y(7) = tel.explorer_wp_total;
y(8) = tel.responder_target_id;
y(9) = tel.n_targets_known;
y(10) = tel.n_targets_visited;
y(11) = guard_active;
if tel.explorer_id == 2
    pe = pose2; pr = pose1;
else
    pe = pose1; pr = pose2;
end
y(12) = norm([pe(1) - pr(1), pe(2) - pr(2)]);
y(13) = pe(3) - pr(3);
y(14) = tel.explorer_done;
home = [tel.home1; tel.home2];
role = [tel.role1 tel.role2];
for i = 1:2
    o = 14 + 18*(i-1);
    if i == 1
        ps = pose1; sp = sp1; v = vel1;
    else
        ps = pose2; sp = sp2; v = vel2;
    end
    y(o+1) = aux(i);
    y(o+2) = aux(2+i);
    y(o+3:o+6) = [ps(1); ps(2); ps(3); ps(4)];
    y(o+7:o+10) = [sp(1); sp(2); sp(3); sp(4)];
    y(o+11:o+13) = [v(1); v(2); v(3)];
    y(o+14) = aux(4+i);
    y(o+15) = home(i,1);
    y(o+16) = home(i,2);
    y(o+17) = role(i);
    y(o+18) = aux(6+i);
end
y(51) = tel.visited_mask;
y(52) = tel.marking_progress;
y(53) = tel.queue_length;
y(54) = tel.cmd_seq_ack;
% 55: alerta (0 = nada): 1 homes demasiado cerca (START rechazado), 2 pausa
% por pérdida de rastreo, 3 timeout de despegue, 4 timeout de aterrizaje,
% 5 E-stop, 6 Takeoff abajo, 7 Arm abajo, 8 respaldo crudo (5-8 las pone la
% capa del operador en tierra), 9 falla a bordo de un dron (FSM 7/8/9, MPC),
% 10 protocolo MPC incorrecto.
y(55) = tel.alerta;
% 56-60: un objetivo de la tabla por mensaje, rotando: [id clase x y estado]
% (estado 1 pendiente, 2 asignado, 3 visitado). id = 0: tabla vacía.
if isfield(tel, 'obj_rot')
    y(56:60) = [tel.obj_rot(1); tel.obj_rot(2); tel.obj_rot(3); tel.obj_rot(4); tel.obj_rot(5)];
end
% 61-63: actitud del EXPLORADOR [roll pitch válido] (rad), de la misma pose
% OptiTrack que y(o+3:o+6). Válido = hay explorador (1 o 2), está rastreado
% (aux) y los dos ángulos son finitos; si no, 0 0 0 y la estación geolocaliza
% nivelada.
if nargin >= 12
    ie = round(tel.explorer_id);
    if ie == 1 || ie == 2
        r = att(2*ie - 1);
        p = att(2*ie);
        if aux(ie) > 0.5 && isfinite(r) && isfinite(p)
            y(61) = r;
            y(62) = p;
            y(63) = 1;
        end
    end
end
% 64: dron_solo (0 = dual). Solo explorador = explorador 1 o 2 sin respondedor:
% y(64) = ese dron y las distancias E-R (12, 13) en 0.
ie2 = round(tel.explorer_id);
ir2 = round(tel.responder_id);
if (ie2 == 1 || ie2 == 2) && ir2 == 0
    y(64) = ie2;
    y(12) = 0;
    y(13) = 0;
end
end
