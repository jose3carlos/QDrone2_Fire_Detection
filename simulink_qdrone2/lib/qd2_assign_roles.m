function [explorer_id, responder_id, home1, home2, ok] = qd2_assign_roles(p1, p2, tracked1, tracked2)
%QD2_ASSIGN_ROLES Asigna EXPLORADOR / RESPONDEDOR y captura los home.
%   [explorer_id, responder_id, home1, home2, ok] =
%       qd2_assign_roles(p1, p2, tracked1, tracked2)
%
%   p1, p2  : poses medidas de los drones 1 y 2 (al menos [x y], RSF).
%   tracked : 1 si OptiTrack rastrea el rigid body.
%   El dron con MENOR x (atrás) es el explorador (id 1 o 2); el otro es el
%   respondedor. Empate exacto en x -> el dron 1 es el explorador.
%   home1/home2 = [x y] actuales de cada dron (1x2).
%   ok = 1 si ambos están rastreados y sus posiciones son finitas.
%
%   Esta función es pura (sin memoria). La condición "rastreados de forma
%   estable durante 1 s" la impone qd2_mission_step (estado WAIT_TRACKING,
%   p.t_tracking_estable) antes de llamarla.
%#codegen

home1 = [p1(1), p1(2)];
home2 = [p2(1), p2(2)];
ok = (tracked1 > 0.5) && (tracked2 > 0.5) && all(isfinite(home1)) && all(isfinite(home2));
if p2(1) < p1(1)
    explorer_id = 2;
    responder_id = 1;
else
    explorer_id = 1;
    responder_id = 2;
end
if ~ok
    explorer_id = 0;
    responder_id = 0;
end
ok = double(ok);
end
