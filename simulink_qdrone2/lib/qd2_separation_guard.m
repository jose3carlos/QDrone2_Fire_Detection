function [spE_o, spR_o, guard_active, gs] = qd2_separation_guard(pE, pR, spE, spR, p, gs, ctx)
%QD2_SEPARATION_GUARD Filtro de consignas de la versión PID ("nunca uno
%   encima del otro"): distancia horizontal >= d_min_h entre poses medidas,
%   entre consignas y a lo largo del camino recto del dron que cede.
%
%   [spE_o, spR_o, guard_active, gs] = qd2_separation_guard(pE, pR, spE, spR, p)
%   [spE_o, spR_o, guard_active, gs] = qd2_separation_guard(pE, pR, spE, spR, p, gs, ctx)
%
%   pE, pR   : poses medidas [x y z ...] del explorador / respondedor.
%   spE, spR : consignas [x y z yaw] de qd2_mission_step (por rol).
%   p        : qd2_params().
%   gs       : estado (qd2_guard_init()). Sin él, el guard es sin memoria.
%   ctx      : [r_marking, prioridad_R, d_min_h, solo_respaldo, look_x, look_y]
%              = tel.ctx_guard de qd2_mission_step (4 o 6 elementos).
%              look = punto de la ruta de E más adelante de su consigna; el
%              dron que cede no espera ni se refugia sobre [consigna, look].
%              r_marking=1: R está marcando -> si E se acercaría, E espera.
%              prioridad_R=1: E ya terminó la N -> R tiene prioridad y E cede.
%              d_min_h: override de la estación; solo se usa si es mayor
%              que p.d_min_h (nunca reduce la separación).
%              solo_respaldo=1: (despegue, regreso, aterrizaje, pausa) la
%              misión ya secuencia los movimientos (con margen D+m); el
%              guard solo retiene horizontalmente al que se acerque a menos
%              de D del otro, sin evasión.
%
%   Reglas (D = d_min_h, m = p.guard_margen):
%   - El dron con prioridad (A, normalmente E) avanza salvo que el otro (B)
%     esté a < D+m de su segmento [pose, consigna]; entonces A espera
%     (mantiene su x,y previos; z sigue libre).
%   - B va a su consigna solo si su camino recto [pose, consigna] queda a
%     >= D+2m del segmento de A y de la consigna previa de A. Si no, espera
%     en un punto fijo seguro; si ese punto deja de ser seguro, se evade a
%     un punto a D+3m de A, dentro de la arena, sin acercarse en el camino.
%   - Si B está marcando, B no se mueve y A espera.
%   - Respaldo final: si las consignas de salida quedan a < D, A espera.
%   guard_active = 1 si alguno está siendo retenido o desviado.
%   Poses o consignas NaN: se usan la última pose finita / la última
%   salida; las salidas nunca son NaN.
%#codegen

if nargin < 6
    gs = qd2_guard_init();
end
% ctx normalizado a 6 elementos sin indexar fuera de rango (codegen)
c6 = [0 0 0 0 NaN NaN];
if nargin >= 7
    for i = 1:min(numel(ctx), 6)
        c6(i) = ctx(i);
    end
end
% d_min_h efectivo: el override solo puede AUMENTAR la separación
D = p.d_min_h;
if isfinite(c6(3)) && c6(3) > D
    D = c6(3);
end
m = p.guard_margen;
b_lock = c6(1) > 0.5;
prioR = c6(2) > 0.5;
respaldo = c6(4) > 0.5;
dA_espera = D + m;
dB_seguro = D + 2*m;
dB_evasion = D + 3*m;

% ---------------- Entradas sin NaN ----------------------------------------
% Pose no finita -> última pose finita. Consigna no finita -> última salida.
% Sin ninguna pose finita todavía -> se repiten las salidas anteriores.
spE = [spE(1) spE(2) spE(3) spE(4)];
spR = [spR(1) spR(2) spR(3) spR(4)];
for i = 1:4
    if ~isfinite(spE(i)), spE(i) = gs.outE(i); end
    if ~isfinite(spR(i)), spR(i) = gs.outR(i); end
end
if isfinite(pE(1)) && isfinite(pE(2))
    gs.pE_ok = [pE(1) pE(2)];
    gs.tiene_pose(1) = 1;
end
if isfinite(pR(1)) && isfinite(pR(2))
    gs.pR_ok = [pR(1) pR(2)];
    gs.tiene_pose(2) = 1;
end
if gs.tiene_pose(1) < 0.5 || gs.tiene_pose(2) < 0.5
    spE_o = gs.outE;
    spR_o = gs.outR;
    guard_active = 1;
    return;
end
qE = gs.pE_ok;
qR = gs.pR_ok;
if gs.init < 0.5
    gs.outE = [qE(1) qE(2) spE(3) spE(4)];
    gs.outR = [qR(1) qR(2) spR(3) spR(4)];
    gs.holdB = qR;
    gs.init = 1;
end
if prioR ~= (gs.prio_prev > 0.5)
    gs.modoB = 0;
end
gs.prio_prev = double(prioR);

if prioR
    pA = qR; spA = spR; prevA = gs.outR;
    pB = qE; spB = spE; prevB = gs.outE;
    b_lock = false;
else
    pA = qE; spA = spE; prevA = gs.outE;
    pB = qR; spB = spR; prevB = gs.outR;
end
gA = spA(1:2);
gB = spB(1:2);
look = gA;
if ~prioR && isfinite(c6(5)) && isfinite(c6(6))
    look = [c6(5) c6(6)];
end

% ---------------- Dron con prioridad (A) ---------------------------------
dB = min(qd2_dist_punto_segmento(pB, pA, gA), qd2_dist_punto_segmento(prevB(1:2), pA, gA));
espera = dB < dA_espera;
if respaldo
    % solo retener si se acerca más de lo que ya está
    espera = (dB < D) && (qd2_dist_punto_segmento(pB, pA, gA) < norm(pB - pA) - 1e-6);
end
if espera
    outA = [prevA(1) prevA(2) spA(3) spA(4)];
else
    outA = spA;
end

% ---------------- Dron que cede (B) ---------------------------------------
if b_lock
    outB = spB;
    gs.modoB = 0;
elseif respaldo
    dseg = min(qd2_dist_segmentos(pB, gB, pA, gA), qd2_dist_punto_segmento(prevA(1:2), pB, gB));
    acerca = dseg < D && dseg < norm(pB - pA) - 1e-6;
    if acerca
        if gs.modoB == 0
            gs.holdB = pB;
            gs.modoB = 1;
        end
        outB = [gs.holdB(1) gs.holdB(2) spB(3) spB(4)];
    else
        gs.modoB = 0;
        outB = spB;
    end
else
    histeresis = 0;
    if gs.modoB > 0
        histeresis = 0.05;
    end
    dseg = min([qd2_dist_segmentos(pB, gB, pA, gA), qd2_dist_segmentos(pB, gB, gA, look), ...
        qd2_dist_punto_segmento(prevA(1:2), pB, gB)]);
    if dseg >= dB_seguro + histeresis
        gs.modoB = 0;
        outB = spB;
    else
        if gs.modoB == 0
            gs.holdB = pB;
            gs.modoB = 1;
        end
        % el punto de espera debe ser seguro y el camino hacia él no debe
        % acercar a B a A (A pudo moverse desde que se eligió)
        d_ahora = min(qd2_dist_punto_segmento(pB, pA, gA), norm(pB - prevA(1:2)));
        dh = min([qd2_dist_punto_segmento(gs.holdB, pA, gA), qd2_dist_punto_segmento(gs.holdB, gA, look), ...
            norm(gs.holdB - prevA(1:2))]);
        dcam = min(qd2_dist_segmentos(pB, gs.holdB, pA, gA), qd2_dist_punto_segmento(prevA(1:2), pB, gs.holdB));
        if dh < dB_seguro || dcam < min(d_ahora, dA_espera) - 0.02
            if min(d_ahora, qd2_dist_punto_segmento(pB, gA, look)) >= dB_seguro
                gs.holdB = pB;          % detenerse donde está
                gs.modoB = 1;
            else
                gs.holdB = evasion(pB, gB, pA, gA, look, prevA(1:2), dB_seguro, dA_espera, dB_evasion, p);
                gs.modoB = 2;
            end
        end
        outB = [gs.holdB(1) gs.holdB(2) spB(3) spB(4)];
    end
end

% ---------------- Respaldo: consignas a >= D -------------------------------
if norm(outA(1:2) - outB(1:2)) < D && norm(prevA(1:2) - outB(1:2)) >= norm(outA(1:2) - outB(1:2))
    outA = [prevA(1) prevA(2) spA(3) spA(4)];
    espera = true;
end

gs.esperaA = double(espera);
guard_active = double(espera || gs.modoB > 0);
if prioR
    spR_o = outA; spE_o = outB;
else
    spE_o = outA; spR_o = outB;
end
gs.outE = spE_o;
gs.outR = spR_o;
end

% =========================================================================
function c = evasion(pB, gB, pA, gA, look, prevA, d_seguro, d_espera, radio, p)
% Punto de evasión para B. Candidatos: círculos alrededor del punto de A
% más cercano (radios 'radio' y d_seguro+0.05) y un retroceso radial.
%  - Restricción dura: el camino recto pB->c no se acerca a la posición
%    actual de A (ni a su consigna previa) más de lo que ya está (o de
%    d_espera). Si ningún candidato la cumple, B se queda donde está.
%  - Preferencia: destino a >= d_seguro de la zona de A ([pA,gA] y
%    [gA,look]) y camino sin cruzar esa zona -> el más cercano;
%    si no hay, el que deje más holgura en el destino.
lo = p.xy_lim(1) + p.xy_margen;
hi = p.xy_lim(2) - p.xy_margen;
[d_seg, q] = qd2_dist_punto_segmento(pB, pA, gA);
d_ahora = min(d_seg, norm(pB - prevA));
req = min(d_ahora, d_seguro) - 0.02;
dref = min([norm(pB - pA), norm(pB - prevA), d_espera]) - 0.02;
nd = p.n_dir_evasion;
c = pB;
mejor1 = inf;
mejor2 = -inf;
c2 = pB;
radios = [radio, d_seguro + 0.05];
v = pB - q;
if norm(v) < 1e-6
    v = [1 0];
end
for ir = 1:3
    for k = 1:64
        if k > nd || (ir == 3 && k > 1)
            break;
        end
        if ir == 3
            cand = min(max(pB + 0.4*v/norm(v), lo), hi);     % retroceso radial
        else
            ang = 2*pi*(k-1)/nd;
            cand = min(max(q + radios(ir)*[cos(ang) sin(ang)], lo), hi);
        end
        if norm(cand - pB) < 0.05
            continue;
        end
        dpos = min(qd2_dist_punto_segmento(pA, pB, cand), qd2_dist_punto_segmento(prevA, pB, cand));
        if dpos < dref
            continue;                                         % restricción dura
        end
        dc = min([qd2_dist_punto_segmento(cand, pA, gA), qd2_dist_punto_segmento(cand, gA, look), norm(cand - prevA)]);
        dcam = min(qd2_dist_segmentos(pB, cand, pA, gA), dpos);
        if dc >= d_seguro && dcam >= req
            costo = norm(cand - pB) + 0.5*norm(cand - gB);
            if costo < mejor1
                mejor1 = costo;
                c = cand;
            end
        elseif dc > mejor2
            mejor2 = dc;
            c2 = cand;
        end
    end
end
if ~isfinite(mejor1)
    if mejor2 > d_ahora
        c = c2;
    else
        c = pB;
    end
end
end
