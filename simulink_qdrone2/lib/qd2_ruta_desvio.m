function [via, n_via, ok] = qd2_ruta_desvio(A, B, O, r, p)
%QD2_RUTA_DESVIO Ruta A -> (via) -> B que mantiene distancia horizontal >= r
%   al obstáculo puntual O (otro dron quieto). Solo usa x,y.
%
%   [via, n_via, ok] = qd2_ruta_desvio(A, B, O, r, p)
%   n_via = 0 : el segmento recto A-B ya libra el obstáculo (via = B).
%   n_via = 1 : pasar primero por via (1x2), luego ir a B.
%   ok    = 0 : no se encontró ruta (via = B); el llamador decide.
%   Si A o B ya están a menos de r de O, la holgura exigida en ese extremo
%   es la distancia que ya tienen (no se puede exigir más).
%   Los puntos de paso se recortan a la arena (p.xy_lim -/+ p.xy_margen).
%#codegen

a = [A(1) A(2)];
b = [B(1) B(2)];
o = [O(1) O(2)];
via = b;
n_via = 0;
ok = 1;
dA = norm(a - o);
dB = norm(b - o);
d0 = qd2_dist_punto_segmento(o, a, b);
if d0 >= min([r, dA, dB]) - 1e-9
    return;
end
lo = p.xy_lim(1) + p.xy_margen;
hi = p.xy_lim(2) - p.xy_margen;
mejor = inf;
nd = 24;
ok = 0;
radios = [r + 0.10, r + 0.30, r + 0.50];
for ir = 1:3
    for k = 1:nd
        ang = 2*pi*(k-1)/nd;
        c = o + radios(ir)*[cos(ang) sin(ang)];
        c = min(max(c, lo), hi);
        if norm(c - o) < r - 1e-9
            continue;
        end
        d1 = qd2_dist_punto_segmento(o, a, c);
        d2 = qd2_dist_punto_segmento(o, c, b);
        if d1 >= min(r, dA) - 1e-9 && d2 >= min(r, dB) - 1e-9
            L = norm(c - a) + norm(b - c);
            if L < mejor
                mejor = L;
                via = c;
                ok = 1;
            end
        end
    end
end
if ok
    n_via = 1;
end
end
