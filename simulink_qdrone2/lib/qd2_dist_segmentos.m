function d = qd2_dist_segmentos(a1, b1, a2, b2)
%QD2_DIST_SEGMENTOS Distancia 2D mínima entre los segmentos [a1,b1] y [a2,b2].
%#codegen
if segmentos_cruzan(a1, b1, a2, b2)
    d = 0;
    return;
end
d1 = qd2_dist_punto_segmento(a1, a2, b2);
d2 = qd2_dist_punto_segmento(b1, a2, b2);
d3 = qd2_dist_punto_segmento(a2, a1, b1);
d4 = qd2_dist_punto_segmento(b2, a1, b1);
d = min([d1 d2 d3 d4]);
end

function c = segmentos_cruzan(a1, b1, a2, b2)
o1 = orient(a1, b1, a2);
o2 = orient(a1, b1, b2);
o3 = orient(a2, b2, a1);
o4 = orient(a2, b2, b1);
c = (o1*o2 < 0) && (o3*o4 < 0);
end

function o = orient(a, b, c)
o = (b(1) - a(1))*(c(2) - a(2)) - (b(2) - a(2))*(c(1) - a(1));
end
