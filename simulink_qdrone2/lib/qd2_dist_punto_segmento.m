function [d, q] = qd2_dist_punto_segmento(pt, a, b)
%QD2_DIST_PUNTO_SEGMENTO Distancia 2D del punto pt al segmento [a,b].
%   [d, q] = qd2_dist_punto_segmento(pt, a, b) devuelve la distancia d y el
%   punto q del segmento más cercano. Solo usa las componentes x,y.
%#codegen
px = pt(1); py = pt(2);
ax = a(1);  ay = a(2);
bx = b(1);  by = b(2);
ux = bx - ax; uy = by - ay;
L2 = ux*ux + uy*uy;
s = 0;
if L2 > 1e-12
    s = ((px - ax)*ux + (py - ay)*uy)/L2;
    s = min(max(s, 0), 1);
end
q = [ax + s*ux, ay + s*uy];
d = sqrt((px - q(1))^2 + (py - q(2))^2);
end
