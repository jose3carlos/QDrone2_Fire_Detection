function y = qd2_rate_limit(y_prev, u, rate, dt, xy_norma)
%QD2_RATE_LIMIT Limitador de razón de consignas [x y z yaw].
%   y = qd2_rate_limit(y_prev, u, rate, dt)
%       elemento a elemento (igual al Rate Limiter2 original: cada eje por
%       separado, por lo que un movimiento diagonal sigue un camino en "L").
%   y = qd2_rate_limit(y_prev, u, rate, dt, 1)
%       limita x,y por la NORMA del vector (|v_xy| <= rate(1)), así la
%       consigna avanza en línea recta. Es lo que suponen el SEPARATION
%       GUARD y la planeación de regreso (camino recto entre consignas):
%       usar este modo en la versión PID.
%   rate puede ser escalar o del mismo tamaño que u.
%#codegen
r = rate(:).*ones(numel(u), 1);
paso = r*dt;
d = u(:) - y_prev(:);
d = min(max(d, -paso), paso);
if nargin >= 5 && xy_norma > 0.5 && numel(u) >= 2
    dxy = [u(1) - y_prev(1); u(2) - y_prev(2)];
    n = sqrt(dxy(1)^2 + dxy(2)^2);
    if n > paso(1)
        dxy = dxy*(paso(1)/n);
    end
    d(1) = dxy(1);
    d(2) = dxy(2);
end
y = reshape(y_prev(:) + d, size(y_prev));
end
