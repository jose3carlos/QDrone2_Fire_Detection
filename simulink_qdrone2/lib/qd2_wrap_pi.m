function a = qd2_wrap_pi(a)
%QD2_WRAP_PI Envuelve un ángulo (o vector) a (-pi, pi].
%#codegen
a = a - 2*pi*floor((a + pi)/(2*pi));
end
