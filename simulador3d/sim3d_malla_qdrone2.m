function M = sim3d_malla_qdrone2()
%SIM3D_MALLA_QDRONE2 Malla procedural del Quanser QDrone 2.
%   M = sim3d_malla_qdrone2()
%
%   Medidas del manual "QDrone 2 User Manual - System Hardware" (tabla 10):
%   largo 442.58 mm (x), ancho 490.8 mm (y), alto 135.53 mm. Cuatro aros
%   de protección de fibra de carbono (doble aro: abajo y arriba, unidos por
%   postes), brazos en X, placas centrales con el Jetson Xavier NX y su
%   ventilador, batería LiPo abajo, 4 motores con hélices de 7" y la
%   RealSense D435 (90 x 25 x 25 mm) al frente, montada mirando hacia abajo
%   como en este proyecto (ARQUITECTURA §3.4, camara.offset_camara_m).
%
%   Marco del cuerpo: +x adelante, +y izquierda, +z arriba. Origen = centro
%   del dron a la altura de las patas (z = 0 es el suelo con el dron
%   aterrizado; coincide con la z de OptiTrack ya corregida con bias_z).
%
%   M.grupos(k): .nombre, .V (Nx3), .F (Mx3), .color (1x3, o [] = color del dron)
%   M.helice   : .V, .F de una hélice de 2 palas en su propio marco
%   M.motores  : 4x3 centro del cubo de cada hélice
%   M.sentido  : 1x4 sentido de giro (+1 antihorario visto desde arriba)
%   M.lente    : 1x3 posición de la lente de la RealSense
%   M.medidas  : [largo ancho alto] (m) de la malla resultante

Lx = 0.44258; Wy = 0.4908; Hz = 0.13553;
Ro = 0.112;               % radio exterior del aro
Ri = 0.100;               % radio interior del aro
a = Lx/2 - Ro;            % centros de los aros en x (+-0.1093)
b = Wy/2 - Ro;            % centros de los aros en y (+-0.1334)
cen = [a b; a -b; -a b; -a -b];   % FL, FR, RL, RR
z_abajo = [0.045 0.049];  % aro inferior / placa inferior
z_arriba = [Hz-0.004 Hz]; % aro superior
z_helice = 0.077;

G = struct('nombre', {}, 'V', {}, 'F', {}, 'color', {});
carbono = [0.13 0.13 0.15];

% ---------------- aros, postes, brazos y placas (carbono) ----------------
V = zeros(0,3); F = zeros(0,3);
for k = 1:4
    [V, F] = juntar(V, F, anillo(cen(k,:), Ri, Ro, z_abajo(1), z_abajo(2), 64));
end
% postes entre aro inferior y superior (se omiten los que caen dentro de otro aro)
for k = 1:4
    for ang = (0:9)*2*pi/10 + pi/10
        q = cen(k,:) + (Ri + Ro)/2*[cos(ang) sin(ang)];
        otros = cen([1:k-1 k+1:4], :);
        if all(sqrt(sum((otros - q).^2, 2)) > Ro - 0.003)
            [V, F] = juntar(V, F, cilindro([q z_abajo(2)], 0.0022, z_arriba(1) - z_abajo(2), 6));
        end
    end
end
% brazos del centro a cada motor
for k = 1:4
    [V, F] = juntar(V, F, barra([0 0], cen(k,:), 0.024, z_abajo(1), z_abajo(2)));
end
% placa inferior, placa superior y separadores
[V, F] = juntar(V, F, caja_c([0 0 mean(z_abajo)], [0.21 0.115 0.004]));
[V, F] = juntar(V, F, caja_c([-0.005 0 0.0995], [0.16 0.095 0.003]));
for sx = [-1 1]
    for sy = [-1 1]
        [V, F] = juntar(V, F, cilindro([0.07*sx 0.04*sy z_abajo(2)], 0.003, 0.0485, 6));
    end
end
% soporte de la cámara
[V, F] = juntar(V, F, caja_c([0.175 0 0.040], [0.05 0.05 0.004]));
G(end+1) = grupo('carbono', V, F, carbono);

% ---------------- aro superior (color del dron, para distinguirlos) -------
V = zeros(0,3); F = zeros(0,3);
for k = 1:4
    [V, F] = juntar(V, F, anillo(cen(k,:), Ri + 0.001, Ro, z_arriba(1), z_arriba(2), 64));
end
% LEDs traseros y tapa de los motores
for sy = [-1 1]
    [V, F] = juntar(V, F, caja_c([-0.105 0.03*sy 0.052], [0.012 0.02 0.006]));
end
for k = 1:4
    [V, F] = juntar(V, F, cilindro([cen(k,:) 0.068], 0.0145, 0.005, 16));
end
G(end+1) = grupo('acento', V, F, []);

% ---------------- motores (metal) -----------------------------------------
V = zeros(0,3); F = zeros(0,3);
for k = 1:4
    [V, F] = juntar(V, F, cilindro([cen(k,:) z_abajo(2)], 0.014, 0.019, 16));
    [V, F] = juntar(V, F, cilindro([cen(k,:) 0.073], 0.0035, 0.006, 8));
end
G(end+1) = grupo('metal', V, F, [0.50 0.50 0.55]);

% ---------------- Jetson, ventilador, patas, lentes (negro) ---------------
V = zeros(0,3); F = zeros(0,3);
[V, F] = juntar(V, F, caja_c([-0.035 0 0.0665], [0.090 0.070 0.030]));
[V, F] = juntar(V, F, cilindro([-0.035 0 0.101], 0.021, 0.006, 20));
for sx = [-1 1]
    for sy = [-1 1]
        [V, F] = juntar(V, F, cilindro([0.075*sx 0.075*sy 0], 0.006, z_abajo(1), 8));
    end
end
% cara inferior de la D435 (lentes hacia abajo)
for yl = [-0.030 -0.010 0.022]
    [V, F] = juntar(V, F, cilindro([0.205 yl 0.0105], 0.0055, 0.001, 14));
end
G(end+1) = grupo('negro', V, F, [0.05 0.05 0.06]);

% ---------------- RealSense D435 (aluminio) -------------------------------
[V, F] = caja([0.205 0 0.024], [0.025 0.090 0.025]);
G(end+1) = grupo('aluminio', V, F, [0.80 0.81 0.84]);

% ---------------- tarjeta (PCB) -------------------------------------------
[V, F] = caja([0.035 0 0.1025], [0.065 0.075 0.003]);
G(end+1) = grupo('pcb', V, F, [0.10 0.42 0.22]);

% ---------------- batería LiPo --------------------------------------------
[V, F] = caja([-0.005 0 0.027], [0.140 0.046 0.034]);
G(end+1) = grupo('bateria', V, F, [0.20 0.24 0.34]);

M.grupos = G;
[M.helice.V, M.helice.F] = helice(0.0889);   % 7 pulgadas de diámetro
M.motores = [cen z_helice*ones(4,1)];
M.sentido = [1 -1 -1 1];
M.lente = [0.205 0 0.010];
M.cen_aros = cen;
M.r_aro = Ro;
% medidas reales de la malla (para comprobar contra el manual)
Vt = vertcat(G.V);
M.medidas = max(Vt) - min(Vt);
end

% =========================================================================
function g = grupo(nombre, V, F, color)
g.nombre = nombre; g.V = V; g.F = F; g.color = color;
end

function [V, F] = juntar(V, F, V2, F2)
if nargin == 3       % juntar(V, F, {V2, F2}) desde una función con 2 salidas
    F2 = V2{2}; V2 = V2{1};
end
F = [F; F2 + size(V, 1)];
V = [V; V2];
end

function out = caja_c(c, s)
[V, F] = caja(c, s);
out = {V, F};
end

function [V, F] = caja(c, s)
h = s/2;
V = [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1].*h + c;
Q = [1 2 3 4; 5 8 7 6; 1 5 6 2; 2 6 7 3; 3 7 8 4; 4 8 5 1];
F = [Q(:, [1 2 3]); Q(:, [1 3 4])];
end

function out = anillo(c, ri, ro, z0, z1, n)
% banda anular extruida (aro de protección)
t = (0:n-1)'*2*pi/n;
ci = [cos(t) sin(t)];
V = [c + ri*ci, z0*ones(n,1); c + ro*ci, z0*ones(n,1); ...
     c + ri*ci, z1*ones(n,1); c + ro*ci, z1*ones(n,1)];
i = (1:n)'; j = [2:n 1]';
IB = i; OB = n + i; IT = 2*n + i; OT = 3*n + i;
IBj = j; OBj = n + j; ITj = 2*n + j; OTj = 3*n + j;
F = [IT OT OTj; IT OTj ITj;          % arriba
     IB OBj OB; IB IBj OBj;          % abajo
     OB OBj OTj; OB OTj OT;          % pared exterior
     IB IT ITj; IB ITj IBj];         % pared interior
out = {V, F};
end

function out = cilindro(c, r, h, n)
% cilindro vertical: base en c(3), altura h
t = (0:n-1)'*2*pi/n;
V = [c(1) + r*cos(t), c(2) + r*sin(t), c(3)*ones(n,1); ...
     c(1) + r*cos(t), c(2) + r*sin(t), (c(3) + h)*ones(n,1); ...
     c(1) c(2) c(3); c(1) c(2) c(3) + h];
i = (1:n)'; j = [2:n 1]';
cb = 2*n + 1; ct = 2*n + 2;
F = [i j n+j; i n+j n+i; ...
     cb*ones(n,1) j i; ct*ones(n,1) n+i n+j];
out = {V, F};
end

function out = barra(p0, p1, ancho, z0, z1)
% caja horizontal entre dos puntos (x, y)
d = p1 - p0; L = norm(d); u = d/L; nrm = [-u(2) u(1)]*ancho/2;
q = [p0 - nrm; p1 - nrm; p1 + nrm; p0 + nrm];
V = [q z0*ones(4,1); q z1*ones(4,1)];
Q = [1 2 3 4; 5 8 7 6; 1 5 6 2; 2 6 7 3; 3 7 8 4; 4 8 5 1];
F = [Q(:, [1 2 3]); Q(:, [1 3 4])];
out = {V, F};
end

function [V, F] = helice(R)
% dos palas con cuerda decreciente y un poco de torsión (pala a lo largo de +x)
r = linspace(0.010, R, 10)';
c = 0.024*(1 - 0.55*r/R);
c(end) = 0.006;                         % punta redondeada
tw = 0.004*(1 - r/R);                   % torsión (diferencia de altura borde de ataque/salida)
LE = [r, 0.6*c, tw];
TE = [r, -0.4*c, -tw];
n = numel(r);
Vb = [LE; TE];
i = (1:n-1)';
Fb = [i i+1 n+i+1; i n+i+1 n+i];
Vb2 = [-Vb(:,1) -Vb(:,2) Vb(:,3)];      % segunda pala (girada 180 grados)
V = [Vb; Vb2];
F = [Fb; Fb + size(Vb, 1)];
% cubo
t = (0:11)'*2*pi/12;
Vh = [0.009*cos(t) 0.009*sin(t) -0.004*ones(12,1); 0.009*cos(t) 0.009*sin(t) 0.004*ones(12,1); 0 0 0.004];
k = (1:12)'; j = [2:12 1]';
Fh = [k j 12+j; k 12+j 12+k; 25*ones(12,1) 12+k 12+j];
F = [F; Fh + size(V, 1)];
V = [V; Vh];
end
