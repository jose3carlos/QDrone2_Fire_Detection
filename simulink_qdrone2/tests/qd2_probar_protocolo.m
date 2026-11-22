function R = qd2_probar_protocolo()
%QD2_PROBAR_PROTOCOLO Prueba el PROTOCOL CHECK a bordo de QD2_DroneStack_MPC.
%   R = qd2_probar_protocolo()
%   Extrae el código del bloque Communication/PROTOCOL CHECK del modelo
%   mpc/QD2_DroneStack_MPC.slx (tal como está) a una carpeta temporal y lo
%   ejecuta como función MATLAB con tramas sintéticas de un vuelo:
%     a) tierra MPC actual (24 doubles, índice 24 = 2): se aceptan todas;
%     b) tierra MPC vieja (índice 24 = 0): se rechazan todas (el dron no arma);
%     c) tierra PID (16 doubles) leída en unidades de 24: se rechazan;
%     d) campos binarios corruptos (arm = 0.5) o timestamp NaN: se rechazan.
%   Una trama rechazada repite la última válida (el timestamp se congela y la
%   detección de pérdida de enlace original lleva la FSM a Error).
raiz = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(raiz, 'lib'));
if exist('qc_check_arguments', 'file') == 0 && exist('C:/Program Files/Quanser/QUARC/quarc', 'dir')
    addpath('C:/Program Files/Quanser/QUARC/quarc', '-end');
end
md = 'QD2_DroneStack_MPC';
w = warning('off', 'all');
load_system(fullfile(raiz, 'mpc', [md '.slx']));
warning(w);
cfg = get_param([md '/Communication/PROTOCOL CHECK'], 'MATLABFunctionConfiguration');
codigo = cfg.FunctionScript;
close_system(md, 0);
d = fullfile(tempdir, 'qd2_probar_protocolo');
if ~exist(d, 'dir'), mkdir(d); end
fid = fopen(fullfile(d, 'u_protocolo.m'), 'w');
fwrite(fid, strrep(codigo, 'function rcv_ok = fcn(rcv)', 'function rcv_ok = u_protocolo(rcv)'));
fclose(fid);
addpath(d); rehash;
limpiar = onCleanup(@() rmpath(d));

% vuelo sintético a 1 kHz: armado, takeoff, pose y consignas que cambian
N = 5000;
t = (1:N)'*0.001;
p16 = zeros(N, 16);
p16(:,1) = t > 0.5;                 % arm
p16(:,2) = t > 1.5;                 % takeoff
p16(:,5) = 1;                       % isTracking
p16(:,6:8) = [0.3*sin(t) 0.2*cos(t) 1.0*min(t/2, 1)];
p16(:,12:14) = [0.3*ones(N,1) -0.2*ones(N,1) ones(N,1)];
p16(:,16) = 100 + t;                % timestamp
extra = repmat([1 0 1.5 0 0 0 1 2], N, 1);
R.a = probar([p16 extra]);
extra0 = extra; extra0(:,8) = 0;
R.b = probar([p16 extra0]);
flujo = reshape(p16', [], 1);       % 16 doubles por paquete, leídos de 24 en 24
nt = floor(numel(flujo)/24);
R.c = probar(reshape(flujo(1:nt*24), 24, nt)');
mal = [p16 extra]; mal(1:2:end,1) = 0.5; mal(2:2:end,16) = NaN;
R.d = probar(mal);
fprintf(['[PROTOCOL CHECK a bordo] a) MPC con 24 = 2: %d/%d aceptadas | b) MPC con 24 = 0: %d/%d | ' ...
    'c) PID 16 leído en 24: %d/%d | d) binarios/timestamp corruptos: %d/%d | arm/takeoff fuera de {0,1} a la salida: %d\n'], ...
    R.a.ok, R.a.n, R.b.ok, R.b.n, R.c.ok, R.c.n, R.d.ok, R.d.n, R.a.mal + R.b.mal + R.c.mal + R.d.mal);
R.pasa = R.a.ok == R.a.n && R.b.ok == 0 && R.c.ok == 0 && R.d.ok == 0 && ...
    (R.a.mal + R.b.mal + R.c.mal + R.d.mal) == 0;
end

function r = probar(F)
clear u_protocolo
r.n = size(F, 1);
r.ok = 0;
r.mal = 0;
for k = 1:r.n
    y = u_protocolo(F(k,:)');
    if isequal(y(:), F(k,:)')
        r.ok = r.ok + 1;            % trama aceptada (sale tal cual)
    end
    if any(y(1:3) ~= 0 & y(1:3) ~= 1)
        r.mal = r.mal + 1;
    end
end
end
