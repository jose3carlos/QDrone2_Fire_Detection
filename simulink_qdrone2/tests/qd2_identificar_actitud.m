function R = qd2_identificar_actitud()
%QD2_IDENTIFICAR_ACTITUD Identifica el retardo de actitud y verifica el trim
%   con el vuelo real del Equipo2 (equipo2_original/saved_data, 20-nov-2026).
%   Fuentes: commander_data (fila 19 = estado FSM, 34 = empuje del
%   controlador, 35/36 = roll/pitch comandados) y QDrone_data (filas 2/3 y
%   11/12 = roll/pitch estimados por las dos IMU, 24 = batería).
%   Ajusta  est = k * lag(cmd retrasado d)  por mínimos cuadrados en una
%   malla (tau, d), y reporta el primer orden equivalente sin retardo.
%   Tarda ~1-2 min (los logs pesan ~100 MB).
raiz = fileparts(fileparts(mfilename('fullpath')));
b = fullfile(raiz, 'equipo2_original', 'saved_data', 'QD2_DroneStack_PID_2021a');
q = load(fullfile(b, 'log_20-Nov-2026_20-05-50_QDrone2.mat'));
c = load(fullfile(b, 'log_20-Nov-2026_20-05-50_Commander.mat'));
s = load(fullfile(b, 'log_20-Nov-2026_20-05-50_Stabilizer.mat'));
Q = q.QDrone_data; C = c.commander_data; S = s.stabilizer_data;
idx = find(C(19,:) == 5);                     % FLIGHT
dt = 0.001;
nombres = {'roll', 'pitch'};
cmd = {C(35,idx), C(36,idx)};
est = {(Q(2,idx) + Q(11,idx))/2, (Q(3,idx) + Q(12,idx))/2};
taus = 0.02:0.01:0.30;
dels = 0:0.02:0.14;
for a = 1:2
    u = cmd{a}; y = est{a} - mean(est{a});
    mejor = [inf 0 0 0];
    for dd = dels
        nd = round(dd/dt);
        ud = [u(1)*ones(1,nd) u(1:end-nd)];
        for tau = taus
            al = dt/tau;
            yh = filter(al, [1 al-1], ud, (1-al)*u(1));
            yh = yh - mean(yh);
            k = (yh*y')/(yh*yh');
            J = sqrt(mean((y - k*yh).^2));
            if J < mejor(1), mejor = [J tau dd k]; end
        end
    end
    sin_ret = [inf 0];
    for tau = 0.02:0.005:0.4
        al = dt/tau;
        yh = filter(al, [1 al-1], u, (1-al)*u(1)); yh = yh - mean(yh);
        k = (yh*y')/(yh*yh');
        J = sqrt(mean((y - k*yh).^2));
        if J < sin_ret(1), sin_ret = [J tau]; end
    end
    R.(nombres{a}).tau = mejor(2);
    R.(nombres{a}).retardo = mejor(3);
    R.(nombres{a}).ganancia = mejor(4);
    R.(nombres{a}).VAF = 100*(1 - mejor(1)^2/var(y));
    R.(nombres{a}).tau_sin_retardo = sin_ret(2);
    fprintf('%s: retardo %.2f s + lag %.3f s, ganancia %.2f, VAF %.1f%% | primer orden equivalente tau = %.3f s\n', ...
        nombres{a}, mejor(3), mejor(2), mejor(4), R.(nombres{a}).VAF, sin_ret(2));
end
% Trim: empuje total del stabilizer en vuelo vs 1.504 kg * 9.81
T = S(8, idx);
R.T_media = mean(T);
R.u_z_media = mean(C(34, idx));
R.bateria = [min(Q(24, idx)) max(Q(24, idx))];
fprintf('Empuje en FLIGHT: media %.3f N (trim del modelo 14.754 N), ctrl z medio %.3f N -> masa equivalente %.3f kg; bateria %.2f-%.2f V\n', ...
    R.T_media, R.u_z_media, R.T_media/9.81, R.bateria(1), R.bateria(2));
end
