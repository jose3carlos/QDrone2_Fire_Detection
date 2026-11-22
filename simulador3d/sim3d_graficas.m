function archivos = sim3d_graficas(R, guardar, carpeta)
%SIM3D_GRAFICAS Gráficas de la corrida (como las del simulador original).
%   archivos = sim3d_graficas(R, guardar, carpeta)
%   Figura 1: posiciones (x, y, z de cada dron y su consigna), banderas
%             (estado de misión, arm/takeoff, guard, marcando, E terminó),
%             timers (permanencia de E en el waypoint, marca acumulada de R) y
%             distancias (horizontal mínima y vertical entre drones).
%   Figura 2: vista cenital de las trayectorias con la pista, los objetos
%             reales y los objetivos publicados.

if nargin < 2, guardar = false; end
archivos = {};
L = R.log;
t = L.t;
if numel(t) < 2, return; end
vis = 'on';
if guardar && ~R.opc.mostrar, vis = 'off'; end
cdr = [0.12 0.50 1.00; 1.00 0.55 0.10];
ver = 'PID'; if R.version == 2, ver = 'MPC'; end

f1 = figure('Name', sprintf('Registro de la corrida (%s)', ver), 'NumberTitle', 'off', ...
    'Position', [80 60 1200 900], 'Visible', vis, 'Color', 'w');
% ---- posiciones
ejes = {'x', 'y', 'z'};
for a = 1:3
    subplot(6, 1, a);
    hold on; grid on;
    for i = 1:2
        plot(t, squeeze(L.pos(a, :, i)), '-', 'Color', cdr(i,:), 'LineWidth', 1.3);
        plot(t, squeeze(L.sp(a, :, i)), '--', 'Color', cdr(i,:)*0.8, 'LineWidth', 0.9);
    end
    ylabel(sprintf('%s (m)', ejes{a}));
    if a == 1
        title(sprintf('Posiciones (continua) y consignas tras guard + limitador (discontinua) · versión %s', ver));
        legend({'D1', 'D1 consigna', 'D2', 'D2 consigna'}, 'Location', 'eastoutside');
    else
        legend({'D1', 'D1 cons.', 'D2', 'D2 cons.'}, 'Location', 'eastoutside');
    end
end
% ---- banderas (desplazadas para que se lean, como el original)
subplot(6, 1, 4); hold on; grid on;
stairs(t, L.estado/10*0.9, 'k', 'LineWidth', 1.2);
stairs(t, L.arm(1,:)*0.8 + 1.2, 'Color', cdr(1,:), 'LineWidth', 1.1);
stairs(t, L.takeoff(1,:)*0.8 + 2.2, 'Color', cdr(1,:)*0.7, 'LineWidth', 1.1);
stairs(t, L.arm(2,:)*0.8 + 3.2, 'Color', cdr(2,:), 'LineWidth', 1.1);
stairs(t, L.takeoff(2,:)*0.8 + 4.2, 'Color', cdr(2,:)*0.7, 'LineWidth', 1.1);
stairs(t, L.guard*0.8 + 5.2, 'r', 'LineWidth', 1.1);
stairs(t, L.r_marking*0.8 + 6.2, 'm', 'LineWidth', 1.1);
stairs(t, L.explorer_done*0.8 + 7.2, 'Color', [0 0.5 0], 'LineWidth', 1.1);
ylim([-0.2 8.2]);
set(gca, 'YTick', [0.5 1.6 2.6 3.6 4.6 5.6 6.6 7.6], 'YTickLabel', ...
    {'estado/10', 'arm D1', 'takeoff D1', 'arm D2', 'takeoff D2', 'guard', 'R marcando', 'E terminó'});
title('Banderas (desplazadas para que se lean)');
% ---- timers (como dwell_timer / photo_timer del original)
subplot(6, 1, 5); hold on; grid on;
plot(t, L.t_dwell, 'k', 'LineWidth', 1.1);
plot(t, L.t_mark_acum, 'm', 'LineWidth', 1.3);
yline(R.p.t_dwell_wp, 'k--', 't\_dwell\_wp');
yline(R.s_final.t_mark, 'm--', 't\_mark');
ylim([0 max(R.s_final.t_mark*1.3, 1)]);
ylabel('s'); title('Timers: permanencia de E en el waypoint y marca acumulada de R');
legend({'dwell de E en el waypoint', 'marca de R sobre el objetivo'}, 'Location', 'eastoutside');
% ---- distancias
subplot(6, 1, 6); hold on; grid on;
plot(t, L.dh, 'b', 'LineWidth', 1.3);
plot(t, abs(L.dz), 'Color', [0.85 0.4 0], 'LineWidth', 1.1);
if R.version == 1
    yline(R.s_final.d_min_h, 'r--', sprintf('d\\_min\\_h = %.2f m', R.s_final.d_min_h));
else
    yline(R.p.dz_min, 'r--', sprintf('dz\\_min = %.2f m', R.p.dz_min));
end
aire = squeeze(L.pos(3,:,1)) > 0.2 & squeeze(L.pos(3,:,2)) > 0.2;
if any(aire)
    [m, im] = min(L.dh + 1e6*~aire);
    plot(t(im), m, 'bo', 'MarkerFaceColor', 'b');
    text(t(im), m, sprintf('  mín. en vuelo %.3f m', m), 'Color', 'b', 'VerticalAlignment', 'top');
end
xlabel('t (s)'); ylabel('m');
title('Distancia horizontal y vertical entre drones');
legend({'horizontal', '|vertical|'}, 'Location', 'eastoutside');

% ---- figura 2: vista cenital
% mismo ancho para todos los subplots (las leyendas los encogen distinto)
hs = findobj(f1, 'Type', 'axes');
w = min(arrayfun(@(h) h.Position(3), hs));
for h = hs.', h.Position(3) = w; end

f2 = figure('Name', 'Trayectorias (vista cenital)', 'NumberTitle', 'off', 'Position', [120 80 760 700], ...
    'Visible', vis, 'Color', 'w');
hold on; grid on; axis equal;
set(gca, 'XDir', 'reverse');
V = R.esc.V;
plot(V(:,2), V(:,1), '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 4);
lx = R.p.xy_lim;
plot(lx([1 2 2 1 1]), lx([1 1 2 2 1]), 'k-');
for i = 1:2
    plot(squeeze(L.pos(2,:,i)), squeeze(L.pos(1,:,i)), '-', 'Color', cdr(i,:), 'LineWidth', 1.3);
    plot(R.esc.home(i,2), R.esc.home(i,1), 'p', 'MarkerSize', 13, 'MarkerFaceColor', cdr(i,:), 'MarkerEdgeColor', 'k');
end
for j = 1:numel(R.esc.obj)
    o = R.esc.obj(j);
    mk = 's'; if o.clase == 0, mk = '^'; end
    plot(o.y, o.x, mk, 'MarkerSize', 10, 'MarkerFaceColor', [0.85 0.15 0.1], 'MarkerEdgeColor', 'k');
end
s = R.s_final;
if s.n_obj > 0
    plot(s.obj_y(1:s.n_obj), s.obj_x(1:s.n_obj), 'kx', 'MarkerSize', 12, 'LineWidth', 2);
    for k = 1:s.n_obj
        text(s.obj_y(k) - 0.05, s.obj_x(k) + 0.05, sprintf('T%d', s.obj_id(k)), 'FontWeight', 'bold');
    end
end
xlabel('y (m)  [+y a la izquierda]'); ylabel('x (m)  [+x adelante]');
xlim(lx + [-0.2 0.2]); ylim(lx + [-0.2 0.2]);
title(sprintf('Trayectorias · %s · %d objetivos publicados (x), %d objetos reales (rojo)', ver, s.n_obj, numel(R.esc.obj)));
legend({'pista', 'arena', 'D1', 'home D1', 'D2', 'home D2'}, 'Location', 'southoutside', 'Orientation', 'horizontal');

if guardar
    if ~isfolder(carpeta), mkdir(carpeta); end
    pref = sprintf('%s_%s', R.opc.prefijo_capturas, lower(ver));
    archivos{1} = fullfile(carpeta, [pref '_graficas.png']);
    archivos{2} = fullfile(carpeta, [pref '_trayectorias.png']);
    exportgraphics(f1, archivos{1}, 'Resolution', 100);
    exportgraphics(f2, archivos{2}, 'Resolution', 100);
    if ~R.opc.mostrar
        close(f1); close(f2);
    end
end
end
