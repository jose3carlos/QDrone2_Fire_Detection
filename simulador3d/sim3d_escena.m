function G = sim3d_escena(accion, G, varargin)
%SIM3D_ESCENA Figura 3D del simulador: arena, pista, objetos, drones y HUD.
%
%   G = sim3d_escena('crear', [], C, esc, opc, raiz)
%   G = sim3d_escena('actualizar', G, E)      E = estado a dibujar (sim3d_correr)
%   G = sim3d_escena('vista', G, modo)        1 general, 2 cenital, 3 sigue al explorador,
%                                             4 primer plano del dron 1
%
%   Solo usa gráficos base de MATLAB (patch, line, text, hgtransform,
%   uicontrol): funciona en R2022a+ sin toolboxes.

switch accion
    case 'crear'
        G = crear(varargin{:});
    case 'actualizar'
        G = actualizar(G, varargin{1});
    case 'vista'
        G = poner_vista(G, varargin{1}, []);
    otherwise
        error('sim3d:escena', 'Acción desconocida: %s', accion);
end
end

% =========================================================================
%                                CREAR
% =========================================================================
function G = crear(C, esc, opc, raiz)
col.fondo = [0.11 0.12 0.15];
col.tapete = [0.22 0.23 0.26];
col.piso = [0.16 0.17 0.19];
col.reja = [0.36 0.38 0.43];
col.texto = [0.92 0.93 0.95];
col.tenue = [0.62 0.65 0.70];
col.dron = [0.12 0.50 1.00; 1.00 0.55 0.10];     % dron 1 azul, dron 2 naranja
col.estado_obj = [0.75 0.75 0.75; 1.00 0.85 0.10; 1.00 0.50 0.05; 0.20 0.90 0.35];
G.col = col;
G.opc = opc;
p = C.p;
visible = 'on';
if ~opc.mostrar, visible = 'off'; end
G.fig = figure('Name', 'QDrone 2 - simulador 3D (Equipo 2)', 'NumberTitle', 'off', ...
    'Color', col.fondo, 'Visible', visible, 'Position', [40 40 opc.tam_figura], ...
    'Renderer', 'opengl', 'MenuBar', 'none', 'ToolBar', 'figure', 'InvertHardcopy', 'off');
setappdata(G.fig, 'cola_cmd', []);
setappdata(G.fig, 'salir', false);
setappdata(G.fig, 'vista', 1);
setappdata(G.fig, 'vel', opc.velocidad);
set(G.fig, 'WindowKeyPressFcn', @tecla);

% ---------------- ejes 3D -------------------------------------------------
ax = axes('Parent', G.fig, 'Position', [0.0 0.065 0.70 0.935], 'Color', col.fondo, ...
    'XColor', col.tenue, 'YColor', col.tenue, 'ZColor', col.tenue, 'Projection', 'perspective', ...
    'DataAspectRatio', [1 1 1], 'Clipping', 'off', 'FontSize', 8);
hold(ax, 'on');
axis(ax, 'off');
xlim(ax, [-2.1 2.1]); ylim(ax, [-2.1 2.1]); zlim(ax, [-0.01 2.6]);
G.ax = ax;

% piso fuera de la arena y tapete 3x3
patch(ax, [-2.1 2.1 2.1 -2.1], [-2.1 -2.1 2.1 2.1], -0.002*[1 1 1 1], col.piso, 'EdgeColor', 'none', ...
    'FaceLighting', 'none');
lx = p.xy_lim;
patch(ax, lx([1 2 2 1]), lx([1 1 2 2]), [0 0 0 0], col.tapete, 'EdgeColor', 'none', 'FaceLighting', 'none');
% retícula cada 0.25 m (más marcada cada 0.5 m)
g = lx(1):0.25:lx(2);
XL = []; YL = []; XM = []; YM = [];
for v = g
    if abs(mod(v + 1e-9, 0.5)) < 1e-6
        XM = [XM v v NaN lx(1) lx(2) NaN]; YM = [YM lx(1) lx(2) NaN v v NaN]; %#ok<AGROW>
    else
        XL = [XL v v NaN lx(1) lx(2) NaN]; YL = [YL lx(1) lx(2) NaN v v NaN]; %#ok<AGROW>
    end
end
line(ax, XL, YL, 0.001*ones(size(XL)), 'Color', col.reja*0.85, 'LineWidth', 0.5);
line(ax, XM, YM, 0.001*ones(size(XM)), 'Color', col.reja, 'LineWidth', 0.8);
line(ax, lx([1 2 2 1 1]), lx([1 1 2 2 1]), 0.002*ones(1,5), 'Color', [0.98 0.78 0.20], 'LineWidth', 2.5);
% marco OptiTrack (RSF): +x adelante, +y izquierda, +z arriba
Lf = 0.45;
flecha(ax, [0 0 0.003], [Lf 0 0], [0.95 0.25 0.25]);
flecha(ax, [0 0 0.003], [0 Lf 0], [0.25 0.90 0.35]);
flecha(ax, [0 0 0.003], [0 0 Lf], [0.35 0.55 1.00]);
text(ax, Lf + 0.06, 0, 0.02, '+x adelante', 'Color', [0.95 0.45 0.45], 'FontSize', 8, 'FontWeight', 'bold');
text(ax, 0, Lf + 0.06, 0.02, '+y izquierda', 'Color', [0.45 0.95 0.55], 'FontSize', 8, 'FontWeight', 'bold');
text(ax, 0, 0, Lf + 0.05, '+z', 'Color', [0.55 0.70 1.00], 'FontSize', 8, 'FontWeight', 'bold');
text(ax, lx(2) + 0.08, lx(1), 0.01, sprintf('arena %gx%g m', diff(lx), diff(lx)), ...
    'Color', col.tenue, 'FontSize', 8);
% cámaras OptiTrack (decorativas) en postes
for q = [1.95 -1.95; 1.95 1.95; 2.05 0].'
    line(ax, [q(1) q(1)], [q(2) q(2)], [0 2.35], 'Color', [0.30 0.31 0.34], 'LineWidth', 2);
    [Vc, Fc] = caja([q(1) q(2) 2.40], [0.10 0.10 0.07]);
    patch(ax, 'Vertices', Vc, 'Faces', Fc, 'FaceColor', [0.08 0.08 0.09], 'EdgeColor', 'none');
    plot3(ax, q(1)*0.97, q(2)*0.97, 2.40, 'o', 'MarkerSize', 5, 'MarkerFaceColor', [0.9 0.15 0.15], ...
        'MarkerEdgeColor', 'none');
end

% ---------------- pista (PVC 3/4" SCH40, Ø 26.7 mm, blanco) ---------------
V = esc.V;
r_tubo = 0.0267/2;
for k = 1:size(V, 1) - 1
    [Vt, Ft] = tubo([V(k,:) r_tubo], [V(k+1,:) r_tubo], r_tubo, 18);
    patch(ax, 'Vertices', Vt, 'Faces', Ft, 'FaceColor', [0.97 0.97 0.97], 'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', 'AmbientStrength', 0.55);
end
[xs, ys, zs] = sphere(12);
for k = 2:size(V, 1) - 1
    surf(ax, V(k,1) + 1.15*r_tubo*xs, V(k,2) + 1.15*r_tubo*ys, r_tubo + 1.15*r_tubo*zs, ...
        'FaceColor', [0.95 0.95 0.95], 'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'AmbientStrength', 0.55);
end
for k = [1 size(V, 1)]     % tapones
    surf(ax, V(k,1) + r_tubo*xs, V(k,2) + r_tubo*ys, r_tubo + r_tubo*zs, ...
        'FaceColor', [0.90 0.90 0.90], 'EdgeColor', 'none');
end

% ---------------- objetos reales (fuegos y tomas) -------------------------
O = sim3d_mallas_objetos(raiz, opc.usar_stl);
G.origen_mallas = O.origen;
G.obj_anillo = gobjects(1, numel(esc.obj));
G.obj_txt = gobjects(1, numel(esc.obj));
t = linspace(0, 2*pi, 40);
for j = 1:numel(esc.obj)
    o = esc.obj(j);
    tr = hgtransform('Parent', ax);
    if o.clase == 0
        piezas = O.fuego.(o.variante);
    else
        piezas = O.toma;
    end
    for q = 1:numel(piezas)
        patch('Parent', tr, 'Vertices', piezas(q).V, 'Faces', piezas(q).F, 'FaceColor', piezas(q).color, ...
            'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'AmbientStrength', 0.45, 'SpecularStrength', 0.2);
    end
    tr.Matrix = makehgtform('translate', [o.x o.y 0], 'zrotate', o.yaw);
    G.obj_anillo(j) = line(ax, o.x + 0.13*cos(t), o.y + 0.13*sin(t), 0.004*ones(size(t)), ...
        'Color', col.estado_obj(1,:), 'LineWidth', 1.2, 'LineStyle', ':');
    G.obj_txt(j) = text(ax, o.x, o.y, o.h + 0.07, '', 'Color', col.texto, 'FontSize', 7, ...
        'HorizontalAlignment', 'center', 'Interpreter', 'none');
end
% objetivos estimados (publicados): marcador x + etiqueta
G.obj_est = line(ax, NaN, NaN, NaN, 'LineStyle', 'none', 'Marker', 'x', 'MarkerSize', 10, ...
    'LineWidth', 2, 'Color', [1 1 1]);
G.obj_est_txt = gobjects(1, 32);
for k = 1:32
    G.obj_est_txt(k) = text(ax, NaN, NaN, NaN, '', 'Color', [1 1 1], 'FontSize', 7, ...
        'FontWeight', 'bold', 'Interpreter', 'none');
end
G.obs_ult = line(ax, NaN, NaN, NaN, 'LineStyle', 'none', 'Marker', '.', 'MarkerSize', 9, ...
    'Color', [1 0.4 0.9]);
% ruta del explorador (waypoints de la N a z_E)
G.ruta = line(ax, NaN, NaN, NaN, 'Color', col.dron(1,:)*0.8, 'LineStyle', ':', 'LineWidth', 1);
G.wp_act = line(ax, NaN, NaN, NaN, 'LineStyle', 'none', 'Marker', 'd', 'MarkerSize', 7, ...
    'MarkerFaceColor', [1 1 1], 'MarkerEdgeColor', 'none');
G.n_wp = -1;

% ---------------- drones ---------------------------------------------------
M = sim3d_malla_qdrone2();
G.malla = M;
% la pose es el punto corregido de Motive (z = 0 con el dron en el piso); el dron gira alrededor del
% pivote del rigid body, C.cam.pivote m más arriba (web/app/geo.py, sim3d_camara 'pose')
G.pivote = 0;
if isfield(C.cam, 'pivote'), G.pivote = C.cam.pivote; end
[Vd, Fd] = disco(0.0889, 28);
for i = 1:2
    c = col.dron(i,:);
    D.tr = hgtransform('Parent', ax);
    for k = 1:numel(M.grupos)
        gk = M.grupos(k);
        cc = gk.color;
        if isempty(cc), cc = c; end
        patch('Parent', D.tr, 'Vertices', gk.V, 'Faces', gk.F, 'FaceColor', cc, 'EdgeColor', 'none', ...
            'FaceLighting', 'gouraud', 'AmbientStrength', 0.5, 'DiffuseStrength', 0.7, ...
            'SpecularStrength', 0.25, 'BackFaceLighting', 'lit');
    end
    D.helice = gobjects(1, 4);
    D.disco = gobjects(1, 4);
    for k = 1:4
        D.helice(k) = hgtransform('Parent', D.tr);
        patch('Parent', D.helice(k), 'Vertices', M.helice.V, 'Faces', M.helice.F, 'FaceColor', c*0.85 + 0.1, ...
            'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'AmbientStrength', 0.6, 'BackFaceLighting', 'lit');
        D.helice(k).Matrix = makehgtform('translate', M.motores(k,:));
        D.disco(k) = patch('Parent', D.tr, 'Vertices', Vd + M.motores(k,:), 'Faces', Fd, 'FaceColor', c, ...
            'FaceAlpha', 0.10, 'EdgeColor', 'none', 'Visible', 'off');
    end
    D.ang = rand(1,4)*pi;
    % sombra, burbuja de separación, consigna, estela, etiqueta
    D.sombra = patch(ax, 'XData', 0.24*cos(t), 'YData', 0.22*sin(t), 'ZData', 0.003*ones(size(t)), ...
        'FaceColor', [0 0 0], 'FaceAlpha', 0.35, 'EdgeColor', 'none');
    D.burbuja = line(ax, NaN, NaN, NaN, 'Color', c, 'LineWidth', 1, 'LineStyle', '--');
    D.sp = line(ax, NaN, NaN, NaN, 'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 9, ...
        'LineWidth', 1.5, 'MarkerEdgeColor', c);
    D.sp_linea = line(ax, NaN, NaN, NaN, 'Color', c*0.8, 'LineWidth', 0.8, 'LineStyle', ':');
    D.estela = line(ax, NaN, NaN, NaN, 'Color', c, 'LineWidth', 1.6);
    % fondo oscuro: la etiqueta se lee aunque pase sobre los ejes o sobre otro texto
    D.etiqueta = text(ax, 0, 0, 0, '', 'Color', c, 'FontSize', 9, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', 'Interpreter', 'none', 'BackgroundColor', [0.10 0.11 0.13], ...
        'Margin', 1, 'EdgeColor', c*0.6);
    G.D(i) = D;
end
% cámara del explorador: frustum + huella
G.frustum = line(ax, NaN, NaN, NaN, 'Color', [1.0 0.95 0.55], 'LineWidth', 0.9);
G.huella = patch(ax, 'XData', NaN(4,1), 'YData', NaN(4,1), 'ZData', 0.006*ones(4,1), ...
    'FaceColor', [1.0 0.95 0.45], 'FaceAlpha', 0.16, 'EdgeColor', [1.0 0.92 0.35], 'LineWidth', 1.2);
% línea entre drones y distancia
G.dist_linea = line(ax, NaN, NaN, NaN, 'Color', [1 1 1], 'LineWidth', 1, 'LineStyle', '--');
G.dist_txt = text(ax, NaN, NaN, NaN, '', 'Color', [1 1 1], 'FontSize', 8, 'Interpreter', 'none', ...
    'HorizontalAlignment', 'center');

light(ax, 'Position', [2.5 -2.0 4.0], 'Style', 'infinite', 'Color', [1 1 1]);
light(ax, 'Position', [-3.0 2.5 2.5], 'Style', 'infinite', 'Color', [0.55 0.55 0.6]);
G = poner_vista(G, opc.vista, []);

% ---------------- HUD ------------------------------------------------------
% fondo opaco: tapa lo que la perspectiva de los ejes 3D dibuje fuera de su recuadro
axes('Parent', G.fig, 'Position', [0.70 0 0.30 1], 'Color', col.fondo, 'XColor', 'none', ...
    'YColor', 'none', 'XTick', [], 'YTick', [], 'Box', 'off', 'HitTest', 'off');
hx = axes('Parent', G.fig, 'Position', [0.705 0.355 0.29 0.635], 'Visible', 'off', ...
    'XLim', [0 1], 'YLim', [0 1]);
G.hud_ax = hx;
G.hud_titulo = text(hx, 0, 0.985, 'QDrone 2 · simulador 3D', 'Color', col.texto, 'FontSize', 13, ...
    'FontWeight', 'bold', 'VerticalAlignment', 'top', 'Interpreter', 'none');
G.hud_estado = text(hx, 0, 0.925, '', 'Color', [0.3 1 0.5], 'FontSize', 12, 'FontWeight', 'bold', ...
    'VerticalAlignment', 'top', 'Interpreter', 'none');
fuente = 'Consolas';
if ~any(strcmpi(listfonts, fuente)), fuente = 'FixedWidth'; end
G.hud_txt = text(hx, 0, 0.865, '', 'Color', col.texto, 'FontSize', 8.5, 'FontName', fuente, ...
    'VerticalAlignment', 'top', 'Interpreter', 'none');
G.hud_obj = text(hx, 0, 0.50, '', 'Color', col.texto, 'FontSize', 8.5, 'FontName', fuente, ...
    'VerticalAlignment', 'top', 'Interpreter', 'none');
% mini mapa cenital (+x arriba, +y a la izquierda)
mx = axes('Parent', G.fig, 'Position', [0.745 0.07 0.215 0.27], 'Color', col.tapete, ...
    'XColor', col.tenue, 'YColor', col.tenue, 'FontSize', 7, 'XDir', 'reverse', 'Box', 'on');
hold(mx, 'on'); axis(mx, 'equal');
xlim(mx, lx + [-0.1 0.1]); ylim(mx, lx + [-0.1 0.1]);
xlabel(mx, 'y (m)  [+y izquierda]'); ylabel(mx, 'x (m)  [+x adelante]');
plot(mx, V(:,2), V(:,1), '-', 'Color', [0.95 0.95 0.95], 'LineWidth', 2);
for j = 1:numel(esc.obj)
    mk = 's'; if esc.obj(j).clase == 0, mk = '^'; end
    plot(mx, esc.obj(j).y, esc.obj(j).x, mk, 'MarkerSize', 5, 'MarkerFaceColor', [0.85 0.15 0.1], ...
        'MarkerEdgeColor', 'none');
end
G.mm_huella = patch(mx, NaN(4,1), NaN(4,1), [1 0.95 0.45], 'FaceAlpha', 0.2, 'EdgeColor', [1 0.92 0.35]);
G.mm_est = plot(mx, NaN, NaN, 'x', 'Color', [1 1 1], 'MarkerSize', 7, 'LineWidth', 1.5);
for i = 1:2
    G.mm_estela(i) = plot(mx, NaN, NaN, '-', 'Color', col.dron(i,:), 'LineWidth', 1);
    G.mm_dron(i) = plot(mx, NaN, NaN, 'o', 'MarkerSize', 8, 'MarkerFaceColor', col.dron(i,:), ...
        'MarkerEdgeColor', [1 1 1]);
    G.mm_txt(i) = text(mx, NaN, NaN, sprintf('D%d', i), 'Color', col.dron(i,:), 'FontSize', 7, ...
        'FontWeight', 'bold');
end
title(mx, 'vista cenital', 'Color', col.tenue, 'FontSize', 8);

% ---------------- botones --------------------------------------------------
nombres = {'START', 'PAUSE', 'RESUME', 'ABORT/RTH', 'LAND ALL', 'Vista', 'x0.5', 'x2', 'Salir'};
acciones = {1, 4, 5, 2, 3, 'vista', 'lento', 'rapido', 'salir'};
w = 0.072;
for k = 1:numel(nombres)
    uicontrol(G.fig, 'Style', 'pushbutton', 'String', nombres{k}, 'Units', 'normalized', ...
        'Position', [0.01 + (k-1)*(w + 0.004) 0.012 w 0.042], 'FontWeight', 'bold', ...
        'Callback', @(~,~) boton(G.fig, acciones{k}));
end
G.ayuda = annotation(G.fig, 'textbox', [0.68 0.005 0.32 0.05], 'String', ...
    'teclas: s START · p PAUSE · r RESUME · a ABORT · l LAND · v vista · +/- velocidad · q salir', ...
    'Color', col.tenue, 'EdgeColor', 'none', 'FontSize', 7.5, 'VerticalAlignment', 'middle');
G.t_ultimo = NaN;
G.cache = containers.Map();      % último valor puesto (para no repetir set: dibujar es lo caro)
end

% =========================================================================
%                              ACTUALIZAR
% =========================================================================
function G = actualizar(G, E)
if ~isgraphics(G.fig), return; end
col = G.col;
p = E.p;
M = G.malla;
X = E.X;
s = E.s;
tel = E.tel;
roles = {'sin rol', 'EXPLORADOR', 'RESPONDEDOR'};
dt_cuadro = 0.04;
if isfinite(G.t_ultimo), dt_cuadro = max(E.t - G.t_ultimo, 0); end
G.t_ultimo = E.t;
th = linspace(0, 2*pi, 40);
rol = [tel.role1 tel.role2];
% modo solo explorador: un dron vuela; el otro ("ausente") no se usa
solo = isfield(E, 'solo') && E.solo;
ausente = 0;
if solo, ausente = 3 - E.dron_solo; end
oculto = solo && isfield(E, 'aus_fuera') && E.aus_fuera;
for i = 1:2
    D = G.D(i);
    x = X(:, i);
    es_aus = (i == ausente);
    % ausente fuera de OptiTrack: oculto; rastreado en el piso: "fantasma" gris
    setc(G.cache, sprintf('vis%d', i), [D.tr D.sombra D.etiqueta D.estela G.mm_dron(i) G.mm_txt(i)], ...
        'Visible', onoff(~(es_aus && oculto)));
    if es_aus && ~oculto && ~isKey(G.cache, sprintf('gris%d', i))
        set(findobj(D.tr, 'Type', 'patch'), 'FaceColor', [0.55 0.56 0.60], 'FaceAlpha', 0.35);
        set(D.etiqueta, 'Color', [0.70 0.71 0.74], 'EdgeColor', [0.45 0.46 0.50]);
        set(G.mm_dron(i), 'MarkerFaceColor', [0.55 0.56 0.60]);
        set(G.mm_txt(i), 'Color', [0.70 0.71 0.74]);
        G.cache(sprintf('gris%d', i)) = true;
    end
    D.tr.Matrix = makehgtform('translate', x(1:3).' + [0 0 G.pivote], 'zrotate', x(9), 'yrotate', x(8), ...
        'xrotate', x(7), 'translate', [0 0 -G.pivote]);
    activo = E.fsm(i) >= 3 && E.fsm(i) <= 6;
    if activo
        w = min(55, 8 + 3*X(10, i));          % rad/s "visuales" (con aliasing se ve girar)
        D.ang = D.ang + M.sentido*w*max(dt_cuadro, 0.02);
    end
    for k = 1:4
        D.helice(k).Matrix = makehgtform('translate', M.motores(k,:), 'zrotate', D.ang(k));
        setc(G.cache, sprintf('disco%d%d', i, k), D.disco(k), 'Visible', onoff(activo));
    end
    esc_s = 1 + 0.6*min(x(3), 2);            % la sombra crece y se aclara con la altura
    set(D.sombra, 'XData', x(1) + 0.24*esc_s*cos(th), 'YData', x(2) + 0.22*esc_s*sin(th), ...
        'FaceAlpha', 0.38/esc_s);
    if E.version == 1 && s.estado >= 3 && ~solo          % (solo explorador: no aplica)
        r = s.d_min_h/2;
        cb = col.dron(i,:);
        if E.dh < s.d_min_h, cb = [1 0.2 0.2]; end
        set(D.burbuja, 'XData', x(1) + r*cos(th), 'YData', x(2) + r*sin(th), ...
            'ZData', 0.005*ones(size(th)), 'Color', cb);
    else
        set(D.burbuja, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
    end
    sp = E.sp(i, :);
    if s.estado >= 3 && E.fsm(i) >= 3 && ~es_aus
        set(D.sp, 'XData', sp(1), 'YData', sp(2), 'ZData', sp(3));
        set(D.sp_linea, 'XData', [x(1) sp(1)], 'YData', [x(2) sp(2)], 'ZData', [x(3) sp(3)]);
    else
        set(D.sp, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
        set(D.sp_linea, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
    end
    if G.opc.estelas
        e = E.estela{i};
        set(D.estela, 'XData', e(:,1), 'YData', e(:,2), 'ZData', e(:,3));
        set(G.mm_estela(i), 'XData', e(:,2), 'YData', e(:,1));
    end
    txt = sprintf('D%d · %s', i, roles{rol(i) + 1});
    if es_aus, txt = sprintf('D%d · NO USADO', i); end
    set(D.etiqueta, 'Position', [x(1) x(2) x(3) + 0.27]);
    setc(G.cache, sprintf('etq%d', i), D.etiqueta, 'String', txt);
    set(G.mm_dron(i), 'XData', x(2), 'YData', x(1));
    set(G.mm_txt(i), 'Position', [x(2) - 0.12, x(1) + 0.1, 0]);
    G.D(i) = D;
end
% cámara del explorador
ie = s.explorer_id;
if ie >= 1 && X(3, ie) > 0.12 && ~isempty(E.huella)
    H = E.huella;
    set(G.huella, 'XData', H(:,1), 'YData', H(:,2), 'ZData', 0.006*ones(4,1));
    set(G.mm_huella, 'XData', H(:,2), 'YData', H(:,1));
    c0 = E.lente;
    FX = []; FY = []; FZ = [];
    for k = 1:4
        FX = [FX c0(1) H(k,1) NaN]; FY = [FY c0(2) H(k,2) NaN]; FZ = [FZ c0(3) 0.006 NaN]; %#ok<AGROW>
    end
    set(G.frustum, 'XData', FX, 'YData', FY, 'ZData', FZ);
else
    set(G.huella, 'XData', NaN(4,1), 'YData', NaN(4,1));
    set(G.mm_huella, 'XData', NaN(4,1), 'YData', NaN(4,1));
    set(G.frustum, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
end
ob = E.obs_ult;
if isempty(ob)
    set(G.obs_ult, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
else
    set(G.obs_ult, 'XData', ob(:,2), 'YData', ob(:,3), 'ZData', 0.03*ones(size(ob,1),1));
end
% ruta del explorador
if s.n_wp ~= G.n_wp
    G.n_wp = s.n_wp;
    if s.n_wp > 0
        set(G.ruta, 'XData', s.wp(1:s.n_wp, 1), 'YData', s.wp(1:s.n_wp, 2), 'ZData', s.wp(1:s.n_wp, 3));
    else
        set(G.ruta, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
    end
end
if s.n_wp > 0 && s.estado == 5 && s.explorer_done < 0.5
    k = min(s.wp_idx, s.n_wp);
    set(G.wp_act, 'XData', s.wp(k,1), 'YData', s.wp(k,2), 'ZData', s.wp(k,3));
else
    set(G.wp_act, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
end
% distancia entre drones (solo explorador: no aplica)
if ~solo && all(X(3, :) > 0.12)
    set(G.dist_linea, 'XData', X(1,:), 'YData', X(2,:), 'ZData', X(3,:));
    cd = [1 1 1];
    if E.version == 1 && E.dh < s.d_min_h, cd = [1 0.3 0.3]; end
    set(G.dist_linea, 'Color', cd);
    set(G.dist_txt, 'Position', mean(X(1:3,:), 2).' + [0 0 0.08], 'Color', cd, ...
        'String', sprintf('dh %.2f m · dz %.2f m', E.dh, abs(X(3,1) - X(3,2))));
else
    set(G.dist_linea, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
    set(G.dist_txt, 'String', '');
end
% objetivos de la misión (tabla de qd2_mission_step) y estado de los objetos reales
n = s.n_obj;
tx = s.obj_x(1:n); ty = s.obj_y(1:n);
set(G.obj_est, 'XData', tx, 'YData', ty, 'ZData', 0.02*ones(n,1));
set(G.mm_est, 'XData', ty, 'YData', tx);
nombres_cls = {'fuego', 'toma'};
nombres_est = {'', 'detectado', 'ASIGNADO', 'VISITADO'};
for k = 1:32
    if k <= n
        setc(G.cache, sprintf('T%d', k), G.obj_est_txt(k), 'Position', [tx(k) + 0.05, ty(k), 0.06], ...
            'String', sprintf('T%d', s.obj_id(k)), 'Color', col.estado_obj(s.obj_estado(k) + 1, :));
    else
        setc(G.cache, sprintf('T%d', k), G.obj_est_txt(k), 'Position', [NaN NaN NaN], 'String', '');
    end
end
for j = 1:numel(E.esc.obj)
    o = E.esc.obj(j);
    ej = E.estado_real(j);
    setc(G.cache, sprintf('anillo%d', j), G.obj_anillo(j), 'Color', col.estado_obj(ej + 1, :), ...
        'LineStyle', ternario(ej == 0, ':', '-'), 'LineWidth', ternario(ej == 0, 1.0, 2.2));
    etq = nombres_cls{o.clase + 1};
    if ej > 0, etq = sprintf('%s · %s', etq, lower(nombres_est{ej + 1})); end
    setc(G.cache, sprintf('objtxt%d', j), G.obj_txt(j), 'String', etq, 'Color', col.estado_obj(ej + 1, :));
end

% ---------------- HUD ------------------------------------------------------
est_es = {'INIT · inicio', 'WAIT_TRACKING · esperando OptiTrack', 'ASSIGN_ROLES · roles asignados', ...
    'TAKEOFF · despegue', 'HOVER_READY · hover listo', 'MISSION · misión', 'RETURN_HOME · regreso a home', ...
    'LANDING · aterrizando', 'DONE · terminado', 'ABORT · abortado (hover en home)', 'PAUSED · en pausa'};
col_est = [0.7 0.7 0.7; 0.7 0.7 0.7; 0.4 0.8 1; 1 0.85 0.3; 0.4 1 0.6; 0.3 1 0.5; 0.5 0.8 1; ...
    1 0.85 0.3; 0.8 0.8 0.8; 1 0.35 0.3; 1 0.6 0.2];
e = min(max(s.estado, 0), 10);
modo = 'tiempo real';
if ~E.tiempo_real, modo = 'rápido'; end
txt_ver = ternario(E.version == 1, 'PID', 'MPC');
if solo, txt_ver = sprintf('%s · SOLO', txt_ver); end
setc(G.cache, 'titulo', G.hud_titulo, 'String', sprintf('QDrone 2 · simulador 3D   [%s]', txt_ver));
setc(G.cache, 'estado', G.hud_estado, 'String', est_es{e + 1}, 'Color', col_est(e + 1, :));
alertas = {'', 'homes demasiado cerca: START rechazado', 'pausa por pérdida de rastreo', ...
    'timeout de despegue', 'timeout de aterrizaje'};
fsm_n = {'IDLE', 'ARMED', 'TAKEOFF_THR', 'TAKEOFF_H', 'FLIGHT', 'AUTOLAND', '?', '?', 'DISABLE'};
L = {};
L{end+1} = sprintf('t = %6.1f s   x%.2g (%s)   sim/real %.2f', E.t, E.vel, modo, E.factor_real);
L{end+1} = sprintf('estación: %s', E.txt_estacion);
L{end+1} = sprintf('%s', E.txt_tcp);
if solo
    L{end+1} = sprintf('modo: SOLO EXPLORADOR (dron %d; el dron %d no se usa)', E.dron_solo, ausente);
end
if tel.alerta > 0
    L{end+1} = sprintf('ALERTA %d: %s', tel.alerta, alertas{min(tel.alerta, 4) + 1});
else
    L{end+1} = 'alerta: ninguna';
end
L{end+1} = '';
for i = 1:2
    f = E.fsm(i);
    nr = roles{rol(i) + 1};
    if i == ausente, nr = 'NO USADO'; end
    L{end+1} = sprintf('D%d %-11s z %4.2f m  %-11s %4.1f V', i, nr, X(3,i), ...
        fsm_n{min(max(f,1),9)}, E.bat(i)); %#ok<AGROW>
end
L{end+1} = '';
if s.explorer_id > 0 && s.n_wp > 0
    L{end+1} = sprintf('E: waypoint %d/%d   N terminada: %s', s.wp_idx, s.n_wp, ternario(s.explorer_done > 0.5, 'sí', 'no'));
else
    L{end+1} = 'E: N sin generar (falta START)';
end
if solo
    L{end+1} = sprintf('R: sin respondedor (solo explorador)  detectados %d', tel.queue_length);
    L{end+1} = 'separación entre drones: no aplica (un solo dron)';
else
    if tel.responder_target_id > 0
        L{end+1} = sprintf('R: objetivo T%d  marcando %3.0f %%  cola %d', tel.responder_target_id, ...
            100*tel.marking_progress, tel.queue_length);
    else
        L{end+1} = sprintf('R: libre  cola %d', tel.queue_length);
    end
    L{end+1} = sprintf('dist. horizontal %4.2f m (mín. en vuelo %s)', E.dh, num2txt(E.dh_min));
    if E.version == 1
        L{end+1} = sprintf('  regla PID: dh >= %.2f m   guard: %s', s.d_min_h, ternario(E.ga > 0.5, 'ACTIVO', 'inactivo'));
    else
        L{end+1} = sprintf('dist. vertical %4.2f m   regla MPC: dz >= %.2f m apilados', E.dz, p.dz_min);
    end
end
L{end+1} = sprintf('alturas: z_E %.2f  z_R %.2f  t_mark %.1f s', s.z_E, s.z_R, s.t_mark);
setc(G.cache, 'hud', G.hud_txt, 'String', L);
% tabla de objetivos
T = {};
T{end+1} = sprintf('OBJETIVOS  conocidos %d  visitados %d', tel.n_targets_known, tel.n_targets_visited);
T{end+1} = ' id  clase    x      y     estado     error';
for k = 1:min(n, 9)
    T{end+1} = sprintf(' T%-2d %-5s %6.2f %6.2f  %-9s %s', s.obj_id(k), nombres_cls{s.obj_cls(k) + 1}, ...
        s.obj_x(k), s.obj_y(k), nombres_est{s.obj_estado(k) + 1}, E.err_obj{k}); %#ok<AGROW>
end
if n > 9, T{end+1} = sprintf(' ... y %d más', n - 9); end
T{end+1} = sprintf('reales %d · sin detectar %d · clusters pendientes %d · falsos %d', ...
    numel(E.esc.obj), sum(E.estado_real == 0), E.n_pend, E.n_falsos);
T{end+1} = sprintf('detector: %s', E.txt_vision);
if isfield(E, 'txt_actitud') && ~isempty(E.txt_actitud)
    T{end+1} = sprintf('geolocalización: %s', E.txt_actitud);
end
setc(G.cache, 'hudobj', G.hud_obj, 'String', T);
% vista que sigue al explorador
v = getappdata(G.fig, 'vista');
if v == 3
    G = poner_vista(G, 3, X(:, max(s.explorer_id, 1)));
elseif v == 4
    G = poner_vista(G, 4, X(:, 1));
end
end

% =========================================================================
function G = poner_vista(G, modo, x)
ax = G.ax;
setappdata(G.fig, 'vista', modo);
switch modo
    case 1
        % operador parado detrás de la arena (lado -x): +x hacia el fondo, +y a la izquierda
        campos(ax, [-5.2 -1.9 3.6]);
        camtarget(ax, [0.25 0.05 0.25]);
        camup(ax, [0 0 1]);
        camva(ax, 34);
    case 2
        campos(ax, [0 0 9.5]);
        camtarget(ax, [0 0 0]);
        camup(ax, [1 0 0]);              % +x hacia arriba de la pantalla, +y a la izquierda
        camva(ax, 21);
    case 3
        if isempty(x), x = [0; 0; 1]; end
        campos(ax, [x(1) - 2.2, x(2) - 1.4, x(3) + 1.4]);
        camtarget(ax, [x(1) + 0.3, x(2), max(x(3) - 0.6, 0)]);
        camup(ax, [0 0 1]);
        camva(ax, 30);
    case 4
        % primer plano del dron (para revisar la malla)
        if isempty(x), x = [0; 0; 0]; end
        campos(ax, [x(1) - 0.75, x(2) - 0.55, x(3) + 0.45]);
        camtarget(ax, [x(1) + 0.02, x(2), x(3) + 0.05]);
        camup(ax, [0 0 1]);
        camva(ax, 38);
end
end

function tecla(fig, ev)
switch lower(ev.Character)
    case 's', boton(fig, 1);
    case 'p', boton(fig, 4);
    case 'r', boton(fig, 5);
    case 'a', boton(fig, 2);
    case 'l', boton(fig, 3);
    case 'v', boton(fig, 'vista');
    case {'+', '='}, boton(fig, 'rapido');
    case {'-', '_'}, boton(fig, 'lento');
    case {'q', char(27)}, boton(fig, 'salir');
end
end

function boton(fig, a)
if isnumeric(a)
    setappdata(fig, 'cola_cmd', [getappdata(fig, 'cola_cmd') a]);
else
    switch a
        case 'vista'
            v = mod(getappdata(fig, 'vista'), 4) + 1;
            setappdata(fig, 'vista_pedida', v);
        case 'lento'
            setappdata(fig, 'vel', max(getappdata(fig, 'vel')/2, 0.125));
        case 'rapido'
            setappdata(fig, 'vel', min(getappdata(fig, 'vel')*2, 16));
        case 'salir'
            setappdata(fig, 'salir', true);
    end
end
end

% ---------------- utilidades geométricas ---------------------------------
function flecha(ax, o, d, c)
line(ax, o(1) + [0 d(1)], o(2) + [0 d(2)], o(3) + [0 d(3)], 'Color', c, 'LineWidth', 3);
L = norm(d); u = d/L;
[Vc, Fc] = cono_dir(o + d, u, 0.03, 0.08, 14);
patch(ax, 'Vertices', Vc, 'Faces', Fc, 'FaceColor', c, 'EdgeColor', 'none');
end

function [V, F] = cono_dir(base, u, r, h, n)
% cono con base en "base" apuntando en la dirección u
if abs(u(3)) < 0.9, a = [0 0 1]; else, a = [1 0 0]; end
e1 = cross(u, a); e1 = e1/norm(e1); e2 = cross(u, e1);
t = (0:n-1)'*2*pi/n;
V = [base + r*(cos(t)*e1 + sin(t)*e2); base + h*u; base];
i = (1:n)'; j = [2:n 1]';
F = [i j (n+1)*ones(n,1); j i (n+2)*ones(n,1)];
end

function [V, F] = tubo(p0, p1, r, n)
d = p1 - p0; L = norm(d); u = d/L;
if abs(u(3)) < 0.9, a = [0 0 1]; else, a = [1 0 0]; end
e1 = cross(u, a); e1 = e1/norm(e1); e2 = cross(u, e1);
t = (0:n-1)'*2*pi/n;
anillo = r*(cos(t)*e1 + sin(t)*e2);
V = [p0 + anillo; p1 + anillo];
i = (1:n)'; j = [2:n 1]';
F = [i j n+j; i n+j n+i];
end

function [V, F] = disco(r, n)
t = (0:n-1)'*2*pi/n;
V = [r*cos(t) r*sin(t) zeros(n,1); 0 0 0];
F = [(1:n)' [2:n 1]' (n+1)*ones(n,1)];
end

function [V, F] = caja(c, s)
h = s/2;
V = [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1].*h + c;
Q = [1 2 3 4; 5 8 7 6; 1 5 6 2; 2 6 7 3; 3 7 8 4; 4 8 5 1];
F = [Q(:, [1 2 3]); Q(:, [1 3 4])];
end

function setc(cache, clave, h, varargin)
% set() solo si el valor cambió desde la última vez
if ~isKey(cache, clave) || ~isequal(cache(clave), varargin)
    set(h, varargin{:});
    cache(clave) = varargin; %#ok<NASGU>  (containers.Map es un handle)
end
end

function s = onoff(b)
if b, s = 'on'; else, s = 'off'; end
end

function r = ternario(c, a, b)
if c, r = a; else, r = b; end
end

function s = num2txt(v)
if isfinite(v), s = sprintf('%.2f m', v); else, s = '--'; end
end
