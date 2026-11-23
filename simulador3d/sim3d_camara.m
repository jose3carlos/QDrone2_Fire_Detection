function varargout = sim3d_camara(accion, cam, varargin)
%SIM3D_CAMARA Geometría de la cámara del explorador (misma que web/app/geo.py).
%
%   [R_wc, p_c] = sim3d_camara('pose', cam, pos3, R_wb [, R_montaje])
%       R_wb = rotación mundo <- cuerpo (3x3); pos3 = punto de la pose de OptiTrack (punto
%       corregido de Motive). Igual que CameraModel.pose:
%           R_wc = R_wb · R_montaje · R_bc
%           p_c  = pos3 + R_wb·offset + pivote·(ẑ - R_wb·ẑ)
%       (el offset gira alrededor del pivote de Motive, cam.pivote m arriba de la pose).
%       R_montaje (por defecto eye(3)) es el error del soporte que la estación NO conoce
%       (opc.sesgo_montaje_deg); solo lo usa el simulador para "ver" con la cámara real.
%   [uv, delante] = sim3d_camara('proyectar', cam, P, pos3, R_wb [, R_montaje])
%       P 3xN puntos del mundo -> uv 2xN píxeles y máscara 1xN.
%   g = sim3d_camara('a_suelo', cam, uv, q [, R_montaje])
%       píxeles 2xN -> 2xN puntos en el plano z = cam.z_plano (CameraModel.pixel_to_ground).
%       q = [x y z yaw] (nivelado) o [x y z yaw roll pitch] (rad, convención de R_mundo_cuerpo).
%       NaN si el rayo no corta.
%   G = sim3d_camara('huella', cam, q [, R_montaje])
%       4x2 esquinas de lo que ve la cámara en el suelo (vacío si alguna no corta).
%   R = sim3d_camara('rwb', yaw, roll, pitch)   R_mundo_cuerpo = Rz(yaw)·Ry(pitch)·Rx(roll)
%       (ZYX intrínseca; roll > 0 sube el lado izquierdo, pitch > 0 baja la nariz). Con
%       roll = pitch = 0 devuelve Rz(yaw) tal cual, como geo.R_mundo_cuerpo.
%   R = sim3d_camara('rz', yaw)   rotación alrededor de z.
%
%   cam sin el campo 'pivote' (modelo viejo) = pivote 0.

switch accion
    case 'rz'
        varargout{1} = rz(cam);
    case 'rwb'
        varargout{1} = rwb(cam, varargin{1}, varargin{2});
    case 'pose'
        [R_wc, p_c] = pose(cam, varargin{:});
        varargout = {R_wc, p_c};
    case 'proyectar'
        P = varargin{1};
        [R_wc, p_c] = pose(cam, varargin{2:end});
        d = R_wc.'*(P - p_c);                      % coordenadas de cámara
        delante = d(3,:) > 1e-6;
        dz = d(3,:); dz(~delante) = 1;
        uv = [cam.fx*d(1,:)./dz + cam.cx; cam.fy*d(2,:)./dz + cam.cy];
        varargout = {uv, delante};
    case 'a_suelo'
        varargout{1} = a_suelo(cam, varargin{:});
    case 'huella'
        q = varargin{1};
        uv = [0 cam.W-1 cam.W-1 0; 0 0 cam.H-1 cam.H-1];
        g = a_suelo(cam, uv, q, varargin{2:end});
        if any(~isfinite(g(:)))
            varargout{1} = zeros(0,2);
        else
            varargout{1} = g.';
        end
    otherwise
        error('sim3d:camara', 'Acción desconocida: %s', accion);
end
end

function R = rz(a)
c = cos(a); s = sin(a);
R = [c -s 0; s c 0; 0 0 1];
end

function R = rwb(yaw, roll, pitch)
% geo.R_mundo_cuerpo: nivelado -> Rz(yaw) idéntico bit a bit
if roll == 0 && pitch == 0
    R = rz(yaw);
    return;
end
cp = cos(pitch); sp = sin(pitch); cr = cos(roll); sr = sin(roll);
R = rz(yaw)*[cp 0 sp; 0 1 0; -sp 0 cp]*[1 0 0; 0 cr -sr; 0 sr cr];
end

function [R_wc, p_c] = pose(cam, pos3, R_wb, R_m)
if nargin < 4 || isempty(R_m)
    R_wc = R_wb*cam.R_bc;
else
    R_wc = R_wb*R_m*cam.R_bc;
end
p_c = pos3(:) + R_wb*cam.offset;
if isfield(cam, 'pivote') && cam.pivote ~= 0
    % nivelado R_wb(:,3) = [0;0;1] exacto y el término vale 0
    p_c = p_c + cam.pivote*([0; 0; 1] - R_wb(:, 3));
end
end

function g = a_suelo(cam, uv, q, R_m)
if nargin < 4, R_m = []; end
roll = 0; pitch = 0;
if numel(q) >= 6
    roll = q(5); pitch = q(6);
end
[R_wc, p_c] = pose(cam, q(1:3), rwb(q(4), roll, pitch), R_m);
n = size(uv, 2);
d = R_wc*[(uv(1,:) - cam.cx)/cam.fx; (uv(2,:) - cam.cy)/cam.fy; ones(1,n)];
g = nan(2, n);
ok = d(3,:) < -1e-9;
t = (cam.z_plano - p_c(3))./d(3,:);
ok = ok & t > 0;
g(1,ok) = p_c(1) + t(ok).*d(1,ok);
g(2,ok) = p_c(2) + t(ok).*d(2,ok);
end
