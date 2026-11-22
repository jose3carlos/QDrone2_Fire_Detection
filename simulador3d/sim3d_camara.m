function varargout = sim3d_camara(accion, cam, varargin)
%SIM3D_CAMARA Geometría de la cámara del explorador (misma que web/app/geo.py).
%
%   [R_wc, p_c] = sim3d_camara('pose', cam, pos3, R_wb)
%       R_wb = rotación mundo <- cuerpo (3x3). La estación usa solo el yaw
%       (roll y pitch ~ 0); el simulador usa la actitud real para "ver".
%   [uv, delante] = sim3d_camara('proyectar', cam, P, pos3, R_wb)
%       P 3xN puntos del mundo -> uv 2xN píxeles y máscara 1xN.
%   g = sim3d_camara('a_suelo', cam, uv, pose4)
%       píxeles 2xN -> 2xN puntos en el plano z = cam.z_plano, usando solo
%       x, y, z y yaw de pose4 (como la estación). NaN si el rayo no corta.
%   G = sim3d_camara('huella', cam, pose4)
%       4x2 esquinas de lo que ve la cámara en el suelo (vacío si no corta).
%   R = sim3d_camara('rz', yaw)   rotación alrededor de z.

switch accion
    case 'rz'
        varargout{1} = rz(cam);
    case 'pose'
        [R_wc, p_c] = pose(cam, varargin{1}, varargin{2});
        varargout = {R_wc, p_c};
    case 'proyectar'
        P = varargin{1};
        [R_wc, p_c] = pose(cam, varargin{2}, varargin{3});
        d = R_wc.'*(P - p_c);                      % coordenadas de cámara
        delante = d(3,:) > 1e-6;
        dz = d(3,:); dz(~delante) = 1;
        uv = [cam.fx*d(1,:)./dz + cam.cx; cam.fy*d(2,:)./dz + cam.cy];
        varargout = {uv, delante};
    case 'a_suelo'
        uv = varargin{1}; q = varargin{2};
        varargout{1} = a_suelo(cam, uv, q);
    case 'huella'
        q = varargin{1};
        uv = [0 cam.W-1 cam.W-1 0; 0 0 cam.H-1 cam.H-1];
        g = a_suelo(cam, uv, q);
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

function [R_wc, p_c] = pose(cam, pos3, R_wb)
R_wc = R_wb*cam.R_bc;
p_c = pos3(:) + R_wb*cam.offset;
end

function g = a_suelo(cam, uv, q)
[R_wc, p_c] = pose(cam, q(1:3), rz(q(4)));
n = size(uv, 2);
d = R_wc*[(uv(1,:) - cam.cx)/cam.fx; (uv(2,:) - cam.cy)/cam.fy; ones(1,n)];
g = nan(2, n);
ok = d(3,:) < -1e-9;
t = (cam.z_plano - p_c(3))./d(3,:);
ok = ok & t > 0;
g(1,ok) = p_c(1) + t(ok).*d(1,ok);
g(2,ok) = p_c(2) + t(ok).*d(2,ok);
end
