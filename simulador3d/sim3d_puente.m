function [pt, out] = sim3d_puente(accion, pt, varargin)
%SIM3D_PUENTE Servidor TCP del protocolo §3.3 (64 doubles salen, 32 entran).
%
%   pt = sim3d_puente('abrir', [], host, puerto, backend)
%   [pt, paquetes] = sim3d_puente('leer', pt)    paquetes: 32xN (N >= 0)
%   pt = sim3d_puente('enviar', pt, y64)
%   pt = sim3d_puente('cerrar', pt)
%   [pt, txt] = sim3d_puente('estado', pt)
%   sim3d_puente('cerrar_todo')   cierra lo que haya quedado abierto (Ctrl+C, error)
%
%   Igual que el Stream Server de Simulink: el simulador es el SERVIDOR en
%   tcpip://host:puerto y la estación Python (web/app/bridge.py) es el
%   cliente. Doubles IEEE-754 little-endian, tamaño fijo, "most recent".
%
%   Backends:
%   - 'java'      : java.nio (ServerSocketChannel no bloqueante). Viene con
%                   MATLAB (R2022a-R2025a) sin toolboxes. Probado.
%   - 'tcpserver' : tcpserver de MATLAB. OJO: tcpserver es de la Instrument
%                   Control Toolbox (no está en MATLAB base); se usa solo si
%                   existe.
%   - 'auto'      : java si hay JVM; si no, tcpserver.
%   Un cliente a la vez: si se conecta otro, reemplaza al anterior (la
%   estación se reconecta sola).

out = [];
switch accion
    case 'abrir'
        [host, puerto, backend] = varargin{:};
        pt = abrir(host, puerto, backend);
    case 'leer'
        [pt, out] = leer(pt);
    case 'enviar'
        pt = enviar(pt, varargin{1});
    case 'cerrar'
        pt = cerrar(pt);
    case 'estado'
        out = estado(pt);
    case 'cerrar_todo'
        cerrar_registro();
    otherwise
        error('sim3d:puente', 'Acción desconocida: %s', accion);
end
end

% =========================================================================
function pt = abrir(host, puerto, backend)
pt.host = char(host);
pt.puerto = double(puerto);
pt.backend = lower(char(backend));
pt.rx = uint8([]);
pt.n_rx = 0; pt.n_tx = 0; pt.n_desc = 0; pt.n_conex = 0;
pt.conectado = false;
pt.cli = [];
pt.srv = [];
pt.cli_id = '';      % tcpserver: "dirección:puerto" del cliente actual
pt.pend = [];        % ByteBuffer con un mensaje a medio enviar (java)
pt.error = '';
if strcmp(pt.backend, 'auto')
    if usejava('jvm')
        pt.backend = 'java';
    else
        pt.backend = 'tcpserver';
    end
end
switch pt.backend
    case 'java'
        cerrar_registro();          % por si una corrida anterior quedó abierta (Ctrl+C)
        ssc = java.nio.channels.ServerSocketChannel.open();
        try
            ssc.socket().setReuseAddress(true);
            ssc.socket().bind(java.net.InetSocketAddress(pt.host, pt.puerto));
            ssc.configureBlocking(false);
        catch err
            try, ssc.close(); catch, end %#ok<NOCOM>   % no dejar el canal abierto si el bind falla
            rethrow(err);
        end
        pt.srv = ssc;
        pt.bufrx = java.nio.ByteBuffer.allocate(8192);
        setappdata(groot, 'sim3d_tcp_srv', ssc);
    case 'tcpserver'
        if isempty(which('tcpserver'))
            error('sim3d:puente', ['tcpserver no está disponible (es de la Instrument Control ' ...
                'Toolbox). Usa backend_tcp = ''java''.']);
        end
        pt.srv = tcpserver(pt.host, pt.puerto, 'ByteOrder', 'little-endian', 'Timeout', 0.5); %#ok<TNMLP>
    otherwise
        error('sim3d:puente', 'Backend desconocido: %s', pt.backend);
end
end

function [pt, P] = leer(pt)
P = zeros(32, 0);
switch pt.backend
    case 'java'
        % ¿cliente nuevo?
        c = pt.srv.accept();
        if ~isempty(c)
            if ~isempty(pt.cli)
                try, pt.cli.close(); catch, end %#ok<NOCOM>
            end
            c.configureBlocking(false);
            c.socket().setTcpNoDelay(true);
            pt.cli = c;
            setappdata(groot, 'sim3d_tcp_cli', c);
            pt.rx = uint8([]);
            pt.pend = [];
            pt.conectado = true;
            pt.n_conex = pt.n_conex + 1;
        end
        if isempty(pt.cli), return; end
        try
            while true
                pt.bufrx.clear();
                n = pt.cli.read(pt.bufrx);
                if n < 0
                    pt = soltar(pt, 'el cliente cerró la conexión');
                    return;
                elseif n == 0
                    break;
                end
                a = pt.bufrx.array();
                pt.rx = [pt.rx; typecast(int8(a(1:n)), 'uint8')];
                if n < 8192, break; end
            end
        catch err
            pt = soltar(pt, err.message);
            return;
        end
    case 'tcpserver'
        con = pt.srv.Connected;
        if ~con
            % se fue el cliente: se tira el paquete a medias para que el siguiente
            % cliente no quede desalineado
            pt.rx = uint8([]);
            pt.conectado = false;
            pt.cli_id = '';
            return;
        end
        id = sprintf('%s:%d', char(string(pt.srv.ClientAddress)), double(pt.srv.ClientPort));
        if ~pt.conectado || ~strcmp(id, pt.cli_id)
            pt.rx = uint8([]);              % cliente nuevo: encuadre desde cero
            pt.cli_id = id;
            pt.n_conex = pt.n_conex + 1;
        end
        pt.conectado = true;
        nb = pt.srv.NumBytesAvailable;
        if nb > 0
            pt.rx = [pt.rx; uint8(read(pt.srv, nb, 'uint8')).'];
        end
end
n = floor(numel(pt.rx)/256);
if n > 0
    b = pt.rx(1:n*256);
    pt.rx = pt.rx(n*256+1:end);
    P = reshape(typecast(b(:).', 'double'), 32, n);
    pt.n_rx = pt.n_rx + n;
end
end

function pt = enviar(pt, y)
y = double(y(:));
switch pt.backend
    case 'java'
        if isempty(pt.cli), return; end
        try
            % termina primero un mensaje a medio enviar (no se rompe el encuadre)
            if ~isempty(pt.pend)
                pt.cli.write(pt.pend);
                if pt.pend.hasRemaining()
                    pt.n_desc = pt.n_desc + 1;      % el cliente no lee: se descarta este
                    return;
                end
                pt.pend = [];
            end
            bb = java.nio.ByteBuffer.wrap(typecast(y, 'int8'));
            pt.cli.write(bb);
            if bb.hasRemaining()
                pt.pend = bb;
            end
            pt.n_tx = pt.n_tx + 1;
        catch err
            pt = soltar(pt, err.message);
        end
    case 'tcpserver'
        if pt.srv.Connected
            try
                write(pt.srv, y, 'double');
                pt.n_tx = pt.n_tx + 1;
            catch err
                pt.error = err.message;
            end
        end
end
end

function pt = soltar(pt, motivo)
try, pt.cli.close(); catch, end %#ok<NOCOM>
pt.cli = [];
pt.pend = [];
pt.rx = uint8([]);
pt.conectado = false;
pt.error = motivo;
end

function cerrar_registro()
for nombre = {'sim3d_tcp_cli', 'sim3d_tcp_srv'}
    if isappdata(groot, nombre{1})
        ch = getappdata(groot, nombre{1});
        try, ch.close(); catch, end %#ok<NOCOM>
        rmappdata(groot, nombre{1});
    end
end
end

function pt = cerrar(pt)
if isempty(pt), return; end
switch pt.backend
    case 'java'
        if ~isempty(pt.cli)
            try, pt.cli.close(); catch, end %#ok<NOCOM>
        end
        try, pt.srv.close(); catch, end %#ok<NOCOM>
        cerrar_registro();
    case 'tcpserver'
        try, delete(pt.srv); catch, end %#ok<NOCOM>
end
pt.cli = []; pt.srv = []; pt.conectado = false;
end

function txt = estado(pt)
if isempty(pt)
    txt = 'TCP apagado';
elseif pt.conectado
    txt = sprintf('TCP :%d cliente conectado (rx %d, tx %d)', pt.puerto, pt.n_rx, pt.n_tx);
else
    txt = sprintf('TCP :%d esperando cliente', pt.puerto);
end
end
