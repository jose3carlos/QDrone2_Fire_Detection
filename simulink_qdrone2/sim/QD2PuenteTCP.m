classdef QD2PuenteTCP < matlab.System
    %QD2PUENTETCP Stream Server TCP para la simulación (PYTHON BRIDGE).
    %   Sustituye al bloque Stream Server de QUARC, que en modo Normal no
    %   abre ningún socket (verificado: salida "state" = 0 y ningún puerto en
    %   escucha). tcpserver de MATLAB necesita Instrument Control Toolbox, así
    %   que se usan sockets de Java NIO (java.nio.channels), que vienen con
    %   cualquier MATLAB R2022a+. El bloque MATLAB System corre en
    %   "Interpreted execution".
    %
    %   Protocolo (ARQUITECTURA.md §3.3): doubles little-endian, tamaño fijo,
    %   semántica "most recent", un cliente a la vez (reconecta solo).
    %     - Envía la telemetría de 64 doubles en cada paso (50 Hz).
    %     - Recibe paquetes de 32 doubles; se queda con el último completo.
    %   Salidas: py (32x1; si nunca llegó nada, ceros), conectado (0/1),
    %   n_rx (paquetes recibidos en total).
    %
    %   TODO es NO bloqueante (como "send/receive most recent" de QUARC): si
    %   el cliente deja de leer (UI congelada, laptop suspendida) y se llenan
    %   los buffers TCP, las tramas nuevas se DESCARTAN enteras (nunca se
    %   mandan a medias, así no se desalinea el flujo) y la simulación sigue.
    %   Si un cliente no lee nada durante t_desconectar s se le cierra la
    %   conexión para que otro pueda entrar.
    %   Si el puerto está ocupado (otra simulación, el simulador 3D, el mock
    %   de la web...) avisa una vez y la simulación sigue sin puente.

    properties (Nontunable)
        Puerto = 18700          % puerto TCP
        Direccion = '127.0.0.1' % '0.0.0.0' para aceptar clientes de otra PC
        Activo = 1              % 0 = no abrir el socket
        TDesconectar = 5        % s sin poder enviar nada -> se cierra el cliente
        Ts = 0.02               % s, periodo del bloque (para TDesconectar)
    end

    properties (Access = private)
        canal = []      % java.nio.channels.ServerSocketChannel (no bloqueante)
        sc = []         % java.nio.channels.SocketChannel del cliente (no bloqueante)
        rx = []         % ByteBuffer de recepción (acumula bytes hasta tramas completas)
        tx = []         % ByteBuffer de la trama en curso (pendiente si quedó a medias)
        ultimo = zeros(32, 1)
        nrx = 0
        k = 0
        ndrop = 0       % tramas de telemetría descartadas (cliente que no lee)
        ndrop_seg = 0   % descartes consecutivos
    end

    methods
        function obj = QD2PuenteTCP(varargin)
            % QD2PuenteTCP('Puerto', 18700, ...) también sirve desde MATLAB
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.ultimo = zeros(32, 1);
            obj.nrx = 0;
            obj.k = 0;
            obj.ndrop = 0;
            obj.ndrop_seg = 0;
            obj.canal = [];
            obj.sc = [];
            if obj.Activo
                try
                    c = java.nio.channels.ServerSocketChannel.open();
                    c.socket().setReuseAddress(true);
                    c.socket().bind(java.net.InetSocketAddress( ...
                        java.net.InetAddress.getByName(obj.Direccion), obj.Puerto));
                    c.configureBlocking(false);     % accept() devuelve [] si no hay cliente
                    obj.canal = c;
                    fprintf('[PYTHON BRIDGE] Stream Server en tcpip://%s:%d (64 doubles -> / <- 32 doubles)\n', ...
                        obj.Direccion, obj.Puerto);
                catch e
                    warning('QD2PuenteTCP:puerto', ...
                        'No se pudo abrir el puerto %d (%s). La simulación sigue sin puente.', ...
                        obj.Puerto, e.message);
                    obj.canal = [];
                end
            end
        end

        function [py, conectado, nrx] = outputImpl(obj, ~)
            obj.k = obj.k + 1;
            if ~isempty(obj.canal) && isempty(obj.sc) && mod(obj.k, 10) == 1
                aceptar(obj);   % cada 0.2 s mientras no haya cliente
            end
            conectado = double(~isempty(obj.sc));
            if conectado
                try
                    % lee lo disponible sin bloquear
                    for it = 1:64
                        n = obj.sc.read(obj.rx);
                        if n < 0
                            error('QD2PuenteTCP:eof', 'fin de flujo');
                        end
                        if n == 0
                            break;
                        end
                    end
                    pos = obj.rx.position();
                    nf = floor(pos/256);
                    if nf > 0
                        % se queda con la ÚLTIMA trama completa ("most recent")
                        a = obj.rx.array();
                        i0 = (nf - 1)*256;
                        v = typecast(int8(a(i0+1:i0+256)), 'double');
                        obj.ultimo = v(:);
                        obj.nrx = obj.nrx + nf;
                        % conserva los bytes de una trama incompleta
                        obj.rx.position(nf*256);
                        obj.rx.limit(pos);
                        obj.rx.compact();
                    end
                catch
                    cerrar_cliente(obj);
                    conectado = 0;
                end
            end
            py = obj.ultimo;
            nrx = obj.nrx;
        end

        function updateImpl(obj, tel)
            if isempty(obj.sc)
                return;
            end
            try
                if ~isempty(obj.tx) && obj.tx.hasRemaining()
                    obj.sc.write(obj.tx);           % termina la trama pendiente
                end
                if ~isempty(obj.tx) && obj.tx.hasRemaining()
                    % el cliente no está leyendo: se descarta esta trama
                    obj.ndrop = obj.ndrop + 1;
                    obj.ndrop_seg = obj.ndrop_seg + 1;
                    if obj.ndrop_seg == 1
                        fprintf('[PYTHON BRIDGE] el cliente no lee: se descartan tramas de telemetría\n');
                    end
                    if obj.ndrop_seg*obj.Ts >= obj.TDesconectar
                        fprintf('[PYTHON BRIDGE] %.0f s sin poder enviar: se cierra el cliente (%d tramas descartadas)\n', ...
                            obj.TDesconectar, obj.ndrop);
                        cerrar_cliente(obj);
                    end
                    return;
                end
                obj.ndrop_seg = 0;
                obj.tx = java.nio.ByteBuffer.wrap(typecast(double(tel(:)'), 'int8'));
                obj.sc.write(obj.tx);               % puede quedar a medias: se termina luego
            catch
                cerrar_cliente(obj);    % el cliente se desconectó
            end
        end

        function releaseImpl(obj)
            cerrar_cliente(obj);
            if ~isempty(obj.canal)
                try
                    obj.canal.close();
                catch
                end
                obj.canal = [];
            end
            if obj.ndrop > 0
                fprintf('[PYTHON BRIDGE] tramas de telemetría descartadas en total: %d\n', obj.ndrop);
            end
        end

        function resetImpl(obj)
            obj.ultimo = zeros(32, 1);
        end

        function n = getNumInputsImpl(~)
            n = 1;
        end

        function n = getNumOutputsImpl(~)
            n = 3;
        end

        function [a, b, c] = getOutputSizeImpl(~)
            a = [32 1]; b = [1 1]; c = [1 1];
        end

        function [a, b, c] = getOutputDataTypeImpl(~)
            a = 'double'; b = 'double'; c = 'double';
        end

        function [a, b, c] = isOutputComplexImpl(~)
            a = false; b = false; c = false;
        end

        function [a, b, c] = isOutputFixedSizeImpl(~)
            a = true; b = true; c = true;
        end

        function flag = isInputDirectFeedthroughImpl(~, ~)
            flag = false;   % py no depende de la telemetría del mismo paso
        end

        function nombre = getInputNamesImpl(~)
            nombre = 'Telemetry [64]';
        end

        function [a, b, c] = getOutputNamesImpl(~)
            a = 'Python Commands [32]'; b = 'Connected'; c = 'Packets Rx';
        end
    end

    methods (Access = private)
        function aceptar(obj)
            try
                s = obj.canal.accept();       % [] si no hay cliente esperando
                if ~isempty(s)
                    s.configureBlocking(false);
                    s.socket().setTcpNoDelay(true);
                    obj.sc = s;
                    obj.rx = java.nio.ByteBuffer.allocate(8192);
                    obj.tx = [];
                    obj.ndrop_seg = 0;
                    fprintf('[PYTHON BRIDGE] cliente conectado: %s\n', char(s.socket().getRemoteSocketAddress().toString()));
                end
            catch
                obj.sc = [];
            end
        end

        function cerrar_cliente(obj)
            if ~isempty(obj.sc)
                try
                    obj.sc.close();
                catch
                end
                fprintf('[PYTHON BRIDGE] cliente desconectado\n');
            end
            obj.sc = [];
            obj.rx = [];
            obj.tx = [];
            obj.ndrop_seg = 0;
        end
    end

    methods (Static, Access = protected)
        function simMode = getSimulateUsingImpl
            simMode = 'Interpreted execution';
        end

        function flag = showSimulateUsingImpl
            flag = false;
        end
    end
end
