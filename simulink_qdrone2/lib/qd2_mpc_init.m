function mst = qd2_mpc_init(p)
%QD2_MPC_INIT Estado inicial (struct de tamaño fijo) de qd2_mpc_position.
%   mst = qd2_mpc_init(p). Llamar al arrancar y cada vez que el dron pase a
%   TAKEOFF_HEIGHT (igual que el reset del integrador del PID original).
%#codegen
N = size(p.mpc.Hxy, 1);
mst.init  = 0;
mst.Ux    = zeros(N,1);   % solución previa (arranque en caliente)
mst.Uy    = zeros(N,1);
mst.Uz    = zeros(N,1);
mst.ux_prev = 0;          % primer comando aplicado en el paso anterior
mst.uy_prev = 0;
mst.uz_prev = 0;
mst.v_hat = 0;            % velocidad vertical estimada por el observador
mst.d_hat = 0;            % perturbación vertical estimada (m/s^2)
mst.a_cmd_prev = 0;       % aceleración vertical comandada en el paso anterior
mst.a_ff_prev  = 0;       % feedforward de downwash usado en el paso anterior
mst.att_model = [0 0];    % [th_x th_y] del modelo si no hay actitud medida
end
