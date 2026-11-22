function ob = qd2_onboard_init(p)
%QD2_ONBOARD_INIT Estado inicial de qd2_onboard_step.
%#codegen
ob.fsm = 1;
ob.T_rampa = 0;
ob.pid.int = zeros(4,1);
ob.mst = qd2_mpc_init(p);
ob.t_mpc = 0;
ob.u_mpc = [p.T_trim; 0; 0; 0; 0];
ob.status = 0;
ob.res = 0;
ob.z_land = 0;
end
