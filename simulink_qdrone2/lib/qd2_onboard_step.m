function [u, ob] = qd2_onboard_step(ob, arm, takeoff, ref, x_est, other, p, dt, modo)
%QD2_ONBOARD_STEP Emulación simplificada del stack a bordo para simulación.
%   [u, ob] = qd2_onboard_step(ob, arm, takeoff, ref, x_est, other, p, dt, modo)
%
%   Reproduce lo esencial de QD2_DroneStack (FSM de vuelo + COMMANDER):
%   1 IDLE, 2 ARMED, 3 TAKEOFF_THROTTLE (rampa de trim a 2.33 N/s como el
%   original), 4 TAKEOFF_HEIGHT, 5 FLIGHT, 6 AUTOLAND (z_ref baja a
%   0.5 m/s), 9 DISABLE (motores apagados).
%   modo = 1: PID original (qd2_pid_position); modo = 2: MPC
%   (qd2_mpc_position, llamado cada p.mpc.Ts; dt debe dividir a Ts).
%   ob = qd2_onboard_init(p) al iniciar.
%   x_est : 9x1 [x y z vx vy vz roll pitch yaw]; ref: [x y z yaw];
%   other : 6x1 pose/velocidad del otro dron (solo MPC).
%   u     : 4x1 [T roll pitch yaw_rate] para qd2_quad_plant.
%#codegen

u = zeros(4,1);
switch ob.fsm
    case 1
        if arm > 0.5
            ob.fsm = 2;
        end
    case 2
        if arm < 0.5
            ob.fsm = 1;
        elseif takeoff > 0.5
            ob.fsm = 3;
            ob.T_rampa = 0;
        end
    case 3
        ob.T_rampa = min(ob.T_rampa + 2.33*dt, p.T_trim);
        u = [ob.T_rampa; 0; 0; 0];
        if takeoff < 0.5 || arm < 0.5
            ob.fsm = 9;
        elseif ob.T_rampa >= p.T_trim
            ob.fsm = 4;
            ob.pid.int = zeros(4,1);
            ob.mst = qd2_mpc_init(p);
            ob.t_mpc = 0;
            ob.z_land = x_est(3);
        end
    case {4, 5, 6}
        r = [ref(1); ref(2); ref(3); ref(4)];
        if ob.fsm == 6
            ob.z_land = max(ob.z_land - 0.5*dt, -0.2);
            r(3) = ob.z_land;
        else
            ob.z_land = x_est(3);
        end
        if modo > 1.5
            ob.t_mpc = ob.t_mpc - dt;
            if ob.t_mpc <= 1e-9
                [uu, ob.mst, info] = qd2_mpc_position(x_est, r, other, p, ob.mst);
                ob.u_mpc = uu;
                ob.status = uu(5);
                ob.res = max(info.res);
                ob.t_mpc = ob.t_mpc + p.mpc.Ts;
            end
            u = ob.u_mpc(1:4);
        else
            [u, ob.pid] = qd2_pid_position(x_est, r, ob.pid, p, dt);
        end
        if arm < 0.5
            ob.fsm = 9;
        elseif ob.fsm == 4 && abs(x_est(3) - ref(3)) <= 0.1
            ob.fsm = 5;
        elseif ob.fsm <= 5 && takeoff < 0.5
            ob.fsm = 6;
            ob.z_land = x_est(3);
        elseif ob.fsm == 6 && x_est(3) <= 0.05
            ob.fsm = 9;
        end
    otherwise
        u = zeros(4,1);
end
end
