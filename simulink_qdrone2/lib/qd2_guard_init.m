function gs = qd2_guard_init()
%QD2_GUARD_INIT Estado inicial del SEPARATION GUARD (qd2_separation_guard).
%#codegen
gs.init  = 0;
gs.outE  = [0 0 0 0];   % última consigna entregada al explorador
gs.outR  = [0 0 0 0];   % última consigna entregada al respondedor
gs.modoB = 0;           % dron que cede: 0 libre, 1 retenido, 2 evadiendo, 3 detenido sin evasión
gs.holdB = [0 0];       % punto de espera / evasión del dron que cede
gs.prio_prev = 0;       % 1 si en el paso anterior el respondedor tenía prioridad
gs.esperaA = 0;         % 1 si el dron con prioridad está esperando
gs.pE_ok = [0 0];       % última pose (x,y) finita del explorador
gs.pR_ok = [0 0];       % última pose (x,y) finita del respondedor
gs.tiene_pose = [0 0];  % 1 cuando ya se recibió una pose finita de cada uno
end
