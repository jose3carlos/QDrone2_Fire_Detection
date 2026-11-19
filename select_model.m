function artifact = select_model()
% Model selection lives ONLY in config.m. Edit MODEL_PATH, then call this.
ei_assert_stopped(); close_ei_model(); artifact=build_model();
if bdIsLoaded('EdgeImpulse_YOLO_Simulink')
    close_system('EdgeImpulse_YOLO_Simulink',0);
end
create_simulink_model();
end
