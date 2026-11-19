function ei_sim_prepare(model)
cfg=config(); data=load(fullfile(cfg.buildRoot,'active_model.mat'),'artifact'); a=data.artifact;
assert(strcmp(a.modelPath,char(cfg.modelPath)),'EI:ModelChanged','Model path changed; run select_model before simulation.');
workspace=get_param(model,'ModelWorkspace');stored=getVariable(workspace,'eiBuildInfo');
assert(isstruct(stored) && strcmp(stored.buildKey,a.key),'EI:StaleSimulinkModel', ...
    'The compiled model changed. Regenerate the Simulink example.');
close_ei_model();
setenv('PATH',[fileparts(a.dll) pathsep getenv('PATH')]);
end
