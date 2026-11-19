function test_model_switch()
% Run after editing ONLY MODEL_PATH in config.m.
setup_project(false); cfg=config(); cache=fullfile(cfg.buildRoot,'switch_reference.mat');
sourceHash=ei_hash(fullfile(cfg.root,'simulink','EI_YOLO_Function.m'),true);
previous=[];if isfile(cache), previous=load(cache); end
a=select_model(); load_ei_model(); info=get_ei_model_info();close_ei_model();
assert(info.width==a.metadata.width && info.height==a.metadata.height);
if ~isempty(previous) && ~strcmp(previous.modelHash,a.modelHash)
    assert(strcmp(previous.sourceHash,sourceHash),'EI:GenericBlock','The generic block source changed.');
    fprintf('Switched from [%s] to [%s]; dimensions %d x %d.\n',strjoin(previous.labels,', '),strjoin(info.labels,', '),info.height,info.width);
else
    warning('EI:SecondModelUnavailable','Workflow verified for current model only. A second distinct real export is required to validate switching.');
end
modelHash=a.modelHash;labels=info.labels; %#ok<NASGU>
save(cache,'modelHash','sourceHash','labels');
end
