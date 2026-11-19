function report=validate_custom_model_integration()
% Validate the actual model bindings and unchanged batch script without a camera.
setup_project(false); cfg=config(); addpath(fullfile(cfg.root,'tests','simulink_tests'));
load_system('simulink');
original='Trayectory_Model'; load_system(original);
assert(strcmp(get_param(original,'Dirty'),'off'),'Save the model before validation.');
originalFile=get_param(original,'FileName'); before=ei_hash(originalFile,true);
data=load(fullfile(cfg.buildRoot,'active_model.mat'),'artifact'); a=data.artifact;
work=fullfile(cfg.buildRoot,'custom_model_validation'); if ~isfolder(work),mkdir(work);end
model='EI_Trayectory_Validation'; file=fullfile(work,[model '.slx']);
if bdIsLoaded(model),close_system(model,0);end
copyfile(originalFile,file);
load_system(file); cleanup=onCleanup(@()finish(model)); %#ok<NASGU>
% Only this disposable copy receives a synthetic camera and logging settings.
replace_block(model,'SearchDepth',1,'Name','From Video Device', ...
    'simulink/Sources/Constant','noprompt');
set_param([model '/From Video Device'],'Value','zeros(720,1280,3,''uint8'')','SampleTime','1/30');
replace_block(model,'SearchDepth',1,'Name','Video Viewer', ...
    'simulink/Sinks/Terminator','noprompt');
set_param([model '/Constant'],'Value','true','OutDataTypeStr','boolean');
set_param(model,'StopTime','5.2','SimulationMode','normal');
root=sfroot;
source=root.find('-isa','Stateflow.EMChart','Path',[original '/FOMO Inference']);
copy=root.find('-isa','Stateflow.EMChart','Path',[model '/FOMO Inference']);
assert(strcmp(source.Script,copy.Script),'Inference script changed in the test copy.');
set_param(model,'StartFcn',sprintf('ei_custom_validation_hook(''attach'',''%s'',eiCustomValidationArtifact);',model), ...
    'StopFcn',sprintf('ei_custom_validation_hook(''detach'',''%s'',[]);',model));
assignin('base','eiCustomValidationArtifact',a);
% Deliberately open standalone ownership: InitFcn must release it for simulation.
load_ei_model();
sim(model);
live=ei_custom_validation_hook('results',model,[]);
assert(~isempty(live) && max(live(:,2))==20,'Expected exactly one 20-image batch.');
assert(live(find(live(:,2)==20,1),1)<5.2,'Inference completed only after simulation.');
assert(all(live(:,4)>=0 & live(:,4)<=1),'Confidence out of range.');
assert(all(abs(live(:,4)*100-round(live(:,4)*100))<1e-9),'Confidence rounding changed.');
% A second run must restart result IDs and release/reacquire the engine cleanly.
sim(model); second=ei_custom_validation_hook('results',model,[]);
assert(max(second(:,2))==20,'Repeated simulation did not reset/release the native engine.');
% Exercise the classic C/MEX path that produced the reported missing-header error.
importMode=get_param(model,'SimParseCustomCode');
set_param(model,'SimParseCustomCode','off');
sim(model); classic=ei_custom_validation_hook('results',model,[]);
assert(max(classic(:,2))==20,'Classic C/MEX simulation failed to process the batch.');
set_param(model,'SimParseCustomCode',importMode);
info=load_ei_model(); single=run_ei_inference(zeros(180,180,'uint8')); close_ei_model();
expected=round(double(single.detected)*single.confidence)/100;
assert(live(end,3)==double(single.detected) && abs(live(end,4)-expected)<1e-12, ...
    'Batch outputs disagree with standalone inference on the same images.');
% Rebinding is idempotent and does not alter solver, rate or callback bodies.
parameters={'SolverType','Solver','FixedStep','SimulationMode','StartFcn','StopFcn', ...
    'SimTargetLang','InitFcn','PostLoadFcn','SimCustomHeaderCode','SimCustomInitializer', ...
    'SimCustomTerminator','SimUserIncludeDirs','SimUserLibraries'};
snapshot=cellfun(@(p)get_param(model,p),parameters,'UniformOutput',false);
ei_custom_model_binding('install',model); ei_custom_model_binding('install',model);
after=cellfun(@(p)get_param(model,p),parameters,'UniformOutput',false);
assert(isequal(snapshot,after),'Repeated binding changed existing settings.');
% Existing unrelated callback bodies and custom-code dependencies survive install.
userCallback=sprintf('\n%% Existing unrelated callback\neiCustomUserSentinel=true;\n');
set_param(model,'InitFcn',[get_param(model,'InitFcn') userCallback], ...
    'PostLoadFcn',[get_param(model,'PostLoadFcn') userCallback], ...
    'SimCustomHeaderCode',[get_param(model,'SimCustomHeaderCode') '/* unrelated header */'], ...
    'SimCustomInitializer',[get_param(model,'SimCustomInitializer') '/* unrelated initialization */'], ...
    'SimCustomTerminator',[get_param(model,'SimCustomTerminator') '/* unrelated termination */'], ...
    'SimUserIncludeDirs',[get_param(model,'SimUserIncludeDirs') newline '"C:\unrelated include folder"'], ...
    'SimUserLibraries',[get_param(model,'SimUserLibraries') newline '"C:\unrelated library.a"']);
ei_custom_model_binding('install',model);
assert(endsWith(get_param(model,'InitFcn'),userCallback));
assert(endsWith(get_param(model,'PostLoadFcn'),userCallback));
assert(endsWith(get_param(model,'SimCustomHeaderCode'),'/* unrelated header */'));
assert(endsWith(get_param(model,'SimCustomInitializer'),'/* unrelated initialization */'));
assert(endsWith(get_param(model,'SimCustomTerminator'),'/* unrelated termination */'));
assert(contains(get_param(model,'SimUserIncludeDirs'),'"C:\unrelated include folder"'));
assert(contains(get_param(model,'SimUserLibraries'),'"C:\unrelated library.a"'));
for k=1:numel(parameters),set_param(model,parameters{k},snapshot{k});end
% Simulate a replaced model artifact and stale absolute paths without editing the real manifest.
workspace=get_param(model,'ModelWorkspace'); binding=getVariable(workspace,'eiCustomBinding');
stale=binding; stale.importLibrary='C:\old_computer\libei_inference_old.dll.a';
stale.includeDir='C:\old_computer\native'; stale.buildKey='old';
assignin(workspace,'eiCustomBinding',stale);
set_param(model,'SimUserLibraries',['"' stale.importLibrary '"'], ...
    'SimUserIncludeDirs',['"' stale.includeDir '"']);
ei_custom_model_binding('refresh',model);
assert(~contains(get_param(model,'SimUserLibraries'),'old_computer'));
assert(contains(get_param(model,'SimUserLibraries'),a.importLibrary));
assert(strcmp(getVariable(workspace,'eiCustomBinding').buildKey,a.key));
correct=getVariable(workspace,'eiCustomBinding'); incompatible=correct;
incompatible.imageContract=[96 96 1]; assignin(workspace,'eiCustomBinding',incompatible);
try
    ei_custom_model_binding('refresh',model);
    error('EI:Validation','An incompatible image contract was accepted.');
catch exception
    assert(strcmp(exception.identifier,'EI:ImageContractChanged'));
end
assignin(workspace,'eiCustomBinding',correct);
assert(strcmp(before,ei_hash(originalFile,true)),'Validation changed the original saved model.');
report=struct('buildKey',a.key,'modelInfo',info,'inferencesPerRun',20, ...
    'completedAt',live(find(live(:,2)==20,1),1),'confidenceAverage',live(end,4), ...
    'objectDetected',logical(live(end,3)),'standaloneComparison',true,'repeatedRun',true, ...
    'idempotentRefresh',true,'unrelatedSettingsPreserved',true,'staleBindingRefresh',true, ...
    'classicCCompileAndLink',true,'incompatibleContractRejected',true,'originalUnchanged',true);
save(fullfile(work,'results.mat'),'report','live','second','classic'); disp(report);
fprintf('CUSTOM_MODEL_INTEGRATION_VALIDATION_PASSED\n');
end

function finish(model)
ei_custom_validation_hook('detach',model,[]);
if bdIsLoaded(model),close_system(model,0);end
close_ei_model();
evalin('base','clear eiCustomValidationArtifact');
end
