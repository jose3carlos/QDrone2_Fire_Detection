function report=validate_fomo_visualization(blockCode)
% blockCode is supplied in memory: this test never saves the user's block code.
setup_project(false); cfg=config(); a=load(fullfile(cfg.buildRoot,'active_model.mat'),'artifact');a=a.artifact;
v=load(fullfile(cfg.buildRoot,'visualization','active_visualization.mat'),'visualization');v=v.visualization;
addpath(v.folder,fullfile(cfg.root,'tests','simulink_tests'));
sourceFile=fullfile(cfg.root,'Trayectory_Model.slx'); originalHash=ei_hash(sourceFile,true);
load_system(sourceFile); root=sfroot;
original=root.find('-isa','Stateflow.EMChart','Path','Trayectory_Model/FOMO Inference');
averageCode=original.Script;
model='EI_Visualization_InMemory_Test';
if bdIsLoaded(model),close_system(model,0);end
new_system(model); cleanup=onCleanup(@()finish(model)); %#ok<NASGU>
configure_simulink_model(model); % Only a disposable, unsaved test model.
set_param(model,'FixedStep','1/30','StopTime','0.9', ...
    'StartFcn',sprintf('ei_visualization_test_hook(''attach'',''%s'');',model), ...
    'StopFcn',sprintf('ei_visualization_test_hook(''detach'',''%s'');',model));
bank=zeros(180,180,20,'uint8');
for k=1:20,bank(:,:,k)=uint8(k);end
workspace=get_param(model,'ModelWorkspace');assignin(workspace,'testImages',bank);
add_block('simulink/Sources/Constant',[model '/Images'],'Value','testImages','SampleTime','1/30');
add_block('simulink/Sources/Constant',[model '/Ready'],'Value','true','SampleTime','1/30');
add_block('simulink/User-Defined Functions/MATLAB Function',[model '/Viewer']);
chart=root.find('-isa','Stateflow.EMChart','Path',[model '/Viewer']);chart.Script=blockCode;
add_line(model,'Images/1','Viewer/1');add_line(model,'Ready/1','Viewer/2');
for k=1:7
    add_block('simulink/Sinks/Terminator',[model '/ViewSink' num2str(k)]);
    add_line(model,['Viewer/' num2str(k)],['ViewSink' num2str(k) '/1']);
end
% Both blocks infer independently in one simulation without fighting ownership.
add_block('simulink/User-Defined Functions/MATLAB Function',[model '/Average']);
chart=root.find('-isa','Stateflow.EMChart','Path',[model '/Average']);chart.Script=averageCode;
add_line(model,'Images/1','Average/1');add_line(model,'Ready/1','Average/2');
for k=1:2
    add_block('simulink/Sinks/Out1',[model '/AverageOutput' num2str(k)],'Port',num2str(k));
    add_line(model,['Average/' num2str(k)],['AverageOutput' num2str(k) '/1']);
end
set_param(model,'SaveOutput','on','SaveFormat','Dataset','OutputSaveName','yout','ReturnWorkspaceOutputs','on');
% A gateway without the simulation owner must fail explicitly before inference.
try
    feval(v.gateway,'infer',a.dll,bank(:,:,1));error('EI:Test','Uninitialized inference was accepted.');
catch exception
    assert(strcmp(exception.identifier,'EI:VisualizationNative') && contains(exception.message,'-2:'));
end
out=sim(model);
records=ei_visualization_test_hook('records',model);
assert(isequal(records(:,2),(1:20)') && sum(records(:,3))==1 && records(end,3)==1);
assert(records(end,1)<0.9,'Figures were not updated until simulation stopped.');
project=ei_hash(cfg.root); tags=sprintf('EI_FOMO_%s_Frame_',project(1:12));
figures=findall(groot,'Type','figure','-regexp','Tag',['^' tags]);
assert(numel(figures)==20,'Expected twenty separate image figures.');
found=false(1,20); positive=0; totalConfidence=0;
for figureHandle=reshape(figures,1,[])
    meta=get(figureHandle,'UserData');k=double(meta.imageIndex);found(k)=true;
    imageHandle=findall(figureHandle,'Tag','EI_FOMO_Pixels');
    assert(isequal(get(imageHandle,'CData'),bank(:,:,k)),'Displayed pixels differ from the inferred image.');
    rect=findall(figureHandle,'Tag','EI_FOMO_Box');
    assert(numel(rect)==double(meta.result.detected));
    positive=positive+double(meta.result.detected);
    totalConfidence=totalConfidence+meta.result.confidence*double(meta.result.detected);
end
assert(all(found));
assert(out.yout{1}.Values.Data(end)==(positive>10));
assert(abs(out.yout{2}.Values.Data(end)-round(totalConfidence/20)/100)<1e-12);
% Reuse figures on a second run, recreating any window closed by the user.
close(figures(1)); sim(model);
figures=findall(groot,'Type','figure','-regexp','Tag',['^' tags]);assert(numel(figures)==20);
% Positive-box rendering is deterministic even when synthetic frames detect nothing.
result=struct('detected',true,'confidence',91,'boundingBox',uint32([0 16 8 8]), ...
    'resultID',uint64(100),'label','Test class','status',1,'detectionCount',1);
f=ei_show_fomo_frame(bank(:,:,1),result,uint32(1),uint32(20),uint64(2));
rect=findall(f,'Tag','EI_FOMO_Box');assert(isequal(get(rect,'Position'),[0.5 16.5 8 8]));
assert(isequal(get(findall(f,'Tag','EI_FOMO_Pixels'),'CData'),bank(:,:,1)));
assert(strcmp(originalHash,ei_hash(sourceFile,true)),'Original model was modified.');
report=struct('nativeKey',a.key,'processedFrames',20,'separateFigures',20, ...
    'completedDuringSimulation',true,'pixelsUnchanged',true,'boxCoordinates',true, ...
    'coexistsWithAverage',true,'averageUnchanged',true,'windowReuse',true, ...
    'uninitializedRejected',true,'originalModelUnchanged',true);
save(fullfile(cfg.buildRoot,'visualization','validation_results.mat'),'report','records');disp(report);
fprintf('FOMO_VISUALIZATION_VALIDATION_PASSED\n');
end

function finish(model)
ei_visualization_test_hook('detach',model);
if bdIsLoaded(model),close_system(model,0);end
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));project=ei_hash(root);
figures=findall(groot,'Type','figure','-regexp','Tag',['^EI_FOMO_' project(1:12) '_Frame_']);
if ~isempty(figures),close(figures);end
end
