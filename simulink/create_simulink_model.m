function model = create_simulink_model()
% Creates a real .slx using MATLAB R2023b APIs. No Simulink Coder license needed.
setup_project(false); cfg=config();
assert(license('test','Simulink'),'EI:Simulink','Simulink is required to generate the example.');
data=load(fullfile(cfg.buildRoot,'active_model.mat'),'artifact'); a=data.artifact;
model='EdgeImpulse_YOLO_Simulink';
if bdIsLoaded(model), close_system(model,0); end
new_system(model); configure_simulink_model(model);
add_block('simulink/Sources/From Workspace',[model '/Image'], ...
    'VariableName','eiImageSequence','Interpolate','off','OutputAfterFinalValue','Holding final value','SampleTime',num2str(cfg.image.sampleTime,17), ...
    'Position',[35 70 170 110]);
add_block('simulink/Sources/From Workspace',[model '/New Image Event'], ...
    'VariableName','eiNewImageSequence','Interpolate','off','OutputAfterFinalValue','Holding final value','SampleTime',num2str(cfg.image.sampleTime,17), ...
    'Position',[35 155 170 195]);
add_block('simulink/User-Defined Functions/MATLAB Function',[model '/Event Detector'], ...
    'Position',[245 65 470 235]);
sf=sfroot; chart=sf.find('-isa','Stateflow.EMChart','Path',[model '/Event Detector']);
chart.Script=fileread(fullfile(cfg.root,'simulink','EI_YOLO_Function.m'));
input=chart.find('-isa','Stateflow.Data','Name','image');
input.DataType='uint8';
if a.metadata.channels==1,input.Props.Array.Size=sprintf('[%d %d]',a.metadata.height,a.metadata.width);
else,input.Props.Array.Size=sprintf('[%d %d 3]',a.metadata.height,a.metadata.width);end
trigger=chart.find('-isa','Stateflow.Data','Name','newImage'); trigger.DataType='boolean'; trigger.Props.Array.Size='1';
add_line(model,'Image/1','Event Detector/1'); add_line(model,'New Image Event/1','Event Detector/2');
names={'confidence','objectClass','detected','status','resultID','resultUpdated'};
for i=1:numel(names)
    add_block('simulink/Sinks/Out1',[model '/' names{i}], ...
        'Port',num2str(i),'Position',[555 40+i*50 585 54+i*50]);
    line=add_line(model,['Event Detector/' num2str(i)],[names{i} '/1']);
    set_param(line,'Name',names{i});
end
set_param(model,'SaveOutput','on','OutputSaveName','yout','SaveFormat','Dataset', ...
    'ReturnWorkspaceOutputs','on');
sequence=prepare_image_sequence({zeros(a.metadata.height,a.metadata.width,a.metadata.channels,'uint8')},0,a.metadata);
load_image_sequence(model,sequence);
note=Simulink.Annotation(model,sprintf(['Event-triggered native Edge Impulse detector\n' ...
    'One rising edge = one inference. Low sample re-arms.\n' ...
    'Synchronous: simulation waits for native inference.\n' ...
    'Outputs latch until the next result; use detected + resultUpdated.\n' ...
    'Supplied export: %s, %d x %d, %d channel(s)'],a.metadata.lastLayer,a.metadata.height,a.metadata.width,a.metadata.channels));
note.Position=[25 395 640 480];
save_system(model,fullfile(cfg.root,'simulink',[model '.slx']));
fprintf('Generated %s\n',fullfile(cfg.root,'simulink',[model '.slx']));
end
