function tests=test_capture_batch_block
tests=functiontests(localfunctions);
end
function test_fixed_size_block_compiles_and_runs(test)
setup_project(false);cfg=config();model='EI_CaptureBatch_Test';
if bdIsLoaded(model),close_system(model,0);end
new_system(model);cleanup=onCleanup(@()close_system(model,0)); %#ok<NASGU>
set_param(model,'Solver','FixedStepDiscrete','FixedStep','1/30','StopTime','4.2', ...
    'SaveOutput','on','OutputSaveName','yout','SaveFormat','Dataset','ReturnWorkspaceOutputs','on');
add_block('simulink/Sources/Constant',[model '/RGB'],'Value','zeros(480,640,3,''uint8'')');
add_block('simulink/Sources/Constant',[model '/Trigger'],'Value','true');
add_block('simulink/Sources/Constant',[model '/NewFrame'],'Value','true');
add_block('simulink/Sources/Digital Clock',[model '/Clock'],'SampleTime','1/30');
add_block('simulink/User-Defined Functions/MATLAB Function',[model '/FrameID']);
add_block('simulink/User-Defined Functions/MATLAB Function',[model '/Capture']);
root=sfroot;id=root.find('-isa','Stateflow.EMChart','Path',[model '/FrameID']);
id.Script=sprintf('function id=f(t)\n id=uint64(round(t*30));\nend');
chart=root.find('-isa','Stateflow.EMChart','Path',[model '/Capture']);
chart.Script=fileread(fullfile(cfg.root,'simulink','CaptureFOMOBatch.m'));
add_line(model,'Clock/1','FrameID/1');add_line(model,'RGB/1','Capture/1');
add_line(model,'Trigger/1','Capture/2');add_line(model,'NewFrame/1','Capture/3');
add_line(model,'FrameID/1','Capture/4');add_line(model,'Clock/1','Capture/5');
add_block('simulink/Sinks/Terminator',[model '/Batch']);add_line(model,'Capture/1','Batch/1');
for k=1:2
    names={'ImgCaptured','Status'};
    add_block('simulink/Sinks/Out1',[model '/' names{k}],'Port',num2str(k));
    line=add_line(model,['Capture/' num2str(k+1)],[names{k} '/1']);set_param(line,'Name',names{k});
end
out=sim(model);ready=out.yout.getElement('ImgCaptured').Values;
status=out.yout.getElement('Status').Values;
verifyClass(test,ready.Data,'logical');verifyClass(test,status.Data,'int32');
index=find(ready.Data,1);verifyNotEmpty(test,index);
verifyEqual(test,ready.Time(index),4,'AbsTol',1e-8);
verifyTrue(test,all(ready.Data(index:end)));verifyEqual(test,status.Data(end),int32(2));
end
