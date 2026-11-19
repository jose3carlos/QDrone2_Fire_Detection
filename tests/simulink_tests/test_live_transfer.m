function tests = test_live_transfer
tests=functiontests(localfunctions);
end
function test_actual_live_transfer(test)
model=create_simulink_model();cleanup=onCleanup(@()close_system(model,0)); %#ok<NASGU>
enable_live_source(model);cfg=config();d=load(fullfile(cfg.buildRoot,'active_model.mat'),'artifact');i=d.artifact.metadata;
image=zeros(i.height,i.width,i.channels,'uint8');
r=submit_live_image(image);verifyTrue(test,r.accepted);
r=submit_live_image(image);verifyFalse(test,r.accepted);verifyEqual(test,r.status,int32(-6));
assignin('base','eiLiveTestImage',image);
set_param(model,'StartFcn',sprintf('ei_test_live_hook(''attach'',''%s'',eiLiveTestImage);',model), ...
    'StopFcn',sprintf('ei_test_live_hook(''detach'',''%s'',[]);',model),'StopTime','1');
out=sim(model);verifyEqual(test,nnz(out.yout.getElement('resultUpdated').Values.Data),2);
verifyEqual(test,out.yout.getElement('resultID').Values.Data(end),uint64(2));
updated=out.yout.getElement('resultUpdated').Values.Data;
status=out.yout.getElement('status').Values.Data;
verifyGreaterThanOrEqual(test,status(logical(updated)),int32(0));
end
