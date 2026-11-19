function tests = test_simulink_events
tests=functiontests(localfunctions);
end
function setupOnce(test)
setup_project(false);close_ei_model();test.TestData.model=create_simulink_model();
cfg=config();data=load(fullfile(cfg.buildRoot,'active_model.mat'),'artifact');
test.TestData.info=data.artifact.metadata;
end
function teardownOnce(test)
close_system(test.TestData.model,0);
end
function test_four_events_and_latch(test)
model=test.TestData.model;i=test.TestData.info;image=zeros(i.height,i.width,i.channels,'uint8');
s=prepare_image_sequence(repmat({image},1,4),[0 40 85 130],i);load_image_sequence(model,s);
out=sim(model);updated=out.yout.getElement('resultUpdated').Values;
ids=out.yout.getElement('resultID').Values;
verifyEqual(test,nnz(updated.Data),4);verifyEqual(test,ids.Data(end),uint64(4));
verifyEqual(test,updated.Time(logical(updated.Data)),[0;40;85;130],'AbsTol',1e-8);
status=out.yout.getElement('status').Values.Data;
verifyGreaterThanOrEqual(test,status(logical(updated.Data)),int32(0));
names={'confidence','objectClass','detected','status','resultID'};
for n=1:numel(names)
    v=out.yout.getElement(names{n}).Values;d=v.Data(:);u=updated.Data(:);
    verifyEqual(test,d(find(~u(2:end))+1),d(find(~u(2:end))));
end
verifyClass(test,out.yout.getElement('confidence').Values.Data,'single');
verifyClass(test,out.yout.getElement('objectClass').Values.Data,'logical');
verifyClass(test,out.yout.getElement('status').Values.Data,'int32');
end
function test_held_high(test)
model=test.TestData.model;i=test.TestData.info;image=zeros(i.height,i.width,i.channels,'uint8');
s=prepare_image_sequence({image},0,i);s.newImage.Data(:)=true;s.stopTime=1;
load_image_sequence(model,s);out=sim(model);
verifyEqual(test,nnz(out.yout.getElement('resultUpdated').Values.Data),1);
verifyEqual(test,out.yout.getElement('resultID').Values.Data(end),uint64(1));
end
function test_no_event_and_restart(test)
model=test.TestData.model;i=test.TestData.info;
s=prepare_image_sequence({zeros(i.height,i.width,i.channels,'uint8')},0,i);
s.newImage.Data(:)=false;s.stopTime=1;load_image_sequence(model,s);out=sim(model);
verifyEqual(test,nnz(out.yout.getElement('resultUpdated').Values.Data),0);
verifyEqual(test,out.yout.getElement('resultID').Values.Data(end),uint64(0));
verifyEqual(test,out.yout.getElement('status').Values.Data(end),int32(0));
end
function test_irregular_fractional(test)
model=test.TestData.model;i=test.TestData.info;
s=prepare_image_sequence(repmat({zeros(i.height,i.width,i.channels,'uint8')},1,4),[0 .3 1.7 4.2],i);
load_image_sequence(model,s);out=sim(model);
verifyEqual(test,nnz(out.yout.getElement('resultUpdated').Values.Data),4);
end
function test_reject_ambiguous_events(test)
i=test.TestData.info;im=zeros(i.height,i.width,i.channels,'uint8');
verifyError(test,@()prepare_image_sequence({im,im},[0 .1],i),'EI:Events');
verifyError(test,@()prepare_image_sequence({im},.123,i),'EI:TimeGrid');
end
