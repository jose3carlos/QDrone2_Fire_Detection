function tests = test_failure_transitions
tests=functiontests(localfunctions);
end
function test_clear_valid_result_on_no_detection_and_failure(test)
% Test-only fake backend isolates latch behavior; separate tests run real TFLM.
model=create_simulink_model();cleanup=onCleanup(@()close_system(model,0)); %#ok<NASGU>
cfg=config();data=load(fullfile(cfg.buildRoot,'active_model.mat'),'artifact');a=data.artifact;
set_param(model,'SimUserLibraries',['"' a.fixtureLibrary '"']);
blank=zeros(a.metadata.height,a.metadata.width,a.metadata.channels,'uint8');valid=blank;valid(1)=1;bad=blank;bad(1)=255;
sequence=prepare_image_sequence({valid,blank,valid,bad},[0 .3 .6 .9],a.metadata);
load_image_sequence(model,sequence);out=sim(model);
u=out.yout.getElement('resultUpdated').Values.Data;
s=out.yout.getElement('status').Values.Data;
d=out.yout.getElement('detected').Values.Data;
c=out.yout.getElement('confidence').Values.Data;
verifyEqual(test,s(logical(u)),int32([1;2;1;-3]));
verifyEqual(test,d(logical(u)),logical([1;0;1;0]));
verifyEqual(test,c(logical(u)),single([91;0;91;0]));
verifyEqual(test,out.yout.getElement('resultID').Values.Data(end),uint64(4));
end
