function test_single_class_adapter(testDir)
% Test-only DLL fixture: no trained one-class model is implied by these checks.
setup_project(false);cfg=config();
if nargin<1
    active=load(fullfile(cfg.buildRoot,'active_model.mat'),'artifact');
    testDir=fullfile(active.artifact.folder,'cmake');
end
dll=fullfile(testDir,'bin','ei_contract_fixture.dll');
assert(isfile(dll),'EI:Fixture','Build native_unit_tests and ei_contract_fixture CMake targets first.');
mex('-R2018a',['-I' fullfile(cfg.root,'native')],fullfile(cfg.root,'native','ei_matlab_mex.cpp'), ...
    '-outdir',testDir,'-output','ei_gateway_single_class_test');
addpath(testDir);setenv('PATH',[fileparts(dll) pathsep getenv('PATH')]);
gateway='ei_gateway_single_class_test';cleanup=onCleanup(@()feval(gateway,'unload')); %#ok<NASGU>
info=feval(gateway,'initialize',dll,.5,0,4096,0,0);
assert(info.classCount==1 && isequal(info.labels,{'Class_A'}));
blank=zeros(info.height,info.width,info.channels,'uint8');valid=blank;valid(1)=1;bad=blank;bad(1)=255;
r=feval(gateway,'infer',valid);assert(r.detected && r.classIndex==0 && ~r.objectClass && r.confidence==91);
r=feval(gateway,'infer',blank);assert(~r.detected && r.classIndex==-1 && isempty(r.label) && r.confidence==0);
r=feval(gateway,'infer',bad);assert(r.status==-3 && ~r.detected && r.resultID==uint64(3));
feval(gateway,'close');
% Simulate a COPY of the existing example with the fixture; leave active_model
% and the production .slx untouched. The event block source stays unchanged.
source=fullfile(cfg.root,'simulink','EdgeImpulse_YOLO_Simulink.slx');
assert(~bdIsLoaded('EdgeImpulse_YOLO_Simulink'),'EI:ModelLoaded','Close the example before testing.');
load_system(source);model='EI_SingleClass_Test';
save_system('EdgeImpulse_YOLO_Simulink',fullfile(testDir,[model '.slx']));
modelCleanup=onCleanup(@()close_system(model,0)); %#ok<NASGU>
set_param(model,'InitFcn','','SimUserLibraries',['"' fullfile(testDir,'lib','libei_contract_fixture.dll.a') '"'], ...
    'SimUserIncludeDirs',['"' fullfile(cfg.root,'native') '"'], ...
    'SimCustomInitializer','if(ei_sim_start(0.5f,0u,4096u,0u,0.0)<0) mexErrMsgIdAndTxt("EI:Fixture","Initialization failed");');
sequence=prepare_image_sequence({valid,blank,bad},[0 .3 .6],info);
load_image_sequence(model,sequence);out=sim(model);
updated=out.yout.getElement('resultUpdated').Values.Data;
status=out.yout.getElement('status').Values.Data;
detected=out.yout.getElement('detected').Values.Data;
assert(nnz(updated)==3 && isequal(status(logical(updated)),int32([1;2;-3])));
assert(isequal(detected(logical(updated)),logical([1;0;0])));
assert(~any(out.yout.getElement('objectClass').Values.Data));
assert(out.yout.getElement('resultID').Values.Data(end)==uint64(3));
data=load(fullfile(cfg.buildRoot,'active_model.mat'),'artifact');
if data.artifact.metadata.classCount~=1
    rejected=false;
    try, inspect_model(data.artifact.metadata.root);
    catch exception, rejected=strcmp(exception.identifier,'EI:UnsupportedExport');end
    assert(rejected,'Two-class exports must be rejected.');
end
fprintf('Single-class fixture passed: MATLAB metadata/results, Simulink events, failure clearing and two-class rejection.\n');
end
