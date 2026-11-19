function tests = test_matlab_interface
tests=functiontests(localfunctions);
end
function setupOnce(test)
setup_project(false);load_ei_model();test.TestData.info=get_ei_model_info();
end
function teardownOnce(~), close_ei_model(); end
function test_metadata(test)
i=test.TestData.info;verifyEqual(test,i.classCount,1);verifyEqual(test,numel(i.labels),1);
verifyEqual(test,i.booleanLabels,i.labels);
verifyTrue(test,ismember(i.channels,[1 3]));
end
function test_invalid_images(test)
i=test.TestData.info;
verifyError(test,@()run_ei_inference(nan(i.height,i.width)),'EI:InvalidImage');
verifyError(test,@()run_ei_inference(zeros(i.height,i.width,2,'uint8')),'EI:InvalidImage');
verifyError(test,@()run_ei_inference(-ones(i.height,i.width)),'EI:InvalidImage');
end
function test_resizing(test)
i=test.TestData.info;rgb=zeros(19,33,3,'uint8');rgb(:,:,1)=11;rgb(:,:,2)=71;rgb(:,:,3)=191;
x=prepare_image(rgb,i);
if i.channels==1
    verifySize(test,x,[i.height i.width]);verifyEqual(test,x(1,1),uint8(67));
else
    verifySize(test,x,[i.height i.width 3]);verifyEqual(test,x(1,1,:),reshape(uint8([11 71 191]),1,1,3));
end
end
function test_actual_inference(test)
i=test.TestData.info;reset_ei_inference();image=zeros(i.height,i.width,i.channels,'uint8');
r1=run_ei_inference(image);r2=run_ei_inference(image);
verifyGreaterThanOrEqual(test,r1.status,0);verifyGreaterThanOrEqual(test,r2.status,0);
verifyEqual(test,r1.resultID,uint64(1));verifyEqual(test,r2.resultID,uint64(2));
verifyEqual(test,r1.detected,r2.detected);verifyEqual(test,r1.confidence,r2.confidence);
verifyFalse(test,r1.objectClass);verifyFalse(test,r2.objectClass);
verifyGreaterThan(test,r1.elapsedMs,0);verifyEqual(test,get_ei_result(),r2);
state=ei_session('get');bad=feval(state.artifact.gateway,'infer',zeros(1,1,i.channels,'uint8'));
verifyEqual(test,bad.status,-1);verifyFalse(test,bad.detected);verifyEqual(test,bad.confidence,0);
verifyEqual(test,get_ei_result(),bad);
reset_ei_inference();r=get_ei_result();verifyEqual(test,r.resultID,uint64(0));verifyFalse(test,r.detected);
end
function test_grayscale_rgb_equivalence(test)
i=test.TestData.info;gray=uint8(mod(reshape(0:i.height*i.width-1,i.height,i.width),256));
a=run_ei_inference(gray);b=run_ei_inference(repmat(gray,1,1,3));
verifyGreaterThanOrEqual(test,a.status,0);verifyGreaterThanOrEqual(test,b.status,0);
verifyEqual(test,a.status,b.status);verifyEqual(test,a.confidence,b.confidence);
verifyEqual(test,a.boundingBox,b.boundingBox);verifyEqual(test,a.detected,b.detected);
end
