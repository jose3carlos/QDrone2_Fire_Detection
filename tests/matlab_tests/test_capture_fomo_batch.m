function tests=test_capture_fomo_batch
tests=functiontests(localfunctions);
end
function setup(~)
setup_project(false);clear CaptureFOMOBatch
end
function test_default_schedule_and_hold(test)
frame=zeros(480,640,3,'uint8');Ts=1/30;
for j=0:150
    frame(:)=uint8(j+1);
    [images,ready,status]=CaptureFOMOBatch(frame,true,true,uint64(j),j*Ts);
    if j<120,verifyFalse(test,ready);else,verifyTrue(test,ready);end
end
verifySize(test,images,[180 180 20]);verifyClass(test,images,'uint8');
verifyEqual(test,status,int32(2));
for k=1:20
    frameIndex=ceil(((k-1)*4/19-1e-9)/Ts);
    verifyEqual(test,images(:,:,k),repmat(uint8(frameIndex+1),180,180));
end
held=images;
[images,ready,status]=CaptureFOMOBatch(frame,true,true,uint64(151),151*Ts);
verifyEqual(test,images,held);verifyTrue(test,ready);verifyEqual(test,status,int32(2));
CaptureFOMOBatch(frame,false,true,uint64(152),152*Ts);
[images,ready,status]=CaptureFOMOBatch(frame,true,true,uint64(153),153*Ts);
verifyFalse(test,ready);verifyEqual(test,status,int32(1));
verifyEqual(test,nnz(images(:,:,2:end)),0);
end
function test_ignores_busy_edges_and_matches_preprocessing(test)
% An asymmetric spatial pattern catches resizing, channel and indexing errors.
frame=zeros(480,640,3,'uint8');frame(:,:,1)=uint8(mod((1:480)'+(1:640),256));
frame(:,:,2)=71;frame(:,:,3)=191;
info=struct('height',180,'width',180,'channels',1,'resizeMode','EI_CLASSIFIER_RESIZE_SQUASH');
expected=prepare_image(frame,info);
for j=0:120
    trigger=~(j==3); % second rising edge at j=4 must not restart the batch
    [images,ready,status]=CaptureFOMOBatch(frame,trigger,true,uint64(j),j/30);
end
verifyTrue(test,ready);verifyEqual(test,status,int32(2));
for k=1:20,verifyEqual(test,images(:,:,k),expected);end
end
function test_no_frame_and_duplicate_id(test)
frame=ones(480,640,3,'uint8')*100;
[images,ready,status]=CaptureFOMOBatch(frame,false,true,uint64(1),0);
verifyFalse(test,ready);verifyEqual(test,status,int32(0));verifyEqual(test,nnz(images),0);
CaptureFOMOBatch(frame,true,true,uint64(2),.1);
for j=1:12
    [images,ready,status]=CaptureFOMOBatch(frame,true,true,uint64(2),.1+j/30);
end
verifyFalse(test,ready);verifyEqual(test,status,int32(-2));
verifyEqual(test,nnz(images(:,:,2:end)),0);
clear CaptureFOMOBatch
CaptureFOMOBatch(frame,true,false,uint64(1),0);
[~,ready,status]=CaptureFOMOBatch(frame,true,false,uint64(1),.1);
verifyFalse(test,ready);verifyEqual(test,status,int32(-2));
end
function test_invalid_clock(test)
frame=zeros(480,640,3,'uint8');
CaptureFOMOBatch(frame,true,true,uint64(1),1);
[~,ready,status]=CaptureFOMOBatch(frame,true,true,uint64(2),.9);
verifyFalse(test,ready);verifyEqual(test,status,int32(-1));
end
