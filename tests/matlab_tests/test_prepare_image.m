function tests=test_prepare_image
tests=functiontests(localfunctions);
end
function info=metadata(channels)
info=struct('height',2,'width',3,'channels',channels,'resizeMode','EI_CLASSIFIER_RESIZE_FIT_SHORTEST');
end
function test_channels_and_luma(test)
gray=uint8([0 64 255;1 128 200]);
verifyEqual(test,prepare_image(gray,metadata(1)),gray);
verifyEqual(test,prepare_image(gray,metadata(3)),repmat(gray,1,1,3));
rgb=zeros(2,3,3,'uint8');rgb(:,1,1)=255;rgb(:,2,2)=255;rgb(:,3,3)=255;
verifyEqual(test,prepare_image(rgb,metadata(1)),repmat(uint8([76 150 29]),2,1));
end
function test_numeric_types(test)
i=metadata(1);
verifyEqual(test,prepare_image(repmat(uint16([0 32768 65535]),2,1),i),repmat(uint8([0 128 255]),2,1));
verifyEqual(test,prepare_image(repmat([0 .5 1],2,1),i),repmat(uint8([0 128 255]),2,1));
verifyEqual(test,prepare_image(true(2,3),i),repmat(uint8(255),2,3));
verifyEqual(test,prepare_image(single(zeros(2,3)),i),zeros(2,3,'uint8'));
end
function test_transparency(test)
rgba=ones(2,3,4,'uint8')*200;rgba(:,:,4)=255;rgba(:,1,4)=0;
expected=ones(2,3,'uint8')*200;expected(:,1)=0;
verifyEqual(test,prepare_image(rgba,metadata(1)),expected);
end
function test_invalid_values_and_shapes(test)
i=metadata(1);
bad={nan(2,3),inf(2,3),-ones(2,3),ones(2,3)*255,zeros(2,3,2,'uint8'), ...
    zeros(2,3,3,2,'uint8'),zeros(0,3,'uint8'),int16(zeros(2,3)),complex(ones(2,3),ones(2,3))};
for k=1:numel(bad),verifyError(test,@()prepare_image(bad{k},i),'EI:InvalidImage');end
end
function test_file_decoding(test)
folder=tempname;mkdir(folder);cleaner=onCleanup(@()rmdir(folder,'s')); %#ok<NASGU>
i=metadata(1);gray=repmat(uint8([0 128 255]),2,1);
file=fullfile(folder,'gray16.png');imwrite(uint16(gray)*257,file);
verifyEqual(test,prepare_image(file,i),gray);
file=fullfile(folder,'indexed.png');map=[1 0 0;0 1 0];indices=uint8([0 1 0;1 0 1]);imwrite(indices,map,file);
verifyEqual(test,prepare_image(file,i),uint8([76 150 76;150 76 150]));
file=fullfile(folder,'alpha.png');alpha=ones(2,3,'uint8')*255;alpha(:,1)=0;
imwrite(ones(2,3,'uint8')*200,file,'Alpha',alpha);
expected=ones(2,3,'uint8')*200;expected(:,1)=0;
verifyEqual(test,prepare_image(file,i),expected);
file=fullfile(folder,'color.jpg');imwrite(repmat(gray,1,1,3),file,'Quality',95);
verifyEqual(test,prepare_image(file,i),prepare_image(imread(file),i));
end
function test_resize_geometry(test)
for channels=[1 3]
    i=metadata(channels);input=ones(4,8,channels,'uint8')*100;
    for mode={'EI_CLASSIFIER_RESIZE_FIT_SHORTEST','EI_CLASSIFIER_RESIZE_FIT_LONGEST','EI_CLASSIFIER_RESIZE_SQUASH'}
        i.resizeMode=mode{1};output=prepare_image(input,i);
        verifyEqual(test,size(output,1),2);verifyEqual(test,size(output,2),3);
        verifyEqual(test,size(output,3),channels);verifyClass(test,output,'uint8');
    end
    i.resizeMode='EI_CLASSIFIER_RESIZE_NONE';
    verifyError(test,@()prepare_image(input,i),'EI:Resize');
end
end
