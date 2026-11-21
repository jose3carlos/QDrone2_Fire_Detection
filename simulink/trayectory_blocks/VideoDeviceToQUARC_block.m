function [RGB, newFrame, framNum] = VideoDeviceToQUARC(image)
%#codegen
%
% INPUT:
%   image: fixed-size uint8 [H W 3].
%   H and W are inherited from the connected acquisition block.
%
% OUTPUTS:
%   RGB:      uint8 [480 640 3]
%   newFrame: logical scalar
%   framNum:  uint64 scalar
%
% Execute at the acquisition rate, 1/30 second.
% Enable Read All Frames on From Video Device.
%
% newFrame assumes one newly delivered acquisition per execution.
% framNum is a software counter, not a hardware frame identifier.

assert(isa(image,'uint8') && isreal(image), ...
    'Input must be a real uint8 RGB image.');

assert(ndims(image)==3 && size(image,3)==3, ...
    'Connect one H-by-W-by-3 RGB signal, not separate color planes.');

assert(size(image,1)>0 && size(image,2)>0, ...
    'Input image must not be empty.');

sourceHeight = double(size(image,1));
sourceWidth  = double(size(image,2));

% false: resize the entire image, allowing aspect-ratio distortion.
% true: center-crop to 4:3 before resizing, preserving proportions.
CENTER_CROP = false;

cropHeight = sourceHeight;
cropWidth  = sourceWidth;
topOffset  = 0;
leftOffset = 0;

if CENTER_CROP
    if sourceWidth*480 > sourceHeight*640
        % Input is wider than the required 4:3 output.
        cropWidth = min(sourceWidth, ...
            max(1,floor(sourceHeight*640/480)));

        leftOffset = floor((sourceWidth-cropWidth)/2);
    else
        % Input is taller than the required 4:3 output.
        cropHeight = min(sourceHeight, ...
            max(1,floor(sourceWidth*480/640)));

        topOffset = floor((sourceHeight-cropHeight)/2);
    end
end

% Fixed-length indices ensure a fixed 480x640 output.
rows = topOffset ...
     + floor(((0:479)+0.5)*(cropHeight/480)) + 1;

cols = leftOffset ...
     + floor(((0:639)+0.5)*(cropWidth/640)) + 1;

% Explicitly protect array boundaries.
rows = min(sourceHeight,max(1,rows));
cols = min(sourceWidth,max(1,cols));

% Preserve RGB channels and uint8 data type.
RGB = image(rows,cols,:);

persistent counter

if isempty(counter)
    counter = uint64(0);
end

assert(counter<intmax('uint64'), ...
    'Frame counter exhausted; restart simulation.');

counter = counter+uint64(1);

framNum = counter;
newFrame = true;
end