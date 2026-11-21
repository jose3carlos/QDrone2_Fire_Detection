function [ImageOut, BoundingBox, Detected, Confidence, FrameIndex, ...
          FrameUpdated, BatchComplete] = VisualizeFOMOBatch(Images, ImgCaptured)
%#codegen
% Images: fixed-size uint8 [180 180 N], currently N=20.
% ImgCaptured: logical scalar; rising edge accepts a complete batch.
%
% ImageOut: exact original uint8 [180 180] grayscale frame.
% BoundingBox: uint32 [x y width height], native ZERO-BASED coordinates.
% Detected: logical detection flag for the current image.
% Confidence: single [0,1], individual image confidence.
% FrameIndex: uint32 index 1..N, initially zero.
% FrameUpdated: one-execution pulse for each processed image.
% BatchComplete: latched true after N images, reset by the next batch.
%
% One inference and one figure update per execution.
% Figures are reused by image index across batches.
% Requires setup_fomo_visualization and existing model initialization.

coder.extrinsic('ei_visualization_step');

assert(~coder.target('Rtw'), ...
    'This viewer requires MATLAB normal simulation.');
assert(isa(Images,'uint8') && isreal(Images));
assert(ndims(Images)<=3 && ...
    size(Images,1)==180 && size(Images,2)==180);
assert(islogical(ImgCaptured) && isscalar(ImgCaptured));

N = size(Images,3);
assert(N>=1 && N<=double(intmax('uint32')));

persistent bank busy previousReady nextIndex batchID lastID
persistent heldImage heldBox heldDetected heldConfidence heldIndex complete

if isempty(previousReady)
    bank = zeros(180,180,N,'uint8');
    busy = false;
    previousReady = false;
    nextIndex = 1;
    batchID = uint64(0);
    lastID = uint64(0);

    heldImage = zeros(180,180,'uint8');
    heldBox = zeros(1,4,'uint32');
    heldDetected = false;
    heldConfidence = single(0);
    heldIndex = uint32(0);
    complete = false;
end

FrameUpdated = false;

newBatch = ImgCaptured && ~previousReady;
previousReady = ImgCaptured;

assert(~(newBatch && busy), ...
    'Visualization is still processing the preceding batch.');

if newBatch
    assert(batchID<intmax('uint64'), ...
        'Restart simulation: batch counter exhausted.');

    % Snapshot protects images from subsequent capture updates.
    bank = Images;
    nextIndex = 1;
    batchID = batchID + uint64(1);
    complete = false;
    busy = true;

    heldImage(:) = uint8(0);
    heldBox(:) = uint32(0);
    heldDetected = false;
    heldConfidence = single(0);
    heldIndex = uint32(0);
end

if busy
    frame = bank(:,:,nextIndex);

    % Explicit types/sizes are required for extrinsic outputs.
    box = zeros(1,4,'uint32');
    found = false;
    score = single(0);
    nativeID = uint64(0);

    [box, found, score, nativeID] = ei_visualization_step( ...
        frame, uint32(nextIndex), uint32(N), batchID);

    assert(nativeID>lastID && ...
        isfinite(score) && score>=0 && score<=1, ...
        'Invalid or stale visualization inference result.');

    lastID = nativeID;

    % The rectangle is a figure overlay; these pixels stay unchanged.
    heldImage = frame;
    heldBox = box;
    heldDetected = found;
    heldConfidence = score;
    heldIndex = uint32(nextIndex);
    FrameUpdated = true;

    if nextIndex==N
        complete = true;
        busy = false;
    else
        nextIndex = nextIndex + 1;
    end
end

ImageOut = heldImage;
BoundingBox = heldBox;
Detected = heldDetected;
Confidence = heldConfidence;
FrameIndex = heldIndex;
BatchComplete = complete;
end