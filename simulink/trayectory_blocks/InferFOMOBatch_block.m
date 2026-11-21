function [ObjectDetected, ConfidenceAverage] = InferFOMOBatch(Images, ImgCaptured)
%#codegen
%
% INPUTS:
% Images       : Fixed-size uint8 [180 180 N], from CaptureFOMOBatch.
% ImgCaptured  : Logical scalar, from CaptureFOMOBatch.
%
% OUTPUTS:
% ObjectDetected    : Logical; true when MORE THAN HALF the images detect Fire.
% ConfidenceAverage : Double [0,1], rounded to two decimal places.
%
% AVERAGING POLICY:
% Each image uses the project's existing confidence threshold and
% highest-confidence detection selection. No detection contributes zero.
% ConfidenceAverage averages over ALL N images, including those zeros.
% It represents mean detection confidence, not classification accuracy.
%
% EXECUTION:
% A rising ImgCaptured edge accepts one complete batch.
% The batch is copied once to protect it from subsequent input changes.
% One image is processed per block execution while simulation is running.
% After N executions, both outputs update and remain held.
% A continuously high ImgCaptured signal never repeats the batch.
% Outputs are false/0 initially and while processing a newly accepted batch.
%
% REQUIRED MODEL SETUP -- MATLAB Command Window, before simulation:
%   setup_project(false);
%   build_model();
%   configure_simulink_model('YourModelName');
%
% The helper changes solver/sample-time settings. Restore the settings
% required by your QUARC model afterwards. Execute this block at a
% discrete sample time, for example the camera's sample time.
%
% Model custom initialization/termination loads/releases the native engine
% once per simulation. Initialization is NOT repeated for each image.
% Each native inference is synchronous and blocks its current execution;
% this is not an asynchronous worker or a hard real-time guarantee.
%
% The existing DLL supports Windows x64. Other QUARC targets require
% a compatible native backend and target-specific linking.
%
% Keep runtime assertions enabled. Native errors or overlapping batches
% stop simulation instead of being silently interpreted as "no detection".
% Use one instance with exclusive access to the inference engine.
%
% Compiled and tested in MATLAB R2023b/Simulink using the existing bridge,
% synthetic detection batches, and the actual Fire model.
% Physical QUARC target execution has not been verified.

assert(isa(Images,'uint8') && isreal(Images));
assert(ndims(Images)<=3 && size(Images,1)==180 && size(Images,2)==180);
assert(islogical(ImgCaptured) && isscalar(ImgCaptured));

N = size(Images,3);
assert(N >= 1);

coder.cinclude('ei_wrapper.h');

persistent batch processing previousReady nextImage positiveCount
persistent sumPercent heldDecision heldAverage lastResultID

if isempty(previousReady)
    batch = zeros(180,180,N,'uint8');
    processing = false;
    previousReady = false;
    nextImage = 1;
    positiveCount = 0;
    sumPercent = 0.0;
    heldDecision = false;
    heldAverage = 0.0;
    lastResultID = uint64(0);
end

% Accept each completed capture exactly once.
newBatch = ImgCaptured && ~previousReady;
previousReady = ImgCaptured;

overlap = newBatch && processing;

if overlap
    processing = false;
    heldDecision = false;
    heldAverage = 0.0;
end

assert(~overlap, ...
    'Previous batch is still processing. Wait before submitting another completed batch.');

if newBatch && ~overlap
    batch = Images;

    nextImage = 1;
    positiveCount = 0;
    sumPercent = 0.0;

    heldDecision = false;
    heldAverage = 0.0;
    processing = true;
end

% Process one snapshot image per simulation execution.
if processing
    image = batch(:,:,nextImage);

    percent = single(0);
    objectClass = uint8(0);
    detected = uint8(0);
    resultID = uint64(0);
    nativeStatus = int32(0);

    nativeStatus = coder.ceval( ...
        'ei_sim_infer', ...
        coder.rref(image), ...
        uint64(numel(image)), ...
        uint32(size(image,1)), ...
        uint32(size(image,2)), ...
        coder.wref(percent), ...
        coder.wref(objectClass), ...
        coder.wref(detected), ...
        coder.wref(resultID));

    % Status 1 = valid detection; status 2 = successful no-detection.
    % Increasing result IDs also prevent accepting stale native results.
    valid = isfinite(percent) && percent>=0 && percent<=100 ...
        && resultID>lastResultID ...
        && ((nativeStatus==int32(1) && detected==uint8(1)) ...
        ||  (nativeStatus==int32(2) && detected==uint8(0)));

    if ~valid
        processing = false;
        heldDecision = false;
        heldAverage = 0.0;
    end

    assert(valid, ...
        'Native FOMO inference failed. Check DLL initialization, ownership, dimensions and native status.');

    if valid
        lastResultID = resultID;

        if detected~=0
            positiveCount = positiveCount + 1;
            sumPercent = sumPercent + double(percent);
        end

        if nextImage == N
            % Strict majority; an exact tie returns false.
            heldDecision = positiveCount > double(N)/2.0;

            % Convert percent to [0,1], averaging across all N images.
            meanConfidence = sumPercent/(100.0*double(N));
            meanConfidence = min(1.0,max(0.0,meanConfidence));

            heldAverage = round(100.0*meanConfidence)/100.0;
            processing = false;
        else
            nextImage = nextImage + 1;
        end
    end
end

ObjectDetected = heldDecision;
ConfidenceAverage = heldAverage;
end