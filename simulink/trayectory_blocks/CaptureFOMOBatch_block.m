function [Images, ImgCaptured, Status] = CaptureFOMOBatch( ...
    rgb, Trigger, newFrame, frameNumber, t)
%#codegen
%
% Images:      uint8 [180 180 N]
% ImgCaptured: true only when the complete batch is available
%
% Status:
%   0 = idle
%   1 = capturing
%   2 = complete
%  -1 = invalid/backward clock
%  -2 = missing/late camera frame

% EDIT THESE BETWEEN SIMULATION RUNS:
N = 25;
CaptureSeconds = 3.0;

assert(N >= 1 && N == floor(N));
assert(isfinite(CaptureSeconds) && CaptureSeconds > 0);
assert(isa(rgb,'uint8') && isequal(size(rgb),[480 640 3]));
assert(islogical(Trigger) && isscalar(Trigger));
assert(islogical(newFrame) && isscalar(newFrame));
assert(isa(frameNumber,'uint64') && isscalar(frameNumber));
assert(isa(t,'double') && isscalar(t));

persistent bank ready active previous count startTime
persistent lastID haveID lastTime code

if isempty(previous)
    bank = zeros(180,180,N,'uint8');
    ready = false;
    active = false;
    previous = false;
    count = 0;
    startTime = 0;
    lastID = uint64(0);
    haveID = false;
    lastTime = 0;
    code = int32(0);
end

rising = Trigger && ~previous;
previous = Trigger;

% Invalid timing must never produce a completed batch.
if ~isfinite(t) || t < lastTime
    active = false;
    ready = false;
    code = int32(-1);

    Images = bank;
    ImgCaptured = ready;
    Status = code;
    return
end
lastTime = t;

% Accept a new request only when not already capturing.
if rising && ~active
    bank(:) = uint8(0);
    count = 0;
    ready = false;
    active = true;
    startTime = t;
    lastID = uint64(0);
    haveID = false;
    code = int32(1);
end

if active
    if N == 1
        interval = CaptureSeconds;
    else
        interval = CaptureSeconds / (N - 1);
    end

    % First target is at the trigger; last target is CaptureSeconds later.
    due = startTime + count*interval;

    % Permit camera timing quantization, but reject missed slots.
    maxLate = min(0.05, interval/2);

    if t > due + maxLate + 1e-9
        active = false;
        ready = false;
        code = int32(-2);

    elseif t + 1e-9 >= due && newFrame && ...
            (~haveID || frameNumber > lastID)

        k = count + 1;

        % Match the current project's squash + nearest-neighbor resize,
        % followed by grayscale conversion.
        for y = 1:180
            sy = floor((y - 0.5)*480/180) + 1;

            for x = 1:180
                sx = floor((x - 0.5)*640/180) + 1;

                value = 0.299*double(rgb(sy,sx,1)) ...
                      + 0.587*double(rgb(sy,sx,2)) ...
                      + 0.114*double(rgb(sy,sx,3));

                bank(y,x,k) = uint8(round(value));
            end
        end

        count = k;
        lastID = frameNumber;
        haveID = true;

        if count == N
            active = false;
            ready = true;
            code = int32(2);
        end
    end
end

Images = bank;
ImgCaptured = ready;
Status = code;
end