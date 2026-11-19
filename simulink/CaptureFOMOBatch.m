function [Images, ImgCaptured, Status] = CaptureFOMOBatch(rgb, Trigger, newFrame, frameNumber, t)
%#codegen
% INPUTS: uint8 [480 640 3], logical, logical, uint64, double seconds.
% OUTPUTS: uint8 [180 180 N], logical, int32.
% Status: 0 idle, 1 capturing, 2 complete, -1 invalid clock, -2 missed slot.
% Run every CAMERA sample, not only when the manual switch changes.

% EDIT BETWEEN SIMULATION RUNS. N changes the fixed output dimensions.
N = 20;
CaptureSeconds = 4.0;
assert(N >= 1 && N == floor(N));
assert(isfinite(CaptureSeconds) && CaptureSeconds > 0);
assert(isa(rgb,'uint8') && isequal(size(rgb),[480 640 3]));
assert(islogical(Trigger) && isscalar(Trigger));
assert(islogical(newFrame) && isscalar(newFrame));
assert(isa(frameNumber,'uint64') && isscalar(frameNumber));
assert(isa(t,'double') && isscalar(t));

persistent bank ready active previous count startTime lastID haveID lastTime code
if isempty(previous)
    bank = zeros(180,180,N,'uint8');
    ready = false; active = false; previous = false;
    count = 0; startTime = 0; lastID = uint64(0); haveID = false;
    lastTime = 0; code = int32(0);
end

rising = Trigger && ~previous;
previous = Trigger;

% A bad/backward clock invalidates the attempt; never present partial data ready.
if ~isfinite(t) || t < lastTime
    active = false; ready = false; code = int32(-1);
    Images = bank; ImgCaptured = ready; Status = code;
    return
end
lastTime = t;

% Ignore trigger edges while busy. A held-high switch never restarts a burst.
if rising && ~active
    bank(:) = uint8(0);
    count = 0; ready = false; active = true;
    startTime = t; lastID = uint64(0); haveID = false;
    code = int32(1);
end

if active
    if N == 1
        interval = CaptureSeconds;
    else
        interval = CaptureSeconds / (N - 1);
    end
    % Nominal times: startTime + (0:N-1)*CaptureSeconds/(N-1).
    % Camera sampling quantizes these times. Allow <=50 ms lateness,
    % but never more than half an interval. Never fabricate catch-up frames.
    maxLate = min(0.05, interval/2);
    due = startTime + count*interval;
    if t > due + maxLate + 1e-9
        active = false; ready = false; code = int32(-2);
    elseif t + 1e-9 >= due && newFrame && (~haveID || frameNumber > lastID)
        k = count + 1;
        % Match prepare_image: squash, nearest-neighbor pixel centers,
        % then ITU-R 601 luma. No toolbox, MEX, file I/O or inference here.
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
        count = k; lastID = frameNumber; haveID = true;
        if count == N
            active = false; ready = true; code = int32(2);
        end
    end
end

Images = bank;
ImgCaptured = ready;
Status = code;
end
