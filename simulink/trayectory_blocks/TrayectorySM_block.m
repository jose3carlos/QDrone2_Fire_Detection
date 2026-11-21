
function [pointDesired, startCamera, homeToggle] = ...
    fcn(startBool, currentPosition, nextPoint,waypoints)
%#codegen
%
% SEQUENTIAL WAYPOINT STATE MACHINE
%
% Inputs:
%   startBool       : Boolean mission start signal
%   currentPosition : [X; Y; Z; Yaw] measured position (4x1)
%   nextPoint       : Boolean camera completion acknowledgment
%
% Outputs:
%   pointDesired    : [X; Y; 1; 0] desired reference (4x1)
%   startCamera     : Boolean camera activation
%   homeToggle      : Boolean mission-active output
%
% Operating principle:
%   1. Wait for startBool.
%   2. Command a waypoint.
%   3. Verify position stability for 3 seconds.
%   4. Activate the camera.
%   5. Wait for a fresh camera acknowledgment.
%   6. Advance or complete the mission.
%
% IMPORTANT:
%   Execute this block at a discrete sample time of 0.05 s.

%% =========================================================
%  1. USER CONFIGURATION
% ==========================================================

% Fixed sample time and verification duration
Ts = 0.05;                 % Seconds
settlingTime = 3.0;        % Seconds
windowSize = 60;           % settlingTime / Ts

% Workspace boundaries (meters)
xMin = -1.5;
xMax =  1.5;
yMin = -1.5;
yMax =  1.5;

% Constant altitude and yaw references
zDesired = 1.0;
yawDesired = 0.0;          % Radians

% Mean-position tolerances
tolMean = [ ...
    0.10;                  % X error (m)
    0.10;                  % Y error (m)
    0.10;                  % Z error (m)
    5.0*pi/180 ];          % Yaw error (rad)

% Instantaneous stabilization tolerances
% Every sample in the verification window must
% remain within these limits.
tolInstant = [ ...
    0.15;                  % X error (m)
    0.15;                  % Y error (m)
    0.15;                  % Z error (m)
    8.0*pi/180 ];          % Yaw error (rad)

%% =========================================================
%  2. HARD-CODED WAYPOINTS
% ==========================================================

% Each row is [X, Y, Z, Yaw].
%
% EDIT THESE POINTS TO DEFINE THE FINAL MISSION.
% All X,Y coordinates must be within [-1.5, 1.5].
%
% These coordinates are examples.

% waypoints = [ ...
%    -1.5, -1.5, 1.0, 0.0;
%    -0.5, -1.5, 1.0, 0.0;
%     0.5, -1.5, 1.0, 0.0;
%     1.5, -1.5, 1.0, 0.0;
%     1.5,  0.0, 1.0, 0.0;
%     1.5,  1.5, 1.0, 0.0
% ];

numberOfWaypoints = size(waypoints, 1);

%% =========================================================
%  3. STATE DEFINITIONS
% ==========================================================

STATE_IDLE     = uint8(0);
STATE_MOVE     = uint8(1);
STATE_VERIFY   = uint8(2);
STATE_CAMERA   = uint8(3);
STATE_ADVANCE  = uint8(4);
STATE_COMPLETE = uint8(5);

%% =========================================================
%  4. PERSISTENT STATE VARIABLES
% ==========================================================

persistent state
persistent waypointIndex

% Position verification memory
persistent errorHistory
persistent errorSum
persistent sampleCount
persistent writeIndex

% Camera handshake memory
persistent cameraArmed

if isempty(state)

    state = STATE_IDLE;
    waypointIndex = 1;

    % Fixed-size circular buffer
    errorHistory = zeros(4, windowSize);
    errorSum = zeros(4,1);

    sampleCount = 0;
    writeIndex = 1;

    cameraArmed = false;

end

%% =========================================================
%  5. DEFAULT OUTPUTS
% ==========================================================

startCamera = false;
homeToggle = true;

% Validate position feedback before using it.
validFeedback = all(isfinite(currentPosition));

% Default command before mission start.
% Hold measured XY, with fixed Z and yaw.
if validFeedback
    pointDesired = [ ...
        currentPosition(1);
        currentPosition(2);
        zDesired;
        yawDesired ];
else
    pointDesired = [0.0; 0.0; zDesired; yawDesired];
end

% Restrict initial references to the workspace.
pointDesired(1) = min(max(pointDesired(1), xMin), xMax);
pointDesired(2) = min(max(pointDesired(2), yMin), yMax);

% Once started, always command the stored waypoint.
if state ~= STATE_IDLE

    pointDesired = waypoints(waypointIndex, :).';

    % Enforce workspace restrictions.
    pointDesired(1) = min(max(pointDesired(1), xMin), xMax);
    pointDesired(2) = min(max(pointDesired(2), yMin), yMax);

    % Enforce constant altitude and yaw.
    pointDesired(3) = zDesired;
    pointDesired(4) = yawDesired;

end

%% =========================================================
%  6. POSITION ERROR CALCULATION
% ==========================================================

positionError = zeros(4,1);

if validFeedback

    positionError(1) = ...
        currentPosition(1) - pointDesired(1);

    positionError(2) = ...
        currentPosition(2) - pointDesired(2);

    positionError(3) = ...
        currentPosition(3) - pointDesired(3);

    % Wrapped yaw error in [-pi, pi].
    yawError = currentPosition(4) - pointDesired(4);

    positionError(4) = atan2( ...
        sin(yawError), cos(yawError));

end

% Instantaneous position convergence condition.
insideTolerance = false;

if validFeedback

    insideTolerance = ...
        all(abs(positionError) <= tolInstant);

end

%% =========================================================
%  7. FINITE-STATE MACHINE
% ==========================================================

switch state

    %% -----------------------------------------------------
    %  STATE 0: IDLE
    % ------------------------------------------------------

    case STATE_IDLE

        startCamera = false;
        homeToggle = true;

        if startBool

            waypointIndex = 1;

            % Reset verification memory.
            errorHistory(:) = 0.0;
            errorSum(:) = 0.0;

            sampleCount = 0;
            writeIndex = 1;

            % Reset camera handshake.
            cameraArmed = false;

            state = STATE_MOVE;

        end

    %% -----------------------------------------------------
    %  STATE 1: MOVE TO WAYPOINT
    % ------------------------------------------------------

    case STATE_MOVE

        startCamera = false;
        homeToggle = true;

        if insideTolerance

            % Enter stabilization verification.
            errorHistory(:) = 0.0;
            errorSum(:) = 0.0;

            sampleCount = 0;
            writeIndex = 1;

            state = STATE_VERIFY;

        end

    %% -----------------------------------------------------
    %  STATE 2: VERIFY POSITION STABILITY
    % ------------------------------------------------------

    case STATE_VERIFY

        startCamera = false;
        homeToggle = true;

        if ~insideTolerance

            % Device left stabilization region or
            % feedback became invalid.
            %
            % Discard all previous verification samples.

            errorHistory(:) = 0.0;
            errorSum(:) = 0.0;

            sampleCount = 0;
            writeIndex = 1;

            state = STATE_MOVE;

        else

            % Add current error to the rolling window.
            if sampleCount < windowSize

                % Window not full yet.
                errorHistory(:, writeIndex) = positionError;

                errorSum = errorSum + positionError;

                sampleCount = sampleCount + 1;

            else

                % Remove oldest sample, insert newest.
                oldestError = errorHistory(:, writeIndex);

                errorSum = errorSum - oldestError;

                errorHistory(:, writeIndex) = positionError;

                errorSum = errorSum + positionError;

            end

            % Advance circular buffer index.
            writeIndex = writeIndex + 1;

            if writeIndex > windowSize
                writeIndex = 1;
            end

            % Evaluate convergence after 3 full seconds.
            if sampleCount >= windowSize

                meanError = errorSum / double(windowSize);

                meanConverged = ...
                    all(abs(meanError) <= tolMean);

                if meanConverged

                    % The waypoint is stable.
                    % Begin a new camera capture cycle.

                    cameraArmed = false;

                    state = STATE_CAMERA;

                end

            end

        end

    %% -----------------------------------------------------
    %  STATE 3: CAMERA ACQUISITION
    % ------------------------------------------------------

    case STATE_CAMERA

        startCamera = true;
        homeToggle = true;

        % Handshake protocol:
        %
        % A low camera-completion signal must first
        % be observed during this capture state.
        %
        % Only a subsequent high signal is accepted.
        %
        % This prevents a previous camera acknowledgment
        % from being reused for a new waypoint.

        if ~nextPoint

            % Camera completion is low.
            % Arm acceptance of a fresh acknowledgment.
            cameraArmed = true;

        elseif cameraArmed

            % Camera has completed the current capture.
            % Accept exactly one acknowledgment.

            cameraArmed = false;

            startCamera = false;

            state = STATE_ADVANCE;

        end

    %% -----------------------------------------------------
    %  STATE 4: ADVANCE WAYPOINT
    % ------------------------------------------------------

    case STATE_ADVANCE

        % Handled by the post-switch advancement logic.

    %% -----------------------------------------------------
    %  STATE 5: MISSION COMPLETE
    % ------------------------------------------------------

    case STATE_COMPLETE

        startCamera = false;
        homeToggle = false;

    otherwise

        % Fail-safe internal state recovery.
        % Do not automatically restart the mission.

        state = STATE_COMPLETE;

end

%% =========================================================
%  8. WAYPOINT ADVANCEMENT / TERMINATION
% ==========================================================

% This section also executes immediately after a camera
% acknowledgment sets STATE_ADVANCE, so no extra
% simulation step is required for mission completion.

if state == STATE_ADVANCE

    startCamera = false;

    % Clear verification memory before the next waypoint.
    errorHistory(:) = 0.0;
    errorSum(:) = 0.0;

    sampleCount = 0;
    writeIndex = 1;

    cameraArmed = false;

    if waypointIndex < numberOfWaypoints

        % Advance exactly one waypoint.
        waypointIndex = waypointIndex + 1;

        state = STATE_MOVE;

    else

        % Final waypoint was photographed.
        % Mission is now permanently complete.

        state = STATE_COMPLETE;

    end

end

%% =========================================================
%  9. FINAL OUTPUT ASSIGNMENT
% ==========================================================

if state ~= STATE_IDLE

    % Command the active waypoint, including updates
    % after state advancement.
    pointDesired = waypoints(waypointIndex, :).';

    % Hard workspace constraints.
    pointDesired(1) = min(max(pointDesired(1), xMin), xMax);
    pointDesired(2) = min(max(pointDesired(2), yMin), yMax);

    pointDesired(3) = zDesired;
    pointDesired(4) = yawDesired;

end

% Outputs are defined by the final active state.
startCamera = (state == STATE_CAMERA);

if state == STATE_COMPLETE

    homeToggle = false;

else

    homeToggle = true;

end

end
