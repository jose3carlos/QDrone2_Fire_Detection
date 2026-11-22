%% Real-time drone waypoint simulation with webcam trigger
% Flight area: 3 m x 3 m, (0,0) at center. +x forward, +y left, +z up.
% Yaw held at 0 deg. The simulation runs in wall-clock time (1 s sim = 1 s real).
% When camera_trigger fires, a snapshot is taken from an external webcam.
%
% Requires (only if use_webcam = true):
%   MATLAB Support Package for USB Webcams  (Add-On Explorer -> "USB Webcams")
clear; clc; close all;

%% ---------------- Parameters (edit here) ----------------
% Waypoints: [x y z yaw_deg], one row per waypoint
waypoints = [-1.5 -1.5  1.5  0;
              0.0 -1.5  1.5  0;
              1.5 -1.5  1.5  0;
             -1.5  1.5  1.5  0
              0.0  1.5  1.5  0;
              1.5  1.5  1.5  0;
              1.5 0.75  1.5  0;
              0.0  0.0  1.5  0];


start_pos = [0 0 0];     % initial position [x y z] (m)
tol_xyz   = 0.1;         % m, tolerance in x, y and z
tol_yaw   = 5;           % deg, yaw tolerance (assumed)
dwell_req = 1.5;         % s inside tolerance before photo window starts
photo_dur = 5.0;         % s photo window
trigger_delay = 0.0;     % s after camera_enable rises before the trigger fires
                         % (use this to leave extra time to angle the webcam)
reset_photo_on_drift = true;

start_delay = 5;         % s of countdown before the mission starts
                         % (time to aim the webcam at the first drawing)

% Webcam
use_webcam = false;      % set true when the webcam is connected
cam_index  = 1;          % index in webcamlist
save_dir   = 'photos';   % images saved here as wp_01.png, wp_02.png, ...

% Timing
dt_nom = 0.02;           % s, nominal loop period (50 Hz)
tmax   = 180;            % s, safety limit

% Simple 2nd-order response (slight overshoot)
wn = 0.5;  zeta = 0.9;

%% ---------------- Setup ----------------
N = size(waypoints,1);
if use_webcam
    if ~exist(save_dir,'dir'), mkdir(save_dir); end
    cam = webcam(cam_index);
    snapshot(cam);                       % warm-up frame
    fprintf('Webcam ready: %s\n', cam.Name);
end

pos = start_pos(:)';  vel = [0 0 0];
yaw = 0;  yaw_rate = 0;

state = 1;               % 1 = travel/dwell, 2 = photo window
idx = 1;  dwell_t = 0;  photo_t = 0;
camera_en = false;  camera_trigger = false;  trigger_done = false;
photos_done = zeros(N,1);

% Log (preallocated; columns below)
nmax = ceil(tmax/dt_nom*1.5);
L = zeros(nmax,12);   % t x y z yaw in_tol dwell_t camera_en photo_t trigger wp_idx dt
k = 0;

%% ---------------- Live figure ----------------
fig = figure('Name','Real-time drone sim','Position',[100 100 700 700]);
ax = axes(fig); hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');
axis(ax,[-1.6 1.6 -1.6 1.6]);
xlabel(ax,'x forward (m)'); ylabel(ax,'y left (m)');
rectangle(ax,'Position',[-1.5 -1.5 3 3],'EdgeColor','k');
plot(ax,waypoints(:,1),waypoints(:,2),'rs','MarkerFaceColor','r');
for i = 1:N, text(ax,waypoints(i,1)+0.05,waypoints(i,2)+0.05,num2str(i)); end
h_path  = plot(ax,pos(1),pos(2),'b-');
h_drone = plot(ax,pos(1),pos(2),'bo','MarkerSize',10,'MarkerFaceColor','b');
h_txt   = title(ax,'');

%% ---------------- Countdown ----------------
fprintf('Aim the webcam at the drawing for WAYPOINT 1. Starting in %d s...\n', start_delay);
for c = start_delay:-1:1
    set(h_txt,'String',sprintf('Aim webcam at waypoint 1 - starting in %d s',c));
    drawnow; pause(1);
end
fprintf('>>> Heading to waypoint %d\n', idx);

%% ---------------- Real-time loop ----------------
t_wall = tic;  t_prev = 0;
while idx <= N && ishandle(fig)
    pause(dt_nom);                                   % pace the loop
    t_now = toc(t_wall);
    dt = min(t_now - t_prev, 0.25);                  % real elapsed time (clamped)
    t_prev = t_now;
    if t_now > tmax, fprintf('Time limit reached.\n'); break; end
    k = k + 1;
    wp = waypoints(idx,:);

    % --- Dynamics (sub-stepped so a slow frame stays numerically stable) ---
    nsub = max(1, ceil(dt/0.01));  h = dt/nsub;
    for s = 1:nsub
        acc = wn^2*(wp(1:3)-pos) - 2*zeta*wn*vel;
        vel = vel + acc*h;  pos = pos + vel*h;
        yacc = wn^2*(wp(4)-yaw) - 2*zeta*wn*yaw_rate;
        yaw_rate = yaw_rate + yacc*h;  yaw = yaw + yaw_rate*h;
    end

    % --- Tolerance check ---
    in_tol = all(abs(pos - wp(1:3)) <= tol_xyz) && abs(yaw - wp(4)) <= tol_yaw;

    % --- Flag / timer logic ---
    camera_trigger = false;                          % one-sample pulse
    next_wp = false;
    if state == 1
        if in_tol, dwell_t = dwell_t + dt; else, dwell_t = 0; end
        if dwell_t >= dwell_req
            state = 2;  photo_t = 0;  camera_en = true;  trigger_done = false;
            fprintf('t=%6.2f s | WP %d: in tolerance, camera_enable = 1\n', t_now, idx);
        end
    else
        if ~in_tol && reset_photo_on_drift
            state = 1;  dwell_t = 0;  photo_t = 0;  camera_en = false;
            fprintf('t=%6.2f s | WP %d: left tolerance, reset\n', t_now, idx);
        else
            photo_t = photo_t + dt;
            if ~trigger_done && photo_t >= trigger_delay
                camera_trigger = true;  trigger_done = true;     % <<< CAMERA TRIGGER FLAG
                fprintf('t=%6.2f s | WP %d: CAMERA TRIGGER\n', t_now, idx);
            end
            if photo_t >= photo_dur
                photos_done(idx) = 1;
                camera_en = false;  state = 1;
                dwell_t = 0;  photo_t = 0;
                idx = idx + 1;  next_wp = true;
            end
        end
    end

    % --- Webcam snapshot on trigger ---
    if camera_trigger && use_webcam
        img = snapshot(cam);
        imwrite(img, fullfile(save_dir, sprintf('wp_%02d.png', idx)));
    end

    % --- Announce next waypoint so the webcam can be re-aimed ---
    if next_wp && idx <= N
        fprintf('>>> Waypoint done. AIM WEBCAM at drawing %d. Heading to waypoint %d\n', idx, idx);
    end

    % --- Log ---
    L(k,:) = [t_now pos yaw in_tol dwell_t camera_en photo_t camera_trigger min(idx,N) dt];

    % --- Live plot (every 5th step to stay light) ---
    if mod(k,5) == 0
        set(h_path,'XData',[get(h_path,'XData') pos(1)],'YData',[get(h_path,'YData') pos(2)]);
        set(h_drone,'XData',pos(1),'YData',pos(2));
        if camera_en, col = [0 0.6 0]; else, col = [0 0 0]; end
        set(h_txt,'Color',col,'String',sprintf( ...
            'WP %d/%d | z=%.2f | in_tol=%d | dwell=%.1f s | camera\\_en=%d | photo=%.1f s', ...
            min(idx,N), N, pos(3), in_tol, dwell_t, camera_en, photo_t));
        drawnow limitrate;
    end
end

if use_webcam, clear cam; end
if idx > N
    fprintf('Mission complete in %.1f s. Photos at %d/%d waypoints.\n', toc(t_wall), sum(photos_done), N);
end
L = L(1:k,:);

%% ---------------- Post-run plots ----------------
if k > 0
    figure('Name','Run log','Position',[850 100 900 700]);
    subplot(3,1,1); plot(L(:,1),L(:,2:4),'LineWidth',1.2); grid on;
    ylabel('Position (m)'); legend('x','y','z','Location','eastoutside'); title('Position');
    subplot(3,1,2); stairs(L(:,1),L(:,6),'LineWidth',1.2); hold on;
    stairs(L(:,1),L(:,8)+1.2,'LineWidth',1.2); stairs(L(:,1),L(:,10)+2.4,'LineWidth',1.2);
    ylim([-0.2 3.8]); grid on; title('Flags (offset for clarity)');
    legend('in\_tolerance','camera\_enable','camera\_trigger','Location','eastoutside');
    subplot(3,1,3); plot(L(:,1),L(:,7),L(:,1),L(:,9),'LineWidth',1.2); grid on;
    yline(dwell_req,'--'); yline(photo_dur,'--');
    xlabel('Time (s)'); ylabel('Time (s)'); title('Timers');
    legend('dwell\_timer','photo\_timer','Location','eastoutside');
end