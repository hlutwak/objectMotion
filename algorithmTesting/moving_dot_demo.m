addpath(genpath('/Applications/Psychtoolbox')) % add psychtoolbox to your path
Screen('Preference', 'SkipSyncTests', 1);
screens = Screen('Screens');
screenNumber = max(screens);

visualize = 1; %turn figures on or off

seed=2;
rng(seed) % to have random dots that appear in the same "random" place each time
ns = 2; % number of seconds
world_speed = 1; % m/s speed of the observer in a straight line
fps =60; % Screen(screenNumber,'FrameRate');

height = .5;
% gaze_angle = 15;
fixation = 3;
speeds = 0.02:0.02:0.1; %speeds m/s, for target
speeds = 0.02;
s = 1;
directions = pi/3:pi/3:2*pi;
d = 5;

dim = [6,0,6]; % extent of where dots can be in m: X, Y, Z. Depth is more than how far you're travelling (ns *speed) + a little extra 
% 5 m across
nClusters = 500; % specify number of clusters
nDotsPerCluster = 2;% number of dots per cluster

view_dist = .25; %m how far the screen is from the observer
viewingdepths = [.5,   15]; % nearest and furthest dots that can show up, m

windowRect = [0           0        1920        1080]; % screen size in pixels (origin, width of screen, height of screen)
pixels = windowRect(3:4); % pixel width and height of screen
screensize = [.712 .312]; % screen size in m

ppcm = [39,39]; % pixel per cm of the screen
xyrat = windowRect(3)/windowRect(4);

nDots = nDotsPerCluster*nClusters+3; % total numbber of dots
clusters = rand(nClusters,3).*dim - [.5*dim(1), -height, 0]; % randomize position of dots, centered around x = 0, and ground pushed down

dots = repmat(clusters,nDotsPerCluster,1);
fixation_dot = [0, height, dim(3)/2];
stationary_dot = [0.5, height, dim(3)/2];
target_dot = [-0.5, height, dim(3)/2];
dots(end+1:end+3,:) = [fixation_dot; stationary_dot; target_dot];

% visualize dots, orient so that Z axis extends from observer to direction
% of gaze
if visualize
    figure(1), scatter3(dots(:,3), dots(:,1), -dots(:,2), 'filled')
    hold on, scatter3(dots(end-2,3), dots(end-2,1), -dots(end-2,2), 'filled', 'r')

    xlabel('Z')
    ylabel('X')
    zlabel('Y')
    axis equal
end


% Observer trajectory

% create matrix where each row is a velocity vector, specify velocity
% between each frame

% first specify position over time
% straight line, don't change X or Y position, just Z coordinate
% start at 0, end at some distance (defined by ns*speed)
% speed/fps defines how far to go over each frame 
trajectory = [zeros(ns*fps+1,1), zeros(ns*fps+1,1),(0:(world_speed/fps):(ns*world_speed))'];

% trajectory = [zeros(ns*fps+1,1), 0.2*sin(0:(speed/fps):(ns*speed))', (0:(speed/fps):(ns*speed))'];
target_trajectory = [speeds(s)*cos(directions(d))*(0:1/fps:ns)', zeros(ns*fps+1,1), speeds(s)*sin(directions(d))*(0:1/fps:ns)'];
target_trajectory = target_trajectory + target_dot;

% rotation
secs = 1/fps:1/fps:ns;
theta = atan(height./(fixation-world_speed*secs)); % update theta for observer fixating at a point at the ground in front of them, fixation m away

% visualize observer trajectory within dots
if visualize
    figure(1), hold on, plot3(trajectory(:,3), trajectory(:,1), -trajectory(:,2), 'LineWidth', 3)
    hold on, plot3(target_trajectory(:,3), target_trajectory(:,1), -target_trajectory(:,2), 'LineWidth', 2)
    legend('dots', 'trajectory', 'moving object')
    title('environment and observer trajectory')
end

% calculate velocity between frames
v = diff(trajectory);
v_target = diff(target_trajectory);
T = NaN(size(v));
v_constraint = nan(2,nDots,ns*fps);

% holder matrices for screen positions
x = nan(nDots,ns*fps);
y = nan(nDots,ns*fps);
Z = nan(nDots,ns*fps);
I = true(nDots,ns*fps);

drawndots = NaN([size(dots) ns*fps]);
for ii=1:ns*fps % 
    velocity = v(ii,:); % how much did the observer move
    t_vel = v_target(ii,:);
    % moving observer = moving world relative to observer in equal and opposite way
    % recalculating world coordinates in terms of observer reference frame,
    % where observer is always at the origin
    dots = dots - velocity; %shift dots in world coordinates
    dots(end,:) = dots(end,:)-t_vel; %add velocity to moving object

    % if the observer rotates, rotate the world based on 3D rotation matrix
    observerRotation = [1, 0, 0; 0, cos(theta(ii)), -sin(theta(ii)); 0, sin(theta(ii)), cos(theta(ii))];
    drawndots(:,:,ii) = (observerRotation*dots')'; %observer coordinate dots
    
    %for calculating velocity based on constraint equation
    Z(:,ii) = drawndots(:,3,ii); %Z value of points
    T(ii,:) = (observerRotation*velocity')'; %rotate velocity vector as well
    
    % Using projective geometry (similar triangles) to calculate where on
    % the screen the dots should appear

    % x = x coordinate on screen, y = y coordinate on screen, convert to cm
    x(:,ii) = 100*view_dist*(drawndots(:,1,ii))./(drawndots(:,3,ii));
    y(:,ii) = 100*view_dist*(drawndots(:,2,ii))./(drawndots(:,3,ii));
    
    % calculate velocity based on constraint eq, make sure x,y in m
    v_constraint(:,:,ii) = constraint_velocity_screen(Z(:,ii), [x(:,ii)';y(:,ii)']./100, T(ii,:), view_dist,Z(end-2,ii));
    v_constraint(:,:,ii) = v_constraint(:,:,ii).*100;
    v_constraint_far(:,:,ii) = constraint_velocity_screen(Z(:,ii)+1, [x(:,ii)';y(:,ii)']./100, T(ii,:), view_dist,Z(end-2,ii));
    v_constraint_far(:,:,ii) = v_constraint_far(:,:,ii)*100;
    
    % Indices of dots to show based on how close/far the dots in the real world are (viewing depths)
    I(:,ii) = drawndots(:,3,ii) > viewingdepths(1)...
        & drawndots(:,3,ii)< viewingdepths(2);
    % and the screensize
    I(:,ii) = I(:,ii) & abs(x(:,ii)*ppcm(1))<windowRect(3)/2 & abs(y(:,ii)*ppcm(2))<windowRect(4)/2;

    
end

rvelocityX = diff(x,1,2);
rvelocityY = diff(y,1,2);

% visualize first frame in pixels
if visualize
    figure, scatter(x(I(:,1),1)*ppcm(1), -y(I(:,1),1)*ppcm(2), 'filled')
    hold on, scatter(x(end-2,1)*ppcm(1), -y(end-2,1)*ppcm(2), 'filled', 'r') %fixation
    hold on, scatter(x(end-1,1)*ppcm(1), -y(end-1,1)*ppcm(2), 'filled', 'b') %stationary
    hold on, scatter(x(end,1)*ppcm(1), -y(end,1)*ppcm(2), 'filled', 'g') %target
    xlim([-windowRect(3)/2, windowRect(3)/2])
    ylim([-windowRect(4)/2, windowRect(4)/2])
    axis equal
    title('first frame')
end



%% Psychtoolbox

% make it skip certain sreen tests
Screen('Preference', 'SkipSyncTests', 1);
screens = Screen('Screens');

% choose external monitor
screenNumber = max(screens);
% choose small window on laptop screen
% screenNumber = 0;


% Find the color values which correspond to white and black.
white=WhiteIndex(screenNumber);
black=BlackIndex(screenNumber);

% Round gray to integral number, to avoid roundoff artifacts with some
% graphics cards:
gray=round((white+black)/2);

PsychImaging('PrepareConfiguration');
PsychImaging('AddTask', 'General', 'NormalizedHighresColorRange', 1);
% Open an on screen window and color it black
% [window, windowRect] = PsychImaging('OpenWindow', screenNumber, 0);
[window, windowRect] = PsychImaging('OpenWindow', screenNumber,0,[0,0,960, 540]);

Screen('BlendFunction', window, GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
screen_rect = Screen('Rect',window);

ctr = windowRect(3:4)/2;
vbl = Screen('Flip',window);

sr_hor = round(screen_rect(3)/2); % Middle of the screen, horizontally, in pixels
sr_ver = round(screen_rect(4)/2); % Middle of the screen, vertically, in pixels
fix_hor = sr_hor;     % Horizontal location of fixation cross, in pixels
fix_ver = sr_ver;     % Vertical location of fixation cross, in pixels


Xdraw = x;
Ydraw = y;


for ii=1:ns*fps
    
    xy = [Xdraw(I(:,ii),ii),Ydraw(I(:,ii),ii)]'.*ppcm';
    % draw dots on each frame, in the correct window, at xy positions,
    % size, color, center of the screen, 2 = draw round dots with
    % anti-aliasing
    Screen('DrawDots', window, xy, 10 ,[1,1,1] ,ctr, 2);
%     Screen('DrawDots', window, xy(:,end), 10 ,[0,1,0] ,ctr, 2);
    Screen('DrawDots', window, xy(:,end-2), 10 ,[1,0,0] ,ctr, 2);

    vbl = Screen('Flip',window);
    
end

sca;


%% plot velocities
figure
for ii = 1:ns*fps-1
    quiver(x(I(:,ii),ii), -y(I(:,ii),ii), rvelocityX(I(:,ii),ii), -rvelocityY(I(:,ii),ii), 'color', [.25, .25, .25], 'AutoScale', 1, 'LineWidth', 2), axis equal

    xlim([-30,30])
    ylim([-20,20])

    pause(1/fps)
end

figure
ii = ns*fps-2;
scatter(x(I(:,ii),ii), -y(I(:,ii),ii), 20,'k', 'filled');
hold on,    quiver(x(I(:,ii),ii), -y(I(:,ii),ii), v_constraint(1,I(:,ii),ii)', -v_constraint(2,I(:,ii),ii)', 'color', [0, 0, .5], 'AutoScale','off', 'LineWidth', 2), axis equal

hold on,    quiver(x(I(:,ii),ii), -y(I(:,ii),ii), v_constraint_far(1,I(:,ii),ii)', -v_constraint_far(2,I(:,ii),ii)', 'color', [.5, 0, 0], 'AutoScale','off', 'LineWidth', 2), axis equal

hold on, quiver(x(I(:,ii),ii), -y(I(:,ii),ii), rvelocityX(I(:,ii),ii), -rvelocityY(I(:,ii),ii), 'color', [.25, .25, .25], 'AutoScale','off', 'LineWidth', 2), axis equal

ii = ns*fps-1;
hold on, scatter(x(I(:,ii),ii), -y(I(:,ii),ii),20, 'b', 'filled');
    hold on, scatter(x(end,ii), -y(end,ii), 10,'c', 'filled');


figure
for ii = 1:ns*fps-1
   quiver(x(I(:,ii),ii), -y(I(:,ii),ii), v_constraint(1,I(:,ii),ii)', -v_constraint(2,I(:,ii),ii)', 'color', [.25, .25, .25], 'AutoScale','off', 'LineWidth', 2), axis equal
    xlim([-30,30])
    ylim([-20,20])

    pause(1/fps)
end

 
% look at difference between screen velocity and constraint velocity
% label moving object in red
figure
for ii = 1:ns*fps-1 %ns*fps-1
    % getting distance to constraint segment
%     vec = v_constraint(:,:,ii)'- v_constraint_far(:,:,ii)'; %vector between close and far velocities
%     slope = vec(:,2)./vec(:,1); 
%     intercept = v_constraint(2,:,ii)' - slope.*v_constraint(1,:,ii)'; %intercept = y-slope*x
%     
    % P1 = (x1, y1) = v_constraint, P2 =(x2,y2) = v_constraint_far, P0 =
    % (x0,y0) = rvelocityX, rvelocityY
    a = (v_constraint_far(1,:,ii)-v_constraint(1,:,ii)).*(v_constraint(2,:,ii)-rvelocityY(:,ii)');
    b = (v_constraint(1,:,ii)-rvelocityX(:,ii)').*(v_constraint_far(2,:,ii)-v_constraint(2,:,ii));
    c = sqrt((v_constraint_far(1,:,ii) - v_constraint(1,:,ii)).^2 + (v_constraint_far(2,:,ii) - v_constraint(2,:,ii)).^2);
    d = (abs(a-b)./c)';
%     d = vecnorm((v_constraint(:,:,ii)'- [rvelocityX(:,ii) rvelocityY(:,ii)])')';
    val = max(d(I(:,ii)));
    idx = find(d == val);
%     scatter(d(end), val)
%     xlim([0,0.25]);
%     pause(1/fps*2)
    
    clf
    quiver(x(I(:,ii),ii), -y(I(:,ii),ii), v_constraint(1,I(:,ii),ii)', -v_constraint(2,I(:,ii),ii)', 'color', [.25, .25, .25], 'AutoScale', 1, 'LineWidth', 2), axis equal
    hold on, quiver(x(idx,ii), -y(idx,ii), v_constraint(1,idx,ii)', -v_constraint(2,idx,ii)', 'color', [1,0,0], 'AutoScale', 1, 'LineWidth', 2), axis equal
    hold on, scatter(x(idx,ii), -y(idx,ii), 10,'r', 'filled');
    text(x(idx, ii), -y(idx, ii), num2str(val))
    xlim([-15,15])
    ylim([-10,10])
    pause(1/fps)
    
end
