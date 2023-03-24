addpath(genpath('/Applications/Psychtoolbox')) % add psychtoolbox to your path

seed=2;
rng(seed) % to have random dots that appear in the same "random" place each time
ns = 10; % number of seconds
speed = 1; %m/s speed of the observer in a straight line
fps = 30; % frames per second

dim = [5,5,ns*speed+5]; % extent of where dots can be in m: X, Y, Z. Depth is more than how far you're travelling (ns *speed) + a little extra 
% 5 m across
nClusters = 2500; % specify number of clusters
nDotsPerCluster = 1;% number of dots per cluster

viewdist = .5; %m how far the screen is from the observer
viewingdepths = [.5,5]; % nearest and furthest dots that can show up, m

windowRect = [0           0        1920        1080]; % screen size in pixels (origin, width of screen, height of screen)
pixels = windowRect(3:4); % pixel width and height of screen
screensize = [.712 .312]; % screen size in m

ppcm = [39,39]; % pixel per cm of the screen
xyrat = windowRect(3)/windowRect(4);

nDots = nDotsPerCluster*nClusters; % total numbber of dots
clusters = rand(nClusters,3).*dim - .5*[dim(1:2),0]; % randomize position of dots, centered around origin

dots = repmat(clusters,nDotsPerCluster,1);

% visualize dots, orient so that Z axis extends from observer to direction
% of gaze
figure(1), scatter3(dots(:,3), dots(:,1), dots(:,2), 'filled')
xlabel('Z')
ylabel('X')
zlabel('Y')
axis equal


% Observer trajectory

% create matrix where each row is a velocity vector, specify velocity
% between each frame

% first specify position over time
% straight line, don't change X or Y position, just Z coordinate
% start at 0, end at some distance (defined by ns*speed)
% speed/fps defines how far to go over each frame 
trajectory = [zeros(ns*fps+1,1), zeros(ns*fps+1,1),(0:(speed/fps):(ns*speed))'];


% visualize observer trajectory within dots
figure(1), hold on, plot3(trajectory(:,3), trajectory(:,1), trajectory(:,2), 'LineWidth', 3)
legend('dots', 'trajectory')
title('environment and observer trajectory')

% calculate velocity between frames
v = diff(trajectory);
theta = atan(v(:,2)./v(:,3))*.01; % calculate any change in angle

% holder matrices for screen positions
x = nan(nDots,ns*fps);
y = nan(nDots,ns*fps);
I = true(nDots,ns*fps);

for ii=1:ns*fps % 
    velocity = v(ii,:); % how much did the observer move
    % moving observer = moving world relative to observer in equal and opposite way
    % recalculating world coordinates in terms of observer reference frame,
    % where observer is always at the origin
    dots = dots - velocity; 
    % if the observer rotates, rotate the world based on 3D rotation matrix
    observerRotation = [1, 0, 0; 0, cos(theta(ii)), -sin(theta(ii)); 0, sin(theta(ii)), cos(theta(ii))];
    dots = (observerRotation*dots')';
    
    % Using projective geometry (similar triangles) to calculate where on
    % the screen the dots should appear

    % x = x coordinate on screen, y = y coordinate on screen, convert to cm
    x(:,ii) = 100*viewdist*(dots(:,1))./(dots(:,3));
    y(:,ii) = 100*viewdist*(dots(:,2))./(dots(:,3));

    % Indices of dots to show based on how close/far the dots in the real world are (viewing depths)
    I(:,ii) = dots(:,3) > viewingdepths(1)...
        & dots(:,3)< viewingdepths(2);
    % and the screensize
    I(:,ii) = I(:,ii) & abs(x(:,ii)*ppcm(1))<windowRect(3)/2 & abs(y(:,ii)*ppcm(2))<windowRect(4)/2;

    
end

% visualize first frame in pixels
figure, scatter(x(I(:,1),1)*ppcm(1), y(I(:,1),1)*ppcm(2), 'filled')
% xlim([-windowRect(3)/2, windowRect(3)/2])
% ylim([-windowRect(4)/2, windowRect(4)/2])
axis equal
title('first frame')



%% Psychtoolbox

% make it skip certain sreen tests
Screen('Preference', 'SkipSyncTests', 1)
screens = Screen('Screens');

% choose external monitor
% screenNumber = max(screens);
% choose small window on laptop screen
screenNumber = 0;


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

    vbl = Screen('Flip',window);
    
end

sca;
