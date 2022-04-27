%% simulate iterations of stimulus

addpath(genpath('/Applications/Psychtoolbox'))
addpath('/Users/hopelutwak/Documents/MATLAB/VisTools/')

translation = [0, .05, 1.4];
depth_structure = 2; % natural stim
devPos = [5;3];
weberFrac = 2;

n_iterations = 10;
xvals = [];
yvals = [];
xdots = [];
ydots= [];
depths = [];
for n = 1:n_iterations
    [velocity_field, dots_deg,dev1, dots_m,z0] = get_velocity_field_full(translation, depth_structure);
    xdots = [xdots,dots_deg(1,1:dev1-1), dots_deg(1,dev1+1:end)];
    ydots = [ydots, dots_deg(2,1:dev1-1), dots_deg(2,dev1+1:end)];
    xvals = [xvals, velocity_field(1,1:dev1-1), velocity_field(1,dev1+1:end)];
    yvals = [yvals, velocity_field(2,1:dev1-1), velocity_field(2,dev1+1:end)];
    depths = [depths, dots_m(3,1:dev1-1), dots_m(3,dev1+1:end)];
end

positions = [xdots;ydots];
velocities = [xvals;yvals];

%% visualize velocities

figure, quiver(xdots, -ydots, xvals, -yvals)

% find surround based on radius
devPos = [5; 3];
distance2d1 = vecnorm(devPos - positions);
radius = 2; % in degrees
window_idx = find(distance2d1<radius);

% visualize surround
hold on, scatter(xdots(window_idx), -ydots(window_idx))
axis equal
xlim([-20,20])
ylim([-15,15])
%take average
avg_velocity = mean(velocities(:,window_idx),2);


%
hold on, quiver(devPos(1),-devPos(2),avg_velocity(1), -avg_velocity(2))

%% assume different depth map

% visualize depth map
figure, scatter3(xdots, -ydots, depths)
% calculate as if surround is tree
hold on, scatter3(xdots(window_idx), -ydots(window_idx), depths(window_idx))
avg_depth = mean(depths(window_idx));
min_depth = min(depths(window_idx));


new_velocities = calculate_cloud_flow(ones(size(depths(window_idx)))*max(depths(window_idx)), [xdots(window_idx); ydots(window_idx)], translation, .35, z0);

figure
quiver(xdots(window_idx), -ydots(window_idx), xvals(window_idx), -yvals(window_idx))
hold on
quiver(xdots(window_idx), -ydots(window_idx), new_velocities(1,:), -new_velocities(2,:))

avg_velocity = mean(new_velocities,2);
%% Mahalanobis Distance

%calculate covariance matrix
C = cov(velocities');

cond = get_conditions(translation, depth_structure, devPos, weberFrac);

% matrix of distances
D = zeros(length(cond), length(cond(1).steps));
xsteps = zeros(size(D));
ysteps = zeros(size(D));
% calculate mahalanobis distance for each tested velocity
for c = 1:length(cond)
    for s = 1:length(cond(c).steps)
        x = cond(c).steps(:,s);
        D(c,s) = sqrt((x - avg_velocity)'*pinv(C)*(x-avg_velocity));
        xsteps(c,s) = cond(c).steps(1,s);
        ysteps(c,s) = cond(c).steps(2,s);
    end
end
% D = sqrt((x-mu)'*pinv(s)*(x-mu)

% figure, for p = 1:100
% hold on, scatter(xsteps(p), -ysteps(p), 'Marker', 'o', 'MarkerEdgeAlpha', D(p)/max(max(D)))
% end

figure, scatter3(xsteps(:), -ysteps(:), D(:))