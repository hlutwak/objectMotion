function [dots_m, dots_deg, dev0, dev1] = make_surround_withflow(density, jitter, cloud_dist, view_window, devPos, extent, depth_structure, ds)
% Place non overlapping patches with one at deviation position, and other
% reflected across y-axis

%
% density tells you the dot density in dots/deg^2 for surfaces.
%        Density from Warren & Hannon (1990) works out to about .22
%        dots/deg^2.
%
% dims tells you the [X,Y] dimensions of the dot field on the screen in
%       deg
%
%
% dots_m is a three row vector containing the [X;Y;Z] positions of the
%       dots, in m
%
% dots_deg is a two row vector containing the [x;y] positions of the dots
%       on the screen, in deg
%
% devPos is a two row vector containtin the [X;Y] position of the
% deviation, it will be the first few dots in arrays dots_m and dots_deg,
% based on how many devPos columns there are

% HL 10/9/2020
% HL 7/20/2021

if nargin==0
    density = .1;
    view_window = [40 32];
    devPos = [10 7];
end


gridx = (linspace(devPos(1)-extent/density, devPos(1)+extent/density, extent*2+1));
gridx = [-flip(gridx) gridx];

% gridy = (linspace(devPos(2)-round(view_window(1)/2*density/2), devPos(2)+round(view_window(1)/2*density/2), round(view_window(1)/2*density/2)));
gridy = (linspace(devPos(2)-extent/density, devPos(2)+extent/density, extent*2+1));

nDots = length(gridy)*length(gridx);
dots_deg = zeros(2, nDots);
% for d = 1:size(devPos, 2);
%     dots_deg(:,2*d-1) = devPos(:,d);
%     dots_deg(:,2*d) = [-devPos(1,d);devPos(2,d)];
% end

X = reshape(meshgrid(gridx, gridy)',1,[]);
% X = X+jitter*(rand(size(X))-.5);
Y = reshape(meshgrid(gridy, gridx),[],1);
% Y = Y+jitter*(rand(size(Y))-.5);

dots_deg(1,:) = X;
dots_deg(2,:) = Y';


[dev1,dist] = dsearchn(dots_deg',devPos');
[dev0,dist] = dsearchn(dots_deg',[-devPos(1) devPos(2)]);

dots_deg(1,[1:dev0-1 dev0+1:dev1-1 dev1+1:end]) = dots_deg(1,[1:dev0-1 dev0+1:dev1-1 dev1+1:end])+ (rand(1,length(dots_deg)-2)-.5)*jitter;
dots_deg(2,[1:dev0-1 dev0+1:dev1-1 dev1+1:end]) = dots_deg(2,[1:dev0-1 dev0+1:dev1-1 dev1+1:end])+(rand(1,length(dots_deg)-2)-.5)*jitter;

%dots have different distances according to range of dist, except deviated
%spots
nDots = length(dots_deg);

switch depth_structure
    case 0
    %uniform dist
    distances = (cloud_dist(2)-cloud_dist(1)).*rand(1,nDots)+cloud_dist(1);
    %normal dist
    % distances = normrnd(mean(cloud_dist), (cloud_dist(2)-cloud_dist(1))/8, [1,nDots]);
    distances(dev0) = mean(cloud_dist);
    distances(dev1) = mean(cloud_dist);

    case 1
    distances = raytrace2eyeZ(dots_deg, ds.height, ds.gaze_angle)+(randn(1,nDots))*ds.bumpiness;%convert to depth based on eye centered coordinate

    case 2
    distances = zeros(1,nDots);
    
    case 3
    distances = zeros(1,nDots);
end

dots_m = zeros(3, nDots);
dots_m(1:2,:) = distances.*tand(dots_deg);

dots_m(3,:) = distances;


