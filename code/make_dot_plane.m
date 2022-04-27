function [dots_m, dots_deg, dev0, dev1] = make_dot_plane(density, jitter, bumpiness, height, gaze_angle, wall, view_window, gabor_diam, exclude, devPos)
% Make a cloud of non overlapping random widows, with one always at deviation
% position, and the other reflected across the y-axis, deviation position
% will have set depth
%
% density tells you the dot density in dots/deg^2 for surfaces.
%        Density from Warren & Hannon (1990) works out to about .22
%        dots/deg^2.
%
% beta angle looking at the ground, where 0 is straight at the ground,
% looking forward is positive angle
%
% dims tells you the [X,Y] dimensions of the dot field on the screen in
%       deg
%
% exclude is a rectangle [X1, Y1, X2, Y2] where no dots will be placed,
%       lets you do things like block out the FOE. Can be set to 0 to
%       display the whole world.
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

% HL 7/2/2020

if nargin==0
    density = .1;
    cloud_dist = [12.5 25];
    view_window = [40 32];
    exclude = [-20 -3 20 3];
end

% nDots = ceil(prod(dims)*density);

%grid with correct density that includes devPos
flank = [view_window(1)/2-devPos(1), view_window(2)/2-devPos(2)];
innergridx = linspace(-devPos(1), devPos(1), round(devPos(1)*2*density));
deltax = mean(diff(innergridx));
flankx = devPos(1)+deltax:deltax:view_window(1)/2;
gridx = [-flip(flankx) innergridx flankx];

innergridy = linspace(-devPos(2), devPos(2), round(devPos(2)*2*density));
deltay = mean(diff(innergridy));
flanky = devPos(2)+deltay:deltay:view_window(2)/2;
gridy = [-flip(flanky) innergridy flanky];

nDots = length(gridx)*length(gridy);
dots_deg = zeros(2, nDots);
% for d = 1:size(devPos, 2);
%     dots_deg(:,2*d-1) = devPos(:,d);
%     dots_deg(:,2*d) = [-devPos(1,d);devPos(2,d)];
% end

X = reshape(meshgrid(gridx, gridy)',1,[]);
Y = reshape(meshgrid(gridy, gridx),[],1);

dots_deg(1,:) = X;
dots_deg(2,:) = Y';

% %get non overlapping windows
% for d = size(devPos, 2)*2+1:nDots
%     dot = [(rand(1)-.5)*dims(1); (rand(1)-.5)*dims(2)];
%     diff = dot - dots_deg;
%     while sum(vecnorm(diff)<(gabor_diam/60)) >0 %check if the random dot is too close to any of the others
%         dot = [(rand(1)-.5)*dims(1); (rand(1)-.5)*dims(2)];
%         diff = dot - dots_deg;
%     end
%     dots_deg(:,d) = dot;
% end

%exclude window around fovea
legal = ones(1,nDots); 
if exclude
    for i=1:nDots
        if IsInRect(dots_deg(1,i),dots_deg(2,i),exclude)
            legal(i) = 0;
        end
    end
end
dots_deg = dots_deg(:,legal==1);

[dev1,dist] = dsearchn(dots_deg',devPos');
[dev0,dist] = dsearchn(dots_deg',[-devPos(1) devPos(2)]);

dots_deg(1,[1:dev0-1 dev0+1:dev1-1 dev1+1:end]) = dots_deg(1,[1:dev0-1 dev0+1:dev1-1 dev1+1:end])+ (rand(1,length(dots_deg)-2)-.5)*jitter;
dots_deg(2,[1:dev0-1 dev0+1:dev1-1 dev1+1:end]) = dots_deg(2,[1:dev0-1 dev0+1:dev1-1 dev1+1:end])+(rand(1,length(dots_deg)-2)-.5)*jitter;

% for k = 1:length(devs)
%     dots_deg(1,[1:devs(k)-1 devs(k)+1:end]) = dots_deg(1,[1:devs(k)-1 devs(k)+1:end])+rand(1,length(dots_deg)-1)*jitter;
%     dots_deg(2,[1:devs(k)-1 devs(k)+1:end]) = dots_deg(2,[1:devs(k)-1 devs(k)+1:end])+rand(1,length(dots_deg)-1)*jitter;
% end

%distances based on place in visual field, and angle looking at ground plane
nDots = length(dots_deg);

% replace with raytrace2eyeZ

distances = raytrace2eyeZ(dots_deg, height, gaze_angle)+(randn(1,nDots))*bumpiness;%convert to depth based on eye centered coordinate

if wall
    points_upperField = dots_deg(2,:)<0;
    length_groundPlane = height*tan(deg2rad(gaze_angle));
    distances(points_upperField) = length_groundPlane*cos(deg2rad(-dots_deg(2,points_upperField)));
end
dots_m = zeros(3, nDots);
dots_m(1:2,:) = distances.*tand(dots_deg);

dots_m(3,:) = distances;




