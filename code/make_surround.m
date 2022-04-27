function [dots_deg, dev0, dev1] = make_surround(density, jitter, view_window, devPos, extent)
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
    exclude = [-20 -3 20 3];
    devPos = [10 7];
end


% gridx = (linspace(devPos(1)-round(view_window(1)/2*density/2), devPos(1)+round(view_window(1)/2*density/2), round(view_window(1)/2*density/2)));
% gridx = [-flip(gridx) gridx];

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


