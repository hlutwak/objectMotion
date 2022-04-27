function [velocity_field] = calculate_cloud_flow(Z, centers_deg, translate, view_dist, z0)
% The world is a cloud of enveloped pattern motion, the observer is translating and and fixating past it.
% The observer is always at the origin, and is looking down the Z axis.
% This program takes a set of gabors and calculates the velocity of each
% one based on given translational components (for phase translating).
%
% XYZ is the XYZ position (in m) of the locations where we are measuring
% the speed.
%
% centers_deg is the x,y positions of the gabors on the screen (in deg).
%
% translate tells you the observer's translation speed (Vx, Vy, Vz) in
% m/sec.
%
% view_dist is how far the observer is the from the screen in meters.
%
% velocity_field is a matrix (2x[number of locations]) reporting
% the x and y velocity in deg/s.
%
% HL
% Updated 2/25/2020
% Updated 9/2/2020 takes in just depth for first argument
% 
% if nargin==0
%     [XYZ, centers_deg] = make_dot_plane(.1, 12.5, [40 32], [-20 -3 20 3]);
%     translate = [0.14 0 1.9];                                               % the observer is walking forward at 1.9 m/s, a brisk walk
%     view_dist = .57;
% end 

% initialize empty matrix to hold velocities
velocity_field = zeros(size(centers_deg));                                 % number of patches, X and Y vel

%convert the positions in deg to m, because the other measurements
%are already in m.
centers_m = view_dist*tand(centers_deg);
% centers_m = XYZ(3,:).*tand(centers_deg);

%solve for the inverse depth at the X,Y,Z position that projects to the
%desired x,y position, then use that to compute the flow vector at each
%location (same Z value for all of the gabors).
for ii=1:length(Z)
        inv_depth = 1/(Z(ii));
        
        % Heeger + Jepson, Lutwak + Bonnen + Simoncelli
        x = centers_m(1,ii); % coordinates of screen
        y = centers_m(2,ii);
        f = view_dist;
        Pxy = inv_depth;
        
        A = [-f 0 x; 0 -f y];
%         B = [(x*y)/f -(f+x^2/f) y; f+y^2/f -(x*y)/f -x];
        B = [f+x^2/f x*y/f 0; x*y/f f+y^2/f 0];
%         z0 = f;
%         velocity_field1(1:2,i) = Pxy*A*translate'+B*rotate';
        velocity_field(1:2,ii) = (z0*Pxy*A+B)*translate'/z0;
end

%convert velocities in m/s back to deg/sec
% velocity_field = atand(velocity_field/view_dist); 
% rescale for velocities in peripherry
for ii = 1:2
    velocity_field(ii,:) = atand(velocity_field(ii,:)*view_dist./(view_dist^2+(velocity_field(ii,:)+centers_m(ii,:)).*centers_m(ii,:)));
end
