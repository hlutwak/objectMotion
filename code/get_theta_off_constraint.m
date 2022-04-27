function [vel_off_constraint] = get_theta_off_constraint(velocity, original_velocity,theta)

% Takes velocities and rotates about original velocity by theta degrees

%-- rotate constraint line around middle distance velocity--
centered = velocity - original_velocity;
theta = deg2rad(theta);
R = [cos(theta) sin(theta); -sin(theta) cos(theta)]; % rotates based on tan(theta)
rotated = R*centered;
vel_off_constraint = rotated+original_velocity;