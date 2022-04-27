function [vel_off_constraint] = get_off_constraint(velocity, original_velocity)

% Takes velocities and reflects about the original velocity (also column vector)
% results in velocities OFF the constraint with the same change speed but opposite direction difference as on the
% constraint

%-- get deviation same change in speed and  direction OFF CONSTRAINT--
unitOriginal = original_velocity/norm(original_velocity);
theta = atan(unitOriginal(2)/unitOriginal(1)); %get angle between unit original velocity and x-axis
R = [cos(theta) sin(theta); -sin(theta) cos(theta)]; % rotates based on tan(theta)
rotated = R*velocity; %rotate the deviated velocity along constraint
rotated(2,:) = -rotated(2,:); %reflect over yaxis (rotated direction of original velocity)
Rback = R'; %rotation matrix to go back
vel_off_constraint = Rback*rotated;

