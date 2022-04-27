function [eyeZ] = raytrace2eyeZ(dots_deg, height, gaze_angle)

%takes place in imaging plane in terms of visual degrees, height off the
%ground, and gaze angle from straight at the ground

raytraces = height./cos(deg2rad(gaze_angle-dots_deg(2,:))); %hypotenuse, flipped y vals
eyeZ = raytraces.*cos(deg2rad(-dots_deg(2,:)));%convert to depth based on eye centered coordinate
