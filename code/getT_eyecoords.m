function [translation_eyecoords] = getT_eyecoords(gaze_angle, translation)
%convert walking translation to eye coordinate based on gaze angle straight
%ahead

%example: height:5.5ft, step length: 2.2, look ~2.5 steps ahead => 45 deg
% rotate translation vector by gaze angle

theta = deg2rad(gaze_angle);
R = [cos(theta) sin(theta); -sin(theta) cos(theta)]; % rotates based on tan(theta)
YZtranslation = translation(2:3);
YZ_eyecoords = R*YZtranslation';
translation_eyecoords = [translation(1), YZ_eyecoords'];