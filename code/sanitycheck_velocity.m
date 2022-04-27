% sanity check velocity

calculatedvel = cond(1).steps(:,end); %display velocity based on equations

% calculate velocity by ray tracing
coordinate = dsearchn([X(:,1) Y(:,1)],devPos');
coordm = dots(coordinate,:); % initial world coordinate of point

oldposition = [X(coordinate,1), Y(coordinate,1)]; %initial visual field position

newcoordm = coordm - translation; % travel and update world coordinate of point
translation_angle = -atan(translate(2)/(z0-translate(3))); %calculate based on fixation on ground and tranlation
translationRotation = [1, 0, 0; 0, cos(translation_angle), -sin(translation_angle); 0, sin(translation_angle), cos(translation_angle)]; % rotate in opposite way of gaze_angle to convert to eye centered coordinates
            
newcoordm = (translationRotation*newcoordm')'; %rotation from thetadots

newposition = [rad2deg(atan(newcoordm(1)/newcoordm(3))), rad2deg(atan(newcoordm(2)/newcoordm(3)))];

velocity = newposition - oldposition;