%% Testing constraint line with simulation of walk through 3d world

addpath(genpath('/Users/hopelutwak/Desktop/objectMotion'))
addpath(genpath('/Applications/Psychtoolbox'))
addpath('/Users/hopelutwak/Documents/MATLAB/VisTools/')
%% Generate scene
    frame_rate = 85;
    translation = [0,  0.05, 1.4]/frame_rate;
    depth_structure = 2;
    devPos = [5;3];
    weberFrac = 1;
    view_dist = .35;
%     displacement = [-.01,-0,0]'/frame_rate;
%     random_displacements = 1;
    
%% Calculate constraint lines for each position
close all
[velocity_field, dots_deg, dev1, dots_m, dots_m_next,z0, gazeRotation, displacements] = get_velocity_field_frames(translation, depth_structure, devPos, displacement,random_displacements,view_dist);

dots_screen = view_dist*tand(dots_deg);
velocity_field_screen = calculate_cloud_flow_screen(dots_m(3,:), dots_deg, translation, view_dist, z0);
plane_field_screen = calculate_cloud_flow_screen(ones(1, length(dots_deg))*.5, dots_deg, translation, view_dist, z0);
plane_field_screen2 = calculate_cloud_flow_screen(ones(1, length(dots_deg))*1000, dots_deg, translation, view_dist, z0);

% plane_field2 = calculate_cloud_flow_screen(ones(1, length(dots_deg))*(min(dots_m(3,:))+max(dots_m(3,:)))/2, dots_deg, translation, view_dist, z0);
% plane_field3 = calculate_cloud_flow_screen(ones(1, length(dots_deg))*max(dots_m(3,:)), dots_deg, translation, view_dist, z0);


% difference = velocity_field_screen-plane_field_screen;
difference = plane_field_screen2-plane_field_screen;
slope = difference(2,:)./difference(1,:); % get slope of each constraint line
intercept = plane_field_screen(2,:)-slope.*plane_field_screen(1,:); % get intercept of each constraint line



% d = abs(velocity_field(2,:)-(slope.*velocity_field(1,:)+intercept));
x1 = plane_field_screen(1,:);
y1 = plane_field_screen(2,:);
x2 = plane_field_screen2(1,:);
y2 = plane_field_screen2(2,:);
x0 = velocity_field(1,:);
y0 = velocity_field(2,:);

d = abs((x2-x1).*(y1-y0)-(x1-x0).*(y2-y1))./sqrt((x2-x1).^2 + (y2-y1).^2);

% d = abs(slope.*velocity_field(1,:) - velocity_field(2,:) + intercept)./(slope.^2 + intercept.^2);

% compute distance to line segment
distancep2a = sqrt((velocity_field(1,:)-plane_field_screen(1,:)).^2 + (velocity_field(2,:)-plane_field_screen(2,:)).^2);
distancep2b = sqrt((velocity_field(1,:)-plane_field_screen2(1,:)).^2 + (velocity_field(2,:)-plane_field_screen2(2,:)).^2);
distancea2b = sqrt((plane_field_screen(1,:)-plane_field_screen2(1,:)).^2 + (plane_field_screen(2,:)-plane_field_screen2(2,:)).^2);
ratio = distancep2a./distancea2b;


idx_mind2point = find(ratio >1);

d(idx_mind2point) = min(distancep2a(idx_mind2point), distancep2b(idx_mind2point));


d = d/max(d);
d = 1-d;
c = repmat(d', 1,3);


figure(1)
scatter(dots_deg(1,:), -dots_deg(2,:),100,c, 'filled')

%find where there's object motion

dev = find(abs(velocity_field(2,:)-(slope.*velocity_field(1,:)+intercept))>10^-6);
figure(2)
quiver(dots_screen(1,:), -dots_screen(2,:), velocity_field(1,:), -velocity_field(2,:), 'color', [.25, .25, .25], 'AutoScale', 1, 'LineWidth', 2), axis equal
hold on, scatter(dots_screen(1,dev), -dots_screen(2,dev), 100)
% set(gcf,'renderer','Painters'), saveas(gcf, 'deviations','eps')
% hold on, scatter(dots_screen(1,:), -dots_screen(2,:),100,c, 'filled') 


% figure, quiver([dots_screen(1,:),dots_screen(1,:),dots_screen(1,:)], -[dots_screen(2,:), dots_screen(2,:), dots_screen(2,:)], [velocity_field(1,:), velocity_field_screen(1,:), plane_field_screen(1,:)], -[velocity_field(2,:), velocity_field_screen(2,:), plane_field_screen(2,:)])

% calculate angle difference between velocity in deg/s vs m/s on screen
% theta = zeros(1,length(velocity_field));
% for v = 1:length(velocity_field)
%     unit_deg = velocity_field./vecnorm(velocity_field);
%     unit_screen = velocity_field_screen./vecnorm(velocity_field_screen);
%     
%     theta(v) = acosd(dot(unit_deg(:,v), unit_screen(:,v)));
% end

    
    
%% plot constraint lines
segment_begin = plane_field_screen+ dots_screen;
segment_end = plane_field_screen2 + dots_screen;
figure
for s = 1:length(dots_deg)
    hold on
    plot([segment_begin(1,s) segment_end(1,s)], -[segment_begin(2,s) segment_end(2,s)], 'k')
end
    
hold on, scatter(velocity_field(1,:)+dots_screen(1,:), -(velocity_field(2,:)+dots_screen(2,:)), 'filled')
hold on, scatter(velocity_field(1,dev)+dots_screen(1,dev), -(velocity_field(2,dev)+dots_screen(2,dev)), 'r', 'filled')



%% recaulculating depth constraint
[velocity_field, dots_deg, dev1, dots_m,z0]= get_velocity_field_full(translation, depth_structure, devPos, view_dist);

translation = [0,-0.0722083049421342,1.39903036446583]/frame_rate;
plane_field_screen = calculate_cloud_flow(ones(1, length(dots_deg))*.5, dots_deg, translation, view_dist, z0);
plane_field_screen2 = calculate_cloud_flow(ones(1, length(dots_deg))*100000, dots_deg, translation, view_dist, z0);

figure
quiver(0, 0, velocity_field(1,dev1), -velocity_field(2,dev1),'Color', 'k','LineWidth', 2,'AutoScaleFactor',1)
hold on, plot([plane_field_screen(1,dev1) plane_field_screen2(1,dev1)],-[plane_field_screen(2,dev1), plane_field_screen2(2,dev1)], 'color', [.5 .5 .5], 'linewidth', 2), axis equal

%% plot velocity field and save
figure, quiver(dots_screen(1,:), -dots_screen(2,:), velocity_field(1,:), -velocity_field(2,:), 'color', [.25, .25, .25], 'AutoScale', 1, 'LineWidth', 2), axis equal


% set(gcf,'renderer','Painters'), saveas(gcf, 'natural_velocity_field','eps')


%% Add in velocity of testing conditions
[cond]= get_conditions(translation, depth_structure, devPos, weberFrac);

% replace deviation with tested velocity
n_correct = 0;
figure
for c = 1:length(cond)
    for step = 1:length(cond(1).steps)
    velocity_field_screen(:,dev1) = cond(c).steps(:,step);
    
    % check with calculated flow field
    moving_object_idx = find(abs(velocity_field_screen(2,:)-(slope.*velocity_field_screen(1,:)+intercept))>10^-8);
    
    if moving_object_idx == dev1
        n_correct = n_correct+1;
    end
    
    % plot detectable conditions in red
    hold on, scatter(velocity_field_screen(1,moving_object_idx), -velocity_field_screen(2,moving_object_idx), 'filled', 'r')
    hold on, scatter(velocity_field_screen(1,dev1), -velocity_field_screen(2,dev1), 'k')
    end
end
axis equal

if n_correct == ((length(cond)-2)*(length(cond(1).steps)-1))
   disp('passed all cases')
end

figure
quiver(dots_screen(1,:), -dots_screen(2,:), velocity_field_screen(1,:), -velocity_field_screen(2,:), 'AutoScaleFactor', 1)


%% Add in realistic moving object in simulated scene 

% bug on tree
% get bug coordinate
% displacements = 
displacement = [0, .05,0]'/frame_rate;

[velocity_field, dots_deg, dev1, dots_m, dots_m_next,z0, gazeRotation] = get_velocity_field_frames(translation, depth_structure, devPos, displacement, random_displacements, view_dist);

dots_screen = view_dist*tand(dots_deg);
velocity_field_screen = calculate_cloud_flow_screen(dots_m(3,:), dots_deg, translation, view_dist, z0);
plane_field_screen = calculate_cloud_flow_screen(ones(1, length(dots_deg))*min(dots_m(3,:))-1, dots_deg, translation, view_dist, z0);
plane_field_screen2 = calculate_cloud_flow_screen(ones(1, length(dots_deg))*max(dots_m(3,:))+1, dots_deg, translation, view_dist, z0);

% plane_field2 = calculate_cloud_flow_screen(ones(1, length(dots_deg))*(min(dots_m(3,:))+max(dots_m(3,:)))/2, dots_deg, translation, view_dist, z0);
% plane_field3 = calculate_cloud_flow_screen(ones(1, length(dots_deg))*max(dots_m(3,:)), dots_deg, translation, view_dist, z0);


% difference = velocity_field_screen-plane_field_screen;
difference = plane_field_screen2-plane_field_screen;
slope = difference(2,:)./difference(1,:); % get slope of each constraint line
intercept = plane_field_screen(2,:)-slope.*plane_field_screen(1,:); % get intercept of each constraint line


%find where there's object motion

find(abs(velocity_field(2,:)-(slope.*velocity_field(1,:)+intercept))>10^-6)

figure, quiver(dots_screen(1,:), -dots_screen(2,:), velocity_field(1,:), -velocity_field(2,:))

x1 = plane_field_screen(1,:);
x2 = plane_field_screen2(1,:);
y1 = plane_field_screen(2,:);
y2 = plane_field_screen2(2,:);
x0 = velocity_field(1,:);
y0 = velocity_field(2,:);
d = abs((x2-x1).*(y1-y0)-(x1-x0).*(y2-y1))./sqrt((x2-x1).^2 + (y2-y2).^2);
d = d/(max(d));

figure
for vel = 1:length(velocity_field)
    hold on
    quiver(dots_screen(1,vel), -dots_screen(2,vel), velocity_field(1,vel), -velocity_field(2,vel), 'color', [d(vel), d(vel), d(vel)])
end
%% Add in moving object in simulated scene to match condition velocities

% get conditions for experiment over one frame
[cond]= get_conditions(translation, depth_structure, devPos, weberFrac);

figure
scatter3(dots_m(1,:), dots_m(2,:), dots_m(3,:))
figure
scatter3(dots_m(1,dev1), dots_m(2,dev1), dots_m(3,dev1))
% scatter3(dots_m(1,dev1-20:dev1+20), dots_m(2,dev1-20:dev1+20), dots_m(3,dev1-20:dev1+20))
% figure
% scatter(dots_deg(1,:), dots_deg(2,:))

% for each velocity dictated by experimental conditions, create 3D motion
% resulting in that velocity, flag which motions must be due to a moving
% object

%save 3d motions that were detected in this matrix
detected = [];
for c = 1:length(cond)
    for s = 1:length(cond(1).steps)
        % calculate new screen velocity
        deviation_screen = cond(c).steps(:,s);
        % add it to point on screen
        updated_screen = dots_screen(:,dev1)+deviation_screen;
        
        % project back into 3D using depth from after rotation/translation
        updated_point = [updated_screen/view_dist*dots_m_next(3,dev1);dots_m_next(3,dev1)];
        
        % undo gaze rotation and translation
        rewound_point = gazeRotation'*updated_point+translation';
        
        displacement = rewound_point - dots_m(:,dev1);
        
        %calculate with 3D displacement
        
        %create new matrix with dots on screen before and after
        dots_step = zeros([2,length(dots_m),2]);
        
        % before translate (dots on the screen)
        dots_step(:,:,1) = view_dist*dots_m(1:2,:)./dots_m(3,:);
        
        
        % change world position of dot corresponding to original deviation position
        dots_m(:,dev1) = dots_m(:,dev1)+displacement; %deviate in real world before calculating redrawing after translation and rotation
        
        % update dots after translation
        dots_m_next = dots_m - translation';
        
        % and rotation
        
        gaze_angle = atand(translation(2)/(z0-translation(3)));
        gaze_angle = atand(translation(2)/(z0));
        gaze_angle = double(gaze_angle);
        
        gazeRotation = [1, 0, 0; 0, cosd(-gaze_angle), -sind(-gaze_angle); 0, sind(-gaze_angle), cosd(-gaze_angle)]; % rotate in opposite way of gaze_angle to convert to eye centered coordinates
        gazeRotation = double(gazeRotation);
        dots_m_next = gazeRotation*dots_m_next;
        
        
        % calculate new on screen positions
        dots_step(:,:,2) = view_dist*dots_m_next(1:2,:)./dots_m_next(3,:);
        
        % calculate screen velocity
        velocity_field = dots_step(:,:,2)-dots_step(:,:,1);
        
        % check what points are seen as moving objects
        deviations = find(abs(velocity_field(2,:)-(slope.*velocity_field(1,:)+intercept))>10^-6);
                        %add deviation to the ones that are detected
        threeDmotion = dots_m_next(:,deviations) - dots_m(:,deviations);
        disp(threeDmotion);
        disp(deviations);

        hold on, scatter3(dots_m_next(1,deviations), dots_m_next(2,deviations), dots_m_next(3,deviations), 'filled', 'r')
            if length(deviations)>1
                disp('found more than one deviation')
                disp(sprintf('%d %d',c, s))
           
            end
    end
    
end

%% Display motions of point to create onscreen velocities tested


%% Calculate ellipse along constraint line

[cond]= get_conditions(translation, depth_structure, devPos, weberFrac);

% centered = zeros(2,length(c));
for c = 1:10
    centered(:,c) = cond(c).steps(:,1)-cond(c).steps(:,end);
end
% vecnorm(velocity_thresh(:,1)- data(cond_idx(2)).steps(:,end)) 
% T1 = 0.2932 inc speed, 0.2750 change in dir, along constraint .3116
% T2 = 0.3190 inc speed, 0.3279 change in dir, along constraint .3308



eig1 = cond(9).steps(:,1)- cond(9).steps(:,end);
eig1 = eig1/norm(eig1);
rotateccw = [cos(pi/2) sin(pi/2); -sin(pi/2) cos(pi/2)];
eig2 = rotateccw*eig1;
eig2 = eig2/norm(eig2);

t = linspace(0,2*pi, 20);
ellipse = [eig1 eig2]*[.3308*1.25*cos(t); .3308*.75*sin(t)];
ellipse = ellipse+cond(9).steps(:,end);

figure, 
hold on, plot(ellipse(1,:),-ellipse(2,:),'k-', 'LineWidth', 2)
axis equal

%% Highlight which velocities are off the constraint line

% does the velocity lie on the line
length(find(velocity_field_screen(2,:) == slope.*velocity_field_screen(1,:)+intercept))

length(find(abs(velocity_field_screen(2,:)-(slope.*velocity_field_screen(1,:)+intercept))<10^-6))


length(find(abs(velocity_field(2,:)-(slope.*velocity_field(1,:)+intercept))<10^-6))


length(find(abs(plane_field_screen(2,:)-(slope.*plane_field_screen(1,:)+intercept))<10^-6))

 
%% compare with predction from algorithm

% eigenvectors

% noise 75% confidence
%% adding noise
% noise to base detection ability (local uniform surround)
% estimating translation/rotation from optic flow to get error in detection



%% 
