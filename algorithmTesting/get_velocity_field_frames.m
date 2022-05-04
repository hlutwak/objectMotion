function  [velocity_field, dots_deg, dev1, dots_m,dots_m_next,z0, gazeRotation, displacements]= get_velocity_field_frames(translate, depth_structure, devPos, displacement, random_displacements, view_dist)
if nargin==0
    translate = [0,  0.05, 1.4];
    depth_structure = 2;
    devPos = [5;3]; % y axis flipped
    view_dist = .35;
    displacement = [0,0,-translate(3)]; %move forward, not rigid
    random_displacements = 0;
end

if random_displacements
    num_disp = 10;
    frame_rate = 25;
end

%-----Rig Settings----------------------
screensize                      = [.405 .298];                          % m psychophysics room: [.405 .298], home: [.495 .312] [.697 .392]
pixels                          = [1600 1200];                          % psychophysics room: [1600 1200], home: [1920 1200]

view_window                     = round([rad2deg(atan(screensize(1)/2/view_dist)) rad2deg(atan(screensize(2)/2/view_dist))]);  % X,Y centered around fixation [60 46] [54 40];                                                                      % [36 27] for laptop, [55 32] for monitor
scale_factor                    = sqrt((view_window(1)*60*view_window(2)*60)/(pixels(1)*pixels(2)))*1;          % Arcmin/pixel 1.78 2.25 1.92dell: 1680x1050, psychophysics room: 1600x1200                                % Screen frame rate (hz) psychophysics room: 85
scale_factorX = view_window(1)*60/pixels(1);
scale_factorY = view_window(2)*60/pixels(2);

% how to calculate velocities based on scene structure
switch depth_structure
    case 0
    case 1
    case 2
        img_label = 'lRange009.mat';
        load(img_label)
        screen_range_map            = [1.940 1.118];
        screen_range_map_pix        = [1920 1080];
        view_window_map              = rad2deg([atan(screen_range_map(1)/2*1/3), atan(screen_range_map(2)/2*1/3)]); % screen 3 m out
        [frac, axis]                = min(view_window_map./view_window);    % push the scene closer so takes up same amoount of visual angle as experimental screen
        new_dist = screen_range_map(axis)/2*1/tan(deg2rad(view_window(axis)));
        view_window_map = rad2deg([atan(1.94/2*1/new_dist), atan(1.118/2*1/new_dist)]);
        
        crop_amt = screen_range_map_pix - pixels;              % how much of dispay screen fits on range image *need better solution
        crop_amt(crop_amt<0) = 0;
        crop = rangeMap(1:end - crop_amt(2), 1:end - crop_amt(1), :);
        range_distance = -crop(:,:,1);                                  % swap coordinate system so have positive distances
        
        range_distance(range_distance == 1) = max(max(range_distance))*1000; % replace all 1s (no laser return - could be sky) with very far distance
        range_distance = range_distance + 3- new_dist;
        
        range_distance_left = range_distance(:,1:pixels(1)/2);
%         mirror = [range_distance_left, flip(range_distance_left,2)]; %mirror left side with right so choosing deviation on left and right is symmetric
%         range_distance = mirror;
        pixels = [size(range_distance, 2), size(range_distance, 1)];
        
    case 3
        % axes: X+ to the right, Y+ down, Z+ forward,
        gaze_angle = -5; % right hand rule over x axis, negative is looking down
        gazeRotation = [1, 0, 0; 0, cosd(-gaze_angle), -sind(-gaze_angle); 0, sind(-gaze_angle), cosd(-gaze_angle)]; % rotate in opposite way of gaze_angle to convert to eye centered coordinates
        viewingdepths = [.5,500];
        ppcm = [29,-29];
        xyrat = pixels(1)/pixels(2);
        %             nframes = round(frame_rate*stimulus_duration);
        nframes = 1;
        dim = [100,0,nframes+100]; %dimensions for ground plane
        xvals = linspace(-15,15,1000);
        zvals = linspace(0, 100,1000);
        [planex planez] = meshgrid(xvals, zvals);
        planex = planex(:);
        planez = planez(:);
        planey = zeros(size(planez))+2;
        dots = [planex, planey, planez];
        dots = (gazeRotation*dots')';
        
        
        
        %           observerTrajectory = repmat(translate/frame_rate, [nframes, 1]); %trajectory over one frame
        translate = (gazeRotation*translate')';
        translate = translate;
        
        observerTrajectory = repmat(translate, [nframes, 1]); %trajectory over a seccond should be frames but gave up on evolution
        
        thetadots = atan(observerTrajectory(:,2)./observerTrajectory(:,3));
        
        nDots = size(dots,1);
        X = nan(nDots,nframes);
        Y = nan(nDots,nframes);
        I = true(nDots,nframes);
        
        for ii=1:nframes
            position = observerTrajectory(ii,:);
            if ii >1
                dots = dots - position;
            end
            depths(:,ii) = dots(:,3); % in eye centered coordinates now
            
            %                 X(:,ii) = view_dist*(dots(:,1))./(dots(:,3));
            %                 Y(:,ii) = view_dist*(dots(:,2))./(dots(:,3));
            
            X(:,ii) = rad2deg(atan(dots(:,1)./depths(:,ii))); %convert to deg
            Y(:,ii) = rad2deg(atan(dots(:,2)./depths(:,ii)));
            
            
            I(:,ii) = dots(:,3) > viewingdepths(1)...
                & dots(:,3)< viewingdepths(2);
            %                 I(:,ii) = I(:,ii) & abs(X(:,ii)*ppcm(1))<pixels(1)/2 & abs(Y(:,ii)*ppcm(2))<pixels(2)/2;
            I(:,ii) = I(:,ii) & abs(X(:,ii))<view_window(1)/2 & abs(Y(:,ii))<view_window(2)/2;
            
        end
end

 %-----Experiment Settings, Don't Change---
    %-----Array Settings
    z0                              = 3;                                    % fixation distance
    middle_dist                     = 3; 
    switch depth_structure
        case 0 
        case 1
        case 2
            center  = ceil(pixels/2);
            z0 = range_distance(center(2), center(1));
            
        case 3
            coordinate = dsearchn([X(:,1) Y(:,1)],[0,0]);
            z0 = depths(coordinate, 1);

    end                                  %meters
    cloud_dist                      = [middle_dist*.5, middle_dist*1.5];  % m range of distances   

    dot_density                     = 1;                                    % original settings: 1; 1/dot_density = average degrees between apertures measured from the centers
    jitter                          = .5;                                   % deg to jitter by
    text_diam                       = 30;                                   % Arcmin original settings: 30
    exclude                         = [-.1, -.1, .1, .1];                       % Don't place elements where the FOE will be

    switch depth_structure
            case 0
            case 1

            case 2
                [dots_m, dots_deg, dev0, dev1] = make_dot_cloud(dot_density, jitter, cloud_dist, view_window, text_diam, exclude, devPos);
                devright = dots_deg(:,dev1); %get indices of deviation
                devleft = dots_deg(:,dev0);
                
                dots_deg_pix = [dots_deg(1,:)*60/scale_factorX; dots_deg(2,:)*60/scale_factorY]; %convert to pixels
                dots_deg_pix = round(center'+dots_deg_pix); 

                ok = find(dots_deg_pix(2,:)>0 & dots_deg_pix(2,:)< pixels(2) & dots_deg_pix(1,:)>0 & dots_deg_pix(1,:)<pixels(1)); %get positions which can be found in range map data
                dots_m = dots_m(:,ok);
                dots_deg = dots_deg(:,ok);
                dots_deg_pix = dots_deg_pix(:,ok);
                
                ind = sub2ind(size(range_distance), dots_deg_pix(2,:), dots_deg_pix(1,:)); %get the indices 
                dots_m(3,:)  = range_distance(ind); %assign depths to distances
                
                dev1 = find(dots_deg(1,:) == devright(1) & dots_deg(2,:) == devright(2)); % reassign index values for dev0 and dev1  bc got messed up from getting rid of patches that don't align with range image and screen dimensions
                dev0 = find(dots_deg(1,:) == devleft(1) & dots_deg(2,:) == devleft(2));
                
                dots_m(1:2,:) = dots_m(3,:).*tand(dots_deg); %update X,Y values
                
                
                
            case 3
                [dots_m, dots_deg, dev0, dev1] = make_dot_cloud(dot_density, jitter, cloud_dist, view_window, text_diam, exclude, devPos);
                
                dots_deg = [dots_deg];
                nearestidx = knnsearch([X(:,1) Y(:,1)],dots_deg');
                distances = vecnorm(dots_deg- [X(nearestidx,1)'; Y(nearestidx,1)']);
                nothingidx = find(distances>.1);
                drawn_dots_deg = dots_deg;
                drawn_dots_deg(:,nothingidx) = [];
                
                dev0 = dsearchn(drawn_dots_deg', dots_deg(:,dev0)'); % get new indicecs for deviation to match new set of dots
                dev1 = dsearchn(drawn_dots_deg', dots_deg(:,dev1)');

                dots_m(3,:) = depths(nearestidx,1)';% assign depths based on defined world
                dots_m(:,nothingidx) = []; %get rid of points that won't show up
                
                dots_deg = drawn_dots_deg; % update dots_deg
                
                dots_m(1:2,:) = dots_m(3,:).*tand(dots_deg); %update X,Y values
        end

%         velocity_field = calculate_cloud_flow(dots_m(3,:), dots_deg, translate, view_dist, z0);
        
        

%create new matrix with dots on screen before and after
dots_step = zeros([2,length(dots_m),2]);

% before translate (dots on the screen)
dots_step(:,:,1) = view_dist*dots_m(1:2,:)./dots_m(3,:);


% find some other random places to deviate
if random_displacements
    r = randi(length(dots_deg), 1,num_disp);
    displacements = zeros(3,num_disp);
    for ii = 1:length(r)
        if dots_m(3,r(ii)) < 6
            displacements(:,ii) = -.05*rand(3,1)+.05;
        else
            displacements(:,ii) = 2*rand(3,1)-1;
        end
    end
    displacements = displacements./frame_rate;
    dots_m(:,r) = dots_m(:,r)+displacements;
else
    displacements = [];
end


% change world position of dot corresponding to original deviation position
dots_m(:,dev1) = dots_m(:,dev1)+displacement; %deviate in real world before calculating redrawing after translation and rotation

% update dots after translation
dots_m_next = dots_m - translate';


% and rotation

gaze_angle = atand(translate(2)/(z0-translate(3)));
gaze_angle = atand(translate(2)/(z0));
gaze_angle = double(gaze_angle);

gazeRotation = [1, 0, 0; 0, cosd(-gaze_angle), -sind(-gaze_angle); 0, sind(-gaze_angle), cosd(-gaze_angle)]; % rotate in opposite way of gaze_angle to convert to eye centered coordinates
gazeRotation = double(gazeRotation);
dots_m_next = gazeRotation*dots_m_next;


% calculate new on screen positions
dots_step(:,:,2) = view_dist*dots_m_next(1:2,:)./dots_m_next(3,:);

% calculate screen velocity
velocity_field = dots_step(:,:,2)-dots_step(:,:,1);
