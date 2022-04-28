function  [cond]= get_conditions(translation, depth_structure, devPos, weberFrac)

if nargin==0
    translation = [0, .05, 1.4];
    depth_structure = 2;
    devPos = [5;3];
    weberFrac = 2;
end


    %-----Rig Settings----------------------
    view_dist                       = .35;                                  % m .57; psychophysics room: .35 .50
    screensize                      = [.405 .298];                          % m psychophysics room: [.405 .298], home: [.495 .312] [.697 .392]
    pixels                          = [1600 1200];                          % psychophysics room: [1600 1200], home: [1920 1200]

    view_window                     = round([rad2deg(atan(screensize(1)/2/view_dist)) rad2deg(atan(screensize(2)/2/view_dist))]);  % X,Y centered around fixation [60 46] [54 40];                                                                      % [36 27] for laptop, [55 32] for monitor
    scale_factor                    = sqrt((view_window(1)*60*view_window(2)*60)/(pixels(1)*pixels(2)))*1;          % Arcmin/pixel 1.78 2.25 1.92dell: 1680x1050, psychophysics room: 1600x1200                                % Screen frame rate (hz) psychophysics room: 85
    scale_factorX = view_window(1)*60/pixels(1);
    scale_factorY = view_window(2)*60/pixels(2);

    %-----Trial Settings------------------                                                                    % deg/s *up is down and down is up 
    theta                           = [0 45 90 135 180 225 270 315 .00001 180.00001];        % range directions to test around base velocity, last two correspond to on constraint
    
    % how to calculate velocities based on scene structure
    switch depth_structure
        case 0 
        case 1
        case 2
            img_label = 'lRange016.mat';
            load (img_label)
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
            mirror = [range_distance_left, flip(range_distance_left,2)]; %mirror left side with right so choosing deviation on left and right is symmetric
            range_distance = mirror; 
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

%           observerTrajectory = repmat(translation/frame_rate, [nframes, 1]); %trajectory over one frame
            translate = (gazeRotation*translation')';
            translation = translate;

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
    
    dot_density                     = 1;                                    % original settings: 1; 1/dot_density = average degrees between apertures measured from the centers

    %-----Experiment Settings, Don't Change---
    %-----Array Settings
    z0                              = 3;                                    % fixation distance
    middle_dist                     = 3; 
    switch depth_structure
        case 0 
            z0                              = 3;                                    % fixation distance
            middle_dist                     = 3; 
        case 1
            z0 = ds.height./cos(deg2rad(ds.gaze_angle));
            middle_dist = z0;
        case 2
            center  = ceil(pixels/2);
            z0 = range_distance(center(2), center(1));
            
        case 3
            coordinate = dsearchn([X(:,1) Y(:,1)],[0,0]);
            z0 = depths(coordinate, 1);

    end                                  %meters
    cloud_dist                      = [middle_dist*.5, middle_dist*1.5];  % m range of distances   

    %-----Staircase Settings
    n_blocks                        = 1;                                    % 4 blocks with pause inbetween
    n_staircases                    = 1;                                    % one staircase per block
    stairFrac                       = 10;                                   % fraction of distance between middle velcity and limit velocity

    n_total_conditions = 0;
    for b = 1:n_blocks
        for j=1:size(translation,1)
            for l = 1:length(theta)
                for k=1:n_staircases
                    n_total_conditions = n_total_conditions+1;
                    cond(n_total_conditions).rotation = theta(l);
                    cond(n_total_conditions).translate = translation(j,:);

                     cond(n_total_conditions).steps = zeros(2,stairFrac); % list of possible velocities for the staircase

                end
            end
        end
    end
    % calculate all possible velocities for the deviations and save in
    % condition struct
    velocity_steps = zeros(2*size(translation,1), stairFrac);

%     angle_btwn_constraints = zeros(2,size(translation,1));
    
    % get velocity steps and set 10% random trials to easy and hard
    
    for c = 1:length(cond)
        depth_limits   = cloud_dist;                           % range of depths at the position in the visual field. in case of wall, same everywhere

        switch depth_structure
            case 0 
                
            case 1 % if depth structure ground plane, calculate distnce based on deviation positon
            % trying different height above/below ground plane

            case 2 % if from natural depth map
                devPosPix = [devPos(1)*60*scale_factorX; devPos(2)*60*scale_factorY];
                devPosPix = round(center' + devPosPix);
                middle_dist = range_distance(devPosPix(2), devPosPix(1));
                surround_extent= round([scale_factorX*60*dot_density scale_factorY*60*dot_density]); % find extent based on dot_density, 1/dot_density = deg between patches
                surround_window = range_distance(devPosPix(2)-surround_extent(2):min(pixels(2),devPosPix(2)+surround_extent(2)), devPosPix(1)-surround_extent(1):devPosPix(1)+surround_extent(1));
            
                surround_distance = [min(min(surround_window)), max(max(surround_window))];
                depth_limits = [max(.5, surround_distance(1)), surround_distance(2)]; % take average of depths in window ~2 deg around (based on dot density .5)
            
            case 3
                coordinate = dsearchn([X(:,1) Y(:,1)],devPos');
                middle_dist = depths(coordinate,1); % assign depth based on defined world
%                 middle_dist = dots(coordinate,1); % assign depth based on defined world
                
                depth_limits =  [middle_dist-2, middle_dist+2]; 
        end

        end_vel = calculate_cloud_flow_screen(middle_dist, devPos, cond(c).translate, view_dist, z0);
%         start_vel = end_vel;
        start_vel = end_vel*(1+weberFrac);
        
        if cond(c).rotation == theta(end-1) || cond(c).rotation == theta(end) % get steps along constraint

            closer_vel = calculate_cloud_flow_screen(depth_limits(:,1), devPos, cond(c).translate, view_dist, z0);
            unit_dev = (closer_vel-end_vel)./vecnorm(closer_vel-end_vel);
            delta_speed = weberFrac*vecnorm(end_vel);
            start_vel = end_vel+delta_speed.*unit_dev;
            %change range so that sampling uniformly in both conditions in
            %velocity space, in each direction a deviation with magnitude but
            %still along deviation line
            %3deg/s
        end
        
        % get steps
        segment = end_vel-start_vel; %vector describing segment between starting velocity, and middle velocity

        steps = [linspace(0, segment(1), stairFrac); linspace(0, segment(2), stairFrac)]; %amount to move along segment
        velocity_steps = start_vel+steps;
        
        
        % rotate by theta
        velocity_steps = get_theta_off_constraint(velocity_steps, end_vel, cond(c).rotation);
        
        cond(c).steps = velocity_steps;
    end
    