%% full flow experiment, natural scene
% updated HL 06/23/2021
% clear all;
% close all;
clc;
addpath(genpath('/Applications/Psychtoolbox'))
addpath('/Users/hopelutwak/Documents/MATLAB/VisTools/')
% addpath('/Volvumes/GoogleDrive/My Drive/opticflow/objectDetection/RangeDatabase1080p')

%vvv
   %-----Subject Settings------------------
    initials                        = 'HL';                                 % Subject initials used to save file
    session                         = 'test';
    feedback                        = 1;                                    % 1 = gives feedback after each trial (training), 0 = no feedback (actual experiment)

    %-----Rig Settings----------------------
    mouseclick                      = 1;                                    % use mouseclick instead of keys
    view_dist                       = .35;                                  % m .57; psychophysics room: .35 .50
    screensize                      = [.405 .298];                          % m psychophysics room: [.405 .298], home: [.495 .312] [.697 .392]
    pixels                          = [1600 1200];                          % psychophysics room: [1600 1200], home: [1920 1200]
    frame_rate                      = 30;                                   % Hz 85 , 60 at hhome

    view_window                     = round([rad2deg(atan(screensize(1)/2/view_dist)) rad2deg(atan(screensize(2)/2/view_dist))]);  % X,Y centered around fixation [60 46] [54 40];                                                                      % [36 27] for laptop, [55 32] for monitor
    scale_factor                    = sqrt((view_window(1)*60*view_window(2)*60)/(pixels(1)*pixels(2)))*1;          % Arcmin/pixel 1.78 2.25 1.92dell: 1680x1050, psychophysics room: 1600x1200                                % Screen frame rate (hz) psychophysics room: 85
    scale_factorX = view_window(1)*60/pixels(1);
    scale_factorY = view_window(2)*60/pixels(2);
    linearize                       = 0;                                    % Use calibrated LUT (do this when available)                    

    %-----Trial Settings------------------ vvvvvvv
    subjectNumber                   = '01';                                 % Number used to save eyelink file
    n_trials                        = 20+.2*20;                             % Number of trials n+.2*n so +10% hard and +10% easy trials 20+.2*20;
    n_anchors                       = n_trials/1.2*.2;                                % n_trials/1.2*.2;
    translation                     = [0, .05, 1.4];                            % m/s [0, .05, 1.4]
%                                                                         % deg/s *up is down and down is up
    weberFrac                       = .5;                                    % magnitude deviation vector deg/s
    scramble                        = 0;                                    % scramble surround with optic flow info, 0 is same velociy in surround
    depth_structure                 = 2;                                    % 0 = random cloud, 1 = ground plane, 2 = natural scene , 3 = defined 3D scene
    ds                              = [];
    
    stim_type                       = 1;                                    % 0 = 1/f noise, 1 = gratings
    clusterDev                      = 0;                                    % 0 = one patch, 1 = cross shape;
    
    theta                           = [0 45 90 135 180 225 270 315 .00001 180.00001];        % range directions to test around base velocity, last two correspond to on constraint
%      theta                           = [0 90];
    fixateMovement                  = 0;                                    % 1 = red fixation dot moves, 0 = stationary red fixation dot
    eyeTracking                     = 0;                                    % 1 = eye tracking on, 0 = eye tracking off
    stimulus_duration               = .5;                                   % seconds
    test                            = 1;                                    % just shows one stim
    if test
        stimulus_duration = 1;
%         theta             = [0 180];
    end
    
    linearize                       = 0;                                    % Use calibrated LUT (do this when available)
    data_path                       = './data/';

    % how to calculate velocities based on scene structure
    switch depth_structure
        case 0 
        case 1
            
            ds.gaze_angle               = 70;                                   %deg from staring straight at ground
                if (ds.gaze_angle + view_window(2)/2) >=90
                    error('gaze angle too high!')
                end
            translation                 = getT_eyecoords(90-ds.gaze_angle, translation);
            ds.height                   = 1.65;                                 % ds.height in meters
            ds.bumpiness                   = 0.001;                               % random noise in ground plane
            ds.wall                        = 0;                                 % ground plane + wall starting in upper visual field
            ds.planeParallax               = 1;
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

    dot_density                     = 1;                                    % original settings: 1; 1/dot_density = average degrees between apertures measured from the centers
    jitter                          = .5;                                   % deg to jitter by
    numSame                         = 5;                                    % number of apertures that have same texture
    cut                             = 3;                                    % cut off freq 1:4 (depends on speed, larger speed have lower cyc/deg corresponding to freq 1
    
    %-----Staircase Settings
    n_blocks                        = 4;                                    % 4 blocks with pause inbetween
    n_staircases                    = 1;                                    % one staircase per block
    stairFrac                       = 10;                                   % fraction of distance between middle velcity and limit velocity

    %-----Element Settings
    text_diam                       = 30;                                   % Arcmin original settings: 30
    gabor_sf                        = 4;                                    % c/deg   original settings: 4
    gabor_contrast                  = 50;                                  % percent, for the pattern; each element will be half this
    
    bps                             = text_diam+1;                          % so the moving texture is centered
    spatial_envelope                = 2;                                    % 0 = disk, 1 = Gabor, 2 = raised cosine
%     devPos                          = [randi(view_window(1)/2);randi((view_window(2)-1)/2)];        
    devPos                          = [5; 3]; % position of deviation (y-axis flipped when displaying)
        switch clusterDev
            case 0 
                cluster = [];
                mirrorcluster = [];
            case 1
                cluster                         = [devPos(1)+.5/dot_density, devPos(1)-.5/dot_density devPos(1), devPos(1); devPos(2), devPos(2), devPos(2)+.5/dot_density, devPos(2)-.5/dot_density];
                mirrorcluster = [-cluster(1,:); cluster(2,:)];
        end
    nDevs                           = size([devPos cluster],2);
%     cluster_window                  = [1.5/dot_density 1.5/dot_density]; % size of cluster window in visual angle


    exclude                         = [-1, -1, 1, 1];                       % Don't place elements where the FOE will be

    H_ecc_fix                       = 0;                                    % Horizontal fixation ecc (degs, neg is left)
    V_ecc_fix                       = 0;                                    % Vertical fixation ecc (degs, neg is up)

    experiment_id                   = 'object_detection';                   % Used in group filename
    fast                            = 1;                                    % Automatically trigger trials
    ITI                             = .25;                                    % Intertrial Inverval, Seconds
    background                      = 127;                                  % Grayscale Units
    fixate                          = 1;                                    % Present fixation spot during motion

    %-----Housekeeping----------------------v
    % Scale things based on viewing distance, and convert other stuff to 
    % the units Psychtoolbox wants...
    tme                             = clock;
    logname = strcat(data_path,initials,'_',experiment_id, '_', date);

    text_pixelW                     = ceil(text_diam/scale_factor); %in pixels
    H_ecc_fix                       = H_ecc_fix*60/scale_factorX;
    V_ecc_fix                       = V_ecc_fix*60/scale_factorY;
    mv_length                       = ceil(stimulus_duration*frame_rate);
    
    stimulus_radius                 = round(text_pixelW/2);
    f                               = (gabor_sf*scale_factor/60)*2*pi;
    angle                           = 0;
    a                               = cos(angle)*f;
    b                               = sin(angle)*f;
    amplitude                       = background;

    %-----Spatial Envelope------------------
    [x,y]=meshgrid(-stimulus_radius:stimulus_radius,-stimulus_radius:stimulus_radius);
    bps = (stimulus_radius)*2+1;
    circle=((stimulus_radius)^2-(x.^2+y.^2));
    for ii=1:bps; for j =1:bps; if circle(ii,j) < 0; circle(ii,j) = 0; else circle(ii,j) = 1; end; end;
    end;
    if spatial_envelope == 1
        circle = 255*(1-ggaus(text_pixelW,.25*text_pixelW));
    elseif spatial_envelope == 2
        if stim_type == 1
            R = (sqrt(x.^2 + y.^2) + eps).*circle;
            R = R/max(max(R));
            cos2D = (cos(R*pi)+1)/2;
            circle = (cos2D.*circle);
            circle = circle*255/2;
        else
            circle = 255*(1-coswin(text_pixelW+1,.3*text_pixelW,.5*text_pixelW));
        end
    end

    %-----Open Screens----------------------
    Screen('Preference', 'SkipSyncTests', 1);
    oldVisualDebugLevel = Screen('Preference', 'VisualDebugLevel', 3);
    oldSupressAllWarnings = Screen('Preference', 'SuppressAllWarnings', 1);
    screens=Screen('Screens');
    screenNumber=max(screens);

    w=Screen('OpenWindow',screenNumber,background,[],[],2);
    Screen('BlendFunction', w, GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    screen_rect = Screen('Rect',w);

    Screen('FillRect',w, background);
    Screen('Flip', w);
    Screen('FillRect',w, background);
    Screen('TextSize',w,20);

    if linearize
        calibration = load('0003_alta_141007.mat');
        table = calibration.calib.table;
        psychtoolbox_calib = repmat(table, 1, 3);
        Screen('LoadNormalizedGammaTable',screenNumber,psychtoolbox_calib);
    end 
    %-----Screen Landmarks------------
    sr_hor = round(screen_rect(3)/2); % Middle of the screen, horizontally, in pixels
    sr_ver = round(screen_rect(4)/2); % Middle of the screen, vertically, in pixels
    fix_hor = sr_hor+H_ecc_fix;     % Horizontal location of fixation cross, in pixels
    fix_ver = sr_ver+V_ecc_fix;     % Vertical location of fixation cross, in pixels
    movie_rect= [0,0,bps,bps];
%     
    %-----Make the grating in all phases-----
    gabor = zeros(1,360);
    for ii=1:360
        grating = round(((sin(a*x+b*y+ii*pi/180)*amplitude)+background));
        gabor(ii) = Screen('MakeTexture',w,cat(3,grating, circle));
    end

    n_total_conditions = 0;
    for b = 1:n_blocks
        for j=1:size(translation,1)
            for l = 1:length(theta)
                for k=1:n_staircases
                    n_total_conditions = n_total_conditions+1;
                    cond(n_total_conditions).block = b;
                    
                    cond(n_total_conditions).rotation = theta(l);
                    cond(n_total_conditions).LR = round(rand(1,n_trials)); %will the deviation be on the left or right, L=0, R=1
                    cond(n_total_conditions).translate = translation(j,:);

                    cond(n_total_conditions).count = 0; %what trial are we on
                    cond(n_total_conditions).resp_history = zeros(1,n_trials); % whether you were right or wrong
                    cond(n_total_conditions).n_correct = 0; % number of correct in a row - keeps track to change staircase
                    cond(n_total_conditions).n_flips = 0;

                    cond(n_total_conditions).anchors = []; %indices of interspersed hard/easy trials

                    cond(n_total_conditions).step_idx = 1; % difficulty level (which velocity to choose)
                    cond(n_total_conditions).step_history = NaN(1, n_trials); % to display staircase
                    cond(n_total_conditions).steps = zeros(2,stairFrac,nDevs); % list of possible velocities for the staircase
                    cond(n_total_conditions).dev_history = zeros(2,n_trials,nDevs); % what deviation vector was it

                    cond(n_total_conditions).velocity_range = zeros(2,2,nDevs); %range of possible velocities base on range of depths
                    cond(n_total_conditions).z0 = z0; %save fixation depth

                end
            end
        end
    end
    % calculate all possible velocities for the deviations and save in
    % condition struct
    velocity_steps = zeros(2*size(translation,1), stairFrac, nDevs);
    velocity_steps_off = zeros(size(velocity_steps));
    velocity_range = zeros(2, 2);
    velocity_range_off = zeros(2*size(translation,1), 2);
%     angle_btwn_constraints = zeros(2,size(translation,1));
    
    % get velocity steps and set 10% random trials to easy and hard
    
    for c = 1:length(cond)
        depth_limits   = cloud_dist;                           % range of depths at the position in the visual field. in case of wall, same everywhere

        switch depth_structure
            case 0 
                
            case 1 % if depth structure ground plane, calculate distnce based on deviation positon
            % trying different height above/below ground plane
                middle_dist = raytrace2eyeZ(devPos, ds.height, ds.gaze_angle);
                depth_limits = [middle_dist-ds.bumpiness, middle_dist+ds.bumpiness];
            case 2 % if from natural depth map
                devPosPix = [devPos(1)*60*scale_factorX; devPos(2)*60*scale_factorY];
                devPosPix = round(center' + devPosPix);
                middle_dist = range_distance(devPosPix(2), devPosPix(1));
                surround_extent= round([scale_factorX*60*dot_density scale_factorY*60*dot_density]); % find extent based on dot_density, 1/dot_density = deg between patches
                surround_window = range_distance(devPosPix(2)-surround_extent(2):min(pixels(2),devPosPix(2)+surround_extent(2)), devPosPix(1)-surround_extent(1):devPosPix(1)+surround_extent(1));
            
                surround_distance = [min(min(surround_window)), max(max(surround_window))];
                depth_limits = [max(.5, surround_distance(1)), surround_distance(2)]; % take average of depths in window ~2 deg around (based on dot density .5)
            
                %isolate tree
                tree = find(range_distance<range_distance(devPosPix(2), devPosPix(1))+1 & range_distance(devPosPix(2), devPosPix(1))-1);
%                 range_distance(tree) = 300;
            case 3
                coordinate = dsearchn([X(:,1) Y(:,1)],[devPos'; cluster']);
                middle_dist = depths(coordinate,1); % assign depth based on defined world
%                 middle_dist = dots(coordinate,1); % assign depth based on defined world
                
                depth_limits =  [middle_dist-2, middle_dist+2]; 
        end

        end_vel = calculate_cloud_flow(middle_dist, [devPos cluster], cond(c).translate, view_dist, z0);
%         start_vel = end_vel;
        start_vel = end_vel*(1+weberFrac);
        
        if cond(c).rotation == theta(end-1) || cond(c).rotation == theta(end) % get steps along constraint

            closer_vel = calculate_cloud_flow(depth_limits(:,1), [devPos cluster], cond(c).translate, view_dist, z0);
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
        devs = [devPos cluster];
        for d = 1:nDevs
            steps = [linspace(0, segment(1,d), stairFrac); linspace(0, segment(2,d), stairFrac)]; %amount to move along segment
            velocity_steps(:,:,d) = start_vel(:,d)+steps;
        
        
            % rotate by theta
            velocity_steps(:,:,d) = get_theta_off_constraint(velocity_steps(:,:,d), end_vel(:,d), cond(c).rotation);

            cond(c).steps(:,:,d) = velocity_steps(:,:,d);
            
            % velocity range
            velocity_range(:,1) = calculate_cloud_flow(depth_limits(d,1), devs(:,d), cond(c).translate, view_dist, z0);
            velocity_range(:,2) = calculate_cloud_flow(depth_limits(d,2), devs(:,d), cond(c).translate, view_dist, z0);
            cond(c).velocity_range(:,:,d) = velocity_range;
        end

        
        %randomly assign some hard/easy trials in second half of exp
        idxanchor = randsample(n_trials/2:n_trials, n_anchors);
        cond(c).anchors = idxanchor;
        cond(c).step_history(idxanchor(:,1:n_anchors/2)) = 1;
        cond(c).step_history(idxanchor(:,n_anchors/2+1:end)) = stairFrac;
    end
    
    
    %----Start the experiment-----
    
    Screen('DrawText',w,'A velocity deviation will occur on the left or the right',fix_hor-200, fix_ver-30,250);
    Screen('DrawText',w,'Use Left/Right Arrows to report which side the deviation occured',fix_hor-250, fix_ver,250);
    Screen('DrawText',w,'press V to start',fix_hor-50, fix_ver+30,250);
    Screen('Flip',w);

    if mouseclick
        validKey = 0;
        while ~validKey
            [clicks,x,y,whichButton] = GetClicks();
            find(clicks)
            if clicks >= 1
                validKey = 1;
            end
        end
    else
        FlushEvents('keyDown');
        validKey = 0;
        while ~validKey
            [secs, keyCode, deltaSecs] = KbWait();
            find(keyCode)
            if keyCode(KbName('v'))
                validKey = 1;
            end
        end
    end

    Screen('FillRect', w, background);
    Screen('Flip', w);
    tic;

    if test
        n_blocks = 1;
        trials_per_block = 1;
    else
    end
    
for b = 1:n_blocks
    
    if b>1
        %pause after first block
        Screen('DrawText',w,'Break',fix_hor-30, fix_ver+20,250);
        Screen('DrawText',w,'press V to start again',fix_hor-100, fix_ver+40,250);
        Screen('Flip',w);

        if mouseclick
            validKey = 0;
            while ~validKey
                [clicks,x,y,whichButton] = GetClicks();
                find(clicks)
                if clicks >= 1
                    validKey = 1;
                end
            end
        else
            FlushEvents('keyDown');
            validKey = 0;
            while ~validKey
                [secs, keyCode, deltaSecs] = KbWait();
                find(keyCode)
                if keyCode(KbName('v'))
                    validKey = 1;
                end
            end
        end

        Screen('FillRect', w, background);
        Screen('Flip', w);
        tic;
    end
    %-----Randomize Staircases------------
    n_conditions = n_total_conditions/n_blocks;
    if ~test
        trials_per_block = n_trials*n_conditions;
    end
    perm = randperm(trials_per_block);
    perm = mod(perm,n_conditions)+1;
    duration_check = zeros(size(perm));
    
    for trial = 1:trials_per_block
        aa = GetSecs; 

        % Draw the white fixation cross
        Screen('FillRect',w, background);
        Screen('DrawDots', w, [fix_hor; fix_ver], 4, 250, [], 2); 
        Screen('Flip',w);
        
        if test
            perm(trial) = 3;
            cond((perm(trial))).LR(1) = 1;
        end

        cond(perm(trial)+(b-1)*n_conditions).count = cond(perm(trial)+(b-1)*n_conditions).count+1;
       
        
        % Set up the world

        translate = cond(perm(trial)+(b-1)*n_conditions).translate;
        switch depth_structure
            case 0
                [dots_m, dots_deg, dev0, dev1] = make_dot_cloud(dot_density, jitter, cloud_dist, view_window, text_diam, exclude, devPos);
            case 1
                [dots_m, dots_deg, dev0, dev1] = make_dot_plane(dot_density, jitter, ds.bumpiness,ds.height, ds.gaze_angle, ds.wall, view_window, text_diam, exclude, devPos);
            
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
                if clusterDev 
                    cluster0 = (length(drawn_dots_deg)-2*size(cluster,2)+1):(length(drawn_dots_deg)-size(cluster,2));  %get indices for clusters on left and right
                    cluster1 = (length(drawn_dots_deg)-size(cluster,2)+1):length(drawn_dots_deg);
                else
                    cluster0 = [];
                    cluster1 = [];
                end
                
            case 3
                [dots_m, dots_deg, dev0, dev1] = make_dot_cloud(dot_density, jitter, cloud_dist, view_window, text_diam, exclude, devPos);
                
                dots_deg = [dots_deg mirrorcluster cluster];
                nearestidx = knnsearch([X(:,1) Y(:,1)],dots_deg');
                distances = vecnorm(dots_deg- [X(nearestidx,1)'; Y(nearestidx,1)']);
                nothingidx = find(distances>.1);
                drawn_dots_deg = dots_deg;
                drawn_dots_deg(:,nothingidx) = [];
                
                dev0 = dsearchn(drawn_dots_deg', dots_deg(:,dev0)'); % get new indicecs for deviation to match new set of dots
                dev1 = dsearchn(drawn_dots_deg', dots_deg(:,dev1)');
                if clusterDev 
                    cluster0 = (length(drawn_dots_deg)-2*size(cluster,2)+1):(length(drawn_dots_deg)-size(cluster,2));  %get indices for clusters on left and right
                    cluster1 = (length(drawn_dots_deg)-size(cluster,2)+1):length(drawn_dots_deg);
                else
                    cluster0 = [];
                    cluster1 = [];
                end
                
                dots_m = [dots_m zeros(3, size(cluster, 2)*2)];
                dots_m(3,:) = depths(nearestidx,1)';% assign depths based on defined world
                dots_m(:,nothingidx) = []; %get rid of points that won't show up
                
                dots_deg = drawn_dots_deg; % update dots_deg
                
        end

        velocity_field = calculate_cloud_flow(dots_m(3,:), dots_deg, translate, view_dist, z0);
        
        % if it's one of the anchors ignore the current step_idx
        if sum(cond(perm(trial)+(b-1)*n_conditions).count == cond(perm(trial)+(b-1)*n_conditions).anchors)
            step = cond(perm(trial)+(b-1)*n_conditions).step_history(cond(perm(trial)+(b-1)*n_conditions).count);
        else
            step = cond(perm(trial)+(b-1)*n_conditions).step_idx; %otherwise continue with the staircase
        end
        
        if cond(perm(trial)+(b-1)*n_conditions).LR(cond(perm(trial)+(b-1)*n_conditions).count) %if it's on the right
            idx = [dev1 cluster1];
            if cluster
                for d = 1:nDevs
                    velocity_field(:,idx(d)) = cond(perm(trial)+(b-1)*n_conditions).steps(:,step,d);
                end
            else
                velocity_field(:,idx) = cond(perm(trial)+(b-1)*n_conditions).steps(:,step);
            end
            
            
        else % if it's on the left
            idx = [dev0 cluster0];
            if cluster
                for d = 1:nDevs
                    velocity_field(:,idx(d)) = [-cond(perm(trial)+(b-1)*n_conditions).steps(1,step,d); cond(perm(trial)+(b-1)*n_conditions).steps(2,step,d)];
                end
            else
                velocity_field(:,idx) = [-cond(perm(trial)+(b-1)*n_conditions).steps(1,step); cond(perm(trial)+(b-1)*n_conditions).steps(2,step)];
            end
            
        end

        %pixels/s
        velocity_field_pix = velocity_field*60*1/scale_factor;
        %convert to pixels/frame
        velocity_field_ppf = velocity_field_pix/frame_rate;
        
        switch stim_type
            case 0
                %-----1/f texture-----------------------
                sz = max(vecnorm(velocity_field_pix))*2*stimulus_duration + text_pixelW;

                numDrifts = ceil(length(velocity_field)/numSame);
                tex = zeros(1,numDrifts);
                for d = 1:numDrifts
                    drift = oneoverfcut(1, ceil(sz), ceil(sz), cut);
                    tex(d)=Screen('MakeTexture', w, 255*drift);
                end
                drift_center = [size(drift,1)/2, size(drift,1)/2];
                which_tex = randi(numDrifts, 1,length(velocity_field)); %assign one of drifts to each patch for the movie

                windowTex = Screen('MakeTexture',w,cat(3,128*ones(text_pixelW),255*(1-coswin(text_pixelW,.3*text_pixelW,.5*text_pixelW))));
                
                % center texture patches
                pos0 = CenterRectOnPoint(movie_rect, drift_center(1), drift_center(1));
                pos = CenterRectOnPoint(movie_rect,dots_deg(1,:)'*60/scale_factorX+sr_hor,dots_deg(2,:)'*60/scale_factorY+sr_ver)'; %big Y numbers are lower on the screen in pixels so we have to flip

                source = cell(1,length(stimulus_duration*frame_rate));
                for tt = 1:ceil(stimulus_duration*frame_rate)
                    for s = 1:length(velocity_field)
                        source{tt}(:,s) = [pos0(1)-(tt-1)*velocity_field_ppf(1,s); pos0(2)-(tt-1)*velocity_field_ppf(2,s); ...
                            pos0(3)-(tt-1)*velocity_field_ppf(1,s); pos0(4)-(tt-1)*velocity_field_ppf(2,s)];
                    end
                end
                
            case 1
                %-------Gabors----------------------------
                % convert speeds to phase change
                posA = CenterRectOnPoint(movie_rect,dots_deg(1,:)'*60/scale_factorX+sr_hor,dots_deg(2,:)'*60/scale_factorY+sr_ver)'; %big Y numbers are lower on the screen in pixels so we have to flip
                pos = [posA posA];
                
                speed_ver = velocity_field(2,:);
                speed_hor = velocity_field(1,:);

                angle_ver = 270*(speed_ver>=0)+90*(speed_ver<0); % in deg  
                angle_hor = 180*(speed_hor>0); % in deg
                speed_ver = abs(velocity_field(2,:));
                speed_hor = abs(velocity_field(1,:));

                angle_all = [];
                speed_all = [];
                pos_all = [];
        
                speed = zeros(1,length(speed_ver)*2); 
                angle = speed;
                speed = [speed_hor speed_ver];
                angle = [angle_hor angle_ver];
                angle_all = [angle_all angle];
                speed_all = [speed_all speed];
                pos_all = [pos_all pos];

                angle = angle_all;   
                speed = speed_all; 
                pos = pos_all;
                phase_step = round(speed*gabor_sf*360/frame_rate)';

                phases = repmat(ceil(rand(size(speed,2),1)*360),1,mv_length)+repmat(phase_step,1,mv_length).*repmat(0:(mv_length-1),size(speed,2),1);
                phases = rem(phases,360);
                phases = phases + (phases==0)*360;
        end
        
        %Finish the ITI
        WaitSecs(ITI-(GetSecs-aa));

        % Draw the red fixation cross 
        Screen('DrawDots', w, [fix_hor; fix_ver], 4, [250 0 0], [], 2);
        Screen('Flip',w);

    %     FlushEvents('keyDown'); 
    %     priorityLevel=MaxPriority(w); 
    %     Priority(priorityLevel);

        % Play the movie
        
        StimulusOnsetTime = zeros(1,mv_length);
        aa = GetSecs;
        tag = clock;

        switch stim_type
            case 0 
                for frame = 1:mv_length 
                    Screen('DrawTextures', w, tex(which_tex),source{frame},pos, 0);
                    Screen('DrawTextures', w, windowTex, [], pos, 0); 
                    if fixate 
                        Screen('DrawDots', w, [fix_hor; fix_ver], 4, [250 0 0], [], 2);
                    end
        %             [VBLTimestamp, StimulusOnsetTime(frame), FlipTimestamp,
        %             Missed, Beampos] = Screen('Flip',w);
                    Screen('Flip', w);
                end
            case 1
                for frame = 1:mv_length
                    Screen('DrawTextures', w, gabor(phases(:,frame)),movie_rect,pos,angle);
                    if fixate
                        Screen('DrawDots', w, [fix_hor; fix_ver], 4, [250 0 0], [], 2);
                    end
                    [VBLTimestamp, StimulusOnsetTime(frame), FlipTimestamp, Missed, Beampos] = Screen('Flip',w);
                end
        end
        duration_check(trial) = GetSecs-aa;
        Screen('FillRect',w, background);
        if fixate
            Screen('DrawDots', w, [fix_hor; fix_ver], 4, [250 0 0], [], 2);
        end
        Screen('Flip',w); 

        if mouseclick
            validKey = 0;
            while ~validKey
                [clicks,x,y,whichButton] = GetClicks([],[0],[]);
        %         key = KbName();
        %         if key(1) == 'E'
        %             Screen('CloseAll');
        %             Screen('Preference', 'VisualDebugLevel', oldVisualDebugLevel);
        %             Screen('Preference', 'SuppressAllWarnings', oldSupressAllWarnings);
        %             ListenChar(1);
        %             ShowCursor;
        %             break
                if whichButton == 1 
                    validKey = 1;
                    resp = 0;
                elseif whichButton == 2
                    validKey = 1;
                    resp = 1;
                else
                    Beeper('low');
                end
            % Provides feedback during training phase
                if feedback
                    if cond(perm(trial)+(b-1)*n_conditions).LR(cond(perm(trial)+(b-1)*n_conditions).count) == resp 
                        Screen('DrawText',w,'Correct',fix_hor-30, fix_ver+20,250);
                        Screen('Flip',w)
                        WaitSecs(ITI)
                    else
                        Screen('DrawText',w,'Incorrect',fix_hor-30, fix_ver+20,250);
                        Screen('Flip',w)
                        WaitSecs(ITI)  
                    end
                end
            end
        else
     %     Get the response
            validKey = 0;
            while ~validKey
                [secs, keyCode, deltaSecs] = KbWait();
                if keyCode(KbName('ESCAPE'))
                    Screen('CloseAll');
                    Screen('Preference', 'VisualDebugLevel', oldVisualDebugLevel);
                    Screen('Preference', 'SuppressAllWarnings', oldSupressAllWarnings);
                    ListenChar(1);
                    ShowCursor;
                    break
                elseif keyCode(KbName('LeftArrow'))
                    validKey = 1;
                    resp = 0;
                elseif keyCode(KbName('RightArrow')) 
                    validKey = 1;
                    resp = 1;
                else
                    Beeper('low');
                end
%            Provides feedback during training phase
                if feedback
                    if cond(perm(trial)+(b-1)*n_conditions).LR(cond(perm(trial)+(b-1)*n_conditions).count) == resp 
                        Screen('DrawText',w,'Correct',fix_hor-30, fix_ver+20,250);
                        Screen('Flip',w)
                        WaitSecs(ITI)
                    else
                        Screen('DrawText',w,'Incorrect',fix_hor-30, fix_ver+20,250);
                        Screen('Flip',w)
                        WaitSecs(ITI)  
                    end
                end
            end
        end
    %     Priority(0); 
    %     % Log the trial
    %     if ~trainingMode
    %         fid = fopen(logname,'a');
    %         fprintf(fid,'%s, %s, %d, %d, %d, %d, %d, %f, %d, %f, %f, %f , %d, %d, %d, %f,%f, %f %f\n', ...
    %                 initials,experiment_id, tag(1), tag(2), tag(3), tag(4), tag(5), tag(6), ...
    %                 trial, cond(perm(trial)+(b-1)*n_conditions).translate(1), cond(perm(trial)+(b-1)*n_conditions).translate(2), cond(perm(trial)+(b-1)*n_conditions).translate(3), ...
    %                 cond(perm(trial)+(b-1)*n_conditions).dev_index, cond(perm(trial)+(b-1)*n_conditions).LR(cond(perm(trial)+(b-1)*n_conditions).count), resp, percentSpeed, diffAngle, ...
    %                 stimulus_duration, duration_check(trial));        
    %         fclose(fid); 
    %     end

    % Tell the staircase what happened
    %report just rhs staircase
    
    cond(perm(trial)+(b-1)*n_conditions).dev_history(:,cond(perm(trial)+(b-1)*n_conditions).count) = cond(perm(trial)+(b-1)*n_conditions).steps(:,step);
    cond(perm(trial)+(b-1)*n_conditions).step_history(cond(perm(trial)+(b-1)*n_conditions).count) = step;
    
%     % percent speed
%     cond(perm(trial)+(b-1)*n_conditions).percentSpeed(cond(perm(trial)+(b-1)*n_conditions).count) = norm(cond(perm(trial)+(b-1)*n_conditions).steps(:,cond(perm(trial)+(b-1)*n_conditions).step_idx))/norm(cond(:,perm(trial)).steps(:,end));
%     
%     % angle difference
%     unitDev = cond(perm(trial)+(b-1)*n_conditions).steps(:,cond(perm(trial)+(b-1)*n_conditions).step_idx)/norm(cond(perm(trial)+(b-1)*n_conditions).steps(:,cond(perm(trial)+(b-1)*n_conditions).step_idx));
%     unitMiddle = cond(perm(trial)+(b-1)*n_conditions).steps(:,end)/norm(cond(perm(trial)+(b-1)*n_conditions).steps(:,end));
%     cond(perm(trial)+(b-1)*n_conditions).dev_angle_history(cond(perm(trial)+(b-1)*n_conditions).count) = abs(rad2deg(acos(unitDev'*unitMiddle)));
%     
    if sum(cond(perm(trial)+(b-1)*n_conditions).count == cond(perm(trial)+(b-1)*n_conditions).anchors) % if an anchor trial just report correct/incorrect, don't change step_idx
        if cond(perm(trial)+(b-1)*n_conditions).LR(cond(perm(trial)+(b-1)*n_conditions).count) == resp
            cond(perm(trial)+(b-1)*n_conditions).resp_history(cond(perm(trial)+(b-1)*n_conditions).count) = 1;
        else
        end
    else %otherwise carry on with staircasing
        %1 up 2 down staircase proceducre
        if cond(perm(trial)+(b-1)*n_conditions).n_flips < 1 % if they haven't flipped yet
            % if they're correct
            if cond(perm(trial)+(b-1)*n_conditions).LR(cond(perm(trial)+(b-1)*n_conditions).count) == resp
                cond(perm(trial)+(b-1)*n_conditions).step_idx = cond(perm(trial)+(b-1)*n_conditions).step_idx + 2; % make it harder by two steps
                cond(perm(trial)+(b-1)*n_conditions).resp_history(cond(perm(trial)+(b-1)*n_conditions).count) = 1; % report a correct response in history
            else %if they're wrong
                cond(perm(trial)+(b-1)*n_conditions).step_idx = cond(perm(trial)+(b-1)*n_conditions).step_idx - 1; % maket it easier by 1 step
            end
            if cond(perm(trial)+(b-1)*n_conditions).count > 1 && cond(perm(trial)+(b-1)*n_conditions).resp_history(cond(perm(trial)+(b-1)*n_conditions).count) ~= cond(perm(trial)+(b-1)*n_conditions).resp_history(cond(perm(trial)+(b-1)*n_conditions).count-1)
                %if after trial one and the last answer is different than the
                %current on, increase n_flip
                cond(perm(trial)+(b-1)*n_conditions).n_flips = 1;
            end
        else %if there has been a flip
            if cond(perm(trial)+(b-1)*n_conditions).LR(cond(perm(trial)+(b-1)*n_conditions).count) == resp %if they are correct
                cond(perm(trial)+(b-1)*n_conditions).n_correct = cond(perm(trial)+(b-1)*n_conditions).n_correct+1;
                cond(perm(trial)+(b-1)*n_conditions).resp_history(cond(perm(trial)+(b-1)*n_conditions).count) = 1; % report a correct response in history
                if cond(perm(trial)+(b-1)*n_conditions).n_correct == 2
                    cond(perm(trial)+(b-1)*n_conditions).step_idx = cond(perm(trial)+(b-1)*n_conditions).step_idx + 1; % make it harder by a step
                    cond(perm(trial)+(b-1)*n_conditions).n_correct = 0; % reset number correct
                else
                    %dont change step idx, keep counting how many correct in a row
                end
            else %if they are wrong
                cond(perm(trial)+(b-1)*n_conditions).n_correct = 0; % reset number correct in a row
                cond(perm(trial)+(b-1)*n_conditions).step_idx = cond(perm(trial)+(b-1)*n_conditions).step_idx - 1; % make it easier by a step
            end
        end
    end
        
    
    % check: if the staircase goes out of bounds/preset velocities reset to
    % nearest index
    if cond(perm(trial)+(b-1)*n_conditions).step_idx > stairFrac
        cond(perm(trial)+(b-1)*n_conditions).step_idx = stairFrac;
    elseif cond(perm(trial)+(b-1)*n_conditions).step_idx < 1
        cond(perm(trial)+(b-1)*n_conditions).step_idx = 1;
    end

    end

    today = date;
    day = today(1:2);
    month = today(4:6);
    year = today(end-3: end);

    fname = sprintf ( '%s-%s-%s_%s_%s.mat', year, month, day,initials, session);
    folder = [data_path fname];
    if ~test
        save(folder, 'cond')
    end
end
    clc;
    sca;


% catch ME
%     %this "catch" section executes in case of an error in the "try" section
%     %above.  Importantly, it closes the onscreen window if its open.
%     ListenChar(1);
%     ShowCursor;
%     Screen('CloseAll');
%     Screen('Preference', 'VisualDebugLevel', oldVisualDebugLevel);
%     Screen('Preference', 'SuppressAllWarnings', oldSupressAllWarnings);
%     Priority(0);v
% end %try..catch   