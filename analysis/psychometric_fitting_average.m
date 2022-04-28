%% Psychometric fitting individuals

% Apr 2022
% aggregate data
% calculate thresholds and confidence intervals for velocities tested

% add psignifit toolbox
addpath('/Users/hopelutwak/Documents/MATLAB/psignifit')
addpath('/Volumes/GoogleDrive/My Drive/opticflow/objectDetection/OpticFlow/phaseExperiment/data')

% assign data folder
dataFolder = '/Volumes/GoogleDrive/My Drive/opticflow/objectDetection/OpticFlow/phaseExperiment/data';

% names of files
S = dir(fullfile(dataFolder,'*.mat'));

% which subjects data to analyze
subjects = ["ABC", "HL","MR", "KZ"]; %"ABC", "HL","MR", "KB", "KZ"
stims = "_Ctruncnatural";

% loop over all subjects
figure
for s  = 1:length(subjects)


    %load appropriate files
    count = 0;
    data = [];
    for f = 1:length(S)
        subj = contains(S(f).name,subjects(s));
        sti = contains(S(f).name,stims);
        
        if subj && sti
            load(S(f).name);
            if count < 1
                data = load(S(f).name);
                data = data.cond;
            else
                data = [data cond];
            end
            count = count+1;
        end
    end
    

    % is it a control stim
    if contains(stims, 'C')
        control = 1;
        if contains(stims, 'trunc')
            scramble = 1;
        else 
            scramble = 0;
        end
    else 
        control = 0;
    end

    t = 1;
    num_t = 1;
    steps = 1:size(cond(1).steps,2);
    n_blocks = length(data)/10;
    n_conditions = length(data)/n_blocks;
    n_steps = length(steps);
    % convert steps into measure of intensity (magnitude of difference vector)
    deltanorm = (vecnorm(cond(1).steps(:,:,1) - cond(1).steps(:,end,1)));

    colors = jet(length(cond)/n_blocks);
    colors = colors*.95;

    % get angles, and reorder conditions so radially ordered
    deltaAngles = zeros(1,length(data));

    for c = 1:length(data)
        % get dir dev in same dir as base velocity
    %     base = data(length(data)/n_blocks+1).steps(:,1,1) - data(length(data)/n_blocks+1).steps(:,end,1);
        if mod(c,40) == 1
            base = data(c).steps(:,1,1) - data(c).steps(:,end,1);
        end
        unit_base = [base/norm(base);0];
        deviation = data(c).steps(:,1,1) - data(c).steps(:,end,1);
        unit_deviation = [deviation/norm(deviation);0];

            if control && mod(data(c).block, 2) && ~scramble
                unit_deviation(1) = -unit_deviation(1);
            end
        cosTheta = unit_base'*unit_deviation;

        crossP = cross(unit_base, unit_deviation);
        sinTheta = crossP(end);
        deltaAngles(c) = mod(-sign(sinTheta)*rad2deg(real(acos(cosTheta))), 360);
    end
    % round deltaAngles


    deltaAngles = round(deltaAngles);
    deltaAngles(deltaAngles == 360) = 0;
    %for slightly off angles
    n_angles = n_conditions;
    base_angles = deltaAngles(1:n_conditions);
    idx = dsearchn( base_angles',deltaAngles');
    deltaAngles = base_angles(idx);

    [sorted_cond, angleidx] = sort(deltaAngles);
    sorted_unique_deltaAngles = unique(sorted_cond);
    
    if ~isempty(data)
    
    % get n correct for each stim
    nCorrect = zeros(n_conditions,size(cond(1).steps,2));
    nTrials = zeros(n_conditions,size(cond(1).steps,2));
    
    %save in correct angle order
    
    for ii = 1:n_conditions
        % which staircases are have this angle, take correct number of blocks
        % based on how many files there were
        cond_idx = angleidx(ii*n_blocks- (n_blocks-1):ii*n_blocks);
        
        for jj = cond_idx % go through this staircase and count num correct
            for step = 1:size(data(1).steps,2)
                idx = find(data(jj).step_history ==step);
                numTrials = length(idx);
                ncorrect = sum(data(jj).resp_history(idx));
                
                nCorrect(ii,step) = nCorrect(ii,step)+ncorrect;
                nTrials(ii,step) = nTrials(ii,step)+numTrials;
            end
        end
    end
    
    
    % set options for psychometric functions
    options             = struct;   % initialize as an empty struct
    options.sigmoidName = 'weibull';
    options.expType     = '2AFC';   % choose 2-AFC as the paradigm of the experiment
    % this sets the guessing rate to .5 and
    % fits the rest of the parameters
    options.fixedPars = NaN(5,1);
    options.fixedPars(5) = 0;       % fix eta (dispersion) to zero
    
    %loop through stim conditions and get threshold and plot curves

    for ii = 1:n_conditions
        results = [flip(steps)' nCorrect(ii,:)' nTrials(ii,:)'];
        results(results(:,end)==0, :) = [];
        result = psignifit(results,options);
        thresholds(s,ii)= exp(result.Fit(1));
        CI(:,ii,s) =exp(result.conf_Intervals(1,:,1))'; %get CI for threshold (first row) at 95% (first layer)
        
        subplot(num_t,n_conditions,ii)
        hold on
        plotPsych(result);
        ylim([0 1])
        title(num2str(sorted_unique_deltaAngles(ii)))
        sgtitle(stims) 
    end
    
    else
    end
    
end


%% individual thresholds and confidence intervals

% plot thresholds in velocity space

% convert threshold and CI to velocity
velocity_thresh = zeros(2,n_angles*num_t);
velocity_CI_low = zeros(2, n_angles*num_t);
velocity_CI_high = zeros(2, n_angles*num_t);

%save in angle order
for ii = 1:n_conditions
    % which staircases have this angle
    cond_idx = angleidx(ii*n_blocks- (n_blocks-1):ii*n_blocks);
     
    segment = data(cond_idx(2)).steps(:,1,1) - data(cond_idx(2)).steps(:,end,1);
    
    velocity_thresh(:,ii) = data(cond_idx(2)).steps(:,end,1)+0.1*thresholds(ii)*segment;
    velocity_CI_low(:,ii) = data(cond_idx(2)).steps(:,end,1)+0.1*CI(1,ii)*segment;
    velocity_CI_high(:,ii) = data(cond_idx(2)).steps(:,end,1)+0.1*CI(2,ii)*segment;
    
end

%plot boudnary with confidence intervals

if control
    colors = [[.5 .5 .5];[.8 .8 .8]];
%     if scramble
%         colors = [[.8 .5 .5];[.9 .6 .6]];
%     end
elseif t ==2
    colors = [[.2 .7 .2];[.8 1 .8]];
else
    colors = [[.2 .2 .7];[.7 .7 1]];
end

thresholdfig = figure;

% plot CIs
hold on, fill([velocity_CI_high(1,:) velocity_CI_high(1,1)],-[velocity_CI_high(2,:) velocity_CI_high(2,1)], colors(2,:), 'LineStyle','none', 'FaceAlpha',0.5)
hold on, fill([velocity_CI_low(1,:) velocity_CI_low(1,1)],-[velocity_CI_low(2,:) velocity_CI_low(2,1)], 'w', 'LineStyle','none')
% plot thresholds
hold on, plot([velocity_thresh(1,:) velocity_thresh(1,1)],-[velocity_thresh(2,:) velocity_thresh(2,1)], 'color',colors(1,:), 'linewidth', 2)
% plot base velocity
hold on, quiver(0, 0, data(cond_idx(2)).steps(1,end,1), -data(cond_idx(2)).steps(2,end,1), 'Color', 'k','LineWidth', 2,'AutoScaleFactor',1)
% % plot constraint line
if ~control
    hold on, plot(data(cond_idx(2)).velocity_range(1,:,1), -data(cond_idx(2)).velocity_range(2,:,1), 'color', [.5 .5 .5], 'linewidth', 2)
end

axis equal


 
% set(gcf,'renderer','Painters')
% saveas(thresholdfig, stim, 'eps')

% figure 
% hold on, errorbar(sorted_unique_deltaAngles, velocity_thresh, thresholds-CI(1,:), thresholds-CI(2,:),'o', "Linewidth",2)


%% Average threshold with plotted individual thresholds
include_individuals = 1;
% plot thresholds in velocity space

avg = mean(thresholds);

sem = std(thresholds); %/sqrt(length(subjects))


% convert threshold and CI to velocity
velocity_thresh = zeros(2,n_angles);
velocity_thresholds = zeros(2,n_angles, s);
velocity_sem_high = zeros(2, n_angles);
velocity_sem_low = zeros(2, n_angles);


%save in angle order
for ii = 1:n_conditions
    % which staircases have this angle
    cond_idx = angleidx(ii*n_blocks- (n_blocks-1):ii*n_blocks);
     
    segment = data(cond_idx(2)).steps(:,1,1) - data(cond_idx(2)).steps(:,end,1);
    
    velocity_thresh(:,ii) = data(cond_idx(2)).steps(:,end,1)+0.1*avg(ii)*segment;
    velocity_sem_high(:,ii) = data(cond_idx(2)).steps(:,end,1)+0.1*(avg(ii)+sem(ii))*segment;
    velocity_sem_low(:,ii) = data(cond_idx(2)).steps(:,end,1)+0.1*(avg(ii)-sem(ii))*segment;

    for s = 1:length(subjects)
    velocity_thresholds(:,ii,s) = data(cond_idx(2)).steps(:,end,1)+0.1*thresholds(s,ii)*segment;
    end
end


%plot boudnary

if control
    colors = [[.5 .5 .5];[.8 .8 .8]];
%     if scramble
%         colors = [[.8 .5 .5];[.9 .6 .6]];
    g = linspace(.05,.8,length(subjects))';
    grays = repmat(g,1,3);
%     end
elseif t ==2
    colors = [[.2 .7 .2];[.8 1 .8]];
else
    colors = [[.2 .2 .7];[.7 .7 1]];
    blues = parula(length(subjects)*2);
    blues(length(subjects)+1:end,:) = [];

end

thresholdfig = figure;

% plot SEMs
hold on, fill([velocity_sem_high(1,:) velocity_sem_high(1,1)],-[velocity_sem_high(2,:) velocity_sem_high(2,1)], colors(2,:), 'LineStyle','none', 'FaceAlpha',0.5)
hold on, fill([velocity_sem_low(1,:) velocity_sem_low(1,1)],-[velocity_sem_low(2,:) velocity_sem_low(2,1)], 'w', 'LineStyle','none')

% plot thresholds
hold on, plot([velocity_thresh(1,:) velocity_thresh(1,1)],-[velocity_thresh(2,:) velocity_thresh(2,1)], 'color',colors(1,:), 'linewidth', 2)

%include individual thresholds
if include_individuals
    for s = 1:length(subjects)
        hold on
        
        if control
            c = grays(s,:);
            scatter(velocity_thresholds(1,:,s), -velocity_thresholds(2,:,s), [],c,'filled')
        else
            c = blues(s,:);
            scatter(velocity_thresholds(1,:,s), -velocity_thresholds(2,:,s), [],c,'filled')
        end
    end
end
% plot base velocity
hold on, quiver(0, 0, data(cond_idx(2)).steps(1,end,1), -data(cond_idx(2)).steps(2,end,1), 'Color', 'k','LineWidth', 2,'AutoScaleFactor',1)
% % plot constraint line
if ~control
    hold on, plot(data(cond_idx(2)).velocity_range(1,:,1), -data(cond_idx(2)).velocity_range(2,:,1), 'color', [.5 .5 .5], 'linewidth', 2)
end

axis equal
