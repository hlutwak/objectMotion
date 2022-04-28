addpath(genpath('/Applications/Psychtoolbox'))
addpath('/Users/hopelutwak/Documents/MATLAB/VisTools/')

seed=2;
rng(seed)
nt = 600;
speed = 100; %cm/s
fps = 60;

dim = [100,0,nt*speed/fps+400];
nClusters =210;
nDotsPerCluster = 40;
nHigh = nClusters/5;
nGround = nClusters-nHigh;

viewdist = .5; %m
viewingdepths = [30,300];

% windowRect =  [0           0        1680        1050];
windowRect = [0           0        3840        2160];
pixels = windowRect(3:4);
screensize = [.712 .312];
view_window                     = round([rad2deg(atan(screensize(1)/2/viewdist)) rad2deg(atan(screensize(2)/2/viewdist))]);  % X,Y centered around fixation [60 46] [54 40];   
scale_factor                    = sqrt((view_window(1)*60*view_window(2)*60)/(pixels(1)*pixels(2)))*1;          % Arcmin/pixel 1.78 2.25 1.92dell: 1680x1050, psychophysics room: 1600x1200                                % Screen frame rate (hz) psychophysics room: 85
scale_factorX = view_window(1)*60/pixels(1);
scale_factorY = view_window(2)*60/pixels(2);
    
    
ppcm = [39,-39];
xyrat = windowRect(3)/windowRect(4);
dotRect = 1*[xyrat*1.5,1,0];
dotLine = 1*[1,1/xyrat,std(viewingdepths)];

nDots = nDotsPerCluster*nClusters;
clusters = rand(nClusters,3).*dim - .5*[dim(1:2),0];
% clusters(:,2) = clusters(:,2) - 15;
idxcluster = 1:length(clusters);
high = randsample(1:length(clusters),nHigh,false);
highidx = ismember(idxcluster, high);
ground =  idxcluster(~highidx);
% clusters(high, 2) = clusters(high,2) +rand(length(high),1)*5+5; % increase height of some clusters
clusters(high,2) = clusters(high,2) - 15;
clusters(ground,2) = clusters(ground,2) - 15;

dots = repmat(clusters,nDotsPerCluster,1);

for d = 1:nDotsPerCluster
    dots(high+nClusters*(d-1),:) = dots(high+nClusters*(d-1),:) + dotRect.*randn(nHigh,3);
%     dots(high+nClusters*(d-1),:) = dots(high+nClusters*(d-1),:) + [0,1,0].*ones(nHigh,3)*d/2;
    dots(ground+nClusters*(d-1),:) = dots(ground+nClusters*(d-1),:) +dotRect.*randn(nGround,3);
end


% 
% observerTrajectory = [zeros(nt,1), zeros(nt,1),(0:(speed/fps):(nt-1)*(speed/fps))'];
% observerTrajectory = [zeros(nt,1), sind(1:5:5*nt)'*2,(0:(speed/fps):(nt-1)*(speed/fps))'];

observerTrajectory = [zeros(nt+1,1), zeros(nt+1,1),(0:(speed/fps):(nt)*(speed/fps))'];
% observerTrajectory = [zeros(nt+1,1), sind(1:4:4*nt+1)'*2,(0:(speed/fps):(nt)*(speed/fps))'] ;

observerTrajectory = diff(observerTrajectory);
theta = atan(observerTrajectory(:,2)./observerTrajectory(:,3))*.01;
% theta = zeros(1,nt);

X = nan(nDots,nt);
Y = nan(nDots,nt);
I = true(nDots,nt);

for ii=1:nt
    pos = observerTrajectory(ii,:);
    dots = dots - pos;
    observerRotation = [1, 0, 0; 0, cos(theta(ii)), -sin(theta(ii)); 0, sin(theta(ii)), cos(theta(ii))];
    dots = (observerRotation*dots')';
    depths(:,ii) = dots(:,3);
%     X(:,ii) = viewdist*(dots(:,1)-pos(1))./(dots(:,3)-pos(3));
%     Y(:,ii) = viewdist*(dots(:,2)-pos(2))./(dots(:,3)-pos(3));
    
    X(:,ii) = 100*viewdist*(dots(:,1))./(dots(:,3));
    Y(:,ii) = 100*viewdist*(dots(:,2))./(dots(:,3));


%     
%     I(:,ii) = dots(:,3)-pos(3) > viewingdepths(1)...
%         & dots(:,3) - pos(3) < viewingdepths(2);
%     I(:,ii) = I(:,ii) & abs(X(:,ii)*ppcm(1))<windowRect(3)/2 & abs(Y(:,ii)*ppcm(2))<windowRect(4)/2;
    
    I(:,ii) = dots(:,3) > viewingdepths(1)...
        & dots(:,3)< viewingdepths(2);
    I(:,ii) = I(:,ii) & abs(X(:,ii)*ppcm(1))<windowRect(3)/2 & abs(Y(:,ii)*ppcm(2))<windowRect(4)/2;

    
end

% view depth change over time
% for n = 1:nt
%     histogram(depths(:,n))
%     pause(0.01);
% end

%% Psychtoolbox demo

Screen('Preference', 'SkipSyncTests', 1)
screens = Screen('Screens');
screenNumber = max(screens);

% Find the color values which correspond to white and black.
white=WhiteIndex(screenNumber);
black=BlackIndex(screenNumber);

% Round gray to integral number, to avoid roundoff artifacts with some
% graphics cards:
gray=round((white+black)/2);

PsychImaging('PrepareConfiguration');
PsychImaging('AddTask', 'General', 'NormalizedHighresColorRange', 1);
% Open an on screen window and color it black
[window, windowRect] = PsychImaging('OpenWindow', screenNumber, 0);
Screen('BlendFunction', window, GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
screen_rect = Screen('Rect',window);

ctr = windowRect(3:4)/2;
vbl = Screen('Flip',window);

sr_hor = round(screen_rect(3)/2); % Middle of the screen, horizontally, in pixels
sr_ver = round(screen_rect(4)/2); % Middle of the screen, vertically, in pixels
fix_hor = sr_hor;     % Horizontal location of fixation cross, in pixels
fix_ver = sr_ver;     % Vertical location of fixation cross, in pixels


% choose a cluster
furtherCluster= find(sum(I(1:nClusters,:),2)>20 & sum(I(1:nClusters,:),2)<100);
movingobj = intersect(furtherCluster, high);
cl=randsample(movingobj,1);
% cl=40;
f = mean(find(I(cl,:)));

% deviation
gain = 10;
% dev = sin((0:nt-1)/2) .* normpdf(0:nt-1,f,4); 
dev = (0:nt-1);
dev = gain*dev/max(dev);


Xdraw = X;
Xdraw(cl:nClusters:end,:)= X(cl:nClusters:end,:)+dev;
Ydraw = Y;
% Ydraw(cl:nClusters:end,:)= Y(cl:nClusters:end,:)+dev;

% We create a Luminance+Alpha matrix for use as transparency mask:
% Layer 1 (Luminance) is filled with luminance value 'gray' of the
% background.
% ms=100;
transLayer=2;
% [x,y]=meshgrid(-ms:ms, -ms:ms);
% [x,y]= meshgrid(-pixels(1)/2:pixels(1)/2, -pixels(2)/2:pixels(2)/2);
maskblob=uint8(ones(screen_rect(4), screen_rect(3), transLayer) * gray);
maskblob=uint8(ones(screen_rect(4), screen_rect(3), 1) * gray);
% Layer 2 (Transparency aka Alpha) is filled with gaussian transparency
% mask.
% xsd=ms/2.0;
% ysd=ms/2.0;
% vignette = uint8(round(255 - exp(-((x/xsd).^2)-((y/ysd).^2))*255));
% windowTex = Screen('MakeTexture',w,cat(3,128*ones(text_pixelW),255*(1-coswin(text_pixelW,.3*text_pixelW,.5*text_pixelW))));
text_pixelW = 254;
vignette = uint8(round(255*(1-coswin(text_pixelW,.2*text_pixelW,.3*text_pixelW))));
% maskblob(1:text_pixelW,1:text_pixelW,transLayer)=vignette;

% 
% visiblesize=2*texsize+1;
% % Definition of the drawn rectangle on the screen:
% dstRect=[0 0 visiblesize visiblesize];
% dstRect=CenterRect(dstRect, windowRect);
% angle = 0;

%-----Element Settings
movie_rect= [0,0,text_pixelW, text_pixelW];

dot_density = .6;
jitter = 0;
cloud_dist =viewingdepths;
text_diam = size(maskblob,1);
exclude = [0,0,0,0];                       % Don't place elements where the FOE will be
devPos = [10;6];

[dots_m, dots_deg, dev0, dev1] = make_dot_cloud(dot_density, jitter, cloud_dist, view_window, text_diam, exclude, devPos);

pos = CenterRectOnPoint(movie_rect,dots_deg(2,:)'*60/scale_factorY+screen_rect(4)/2,dots_deg(1,:)'*60/scale_factorX+screen_rect(3)/2)'; %big Y numbers are lower on the screen in pixels so we have to flip

valid =  pos([1 3],:) >0 & pos([1 3],:) < screen_rect(4) & pos([2 4],:) > 0 & pos([2 4],:) < screen_rect(3);
valididx = valid(1,:) & valid(2,:);
validPos = round(pos(:,valididx));

for p = 1:length(validPos)
    maskblob(validPos(1,p):validPos(3,p)-1, validPos(2,p):validPos(4,p)-1,2) = vignette;
end

% Build a single transparency mask texture
masktex=Screen('MakeTexture', window, maskblob);
mRect=Screen('Rect', masktex);



for ii=1:nt
    
    xy = [Xdraw(I(:,ii),ii),Ydraw(I(:,ii),ii)]'.*ppcm';
    Screen('DrawDots', window, xy, 3 ,[1,1,1] ,ctr, 2);

%     Screen('DrawTextures', window, masktex, [], mRect);
    

    vbl = Screen('Flip',window);
    
end

sca;
