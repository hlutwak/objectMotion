cd ../Data
v = VideoReader('world_undistorted.mp4');
load por
cd ../EnvironmentalStatistics
addpath('flow_code')
addpath('flow_code/utils')

%% change start and end times
por.startFrame = 11*60*30;
por.startTime = por.startFrame/30;
por.endFrame = 11*60*30 + 10;
por.endTime = por.endFrame/30;
porX = por.X;
porY = por.Y; 
df = 2;
%%
v.CurrentTime = por.startTime; %start replaying the video at this time
%neighborhood size i think is size of pixel neighborhood usd during opt
%flow calc
%filter size is averaging over neighborhood of pixels after optic flow is
%calculated. uses gaussian.
opticFlowFarneback = opticalFlowFarneback('NeighborhoodSize', 20,'FilterSize', 50); %intialize optic flow object
ind = por.startFrame;

v_write = VideoWriter('output_video', 'MPEG-4');
v_write.FrameRate = v.FrameRate/2;
[col, row] = meshgrid(1:v.Width/df, 1:v.Height/df);

%%
    open(v_write);
   
while v.CurrentTime < por.endTime
    %pull frame
    vidFrame = readFrame(v);
    im1 = vidFrame(1:df:end,1:df:end,:);
   
    %fixation
    pX = round(porX(ind));
    pY = round(porY(ind));
    
    %show vid frame and fixation
    image(im1)
    drawnow;
    hold on;
    scatter(pX/df, pY/df, 'r*');
    drawnow;
    hold on
    
    % Get optic flow
    frameGray = rgb2gray(im1); %need to convert to grayscale
    flowLK = estimateFlow(opticFlowFarneback,frameGray);
    % decimation factor is how many pixels you pool over, i believe
    % scale factor scales arrows up or down
    plot(flowLK,'DecimationFactor',[8 8],'ScaleFactor',30) 
    drawnow
    
    box off;
    set(gca,'YTickLabel',[]);
    set(gca,'XTickLabel',[]);
    %pause to show
    pause(1/(v.FrameRate));
    ind = ind+1;
    writeVideo(v_write, getframe(gcf));
    cla
end
    close(v_write);