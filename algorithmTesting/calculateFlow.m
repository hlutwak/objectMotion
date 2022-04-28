%% change start and end times
v = VideoReader('desk.mp4');

startFrame = 1;
startTime = startFrame/v.FrameRate;
endFrame = 60;
endTime = endFrame/v.FrameRate;
df = 2;
%%
v.CurrentTime = startTime; %start replaying the video at this time
%neighborhood size i think is size of pixel neighborhood usd during opt
%flow calc
%filter size is averaging over neighborhood of pixels after optic flow is
%calculated. uses gaussian.
opticFlowFarneback = opticalFlowFarneback('NeighborhoodSize', 10,'FilterSize', 25); %intialize optic flow object
ind = startFrame;

v_write = VideoWriter('desk_output', 'MPEG-4');
v_write.FrameRate = v.FrameRate/2;
[col, row] = meshgrid(1:v.Width/df, 1:v.Height/df);

%%
    open(v_write);
   
while v.CurrentTime < endTime
    %pull frame
    vidFrame = readFrame(v);
    im1 = vidFrame(1:df:end,1:df:end,:);
  
    
    %show vid frame and fixation
    image(im1)
    drawnow;
    hold on
    
    % Get optic flow
    frameGray = rgb2gray(im1); %need to convert to grayscale
    flowLK = estimateFlow(opticFlowFarneback,frameGray);
    % decimation factor is how many pixels you pool over, i believe
    % scale factor scales arrows up or down
    plot(flowLK,'DecimationFactor',[8 8],'ScaleFactor',5) 
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