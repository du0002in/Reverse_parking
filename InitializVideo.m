%initialization of video input
function [status, vid]=InitializVideo
imaqreset;
try
    vid=videoinput('winvideo', 5, 'RGB24_640x480');
    set(vid,'FramesPerTrigger',1);
    set(vid,'TriggerRepeat',Inf);
    triggerconfig(vid,'manual');
catch
    status=0;
    return;
end
status=1;
start(vid);
preview(vid);
end