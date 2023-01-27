clear all;
clc;

%%%%%%%%%%%%%%%%
% Start Kinect
%%%%%%%%%%%%%%%%
colorVid = videoinput('kinect',1);
framesPerTrig = 6;
set(colorVid,'TriggerRepeat',Inf);%TriggerRepeat
set(colorVid,'FramesPerTrigger',framesPerTrig);%FramesPerTrigger
set(colorVid,'FrameGrabInterval',1);%FrameGrabInterval
start([colorVid]);

pause(1);
for i=1:3
    flushdata(colorVid);
    colorImg = getdata(colorVid);
    lastColorImage = colorImg(:, :, :, 1);
    
    % Marker colors for up to 6 bodies.
    colors = ['r';'g';'b';'c';'y';'m'];
    lastColorImage = flipdim(lastColorImage,2);
    % Display the RGB image.
    imshow(lastColorImage);
    imwrite(lastColorImage, 'calib.jpg');
    continue
end
stop(colorVid);
disp("Done!");