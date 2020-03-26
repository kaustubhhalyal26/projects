%% Copyright 2013-2015 The MathWorks, Inc.                                 
function security_cam()
% This is a simple demo to show video processing using a Webcam in MATLAB

clear all; close all; clc; 

%% Setup Image Acquisition
hCamera = webcam;

%  Create a handle to an imshow figure for faster updating
hShow = imshow(zeros(480,640,3,'uint8'));title('Security Camera');

%% Acquire reference image
ref_vid_img = snapshot(hCamera);

%% Quantize images and outputing to the screen

frames = 500;
for i = 1:frames

% Acquire an image from the webcam
    vid_img = snapshot(hCamera);

% Call the live segmentation function
    object_detected = Segmentation_Fn(vid_img, ref_vid_img);

%  Update the imshow handle with a new image 
    set(hShow,'CData',object_detected);
    drawnow;
end

%%  Shutdown webcam by deleting handle
stop(hCamera);
delete(hCamera);
