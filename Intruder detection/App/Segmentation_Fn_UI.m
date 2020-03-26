%% Copyright 2013-2015 The MathWorks, Inc.                                 
function [highlighted_img, imgThresh, alert] = Segmentation_Fn(img1, img2, objThresh, objSize)

%% Convert Images to Black and White
img1BW = rgb2gray(img1);
img2BW = rgb2gray(img2);


%% Subtract Images
imgDiff = abs(img2BW - img1BW);

%% Threshhold Image
imgThresh = imgDiff > objThresh;

%% Fill in Regions
imgFilled = bwareaopen(imgThresh, 15);

%% Overlay Onto Original Image
% Utility from File Exchange

imgBoth = imoverlay(img2,imgFilled,[1 0 0]);
highlighted_img = imgBoth;

%% Only Care About Things Large Than objSize

imageStats = regionprops(imgFilled, 'MajorAxisLength');

imgLengths = [imageStats.MajorAxisLength];
idx = imgLengths > objSize;
imageStatsFinal = imageStats(idx);
%disp(imageStatsFinal)

%% Determine if Change is Significant
if isempty(imageStatsFinal);
    %disp('Nothing Different Here')
    alert = false;
else
    %disp('Something is Here!')
    alert = true;
end

