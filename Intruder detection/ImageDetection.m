                                
%% Reading of Images
img1 = imread('TestImage1.jpg');
img2 = imread('TestImage2.jpg');

%%  Display Color Images
figure
imshow(img1)
figure
imshow(img2)

%% Convert RGB to Grayscale
img1BW = rgb2gray(img1);
img2BW = rgb2gray(img2);

%%  Display Grayscale Images
figure
imshow(img1BW)
figure
imshow(img2BW)

%% Subtract Images %background subtraction
imgDiff = abs(img1BW - img2BW);
%imDiff = imsubtract(img1BW - img2BW);
figure
imshow(imgDiff)

% background subtraction is not just enough we need to do more processing as below

%% Find Maximium Location of Difference     % Here we are doing is just finding position of highest intensity
maxDiff = max(max(imgDiff));
[iRow,iCol] = find(imgDiff == maxDiff);
[m,n] = size(imgDiff);

imshow(imgDiff)
hold on
plot(iCol,iRow,'r*')      % position of highest intensity is found out by denoting it in image by *
maxDiff = max(max(imgDiff));

%% Use imtool to Determine Threshold and Length
imtool(imgDiff)

%% Threshhold Image          % Here we are converting the image from grayscale to binary image
imgThresh = imgDiff > 8;     % Threshhold is set >8
figure
imshow(imgThresh)
hold on
plot(iCol,iRow,'r*')
hold off

%% Fill in Regions        % bwareaopen is a morphological operator the is used to remve small objects from binary image %morphology is just a fancy word for shape based filters
imgFilled = bwareaopen(imgThresh, 15);  %in image >15 are kept and <15 are removed
figure
imshow(imgFilled)

%% Overlay Onto Original Image   % imoverlay is a utility function(imoverlay.m) which is taken from matlab central which is used to highlight the intruder using th red color([1,0,0])
																																												   |
imgBoth = imoverlay(img2,imgFilled,[1 0 0]);---------------------------------------------------------------------------------------------------------------------------------------|
figure
imshow(imgBoth)

%% Only Care About Things Large Than 80
											% regionprops is function used to measure properties of image regions
imageStats = regionprops(imgFilled, 'MajorAxisLength');

imgLengths = [imageStats.MajorAxisLength];
idx = imgLengths > 80;            % >80 is taken into consideration hat us height of intruder
imageStatsFinal = imageStats(idx);
disp(imageStatsFinal)

%% Determine if Change is Significant

if isempty(imageStatsFinal)
    disp('No Intruder Present')
else
    disp('Intruder Detected')
end

