% clear and ready to start
clear
clc

% Constances

% read camera parameters from a file and display the result
load cameraParams.mat cameraParams
display(cameraParams);

%read image and remove distoration from it
object_image = imread('object.jpg');
figure;
imshow(object_image);
title('Original Image of object');
drawnow;

[undist_image, newOrigin] = undistortImage(object_image, cameraParams, 'OutputView', 'full');
figure; 
imshow(undist_image, 'InitialMagnification', 100);
title('Undistorted Image');
drawnow;

% Convert the image to the HSV color space.
imHSV = rgb2hsv(undist_image);

% Get the saturation channel.
saturation = imHSV(:, :, 2);

% Threshold the image
t = graythresh(saturation);
imobj = (saturation > t);

figure; imshow(imobj, 'InitialMagnification', 100);
title('Segmented objects');

% Find connected components.
blobAnalysis = vision.BlobAnalysis('AreaOutputPort', true,...
    'CentroidOutputPort', false,...
    'BoundingBoxOutputPort', true,...
    'MinimumBlobArea', 500, 'ExcludeBorderBlobs', true);
%[areas,boxes] = blobAnalysis(imobj);
[areas, boxes] = step(blobAnalysis, imobj);

% Sort connected components in descending order by area
[~, idx] = sort(areas, 'Descend');

% Get the largest components.
boxes = double(boxes(idx(2), :));

% Reduce the size of the image for display.
scale = 1;
imDetectedobj = imresize(undist_image, scale);

% Insert labels 
imDetectedobj = insertObjectAnnotation(imDetectedobj, 'rectangle', ...
    scale * boxes, 'LEGO brick', 'LineWidth', 10, 'FontSize', 50);
figure; imshow(imDetectedobj);
title('Detected object');

% Detect the checkerboard.
squareSize = 23.5;
[imagePoints, boardSize] = detectCheckerboardPoints(undist_image);
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
% Adjust the imagePoints so that they are expressed in the coordinate system
% used in the original image, before it was undistorted.  This adjustment
% makes it compatible with the cameraParameters object computed for the original image.
imagePoints = imagePoints + newOrigin; % adds newOrigin to every row of imagePoints

% Compute rotation and translation of the camera.
[R, t] = extrinsics(imagePoints, worldPoints, cameraParams);

boxes = boxes + [newOrigin, 0, 0]; % zero padding is added for widht and height

box1 = double(boxes(1, :));
% Get the top-left corner with the width and height
imagePoints1 = [box1(1:2); ...
                box1(1), box1(2)+box1(4)];
imagePoints2 = [box1(1:2); ...
                box1(1)+ box1(3),box1(2)];

% Get the world coordinates of the corners            
worldPoints1 = pointsToWorld(cameraParams, R, t, imagePoints1);
worldPoints2 = pointsToWorld(cameraParams, R, t, imagePoints2);
% Compute the length in millimeters.
h = worldPoints1(2, :) - worldPoints1(1, :);
w = worldPoints2(2, :) - worldPoints2(1, :);
height = hypot(h(1), h(2));
width = hypot(w(1), w(2));
fprintf('Measured height = %0.2f mm\n', height);
fprintf('Measured width = %0.2f mm\n', width);