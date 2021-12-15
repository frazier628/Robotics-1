function [cameraParams, R, t] = intrinsic_calc()

    %Calculates Camera Intrinsic Parameters for a collection of images
    
    numImages = 18;
    files = cell(1, numImages);
    for i = 1:numImages
        files{i} = fullfile('C:\Users\frazic\Documents\MATLAB\Robotics Project\Photos', sprintf('image%d.jpg', i));
    end
    
    % Detect the checkerboard corners in the images.
    [imagePoints, boardSize] = detectCheckerboardPoints(files);
    
    % Generate the world coordinates of the checkerboard corners in the
    % pattern-centric coordinate system, with the upper-left corner at (0,0).
    squareSize = 8.4; % in millimeters
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);
    
    % Calibrate the camera.
    I = imread(files{1});
    imageSize = [size(I, 1), size(I, 2)];
    cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
        'ImageSize', imageSize);
    
    % Since the lens introduced little distortion, use 'full' output view to illustrate that
    % the image was undistored. If we used the default 'same' option, it would be difficult
    % to notice any difference when compared to the original image. Notice the small black borders.
    
    %The good picture that captures the maze
    imOrig = imread(files{11});
    
    [im, newOrigin] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
    
    % Detect the checkerboard.
    [imagePoints, boardSize] = detectCheckerboardPoints(im);
    
    % Adjust the imagePoints so that they are expressed in the coordinate system
    % used in the original image, before it was undistorted.  This adjustment
    % makes it compatible with the cameraParameters object computed for the original image.
    imagePoints = imagePoints + newOrigin; % adds newOrigin to every row of imagePoints
    
    % Compute rotation and translation of the camera.
    [R, t] = extrinsics(imagePoints, worldPoints, cameraParams);

end









