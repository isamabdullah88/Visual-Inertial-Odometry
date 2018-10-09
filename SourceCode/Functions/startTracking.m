function[ State, Parameters ] = startTracking( currImg, pointTracker, State, camera, Parameters )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here


currTrackedPts = extractPointsToTrack(currImg);
% initialize(pointTracker, trackedPts, currImg);

State = augmentStateStart(State, camera, currTrackedPts);

% Instead of re - initializeing, we can setPoints to reduce time EXCEPT for
% the first time
if Parameters.firstTime == 1
initialize(pointTracker, currTrackedPts, currImg);
Parameters.firstTime = 0;
else
setPoints(pointTracker, currTrackedPts);
end


% Plotting and showing Img
% imshow(currImg); hold on;
% plot(trackedPts( : , 1), trackedPts( : , 2), '--rs', 'LineWidth', 1, 'MarkerSize', 5, ...
%     'MarkerEdgeColor', 'c', 'MarkerFaceColor', 'b');
% drawnow;
% -----------

    end

