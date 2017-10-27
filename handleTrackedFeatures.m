function [ State, currTrackedPts, costVec ] = handleTrackedFeatures(pointTracker, currImg, ...
    State, camParams, costVec, noiseParams)
%handleTrackedFeatures handles the tracked features, and uses them to
%update the state

% --- Check if features got out of view and Remove them from both ---------
% currTrackedPts and trackedPts: IMPORTANT: Remove from BOTH --------------
[currTrackedPts, ptsValidity] = step(pointTracker, currImg);

% % --- capture points which are out of view, and update using them
outOfView = find(not(ptsValidity));

%%% Before augmenting the state (where we are removing all the out of view
%%% features), we are checking for out of view features and updating them
%%% to correct the state.

% if (State.N >= 5 && ~isempty(outOfView))
%     
%     [State, numFeatUsed] = updateState(State, noiseParams, camParams, outOfView);
%     fprintf('Number of features IN-BETWEEN: %d\n', numFeatUsed);
% end


% % --- now remove them
currTrackedPts(outOfView,:) = []; % But this might be efficient
% trackedPts(outOfView,:) = [];
% % Detect inliers using RANSAC, and concat the whole
% trackedPts = rejectOutliers(trackedPts, currTrackedPts);
% trackedPts = [trackedPts, currTrackedPts];

%%%-------- Now if the number of features fall below a threshold, detect
%%%new features----

[State, currTrackedPts] = augmentState(State, camParams, currTrackedPts, currImg, outOfView);

% show features, with images
plotTrajectories(currImg, currTrackedPts);


end

