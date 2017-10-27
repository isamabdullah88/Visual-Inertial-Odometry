function [ trackedPts ] = rejectOutliers( trackedPts, currTrackedPts )
%rejectOutliers rejects the outliers in the tracked features using ransac
%and then concat them to use for mesurement model

[F, indicesPts] = estimateFundamentalMatrix(trackedPts(:,end-1:end), ...
    currTrackedPts, 'Method','RANSAC', 'NumTrials',700,'DistanceThreshold',0.7, ...
    'DistanceType', 'Algebraic');

inlierIndices = find(indicesPts);
currTrackedPts = currTrackedPts(inlierIndices,:);
trackedPts = [trackedPts(inlierIndices,:), currTrackedPts];

end

