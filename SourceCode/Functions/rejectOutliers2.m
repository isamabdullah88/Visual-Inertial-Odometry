function[ trackedPts, enoughPts ] = rejectOutliers2( trackedPts, currTrackedPts )
%rejectOutliers rejects the outliers in the tracked features using ransac
%and then concat them to use for mesurement model

try
[F, indicesPts] = estimateFundamentalMatrix(trackedPts( : , end - 1 : end), ...
currTrackedPts, 'Method', 'RANSAC', 'NumTrials', 500, 'DistanceThreshold', 1e - 01, ...
'DistanceType', 'Algebraic');

inlierIndices = find(indicesPts);
trackedPts =[trackedPts(inlierIndices, : ), currTrackedPts(inlierIndices, : )];
enoughPts = 1;
catch
trackedPts =[];
warning('Not enough inlier points to update in between');
enoughPts = 0;
end

end

