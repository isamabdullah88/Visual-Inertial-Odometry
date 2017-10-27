function [ camState, currTrackedPts ] = detectThreshold( camState, currImg, N )
%detectThreshold detects new set of features from current image, when the
%number of tracked features drop below a threshold
threshold = 100;

commonFeats = camState{N}.features;
if (size(camState,2) <= 10 && size(commonFeats,1) <= threshold)
    pts = extractMidPoints(currImg);
    
    % delete common features from pts
    for i=1:size(commonFeats,1)
        commonE = commonFeats(i,:);
        indFeat = find(all(repmat(commonE,size(pts,1),1)==pts,2));
        pts(indFeat,:) = [];
    end
    
currTrackedPts = [camState{N}.features; pts];
camState{N}.features = currTrackedPts;
else
    currTrackedPts = camState{N}.features;
end


end

