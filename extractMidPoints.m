function [ points ] = extractMidPoints(img)
%extractMidPoints This function extracts good features to track
%   This function is called when the no of features fall below a threshold
%   during tracking

%%% The following feature detectors can be used:
% 1) detectBRISKFeatures
% 2) detectFASTFeatures
% 3) detectHarrisFeatures
% 4) detectSURFFeatures
% 5) detectMinEigenFeatures

% numPts = 150;
%  
% featurePts = detectSURFFeatures(img);
% points = featurePts.selectStrongest(numPts);
% points = points.Location;

% [~, ~, loc] = sift(img);
% % Randomly selecting points
% ptsInd = randperm(size(loc,1), numPts);
% points = loc(ptsInd,[2 1]);


%----------------------------------------------
[h,b] = size(img);
h_break = 2;
b_break = 7;
numCorners = 10;

y = floor(linspace(1, h - h/h_break, h_break));
x = floor(linspace(1, b - b/b_break, b_break));
ind = [1 2 3 4 5 6 7];

final_points = [];
for i=1:length(y)
    for j=1:length(ind)
    roi =   [x(ind(j)),y(i),floor(b/b_break),floor(h/h_break)];
    corners = detectSURFFeatures(img, 'ROI',roi );
    corners = corners.selectStrongest(numCorners);
    final_points = vertcat(final_points, corners.Location);
    end
end
points = cornerPoints(final_points);
points = points.Location;


end
