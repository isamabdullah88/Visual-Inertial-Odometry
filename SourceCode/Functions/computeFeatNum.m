function[ numFeatures, outFeat ] = computeFeatNum( camState, outOfView )
%computeFeatNum Computes the number of features, that have to be used for
%State update. It also states true, if this is the case of outOfView

if (isempty(outOfView))
numFeatures = size(camState{end}.features, 1);
outFeat = 0;
else
numFeatures = length(outOfView);
outFeat = 1;
end

end

