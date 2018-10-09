function[ camState ] = removeOutOfView( camState, currTrackedPts, N, outOfView )
%removeOutOfView makes sure that all the camera frames have appropriate
%tracked features, removing any out of view features.

    if (outOfView ~ = 0)
for i = 1 : N
sz = size(camState{i}.features, 1);
outUpdated = outOfView(find(outOfView <= sz));
camState{i}.features(outUpdated, : ) = [];
end
end
% Assign currTrackedPts to curr frame
camState{N + 1}.features = currTrackedPts;

end

