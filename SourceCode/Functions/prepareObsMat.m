function[ obsMat, trackRange, goodLength ] = prepareObsMat( camState, outFeat, featInd, outOfView )
%prepareObsMat Extracts the feature tracks from all the camera states and
%assemble them into a matrix in order to exploit for correction. It also
%outputs the trackRange of the feature, which is the states, in which the
%features are tracked.


    N = size(camState, 2);
goodLength = true;

trackLen = N;
if (outFeat)
for i = 1 : N
ind = outOfView(featInd);
if (ind > size(camState{i}.features, 1))
trackLen = trackLen - 1;
else
obsMat(2 * i - 1 : 2 * i) = camState{i}.features(ind, : );
end
end
trackRange =[N - trackLen + 1, N];
% if trackRange is less than 5 threshold, don't correct
if (trackLen < 5)
goodLength = false;
end
else
for i = 1 : N
sz = size(camState{i}.features, 1);
if (featInd <= sz)
obsMat(2 * i - 1 : 2 * i) = camState{i}.features(featInd, : );
else
trackLen = trackLen - 1;
end
end
trackRange =[N - trackLen + 1, N];
end

end

