function [ State, costVec ] = updateStateAndCov( State, noiseParams, camParams, costVec)
%updateState2 updates the state, when number of camera frame reaches Nmax
%by correcting using features of Nmax/3 camera poses, starting from second
%left
%   Detailed explanation goes here
%__________________________________________________________________________

[State, numFeatUsed] = updateState(State, noiseParams, camParams, []);
costVec = [costVec numFeatUsed];

% Reset camStates and N
State.camState = [];
State.N = 0;

% Reset camera Covariance and imu to cam Covariance
State.camCovMat = zeros(6*State.N,6*State.N); % 6N X 6N camState Covariance Matrix
State.cam_imu_CovMat = zeros(15, 6*State.N); % 15 X 6N Cam to IMU cross Covariance Matrix

end

