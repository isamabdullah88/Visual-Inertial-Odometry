function[ State ] = propagateImu( State, imuReadings, noiseParams )
%propagateImu propagates both imu state and covariance matrix

% propagate state
State.imuState = propagateImuState(State.imuState, imuReadings);
% Using rk4
% State.imuState = integrateIMU(State.imuState, imuReadings.acc, imuReadings.omega, imuReadings.dT, [0;0;0]);

[State.imuCovMat, State.cam_imu_CovMat] = propagateImuCovMat(State, imuReadings, noiseParams);

end

