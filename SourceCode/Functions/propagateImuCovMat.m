function[ imuCovMat, cam_imu_CovMat ] = propagateImuCovMat( State, imuReadings, noiseParams )
%propogateImuCovMat Propagates the IMU State Covariance matrix

% Jacobians
Q_imu = noiseParams.Q_imu;
F = calcF(State.imuState, imuReadings);
G = calcG(State.imuState);

% State Transition Matrix
Phi = eye(size(F, 1)) + F * imuReadings.dT; % Leutenegger 2013

% IMU - IMU Covariance
imuCovMat = State.imuCovMat + ...
( F * State.imuCovMat ...
    + State.imuCovMat * F' ...
    + G * Q_imu * G' ) ...
    * imuReadings.dT;

%     msckfState_prop.imuCovar = Phi * msckfState.imuCovar * Phi' ...
%                                 + G * Q_imu * G' * measurements_k.dT; % Leutenegger 2013

% Enforce PSD - ness
imuCovMat = enforcePSD(imuCovMat);

% IMU - Camera Covariance
cam_imu_CovMat = Phi * State.cam_imu_CovMat;

end

