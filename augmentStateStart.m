function State_aug = augmentStateStart(State, camera, trackedPts)
%augmentState Augments the MSCKF state with a new camera pose. It would 
% also handle the tracked features in each camera frame removing any out of
% view features.

    N = size(State.camState,2);

    C_IG = quatToRotMat(State.imuState.q_IG);
    
    % Compute camera pose from current IMU pose
    q_CG = quatLeftComp(camera.q_CI) * State.imuState.q_IG;
    p_C_G = State.imuState.p_I_G + C_IG' * camera.p_C_I;

    % Build MSCKF covariance matrix
    P = [State.imuCovMat, State.cam_imu_CovMat;
        State.cam_imu_CovMat', State.camCovMat];
    
    % Break everything into appropriate structs
    State_aug = State;
    State_aug.camState{N+1}.p_C_G = p_C_G;
    State_aug.camState{N+1}.q_CG = q_CG;
    
    % Append (and remove out-of-view) the camera features
    State_aug.camState = removeOutOfView(State_aug.camState, trackedPts, N, 0);
    
    % Camera state Jacobian
    J = calcJ(State, camera);
    
    tempMat = [eye(15 + 6*N); J];
    
    % Augment the MSCKF covariance matrix
    P_aug = tempMat * P * tempMat';
    
    State_aug.imuCovMat = P_aug(1:15,1:15);
    State_aug.camCovMat = P_aug(16:end,16:end);
    State_aug.cam_imu_CovMat = P_aug(1:15, 16:end);
    State_aug.N = N+1;
end