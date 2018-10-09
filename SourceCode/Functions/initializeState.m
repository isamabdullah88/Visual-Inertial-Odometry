function[ State ] = initializeState()
%INITSTATE This function initializes the state and all the parameters


%%%%%%%%%%%%%%%%%%%%%%%%%%%%% THE IMU STATE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
imuState.q_IG =[0;0;0;1]; % 4X1 Global to IMU rotation quaternion,
% The specific dataset says that initial angle was not zero, it had some
% value which had a massive effect on the output
% imuState.q_IG =[0.0436; -0.9988; -0.0230; 0.0059];
% imuState.q_IG =[0.2632; -0.9646; -0.0161; 0.0091];
imuState.b_g = zeros(3, 1); % 3X1 Angular Acceleration bias
imuState.v_I_G = zeros(3, 1); % 3X1 velocity of IMU in Global Frame
% The specific dataset says that initial velocity was not zero, it had some
% value which had a massive effect on the output

% imuState.v_I_G =[13.1727166637690;-0.124752642931640;-0.0329199030473540];
imuState.v_I_G =[8.5187; 0.0547; -0.0395];
% imuState.v_I_G =[10.2502594620480;-0.0277261427025090;-0.0680871415633240];

imuState.b_a = zeros(3, 1); % 3X1 Linear Acceleration bias
imuState.p_I_G = zeros(3, 1); % 3X1 position of IMU in Global Frame

% imuState.covMat = zeros(9, 9); % 9X9 IMU State Covariance Matrix


%%%%%%%%%%%%%%%%%%%%%%%%%%%%% THE CAMERA STATE %%%%%%%%%%%%%%%%%%%%%%%%%%
camState.q_IG = zeros(4, 1); % 4X1 Global to Camera rotation quaternion,
camState.p_I_G = zeros(3, 1); % 3X1 position of IMU in Global Frame
camState.features =[]; % matrix for storing features corresponding to each camera frame


%%%%%%%%%%%%%%%%%%%%%%%%%%%%% THE MSCKF STATE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
State.imuState = imuState;
State.camState = {};
State.N = size(State.camState, 2); % Number of camStates currently augmented
State.imuCovMat = 1e - 6 * ones(15, 15); % 15X15 IMU State Covariance Matrix
State.camCovMat = 1e - 6 * ones(6 * State.N, 6 * State.N); % 6N X 6N camState Covariance Matrix
State.cam_imu_CovMat = 1e - 6 * ones(15, 6 * State.N); % 15 X 6N Cam to IMU cross Covariance Matrix


end

