clear all; clc; %hold on;

addpath('Utils');

%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ NOTATION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% X_sub_super
% q_ToFrom
% p_ofWhat_expressedInWhatFrame

Parameters.NMAX_FRAMES = 5;
Parameters.firstTime = 1;

%% ~~~~~~~~~~~~~~~~~~~~~LOAD CALIBRATION~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
dataCalibDir = 'Data\2011_09_26_calib\2011_09_26';
[T_camvelo_struct, P_rect_cam1] = loadCalibration(dataCalibDir);
T_camvelo = T_camvelo_struct{1};
T_veloimu = loadCalibrationRigid(fullfile(dataCalibDir,'calib_imu_to_velo.txt'));
T_camimu = T_camvelo*T_veloimu;
R_ci = T_camimu(1:3,1:3);
T_imucam = inv(T_camimu);
p_ci_i = T_imucam(1:3,4);
C_c_v = R_ci;
rho_v_c_v = p_ci_i;

%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ LOADING DATA ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

% ################## LOADING IMU DATA #####################################
% imuData = csvread('Data/DATA_20170120_203131.txt');
% First 5-min data is used to find the bias in acceleration
% accBias = [mean(imuData(1:15000,1)); mean(imuData(1:15000,2)); mean(imuData(1:15000,3))];
% accBias = [0; 0; 0];
%
% dataSize = length(imuData(:,1));
% imuReadings.acc = imuData(:,1:3);
% imuReadings.acc(:,3) = imuReadings.acc(:,3) - mean(imuReadings.acc(:,3));
% imuReadings.omega = imuData(:,4:6);
% imuReadings.dT = [0; diff(imuData(:,7))];

%%%% KITTI DATASET %%%%%
imuDir = 'Data\2011_09_26_drive_0095_sync\oxts';
[imuReadings, gpsReadings, accuracy] = loadImuData(imuDir);
% loading Malaga dataset
% imuReadings = loadMalagaData();
dataSize = length(imuReadings.dT);

% ################## LOADING IMAGE DATA #####################################
% % Read all data to memory
%%%% KITTI DATASET %%%%%
fileDir = 'Data\2011_09_26_drive_0095_sync\image_01\data';
fileNames = dir(fullfile(fileDir, '*.png'));

%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Initializing States, Parameters ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%### Initialize the State
State = initState();

%### Setup Noise Parameters
% % %## IMU
q_var = 4e-2 * ones(1,3);               % rot vel var
a_var = 4e-2 * ones(1,3);               % lin vel var
dbq_var = 1e-6 * ones(1,3);            % gyro bias change var
dba_var = 1e-6 * ones(1,3);            % vel bias change var
% q_var = [0.00010156, 0.00010774, 0.00016595].^2; % orientation var
% dbq_var = [1.4340e-05, 7.5519e-06, 6.4870e-04].^2; % bias in orient
% a_var = [0.010156, 0.010774, 0.036595].^2; % lin vel var
% dba_var = [1.4340e-05, 7.5519e-06, 6.4870e-04].^2; % vel bias change var
noiseParams.Q_imu = diag([q_var, dbq_var, a_var, dba_var]);

%## CAMERA
%Set up the camera parameters
camera.c_u      = 609.5593;                   % Principal point [u pixels]
camera.c_v      = 172.8540;                   % Principal point [v pixels]
camera.f_u      = 721.5377;                   % Focal length [u pixels]
camera.f_v      = 721.5377;                   % Focal length [v pixels]
camera.q_CI     = rotMatToQuat(C_c_v);  % 4x1 IMU-to-Camera rotation quaternion
camera.p_C_I    = rho_v_c_v;            % 3x1 Camera position in IMU frame

y_var = 11^2 * ones(1,4);               % pixel coord var
noiseParams.u_var_prime = y_var(1)/camera.f_u^2;
noiseParams.v_var_prime = y_var(2)/camera.f_v^2;

%%% TODO: Make permenant solution to this
% State.imuState.covMat = State.imuCovMat;

%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ STATE ESTIMATE PROPAGATION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

% Point tracker to track
pointTracker = vision.PointTracker('MaxBidirectionalError',1);
% data structure to store tracked points
% trackedPts = [];
countImg = 0;
countFrame = 1;
imgIndex = 1;
trackOutOfView = [];

% Data structure for plotting
statePos = [0,0,0];
stateVel = [0,0,0];

% Looping through each data point to propagate
% k starts from 2, because 1 is used to initialize
for k=1:dataSize-7
    imuPatch.acc = imuReadings.acc(k,:)';
    imuPatch.omega = imuReadings.omega(k,:)';
    imuPatch.dT = imuReadings.dT(k);
    State = propagateImu(State, imuPatch, noiseParams);
    statePos = [statePos; State.imuState.p_I_G'];
    stateVel = [stateVel; State.imuState.v_I_G'];
    
    % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ STATE ESTIMATE UPDATE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    currImg = imread([fileDir '/' fileNames(imgIndex).name]);
    imgIndex = imgIndex + 1;
    if mod(countImg, Parameters.NMAX_FRAMES) == 0
        
        if countImg ~= 0 % This is just condition for first-time update
            State = updateState(State, trackedPts, noiseParams, camera);
            State = resetState(State);
            % Set outOfView tracks to empty in order to restart
            trackOutOfView = [];
%             release(pointTracker); % Instead of release, use setPoints
        end
        
        % After Update, Again start tracking points
        [State, trackedPts, countImg, Parameters] = startTracking( currImg, pointTracker, State, camera, countImg, Parameters);
        
    else
        
        % Continue track and handle out-of-view features
        [ State, trackedPts, countImg, trackOutOfView ] = handleTrackedFeatures(pointTracker, currImg, trackedPts, State, ...
            camera, countImg, noiseParams, camera, trackOutOfView);
    end
    
    countFrame = countFrame + 1;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %     end
    
    
%     countFrame = countFrame + 1;
    
end
figure; line(statePos(:,1), statePos(:,2), statePos(:,3),'Color', 'k');
xlabel('x');ylabel('y');zlabel('z'); grid on;
title('Integrated');
% % legend('1:imu','1:gT','1:msckf','myGPS','myInteg');
% % axis equal;
% 
% %%% GPS Plot
tGPS = toGlobalCoords1(gpsReadings);
% tGPS = gpsData;
hold on; line(tGPS(:,1), tGPS(:,2), tGPS(:,3),'Color','m');
title('Ground Truth GPS in x-y-z plot');
xlabel('pos X'); ylabel('pos Y'); zlabel('pos Z'); grid on;