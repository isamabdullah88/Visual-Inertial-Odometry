clear all; clc; %hold on;

addpath('Utils');
addpath('siftDemoV4');

%% ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ NOTATION ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~
% X_sub_super
% q_ToFrom
% p_ofWhat_expressedInWhatFrame

Parameters.NMAX_FRAMES = 5;
Parameters.firstTime = 1;

%% ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~ LOAD CALIBRATION ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

[C_c_v, rho_v_c_v] = setCalibration();

%% ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ LOADING DATA ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

% ################## LOADING IMU DATA #####################################
%%%% MAV ETH DATASET %%%%%
imuDir = 'Data\2011_09_30_drive_0016_sync\oxts';
[imuReadings, gpsReadings, accuracy] = loadImuData(imuDir);
% imuReadings = loadImuData2(imuDir);
dataSize = length(imuReadings.dT);

% ################## LOADING IMAGE DATA ###################################
%%%% MAV ETH DATASET %%%%%
fileDir = 'Data\2011_09_30_drive_0016_sync\image_00\data';
fileNames = dir(fullfile(fileDir, ' * .png'));

%% ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ Initializing States, Parameters ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~
%### Initialize the State
State = initializeState();

%### Setup Noise Parameters
[noiseParams, camera] = setNoiseParams(C_c_v, rho_v_c_v);


%% ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~ STATE ESTIMATE PROPAGATION ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

% Point tracker to track
pointTracker = vision.PointTracker('MaxBidirectionalError', 0.1);
% data structure to store tracked points
% trackedPts =[];
countImg = 0;
countFrame = 1;
imgIndex = 1;
trackOutOfView =[];

% Data structure for plotting
statePos =[0, 0, 0];
stateVel =[0, 0, 0];
costVec =[];
costVec2 =[];

% Looping through each data point to propagate
% k starts from 2, because 1 is used to initialize
for k = 1 : dataSize
imuPatch.acc = imuReadings.acc(k, : )';
imuPatch.omega = imuReadings.omega(k, : )';
imuPatch.dT = imuReadings.dT(k);
State = propagateImu(State, imuPatch, noiseParams);
statePos =[statePos; State.imuState.p_I_G'];
stateVel =[stateVel; State.imuState.v_I_G'];


% ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~ STATE ESTIMATE UPDATE ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
currImg = imread([fileDir ' /' fileNames(k).name]);

if (mod(State.N, Parameters.NMAX_FRAMES) == 0)
if (countImg > 0)
% Update using all the features of Nmax / 3 frames
[State, costVec] = updateStateAndCov(State, trackedPts, noiseParams, camera, costVec);
[State, trackedPts, Parameters] = startTracking( currImg, pointTracker, State, camera, Parameters);

else

[State, trackedPts, Parameters] = startTracking( currImg, pointTracker, State, camera, Parameters);
end
countImg = countImg + 1;
else
countImg = countImg + 1;

% Continue track and handle out - of - view features
[ State, trackedPts, costVec2 ] = handleTrackedFeatures(pointTracker, ...
currImg, trackedPts, State, camera, costVec2, noiseParams, camera);

pts = trackedPts( : , end - 1 : end);
setPoints(pointTracker, pts);

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~ STATE ESTIMATE UPDATE ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~ ~~

end

hold on; line(statePos( :, 1), statePos( : , 2), statePos( : , 3), 'Color', 'g');
xlabel('x');ylabel('y');zlabel('z'); grid on;
title('Integrated');
% % legend('1 : imu', '1 : gT', '1 : msckf', 'myGPS', 'myInteg');
% % axis equal;
%
%%% GPS Plot
tGPS = toGlobalCoords5(gpsReadings);
% tGPS = gpsReadings;
hold on; line(tGPS(1 : end, 1), tGPS(1 : end, 2), tGPS(1 : end, 3), 'Color', 'm');
title('Ground Truth GPS in x - y - z plot');
xlabel('pos X'); ylabel('pos Y'); zlabel('pos Z'); grid on;


