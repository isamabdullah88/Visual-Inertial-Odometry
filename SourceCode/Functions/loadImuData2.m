function[ imuData ] = loadImuData2( imuDir )
%loadImuData loads the imu data from kitti dataset in required format i.e.
%gyro, accelerometer and timestamps

fileDir =[imuDir ' / data.csv'];
fileName = fullfile(fileDir);
dataFull = csvread(fileName);
len = size(dataFull, 1);

% seperating required data
for i = 1 : len
dataF = dataFull(i, : );

timeStamps(i) = dataF(1) * (1e - 09);
omega = dataF(2 : 4);
acc = dataF(5 : 7);

imuData.omega(i, : ) = omega;
imuData.acc(i, : ) = acc;
end

imuData.dT = diff(timeStamps);
