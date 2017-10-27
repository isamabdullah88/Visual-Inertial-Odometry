function [ imuData, gpsData, velocity ] = loadImuData( imuDir )
%loadImuData loads the imu data from kitti dataset in required format i.e.
%gyro, accelerometer and timestamps

fileDir = [imuDir '/data/'];
fileNames = dir(fullfile(fileDir, '*.txt'));
dateStrings = loadTimestamps(imuDir);
len = size(dateStrings,1);

wgs84 = wgs84Ellipsoid('meters');
% seperating required data
for i=1:len
    dataF = dlmread([fileDir '/' fileNames(i).name]);
    tmpGps = dataF(1:3);
    gpsLatLong(i,:) = tmpGps;
    [x, y, z] = geodetic2ecef(wgs84, tmpGps(1), tmpGps(2), tmpGps(3));
    gpsData(i,:) = [x, y, z];
    acc(i,:) = [dataF(12), dataF(13), dataF(14)];
    omega(i,:) = dataF(18:20);
    velocity(i,:) = [dataF(9), dataF(10), dataF(11)];
end
imuData.acc = acc;
imuData.omega = omega;

% Subtract GPS from first value and plot ground truth
gpsData(:,1) = gpsData(:,1) - gpsData(1,1);
gpsData(:,2) = gpsData(:,2) - gpsData(1,2);
gpsData(:,3) = gpsData(:,3) - gpsData(1,3);

% tGPS = toGlobalCoords(gpsData);
% % tGPS = gpsData;
% figure; line(tGPS(:,1), tGPS(:,2), tGPS(:,3),'Color','m');
% title('Ground Truth GPS in x-y-z plot');
% xlabel('pos X'); ylabel('pos Y'); zlabel('pos Z');

% converting from date to time
for i=1:len
    dataTs(i) = dateToTime(datenum(dateStrings(i)));
end

imuData.dT = diff(dataTs);



%%% Function to convert data to time --------------------------------------
    function t = dateToTime(date)
        t =  (date - 719529)*86400;         %# 719529 == datenum(1970,1,1)
    end
end

