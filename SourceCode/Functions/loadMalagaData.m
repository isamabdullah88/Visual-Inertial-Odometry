function[ imuData ] = loadMalagaData()
%loadMalagaData extracts and formats the data from malaga dataset

fileDir = 'Data\Malaga\malaga - urban - dataset - extract - 02_all - sensors_IMU.txt';
m = csvread(fileDir);
dataSize = size(m);

j = 1;
for i = 1 : 16 : dataSize - 16
timeStamps(j) = m(i);
acc = m(i + 1 : i + 3);
imuData.acc(j, : ) = acc;
omega =[m(i + 5 : i + 6); m(i + 4)];
imuData.omega(j, : ) = omega;
j = j + 1;
end

imuData.dT = diff(timeStamps);

end

