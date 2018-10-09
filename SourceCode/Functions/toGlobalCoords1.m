function[ tGPS ] = toGlobalCoords1( gpsReadings )
%toGlobalCoords transforms the ecef coordinates to initial global
%coordinates of the vehicle in the dataset

len = size(gpsReadings, 1);

theta1 = 60.5;
Rz =[cosd(theta1) - sind(theta1) 0; sind(theta1) cosd(theta1) 0; 0 0 1];

for i = 1 : len
tGPS1(i, : ) = Rz * gpsReadings(i, : )';
end

theta2 = -2;
Rx =[1 0 0; 0 cosd(theta2) - sind(theta2); 0 sind(theta2) cosd(theta2)];
for i = 1 : len
tGPS2(i, : ) = Rx * tGPS1(i, : )';
end

theta3 = -16.5;
Ry =[cosd(theta3) 0 sind(theta3); 0 1 0; -sind(theta3) 0 cosd(theta3)];
for i = 1 : len
tGPS(i, : ) = Ry * tGPS2(i, : )';
end
% hold on; line(tGPS( :, 1), tGPS( : , 2), tGPS( : , 3));


end

