This result is obtained using fixed imu parameters and varing camera parameters. Specifically the
number of camera States tracked has been increased. and the number of detected features have been kept unlimited.
Also, the in-between update has been DISABLED.
Detect Threshold has been ENABLED
Following are the values:

camStates: 15
no of features detected: unlimited (corner)

BEST results are:

imuNoise : 1.08e-4;
imuBias : 1.00e-5;

camNoise: 1:0.1:6

ENABLING in-between update proves, drastic decrease in accuracy.