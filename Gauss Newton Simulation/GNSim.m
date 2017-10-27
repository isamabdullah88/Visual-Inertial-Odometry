

clear all;
addpath('Utils');

% The 3D feature point to optimize in C1
C1_p_f = [3; 5; 10];


%% Setting up Camera

camera.px      = 320;                   % Principal point [px pixels]
camera.py      = 240;                   % Principal point [py pixels]
camera.mx      = 600;                   % Focal length [mx pixels]
camera.my      = 600;                   % Focal length [my pixels]

% The Intrinsics
K = [camera.mx,     0,     camera.px, 0;
     0,         camera.my, camera.py, 0;
     0,             0,         1    , 0];
 
camera.K = K;

noiseParams.u_var_prime = 1e-30;
noiseParams.v_var_prime = 1e-30;
 
%% Image/Feature projection

% "Ideal" coordinates in the C1 camera frame

% {C1} frame origin is (0,0,0).
% Given a 3d point G_p_f in G frame, it's 3d coordinate in camera frame
% {C1} can be found as follows:
%               C1_p_f = C_C1_C1 * (C1_p_f - C1_p_C1)


theta = zeros(20,1);                % Angle in degrees around Y
% Rotation
           
C1_p_C2 = [1; 0; 0];       % Camera is 0.1m along x

[observations, camStates] = simCamera(C1_p_f, camera, C1_p_C2, theta, 20);

% Plotting the poses in C1
% P = [obs(1,:)' obs(2,:)' zeros(5,1)];
% plot3(C1_p_f(1),C1_p_f(2),C1_p_f(3),'*');
% hold on;plot3(pose(1,:),pose(2,:),pose(3,:),'o');
% hold on; plot3(P(:,1), P(:,2), P(:,3),'o');
% hold on;plot(obs(1,:),obs(2,:),'o')
% grid on;
% xlabel('X'); ylabel('Y');

[p_f_G, Jnew,RCOND] = calcGNPosEst(camStates, observations, noiseParams);



