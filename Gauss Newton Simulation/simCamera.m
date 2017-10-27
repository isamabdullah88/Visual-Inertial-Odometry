function [ obs, camStates ] = simCamera( C1_p_f, camera, C1_p_C2, thetas, n )
%simCamera simulates a camera of n frames given the parameters and returns 2D feature
%coordinates in each frame
%   C1_p_f  : 3D position coordinates of feature in C1, which would be
%             tracked in sub-sequent camera frames
%   camera  : The camera parameter matrix
%   thetas  : List of angles of each frame with respect to first frame
%   n       : The number of frames in which it is to track
%   C1_p_Ci : List of poses of each frame from first frame

%   obs     : 2Xn matrix. Each column contains 3D coordinates of feature in
%             each frame
%   poseC1  : 3D poses of each camera frame in C1

% Note: The maximum number of frames would only be, in which the feature
% does not go out of view.

% Poses of i frame in C1
C1_p_Ci = C1_p_C2;
K = camera.K;
tmpP = K * [C1_p_f; 1];
obs(:,1) = [tmpP(1)/tmpP(3); tmpP(2)/tmpP(3)];

for i=1:n
%     camStates{i}.q_CG = (angle2quat(0, thetas(i), 0, 'XYZ'))';
    camStates{i}.q_CG = [0;0;0;1];
    if (i==1)
        camStates{i}.p_C_G = [0;0;0];
    else
        camStates{i}.p_C_G = C1_p_Ci;
        % Translating camera Ci from Ci-1
        C1_p_Ci = C1_p_Ci + C1_p_C2;
    end

    C_C1_Ci = rotMat(thetas(i));
    % 3D feature pos in Ci
    Ci_p_f = C_C1_Ci * (C1_p_f - camStates{i}.p_C_G);
   
    % 2D pos of feature in Ci
    tmpPi = K * [Ci_p_f; 1];
    obs(:,i+1) = [tmpPi(1)/tmpPi(3); tmpPi(2)/tmpPi(3)];
    
end

% Coverting pixel coordinates to 'ideal' coordinates
obs(1,:) = (obs(1,:) - camera.px)/camera.mx;
obs(2,:) = (obs(2,:) - camera.py)/camera.my;


    function C_C1_C2 = rotMat(theta)
        
       C_C1_C2 = [cosd(theta)  0 sind(theta);
                  0            1           0
                  -sind(theta) 0 cosd(theta)];
    end


end



