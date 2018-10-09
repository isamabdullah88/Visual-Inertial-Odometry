function[p_f_G, cost, RCOND] = calcGNPosEst(camState, observations, noiseParams)
%CALCGNPOSEST Calculate the position estimate of the feature using Gauss
%Newton optimization
%   INPUT :
%   observations : 2xM matrix of pixel values of the current landmark
%   camStates : Cell array of M structs of camera poses
%   camera : intrinsic calibration
%   OUTPUT :
%   p_f_G : 3x1 feature vector in the global frame

%K is not needed if we assume observations are not pixels but x' = (u -
%c_u) / f_u

%K =[camera.f_u 0 camera.c_u; 0 camera.f_v camera.c_v; 0 0 1];



%% Actual Problem
%Get initial estimate through intersection
%Use the first 2 camStates
CnInd = length(camState);

C_1n = quatToRotMat(camState{CnInd}.q_CG)' * quatToRotMat(camState{1}.q_CG);
% t_21_1 = quatToRotMat(camState{secondViewIdx}.q_CG) * (camState{secondViewIdx}.p_C_G - camState{1}.p_C_G);
t_1n_1 = quatToRotMat(camState{1}.q_CG) * (camState{CnInd}.p_C_G - camState{1}.p_C_G);

p_f1_1_bar = triangulate(observations( :, 1), observations( : , CnInd), C_1n, t_1n_1);
x0 =[p_f1_1_bar(1) / p_f1_1_bar(3); p_f1_1_bar(2) / p_f1_1_bar(3); 1 / p_f1_1_bar(3)];

[params, cost] = lsqnonlin(@objFn, double(x0));
p_f_G = params;
RCOND = 1;




%% The Objective Function
function r = objFn(beta)
% beta =[X, Y, Z]of the feature position in global frame. These
% are our parameters to estimate
% r = f(X, beta) - Y : These are the list of residuals for all the
% data points

lastInd = length(camState);

for i = 2 : lastInd
C_1i = quatToRotMat(camState{i}.q_CG)' * quatToRotMat(camState{i}.q_CG);
%             t_i1_1 = quatToRotMat(camState{1}.q_CG) * (camState{i}.p_C_G - camState{1}.p_C_G);
t_i1_1 = quatToRotMat(camState{1}.q_CG) * (camState{i}.p_C_G - camState{1}.p_C_G);

%             C1 = C_1i( : , 1); C2 = C_1i( : , 2); C3 = C_1i( : , 3);
%             B1 = beta(1); B2 = beta(2); B3 = beta(3);
%             X1 = t_i1_1(1); X2 = t_i1_1(2); X3 = t_i1_1(3);

% Calculate f(X, B)
fXBVec = C_1i * (beta - (t_i1_1 * beta(3)));
f1XB = fXBVec(1) / fXBVec(3); f2XB = fXBVec(2) / fXBVec(3);

% Compute residual of norms
fXB =[f1XB, f2XB];
obs = double(observations( : , i));
y =[obs(1), obs(2)];

r(i - 1) = norm(y - fXB);
end
end


function[p_f1_1] = triangulate(obs1, obs2, C_12, t_21_1)
% triangulate Triangulates 3D points from two sets of feature vectors and a
% a frame - to - frame transformation

%Calculate unit vectors
v_1 =[obs1;1];
v_2 =[obs2;1];
v_1 = v_1 / norm(v_1);
v_2 = v_2 / norm(v_2);

A =[v_1 - C_12 * v_2];
b = t_21_1;
%            disp(b);

scalar_consts = A\b;
p_f1_1 = scalar_consts(1) * v_1;
end

end

