function [p_f_G, Jderiv,RCOND] = calcGNPosEst(camStates, observations, trackRange)
%CALCGNPOSEST Calculate the position estimate of the feature using Gauss
%Newton optimization
%   INPUT:
%   observations: 2xM matrix of pixel values of the current landmark
%   camStates: Cell array of M structs of camera poses
%   camera: intrinsic calibration
%   OUTPUT:
%   p_f_G: 3x1 feature vector in the global frame

%K is not needed if we assume observations are not pixels but x' = (u -
%c_u)/f_u

%K = [camera.f_u 0 camera.c_u; 0 camera.f_v camera.c_v; 0 0 1];

%Get initial estimate through intersection
%Use the first 2 camStates
firstViewInd = trackRange(1);
secondViewInd = trackRange(2);

C_12 = quatToRotMat(camStates{firstViewInd}.q_CG)*quatToRotMat(camStates{secondViewInd}.q_CG)';
t_21_1 = quatToRotMat(camStates{firstViewInd}.q_CG)*(camStates{secondViewInd}.p_C_G - camStates{firstViewInd}.p_C_G);

p_f1_1_bar = triangulate(observations(:,1), observations(:,secondViewInd),C_12, t_21_1);

%initialEst = quatToRotMat(camStates{1}.q_CG)'*p_f1_1_bar + camStates{1}.p_C_G;


xBar = p_f1_1_bar(1);
yBar = p_f1_1_bar(2);
zBar = p_f1_1_bar(3);

alphaBar = xBar/zBar;
betaBar = yBar/zBar;
rhoBar = 1/zBar;

xEst = [alphaBar; betaBar; rhoBar];

Cnum = length(camStates);

%Optimize
maxIter = 15;
Jprev = Inf;

for optI = 1:maxIter
    %optI
    Jf = zeros(2*Cnum, 3);
    W = zeros(2*Cnum, 2*Cnum);
    residual = zeros(2*Cnum, 1);

    for iState = 1:Cnum
        %Form the weight matrix
%         W((2*iState - 1):(2*iState),(2*iState - 1):(2*iState)) = diag([noiseParams.u_var_prime noiseParams.v_var_prime]);

        C_i1 = quatToRotMat(camStates{iState}.q_CG)*(quatToRotMat(camStates{firstViewInd}.q_CG)');
        t_1i_i = -quatToRotMat(camStates{iState}.q_CG)*(camStates{iState}.p_C_G - camStates{firstViewInd}.p_C_G);
        

        %Form the error vector
        zHat = observations(:, iState);
        h = C_i1*[alphaBar; betaBar; 1] + rhoBar*t_1i_i;

        residual((2*iState - 1):(2*iState),1) = zHat - [h(1); h(2)]/h(3);


        %Form the Jacobian
        dEdalpha = [-C_i1(1,1)/h(3) + (h(1)/h(3)^2)*C_i1(3,1); ...
                    -C_i1(2,1)/h(3) + (h(2)/h(3)^2)*C_i1(3,1)];

        dEdbeta =  [-C_i1(1,2)/h(3) + (h(1)/h(3)^2)*C_i1(3,2); ...
                    -C_i1(2,2)/h(3) + (h(2)/h(3)^2)*C_i1(3,2)];

        dEdrho =   [-t_1i_i(1)/h(3) + (h(1)/h(3)^2)*t_1i_i(3); ...
                    -t_1i_i(2)/h(3) + (h(2)/h(3)^2)*t_1i_i(3)];

        Eblock = [dEdalpha dEdbeta dEdrho];
        Jf((2*iState - 1):(2*iState), :) = Eblock;
    end
    
    %Calculate the cost function
    Jnew = 0.5*residual'*(residual);
    %Solve!
    
%     EWE = E'*(W\E);
    RCOND = rcond(Jf'*Jf);
    dx_star =  -(Jf' * Jf) \ (Jf' * residual); 
    
    xEst = xEst + dx_star;
    
    Jderiv = abs(Jnew - Jprev);
    
    Jprev = Jnew;

    if (Jderiv < eps || Jnew < eps)
        break;
    else
        alphaBar = xEst(1);
        betaBar = xEst(2);
        rhoBar = xEst(3);
    end
    
end

p_f_G = (1/xEst(3))*quatToRotMat(camStates{firstViewInd}.q_CG)'*[xEst(1:2); 1] + camStates{firstViewInd}.p_C_G; 


    function [p_f1_1] = triangulate(obs1, obs2, C_12, t_21_1)
        % triangulate Triangulates 3D points from two sets of feature vectors and a
        % a frame-to-frame transformation

           %Calculate unit vectors 
           v_1 = [obs1;1];
           v_2 = [obs2;1];
           v_1 = v_1/norm(v_1);
           v_2 = v_2/norm(v_2);

           A = [v_1 -C_12*v_2];
           b = t_21_1;

           scalar_consts = A\b;
           p_f1_1 = scalar_consts(1)*v_1;
    end

end

