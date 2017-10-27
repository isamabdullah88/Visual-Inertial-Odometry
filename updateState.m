function [ State, numFeatUsed ] = updateState( State, noiseParams, camParams, outOfView )
%updateState Updates and corrects the state estimate and covariance using
%feature position estimation

H_o = [];
r_o = [];
R_o = [];

%%% Find only the number of features before loop
%%% Also handle if this is a case of outOfView or not

% It also outputs true if it is the case of outofview
[numFeatures, outFeat] = computeFeatNum(State.camState, outOfView);

% obsMat = prepareObsMat(State.camState, outOfView);
% Limiting number of features to avoid weighting towards camera
% if (numFeatures > 20)
%     numFeatures = 20;
% end
numFeatUsed = 0;
costVec = [];
for featInd=1:numFeatures
    % Process one set of observations at a time
    [tmpObs, trackRange, goodLength] = prepareObsMat(State.camState, outFeat, featInd, outOfView);
%     tmpObs = obsMat(featInd,:);
    % if goodLength is true, i.e. camStates are greater than threshold,
    % then do this
    if (goodLength)
        obs = reshape(tmpObs, [2,length(tmpObs)/2]);
        % Making observations ideal (intrinsic) before employing 3D Gauss newton
        obs(1,:) = (obs(1,:) - camParams.c_u)/camParams.f_u;
        obs(2,:) = (obs(2,:) - camParams.c_v)/camParams.f_v;

        %Estimate feature 3D location through Gauss Newton inverse depth
        %optimization
        [p_f_G, Jcost, RCOND] = calcGNPosEst(State.camState, obs, trackRange);
        fprintf('Jcost = %f\n', Jcost);
        if (Jcost < eps)
            numFeatUsed = numFeatUsed + 1;
    %         plot3(p_f_G(1),p_f_G(2),p_f_G(3), '.'); hold on; grid on;
    %         drawnow;

            nObs = size(obs,2);
            JcostNorm = Jcost / nObs^2;
            %     fprintf('Jcost = %f | JcostNorm = %f | RCOND = %f\n',...
            %         Jcost, JcostNorm,RCOND);
            %
    %         fprintf('Jcost = %f\n', Jcost);
    %         costVec = [costVec, Jcost];

            %Calculate residual and Hoj
            r_j = calcResidual(p_f_G, State.camState, obs);
            R_j = diag(repmat([noiseParams.u_var_prime, noiseParams.v_var_prime], [1, numel(r_j)/2]));
            %     R_j = diag(repmat([noiseParams.u_var_prime, noiseParams.v_var_prime], [1, numel(r_j)/2]));

            [H_o_j, A_j, H_x_j] = calcHoj(p_f_G, State);

            % Stacked residuals and friends
            H_o = [H_o; H_o_j];

            if ~isempty(A_j)
                r_o_j = A_j' * r_j;
                r_o = [r_o ; r_o_j];

                R_o_j = A_j' * R_j * A_j;
                %         R_o_j = R_j(1:2*State.N-3,1:2*State.N-3);
                R_o(end+1 : end+size(R_o_j,1), end+1 : end+size(R_o_j,2)) = R_o_j;
            end

        end
    end
end

fprintf('Number of features used: %d\n', numFeatUsed);
fprintf('Total Number of features: %d\n', numFeatures);

% Put residuals into their final update-worthy form
% if msckfParams.doQRdecomp
if (~isempty(H_o))
%     fprintf('\n');
%     fprintf('Yes\n');
%     fprintf('\n');
%     disp('Yes');
    [T_H, Q_1] = calcTH(H_o);
    r_n = Q_1' * r_o;
    R_n = diag(diag(Q_1' * R_o * Q_1));
    % else
    %     T_H = H_o;
    %     r_n = r_o;
    %     R_n = R_o;
    % end
    
    % Build MSCKF covariance matrix
    P = [State.imuCovMat, State.cam_imu_CovMat;
        State.cam_imu_CovMat', State.camCovMat];
    
    % Calculate Kalman gain
    K = (P*T_H') / ( T_H*P*T_H' + R_n ); % == (P*T_H') * inv( T_H*P*T_H' + R_n )
    
    % State correction
    deltaX = K * r_n;
    % disp(deltaX(13:15));
    State = updateStateParams(State, deltaX);
    
    % Covariance correction
    tempMat = (eye(15 + 6*size(State.camState,2)) - K*T_H);
    %             tempMat = (eye(12 + 6*size(msckfState.camStates,2)) - K*H_o);
    
    P_corrected = tempMat * P * tempMat' + K * R_n * K';
    
    State.imuCovMat = P_corrected(1:15,1:15);
    State.camCovMat = P_corrected(16:end,16:end);
    State.cam_imu_CovMat= P_corrected(1:15, 16:end);
    
    % figure(1); clf; imagesc(deltaX); axis equal; axis ij; colorbar;
    % drawnow;
end
    


end

