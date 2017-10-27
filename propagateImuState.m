function propImuState = propagateImuState(imuState, imuReadings)
        
    %%% Propagating Quaternions
    % Rotation matrix from current quaternions
    C_IG = quatToRotMat(imuState.q_IG);
    
    omega = (imuReadings.omega - imuState.b_g) * imuReadings.dT;
    propImuState.q_IG = imuState.q_IG + (0.5 * omegaMat(omega) * imuState.q_IG);
    % normalize quaternions
    propImuState.q_IG = propImuState.q_IG/norm(propImuState.q_IG);
    
    
    %%% Propagating Position, Velocity
    %%% Correction to gravity
    % Assumptions:
    % 1) The car first rotates then translates
    % 2) Since the initial orientation is unknown, the car's z component is
    % assumed to be perfectly aligned with gravity vector.
    
    g_I = C_IG * [0;0;9.80964];
    % compensate for gravity
    acc = imuReadings.acc - g_I;
    
    % Velocity state
    dV = (acc - imuState.b_a) * imuReadings.dT;
    propImuState.v_I_G = imuState.v_I_G + (C_IG')*dV;
    
    % Position state
    vel = (imuState.v_I_G + propImuState.v_I_G)/2;
    pos = vel * imuReadings.dT;
    propImuState.p_I_G = imuState.p_I_G + pos;
    
    %%% Propagating Biases
    propImuState.b_g = imuState.b_g;
    propImuState.b_a = imuState.b_a;
    
    
end