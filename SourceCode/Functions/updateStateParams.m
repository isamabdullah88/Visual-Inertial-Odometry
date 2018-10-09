function State_up = updateStateParams(State, deltaX)
% Updates MSCKF state with deltaX

% Initialize updated state with current state
State_up = State;

% Update IMU State
deltatheta_IG = deltaX(1 : 3);
deltab_g = deltaX(4 : 6);
deltav_I_G = deltaX(7 : 9);
deltab_a = deltaX(10 : 12);
deltap_I_G = deltaX(13 : 15);

deltaq_IG = buildUpdateQuat(deltatheta_IG);

State_up.imuState.q_IG = quatLeftComp(deltaq_IG) * State.imuState.q_IG;
State_up.imuState.b_g = State.imuState.b_g + deltab_g;
State_up.imuState.v_I_G = State.imuState.v_I_G + deltav_I_G;
State_up.imuState.b_a = State.imuState.b_a + deltab_a;
State_up.imuState.p_I_G = State.imuState.p_I_G + deltap_I_G;

% Update camera states
for i = 1 : size(State.camState, 2)
qStart = 15 + 6 * (i - 1) + 1;
pStart = qStart + 3;

deltatheta_CG = deltaX(qStart : qStart + 2);
deltap_C_G = deltaX(pStart : pStart + 2);

deltaq_CG = buildUpdateQuat(deltatheta_CG);

State_up.camState{i}.q_CG = quatLeftComp(deltaq_CG) * State.camState{i}.q_CG;
State_up.camState{i}.p_C_G = State.camState{i}.p_C_G + deltap_C_G;
end

end
