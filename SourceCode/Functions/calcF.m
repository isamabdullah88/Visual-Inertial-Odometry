function F = calcF(imuState, imuReadings)
% Multiplies the error state in the linearized continuous - time
% error state model

F = zeros(15, 15);

omegaHat = imuReadings.omega - imuState.b_g;
aHat = imuReadings.acc - imuState.b_a;
C_IG = quatToRotMat(imuState.q_IG);

F(1 : 3, 1 : 3) = -crossMat(omegaHat);
F(1 : 3, 4 : 6) = -eye(3);
F(7 : 9, 1 : 3) = -C_IG' * crossMat(aHat);
F(7 : 9, 10 : 12) = -C_IG';
F(13 : 15, 7 : 9) = eye(3);

end
