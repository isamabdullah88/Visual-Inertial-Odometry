function G = calcG(imuState)
% Multiplies the noise vector in the linearized continuous - time
% error state model

G = zeros(15, 12);

C_IG = quatToRotMat(imuState.q_IG);

G(1 : 3, 1 : 3) = -eye(3);
G(4 : 6, 4 : 6) = eye(3);
G(7 : 9, 7 : 9) = -C_IG';
G(10 : 12, 10 : 12) = eye(3);
%     G(13 : 15, 7 : 9) = -C_IG';

end
