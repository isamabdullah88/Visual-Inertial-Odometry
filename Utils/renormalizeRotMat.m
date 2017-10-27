function C_unitary = renormalizeRotMat(C)
    % Enforce det(C) = 1 by finding the nearest orthogonal matrix
    [U,S,V] = svd(C);
    first = U*eye(size(S,1));
    C_unitary = first*V';
end