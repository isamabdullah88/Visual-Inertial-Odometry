X = linspace(0, 3);
Xn1 = X + 0.1 * randn(size(X));
Xn2 = X + ((X.^2). * rand(size(X)));

Y = exp(-1.5 * X) + 0.05 * randn(size(X));

modelFun1 = @(X, beta)exp(-X * beta);
fun2 = @(r)exp(-Xn1 * r) - Y;
fun3 = @(r)exp(-Xn2 * r) - Y;


%% Non - Weighted Fitting

beta0 = 4;
mdl1 = fitnlm(Xn2, Y, modelFun1, beta0);
% p2 = lsqnonlin(fun2, x0);
% p3 = lsqnonlin(fun3, x0);

beta1 = mdl1.Coefficients.Estimate;

plot(Xn2, Y, 'go'); hold on;
plot(Xn2' , predict(mdl1, Xn2'), 'r', 'linestyle', '--');
% figure; plot(Xn1, Y, 'ko', Xn1, exp(-p2 * Xn1), 'b -');
% figure; plot(Xn2, Y, 'ko', Xn2, exp(-p3 * Xn2), 'b -');

%% Weighted Non - linear Fitting

w = 1 : length(Xn2);
wnlm1 = fitnlm(Xn2, Y, modelFun1, beta0, 'Weight', w);
line(Xn2', predict(wnlm1, Xn2'), 'color', 'b')

% plot(X, Y, '.');
% figure; plot(Xn1, Y, '.');
% figure; plot(Xn2, Y, '.');

