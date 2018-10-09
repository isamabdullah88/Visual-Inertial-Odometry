function[xUpdate] = integrateIMU(xPrev, a, omega, dt, g_w)
%INTEGRATEIMUSTREAM Integrate IMU stream and return state and covariance
%estimates


%The state is given by :
%x =[p_I_G q_IG v_I_G b_g b_a];

%Set noise to 0
noise_params.sigma_g = 0;
noise_params.sigma_a = 0;
noise_params.sigma_bg = 0;
noise_params.sigma_ba = 0;
noise_params.tau = 10^12;



%Calculate xDot
w = getWhiteNoise(noise_params);

%RK4 Integration
% Given x' = f(x), RK4 integration proceeds as follows :
% x(n + 1) = x(n) + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4); where
% k1 = f(x);
% k2 = f(x + dt / 2 * k1)
% k3 = f(x + dt / 2 * k2)
% k4 = f(x + dt * k3)

k = {};

k_coeffs =[1 0.5 0.5 1];
b_coeffs =[1 / 6 1 / 3 1 / 3 1 / 6]; %RK4
%b_coeffs =[1 0 0 0]; %Forward Euler

k{1} = getxDot(xPrev, omega, a, w, noise_params, g_w);

for i_k = 2 : 4
xTemp.p_I_G = xPrev.p_I_G + (k{i_k - 1}.p_I_G) * k_coeffs(i_k) * dt;
xTemp.q_IG = xPrev.q_IG + (k{i_k - 1}.q_IG) * k_coeffs(i_k) * dt; %Possibly renormalize here
xTemp.v_I_G = xPrev.v_I_G + (k{i_k - 1}.v_I_G) * k_coeffs(i_k) * dt;
xTemp.b_g = xPrev.b_g + (k{i_k - 1}.b_g) * k_coeffs(i_k) * dt;
xTemp.b_a = xPrev.b_a + (k{i_k - 1}.b_a) * k_coeffs(i_k) * dt;
k{i_k} = getxDot(xTemp, omega, a, w, noise_params, g_w);
end

xUpdate.p_I_G = xPrev.p_I_G + dt * (b_coeffs(1) * k{1}.p_I_G + b_coeffs(2) * k{2}.p_I_G + b_coeffs(3) * k{3}.p_I_G + b_coeffs(4) * k{4}.p_I_G);
xUpdate.v_I_G = xPrev.v_I_G + dt * (b_coeffs(1) * k{1}.v_I_G + b_coeffs(2) * k{2}.v_I_G + b_coeffs(3) * k{3}.v_I_G + b_coeffs(4) * k{4}.v_I_G);
xUpdate.b_g = xPrev.b_g + dt * (b_coeffs(1) * k{1}.b_g + b_coeffs(2) * k{2}.b_g + b_coeffs(3) * k{3}.b_g + b_coeffs(4) * k{4}.b_g);
xUpdate.b_a = xPrev.b_a + dt * (b_coeffs(1) * k{1}.b_a + b_coeffs(2) * k{2}.b_a + b_coeffs(3) * k{3}.b_a + b_coeffs(4) * k{4}.b_a);
xUpdate.q_IG = quat_normalize(xPrev.q_IG + dt * (b_coeffs(1) * k{1}.q_IG + b_coeffs(2) * k{2}.q_IG + b_coeffs(3) * k{3}.q_IG + b_coeffs(4) * k{4}.q_IG)); %Note the renormalization



%Non linear kinematics
% x - state
% w - noise vector
% omega, a - ang. rates and accels.
% tau - time constant for accelerometer bias
function xdot = getxDot(x, omega, a, w, noise_params, g_w)
xdot.p_I_G = x.v_I_G;
xdot.q_IG = 0.5 * omegaMat(omega, zeros(3, 1), x.b_g) * x.q_IG;
xdot.v_I_G = rotmat_from_quat(quat_normalize(x.q_IG)) * (a - x.b_a) + rotmat_from_quat(quat_normalize(x.q_IG)) * g_w;
xdot.b_g = zeros(3, 1);
xdot.b_a = zeros(3, 1);

end

%White noise vector
function w = getWhiteNoise(sigma_params)
w.w_g = sigma_params.sigma_g. * randn(3, 1);
w.w_a = sigma_params.sigma_a. * randn(3, 1);
w.w_bg = sigma_params.sigma_bg. * randn(3, 1);
w.w_ba = sigma_params.sigma_ba. * randn(3, 1);
end
%Quaternion kinematic matrix
function bigOmega = omegaMat(omega_meas, w_g, b_g)
%Is it + w_g or - w_g? Does it matter?
    omega_true = omega_meas + w_g - b_g;
ox = omega_true(1);
oy = omega_true(2);
oz = omega_true(3);
bigOmega =[0 - ox - oy - oz; ...
  ox 0 oz - oy;
  oy - oz 0 ox;
  oz oy - ox 0;];
%See technical note on quaternions by Sola. Page 6.
%This formulation assumes that the quaternion has the scalar part
%at the start
end

end

