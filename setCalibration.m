function [ C_c_v, rho_v_c_v ] = setCalibration()
%loadCalibration loads the calibration files like camera to imu translation
%and rotation

dataCalibDir = 'Data\2011_09_26_calib\2011_09_26';
[T_camvelo_struct, P_rect_cam1] = loadCalibration(dataCalibDir);
T_camvelo = T_camvelo_struct{1};
T_veloimu = loadCalibrationRigid(fullfile(dataCalibDir,'calib_imu_to_velo.txt'));
T_camimu = T_camvelo*T_veloimu;
R_ci = T_camimu(1:3,1:3);
T_imucam = inv(T_camimu);
p_ci_i = T_imucam(1:3,4);
C_c_v = R_ci;
rho_v_c_v = p_ci_i;


end

