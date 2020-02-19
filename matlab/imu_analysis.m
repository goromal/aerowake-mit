% bagfile = '../bags/flight_tests/Fext_NOLOAD_TRIAL1.bag';
% 
% addpath(genpath('matlab_utilities/'));
% bagdata = processAllROSBagTopics(bagfile);
% 
% close all
% 
% disp('============================')

%% DATA EXTRACTION

% % IMU sensor time (s)
% t_imu     = bagdata.imu.data.t;
% % IMU accelerometer measurements about axes (m/s^2)
% x_acc_imu = bagdata.imu.data.acc(1,:);
% y_acc_imu = bagdata.imu.data.acc(2,:);
% z_acc_imu = bagdata.imu.data.acc(3,:);
% % IMU gyro measurements about axes (rad/s)
% x_gyr_imu = bagdata.imu.data.gyro(1,:);
% y_gyr_imu = bagdata.imu.data.gyro(2,:);
% z_gyr_imu = bagdata.imu.data.gyro(3,:);
% 
% % IMU bias estimates time (s)
% t_imub     = bagdata.imu_bias.t;
% % IMU accelerometer bias estimates about axes (m/s^2)
% x_acc_imub = bagdata.imu_bias.acc(1,:);
% y_acc_imub = bagdata.imu_bias.acc(2,:);
% z_acc_imub = bagdata.imu_bias.acc(3,:);
% % IMU gyro bias estimates about axes (rad/s)
% x_gyr_imub = bagdata.imu_bias.gyro(1,:);
% y_gyr_imub = bagdata.imu_bias.gyro(2,:);
% z_gyr_imub = bagdata.imu_bias.gyro(3,:);
% 
% % ROSflight command time (s)
% t_com = bagdata.command.t;
% % ROSflight throttle command [0-1]
% F_com = bagdata.command.F;
% 
% % Truth state time (s)
% t_tru = bagdata.odometry.t;
% % North position (m)
% N_tru = bagdata.odometry.pose.position(1,:);
% % East position (m)
% E_tru = bagdata.odometry.pose.position(2,:);
% % Down position (m)
% D_tru = bagdata.odometry.pose.position(3,:);
% % Body-x velocity (m/s)
% u_tru = bagdata.odometry.twist.linear(1,:);
% % Body-y velocity (m/s)
% v_tru = bagdata.odometry.twist.linear(2,:);
% % Body-z velocity (m/s)
% w_tru = bagdata.odometry.twist.linear(3,:);
% % Roll (phi), Pitch (tht), Yaw (psi) (rad)
% [phi_tru, tht_tru, psi_tru] = quat_to_euler(bagdata.odometry.pose.orientation(4,:),...
%                                 bagdata.odometry.pose.orientation(1,:),...
%                                 bagdata.odometry.pose.orientation(2,:),...
%                                 bagdata.odometry.pose.orientation(3,:));
% % Angular velocities about x, y, z axes (rad/s)
% p_tru   = bagdata.odometry.twist.angular(1,:);
% q_tru   = bagdata.odometry.twist.angular(2,:);
% r_tru   = bagdata.odometry.twist.angular(3,:);
% 
% % Regularize time
% t0 = min([t_tru t_com t_imub t_imu]);
% t_tru  = t_tru - t0;
% t_com  = t_com - t0;
% t_imu  = t_imu - t0;
% t_imub = t_imub - t0;

%% DATA PROCESSING

% NOTES
% - see report.pdf for a brief overview of the IMU sensor model
%   (particularly the Accelerometer and Rate Gyro); the sections on filters
%   are an interesting read, but not critical to this analysis
% - REMEMBER that the frames in use here are North-East-Down for the
%   inertial frame and Front-Right-Down for the UAV body frame
% - load the .mat file to get the above data vectors with:
%   load('IMUdata.mat');
% - use https://github.com/goromal/matlab_utilities to determine the alleged 
%   orientation (ignoring yaw) of the uav according to the imu accel vector:
%   addpath('matlab-utils/');
%   grav_vector = [x_acc_imu(i); y_acc_imu(i); z_acc_imu(i)];
%   q = Quatd_from_two_unit_vectors([0; 0; -1],grav_vector);
%   phi_acc = q.roll();
%   tht_acc = q.pitch();
  