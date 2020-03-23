function [] = process_ekf_logs(logname)

% logname = "roof1";

%% LOG HANDLING

% Frontmatter
% set(0, 'DefaultFigureRenderer', 'painters');
addpath(genpath('matlab_utilities'));
addpath(genpath('matlab-utils'));
abs_logdir = strcat('../bags/ekf_bags/',logname,'_logs/absolute');
rel_logdir = strcat('../bags/ekf_bags/',logname,'_logs/relative');

% PLOTS:
% A. ABSOLUTE
%   1. State + Covariance Plots <- LOG_STATE, LOG_COV [6 PLOTS]
%   2. IMU Updates <- LOG_IMU
%   3. GNSS Updates <- LOG_GNSS_RES
%   4. Baro Updates <- LOG_BARO_RES
%   5. Zero-Vel Updates <- LOG_ZERO_VEL_RES
% B. RELATIVE
%   1. State + Covariance Plots <- LOG_STATE, LOG_COV [6 PLOTS]
%   2. IMU Updates <- LOG_IMU
%   3. Pose Ref Updates <- LOG_MOCAP_RES
%   4. DGPS Position Updates <- LOG_REL_POS
%   5. DGPS Velocity Updates <- LOG_REL_VEL
%   6. Baro Updates <- LOG_BARO_RES
%   7. Zero-Vel Updates <- LOG_ZERO_VEL_RES

red_sig_color = [255 100 100]./255;
grn_sig_color = [100 255 100]./255;
blu_sig_color = [100 100 255]./255;
blk_sig_color = [  0   0   0]./255;

% Extract and plot all relevant log data
close all
figsize = [10 10 3500 1750];
linewidth = 1.0;

% Plot A.1
ABS_state_data = get_n_datapoints_from_log(strcat(abs_logdir,'/state.bin'), 29, 1000);
ABS_cov_data   = get_n_datapoints_from_log(strcat(abs_logdir,'/cov.bin'),   18, 1000);

t_A1_state = ABS_state_data(1,:);
p_A1_state = ABS_state_data(3:5,:);
e_A1_state = ABS_state_data(27:29,:);
v_A1_state = ABS_state_data(10:12,:);
ba_A1_state = ABS_state_data(13:15,:);
bg_A1_state = ABS_state_data(16:18,:);
bb_A1_state = ABS_state_data(19,:);

t_A1_cov   = ABS_cov_data(1,:);
p_A1_cov = sqrt(abs(ABS_cov_data(2:4,:)));
e_A1_cov = sqrt(abs(ABS_cov_data(5:7,:)));
v_A1_cov = sqrt(abs(ABS_cov_data(8:10,:)));
ba_A1_cov = sqrt(abs(ABS_cov_data(11:13,:)));
bg_A1_cov = sqrt(abs(ABS_cov_data(14:16,:)));
bb_A1_cov = sqrt(abs(ABS_cov_data(17,:)));

figure('position',figsize)
sgtitle('Absolute Estimator States')
subplot(2,3,1)
plot_sigma_line(t_A1_state, p_A1_state(1,:), t_A1_cov, p_A1_cov(1,:), true, 'r-', red_sig_color, linewidth, true); grid on
plot_sigma_line(t_A1_state, p_A1_state(2,:), t_A1_cov, p_A1_cov(2,:), true, 'g-', grn_sig_color, linewidth, true)
plot_sigma_line(t_A1_state, p_A1_state(3,:), t_A1_cov, p_A1_cov(3,:), true, 'b-', blu_sig_color, linewidth, false)
title('N, E, D')
subplot(2,3,2)
plot_sigma_line(t_A1_state, e_A1_state(1,:), t_A1_cov, e_A1_cov(1,:), true, 'r-', red_sig_color, linewidth, true); grid on
plot_sigma_line(t_A1_state, e_A1_state(2,:), t_A1_cov, e_A1_cov(2,:), true, 'g-', grn_sig_color, linewidth, true)
plot_sigma_line(t_A1_state, e_A1_state(3,:), t_A1_cov, e_A1_cov(3,:), true, 'b-', blu_sig_color, linewidth, false)
title('\phi, \theta, \psi')
subplot(2,3,3)
plot_sigma_line(t_A1_state, v_A1_state(1,:), t_A1_cov, v_A1_cov(1,:), true, 'r-', red_sig_color, linewidth, true); grid on
plot_sigma_line(t_A1_state, v_A1_state(2,:), t_A1_cov, v_A1_cov(2,:), true, 'g-', grn_sig_color, linewidth, true)
plot_sigma_line(t_A1_state, v_A1_state(3,:), t_A1_cov, v_A1_cov(3,:), true, 'b-', blu_sig_color, linewidth, false)
title('u, v, w')
subplot(2,3,4)
plot_sigma_line(t_A1_state, ba_A1_state(1,:), t_A1_cov, ba_A1_cov(1,:), true, 'r-', red_sig_color, linewidth, true); grid on
plot_sigma_line(t_A1_state, ba_A1_state(2,:), t_A1_cov, ba_A1_cov(2,:), true, 'g-', grn_sig_color, linewidth, true)
plot_sigma_line(t_A1_state, ba_A1_state(3,:), t_A1_cov, ba_A1_cov(3,:), true, 'b-', blu_sig_color, linewidth, false)
title('\beta_{accel}')
subplot(2,3,5)
plot_sigma_line(t_A1_state, bg_A1_state(1,:), t_A1_cov, bg_A1_cov(1,:), true, 'r-', red_sig_color, linewidth, true); grid on
plot_sigma_line(t_A1_state, bg_A1_state(2,:), t_A1_cov, bg_A1_cov(2,:), true, 'g-', grn_sig_color, linewidth, true)
plot_sigma_line(t_A1_state, bg_A1_state(3,:), t_A1_cov, bg_A1_cov(3,:), true, 'b-', blu_sig_color, linewidth, false)
title('\beta_{gyro}')
subplot(2,3,6)
plot_sigma_line(t_A1_state, bb_A1_state, t_A1_cov, bb_A1_cov, true, 'r-', red_sig_color, linewidth, false); grid on 
title('\beta_{baro}')

savefig(strcat('../bags/ekf_bags/',logname,'_logs/ABS_STATE'))
% saveas(gcf, strcat('../bags/ekf_bags/',logname,'_logs/ABS_STATE'), 'svg')
 
figure('position',figsize)
sgtitle('Absolute Estimator Updates')
% Plot A.2
ABS_imu_data = get_n_datapoints_from_log(strcat(abs_logdir,'/imu.bin'), 7, 1000);

t_A2 = ABS_imu_data(1,:);
accx_A2 = ABS_imu_data(2,:);
accy_A2 = ABS_imu_data(3,:);
accz_A2 = ABS_imu_data(4,:);
gyrx_A2 = ABS_imu_data(5,:);
gyry_A2 = ABS_imu_data(6,:);
gyrz_A2 = ABS_imu_data(7,:);

subplot(2,3,1)
plot(t_A2, accx_A2, 'r-', ...
     t_A2, accy_A2, 'g-', ...
     t_A2, accz_A2, 'b-', 'Linewidth', linewidth); grid on
title('IMU Accels')
 
subplot(2,3,4)
plot(t_A2, gyrx_A2, 'r-', ...
     t_A2, gyry_A2, 'g-', ...
     t_A2, gyrz_A2, 'b-', 'Linewidth', linewidth); grid on
title('IMU Gyros')
 
% saveas(gcf, strcat('../bags/ekf_bags/',logname,'_logs/ABS_UPDATES'), 'svg')

% Plot A.3
ABS_gnss_data = get_n_datapoints_from_log(strcat(abs_logdir,'/gnss_res.bin'), 25, 1000);

t_A3 = ABS_gnss_data(1,:);
p_A3 = ABS_gnss_data(8:10,:);
v_A3 = ABS_gnss_data(11:13,:);
phat_A3 = ABS_gnss_data(14:16,:);
vhat_A3 = ABS_gnss_data(17:19,:);
psig_A3 = sqrt(abs(ABS_gnss_data(20:22,:)));
vsig_A3 = sqrt(abs(ABS_gnss_data(23:25,:)));

subplot(2,3,2)
plot_sigma_line(t_A3, p_A3(1,:), t_A3, sqrt(psig_A3(1,:)), false, 'r*', red_sig_color, linewidth, true); grid on
plot_sigma_line(t_A3, p_A3(2,:), t_A3, sqrt(psig_A3(2,:)), false, 'g*', grn_sig_color, linewidth, true)
plot_sigma_line(t_A3, p_A3(3,:), t_A3, sqrt(psig_A3(3,:)), false, 'b*', blu_sig_color, linewidth, true)
plot(t_A3, phat_A3(1,:), 'r-', ...
     t_A3, phat_A3(2,:), 'g-', ...
     t_A3, phat_A3(3,:), 'b-', 'Linewidth', linewidth)
hold off
title('GPS Position')

subplot(2,3,5)
plot_sigma_line(t_A3, v_A3(1,:), t_A3, vsig_A3(1,:), false, 'r*', red_sig_color, linewidth, true); grid on
plot_sigma_line(t_A3, v_A3(2,:), t_A3, vsig_A3(2,:), false, 'g*', grn_sig_color, linewidth, true)
plot_sigma_line(t_A3, v_A3(3,:), t_A3, vsig_A3(3,:), false, 'b*', blu_sig_color, linewidth, true)
plot(t_A3, vhat_A3(1,:), 'r-', ...
     t_A3, vhat_A3(2,:), 'g-', ...
     t_A3, vhat_A3(3,:), 'b-', 'Linewidth', linewidth)
hold off
title('GPS Velocity')

% Plot A.4
ABS_baro_data = get_n_datapoints_from_log(strcat(abs_logdir,'/baro_res.bin'), 6, 1000);

t_A4 = ABS_baro_data(1,:);
z_A4 = ABS_baro_data(3,:);
zhat_A4 = ABS_baro_data(4,:);
zsig_A4 = ABS_baro_data(5,:);

subplot(2,3,3)
plot_sigma_line(t_A4, z_A4, t_A4, zsig_A4, false, 'r*', red_sig_color, linewidth, true); grid on
plot(t_A4, zhat_A4, 'r-', 'Linewidth', linewidth)
hold off
title('Barometer Altitude')

% Plot A.5
ABS_zvel_data = get_n_datapoints_from_log(strcat(abs_logdir,'/zero_vel_res.bin'), 5, 1000);

t_A5 = ABS_zvel_data(1,:);
v_A5 = ABS_zvel_data(2:4,:);
y_A5 = ABS_zvel_data(5,:);

subplot(2,3,6)
plot(t_A5, v_A5(1,:), 'r.', ...
     t_A5, v_A5(2,:), 'g.', ...
     t_A5, v_A5(3,:), 'b.', ...
     t_A5, y_A5, 'k.', 'Linewidth', linewidth); grid on
title('Zero-Velocity Update')

savefig(strcat('../bags/ekf_bags/',logname,'_logs/ABS_UPDATES'))

% Plot B.1
REL_state_data = get_n_datapoints_from_log(strcat(rel_logdir,'/state.bin'), 36, 1000);
REL_cov_data   = get_n_datapoints_from_log(strcat(rel_logdir,'/cov.bin'),   21, 1000);

t_B1_state = REL_state_data(1,:);
p_B1_state = REL_state_data(3:5,:);
e_B1_state = REL_state_data(31:33,:);
eR_B1_state = REL_state_data(34:36,:);
v_B1_state = REL_state_data(10:12,:);
ba_B1_state = REL_state_data(13:15,:);
bg_B1_state = REL_state_data(16:18,:);
bb_B1_state = REL_state_data(19,:);

t_B1_cov   = REL_cov_data(1,:);
p_B1_cov = sqrt(abs(REL_cov_data(2:4,:)));
e_B1_cov = sqrt(abs(REL_cov_data(5:7,:)));
v_B1_cov = sqrt(abs(REL_cov_data(8:10,:)));
ba_B1_cov = sqrt(abs(REL_cov_data(11:13,:)));
bg_B1_cov = sqrt(abs(REL_cov_data(14:16,:)));
bb_B1_cov = sqrt(abs(REL_cov_data(17,:)));
eR_B1_cov = sqrt(abs(REL_cov_data(19:21,:)));

figure('position',figsize)
sgtitle('Relative Estimator States')
subplot(2,3,1)
plot_sigma_line(t_B1_state, p_B1_state(1,:), t_B1_cov, p_B1_cov(1,:), true, 'r-', red_sig_color, linewidth, true); grid on
plot_sigma_line(t_B1_state, p_B1_state(2,:), t_B1_cov, p_B1_cov(2,:), true, 'g-', grn_sig_color, linewidth, true)
plot_sigma_line(t_B1_state, p_B1_state(3,:), t_B1_cov, p_B1_cov(3,:), true, 'b-', blu_sig_color, linewidth, true)
plot_sigma_line(t_B1_state, bb_B1_state, t_B1_cov, bb_B1_cov, true, 'k-', blk_sig_color, linewidth, false)
title('p^{SHIP} and \beta_{baro}')
subplot(2,3,2)
plot_sigma_line(t_B1_state, e_B1_state(1,:), t_B1_cov, e_B1_cov(1,:), true, 'r-', red_sig_color, linewidth, true); grid on
plot_sigma_line(t_B1_state, e_B1_state(2,:), t_B1_cov, e_B1_cov(2,:), true, 'g-', grn_sig_color, linewidth, true)
plot_sigma_line(t_B1_state, e_B1_state(3,:), t_B1_cov, e_B1_cov(3,:), true, 'b-', blu_sig_color, linewidth, false)
title('\phi^{NED=SHIP}, \theta^{NED=SHIP}, \psi^{SHIP}')
subplot(2,3,3)
plot_sigma_line(t_B1_state, v_B1_state(1,:), t_B1_cov, v_B1_cov(1,:), true, 'r-', red_sig_color, linewidth, true); grid on
plot_sigma_line(t_B1_state, v_B1_state(2,:), t_B1_cov, v_B1_cov(2,:), true, 'g-', grn_sig_color, linewidth, true)
plot_sigma_line(t_B1_state, v_B1_state(3,:), t_B1_cov, v_B1_cov(3,:), true, 'b-', blu_sig_color, linewidth, false)
title('u, v, w')
subplot(2,3,4)
plot_sigma_line(t_B1_state, ba_B1_state(1,:), t_B1_cov, ba_B1_cov(1,:), true, 'r-', red_sig_color, linewidth, true); grid on
plot_sigma_line(t_B1_state, ba_B1_state(2,:), t_B1_cov, ba_B1_cov(2,:), true, 'g-', grn_sig_color, linewidth, true)
plot_sigma_line(t_B1_state, ba_B1_state(3,:), t_B1_cov, ba_B1_cov(3,:), true, 'b-', blu_sig_color, linewidth, false)
title('\beta_{accel}')
subplot(2,3,5)
plot_sigma_line(t_B1_state, bg_B1_state(1,:), t_B1_cov, bg_B1_cov(1,:), true, 'r-', red_sig_color, linewidth, true); grid on
plot_sigma_line(t_B1_state, bg_B1_state(2,:), t_B1_cov, bg_B1_cov(2,:), true, 'g-', grn_sig_color, linewidth, true)
plot_sigma_line(t_B1_state, bg_B1_state(3,:), t_B1_cov, bg_B1_cov(3,:), true, 'b-', blu_sig_color, linewidth, false)
title('\beta_{gyro}')
subplot(2,3,6)
plot_sigma_line(t_B1_state, eR_B1_state(1,:), t_B1_cov, eR_B1_cov(1,:), true, 'r-', red_sig_color, linewidth, true); grid on
plot_sigma_line(t_B1_state, eR_B1_state(2,:), t_B1_cov, eR_B1_cov(2,:), true, 'g-', grn_sig_color, linewidth, true)
plot_sigma_line(t_B1_state, eR_B1_state(3,:), t_B1_cov, eR_B1_cov(3,:), true, 'b-', blu_sig_color, linewidth, false)
title('\phi_{NED}^{SHIP}, \theta_{NED}^{SHIP}, \psi_{NED}^{SHIP}')

savefig(strcat('../bags/ekf_bags/',logname,'_logs/REL_STATE'))

figure('position',figsize)
sgtitle('Relative Estimator Updates')
% Plot B.2
REL_imu_data = get_n_datapoints_from_log(strcat(rel_logdir,'/imu.bin'), 7, 1000);

t_B2 = REL_imu_data(1,:);
accx_B2 = REL_imu_data(2,:);
accy_B2 = REL_imu_data(3,:);
accz_B2 = REL_imu_data(4,:);
gyrx_B2 = REL_imu_data(5,:);
gyry_B2 = REL_imu_data(6,:);
gyrz_B2 = REL_imu_data(7,:);

subplot(2,4,1)
plot(t_B2, accx_B2, 'r-', ...
     t_B2, accy_B2, 'g-', ...
     t_B2, accz_B2, 'b-', 'Linewidth', linewidth); grid on
title('IMU Accels')
 
subplot(2,4,5)
plot(t_B2, gyrx_B2, 'r-', ...
     t_B2, gyry_B2, 'g-', ...
     t_B2, gyrz_B2, 'b-', 'Linewidth', linewidth); grid on
title('IMU Gyro')

% Plot B.3
REL_ref_data = get_n_datapoints_from_log(strcat(rel_logdir,'/mocap_res.bin'), 25, 1000);

t_B3 = REL_ref_data(1,:);
p_B3 = REL_ref_data(8:10,:);
e_B3 = REL_ref_data(11:13,:);
phat_B3 = REL_ref_data(14:16,:);
ehat_B3 = REL_ref_data(17:19,:);
psig_B3 = REL_ref_data(20:22,:);
esig_B3 = REL_ref_data(23:25,:);

subplot(2,4,2)
plot_sigma_line(t_B3, p_B3(1,:), t_B3, psig_B3(1,:), false, 'r*', red_sig_color, linewidth, true); grid on
plot_sigma_line(t_B3, p_B3(2,:), t_B3, psig_B3(2,:), false, 'g*', grn_sig_color, linewidth, true)
plot_sigma_line(t_B3, p_B3(3,:), t_B3, psig_B3(3,:), false, 'b*', blu_sig_color, linewidth, true)
plot(t_B3, phat_B3(1,:), 'r-', ...
     t_B3, phat_B3(2,:), 'g-', ...
     t_B3, phat_B3(3,:), 'b-', 'Linewidth', linewidth)
hold off
title('Vision Position')

subplot(2,4,6)
plot_sigma_line(t_B3, e_B3(1,:), t_B3, esig_B3(1,:), false, 'r*', red_sig_color, linewidth, true); grid on
plot_sigma_line(t_B3, e_B3(2,:), t_B3, esig_B3(2,:), false, 'g*', grn_sig_color, linewidth, true)
plot_sigma_line(t_B3, e_B3(3,:), t_B3, esig_B3(3,:), false, 'b*', blu_sig_color, linewidth, true)
plot(t_B3, ehat_B3(1,:), 'r-', ...
     t_B3, ehat_B3(2,:), 'g-', ...
     t_B3, ehat_B3(3,:), 'b-', 'Linewidth', linewidth)
hold off
title('Vision \phi, \theta, \psi')

% Plot B.4
REL_pos_data = read_log(strcat(rel_logdir,'/rel_pos.bin'), 13);

t_B4 = REL_pos_data(1,:);
p_B4 = REL_pos_data(5:7,:);
phat_B4 = REL_pos_data(8:10,:);
psig_B4 = REL_pos_data(11:13,:);

subplot(2,4,3)
plot_sigma_line(t_B4, p_B4(1,:), t_B4, psig_B4(1,:), false, 'r*', red_sig_color, linewidth, true); grid on
plot_sigma_line(t_B4, p_B4(2,:), t_B4, psig_B4(2,:), false, 'g*', grn_sig_color, linewidth, true)
plot_sigma_line(t_B4, p_B4(3,:), t_B4, psig_B4(3,:), false, 'b*', blu_sig_color, linewidth, true)
plot(t_B4, phat_B4(1,:), 'r-', ...
     t_B4, phat_B4(2,:), 'g-', ...
     t_B4, phat_B4(3,:), 'b-', 'Linewidth', linewidth)
hold off
title('DGPS Position')

% Plot B.5
REL_vel_data = read_log(strcat(rel_logdir,'/rel_vel.bin'), 13);

t_B5 = REL_vel_data(1,:);
v_B5 = REL_vel_data(5:7,:);
vhat_B5 = REL_vel_data(8:10,:);
vsig_B5 = REL_vel_data(11:13,:);

subplot(2,4,7)
plot_sigma_line(t_B5, v_B5(1,:), t_B5, vsig_B5(1,:), false, 'r*', red_sig_color, linewidth, true); grid on
plot_sigma_line(t_B5, v_B5(2,:), t_B5, vsig_B5(2,:), false, 'g*', grn_sig_color, linewidth, true)
plot_sigma_line(t_B5, v_B5(3,:), t_B5, vsig_B5(3,:), false, 'b*', blu_sig_color, linewidth, true)
plot(t_B5, vhat_B5(1,:), 'r-', ...
     t_B5, vhat_B5(2,:), 'g-', ...
     t_B5, vhat_B5(3,:), 'b-', 'Linewidth', linewidth)
hold off
title('DGPS Velocity')

% Plot B.6
REL_baro_data = read_log(strcat(rel_logdir,'/baro_res.bin'), 6);

t_B6 = REL_baro_data(1,:);
z_B6 = REL_baro_data(3,:);
zhat_B6 = REL_baro_data(4,:);
zsig_B6 = REL_baro_data(5,:);

subplot(2,4,4)
plot_sigma_line(t_B6, z_B6, t_B6, zsig_B6, false, 'r*', red_sig_color, linewidth, true); grid on
plot(t_B6, zhat_B6, 'r-', 'Linewidth', linewidth)
hold off
title('Baro Altitude')

% Plot B.7
REL_zvel_data = read_log(strcat(rel_logdir,'/zero_vel_res.bin'), 5);

subplot(2,4,8)
plot([],[])
title('Zero-Velocity Update (DEACTIVATED)')

savefig(strcat('../bags/ekf_bags/',logname,'_logs/REL_UPDATES'))

end


%% HELPER FUNCTIONS

function [] = plot_sigma_line(t, data, t_sigma, sigma, interpolate_data, linespec, sig_color, linewidth, hold_on)

if ~isrow(t)
    t = t';
end
if ~isrow(t_sigma)
    t_sigma = t_sigma';
end
if ~isrow(data)
    data = data';
end
if ~isrow(sigma)
    sigma = sigma';
end

if interpolate_data
    sigma_interp = interp1(t_sigma', sigma', t')';
else
    sigma_interp = sigma;
end

x_vector = [t, fliplr(t)];
patch = fill(x_vector, [data + sigma_interp, fliplr(data - sigma_interp)], sig_color);
set(patch, 'edgecolor','none');
set(patch, 'FaceAlpha', 0.2);
hold on
plot(t, data, linespec, 'Linewidth', linewidth)

if ~hold_on
    hold off
end

end

function data = get_n_datapoints_from_log(fname, rowsize, num_datapoints)

data = read_log(fname, rowsize);
n = size(data,2);
if n > num_datapoints
    factor = floor(n / num_datapoints);
    data = data(:,1:factor:end);
end

end