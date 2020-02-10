bagfile = '../scripts/Fext_NOLOAD_TRIAL3.bag';

addpath(genpath('matlab_utilities/'));
bagdata = processAllROSBagTopics(bagfile);

close all

disp('============================')

%% DATA

t0 = bagdata.imu.data.t(1);

has_commands = false;
if isfield(bagdata, 'full_hl_command')
    disp('TRACKING DATA PRESENT')
    
    % Commanded states
    com_t   = bagdata.full_hl_command.t - t0;
    com_N   = bagdata.full_hl_command.pose.position(1,:);
    com_E   = bagdata.full_hl_command.pose.position(2,:);
    com_D   = bagdata.full_hl_command.pose.position(3,:);
    com_u   = bagdata.full_hl_command.twist.linear(1,:);
    com_v   = bagdata.full_hl_command.twist.linear(2,:);
    com_w   = bagdata.full_hl_command.twist.linear(3,:);
    com_phi = bagdata.full_hl_command.pose.orientation(1,:)*180/pi;
    com_tht = bagdata.full_hl_command.pose.orientation(2,:)*180/pi;
    com_psi = bagdata.full_hl_command.pose.orientation(3,:)*180/pi;
    com_r   = bagdata.full_hl_command.twist.angular(3,:);

    % Actual states
    act_t = bagdata.odometry.t - t0;
    act_N = bagdata.odometry.pose.position(1,:);
    act_E = bagdata.odometry.pose.position(2,:);
    act_D = bagdata.odometry.pose.position(3,:);
    act_u = bagdata.odometry.twist.linear(1,:);
    act_v = bagdata.odometry.twist.linear(2,:);
    act_w = bagdata.odometry.twist.linear(3,:);
    [phi, tht, psi] = quat_to_euler(bagdata.odometry.pose.orientation(4,:),...
                                    bagdata.odometry.pose.orientation(1,:),...
                                    bagdata.odometry.pose.orientation(2,:),...
                                    bagdata.odometry.pose.orientation(3,:));
    act_phi = phi*180/pi;
    act_tht = tht*180/pi;
    act_psi = psi*180/pi;
    act_r   = bagdata.odometry.twist.angular(3,:);

    % ROSflight's states
    rsf_t = bagdata.attitude.t - t0;
    rsf_phi = bagdata.attitude.q_euler(1,:);
    rsf_tht = bagdata.attitude.q_euler(2,:);
    rsf_psi = bagdata.attitude.q_euler(3,:);
    rsf_r = bagdata.attitude.omega(3,:);
    
    has_commands = true;
else
    disp('TRACKING DATA ABSENT')
end

% UKF states (if they exist)
has_ukf = false;
if isfield(bagdata, 'Fext')
    disp('UKF DATA PRESENT')
    
    ukf_t = bagdata.Fext.t - t0;
    ukf_Fx = bagdata.Fext.force(1,:);
    ukf_Fy = -bagdata.Fext.force(2,:);
    ukf_Fz = -bagdata.Fext.force(3,:);
    ukf_Tx = bagdata.Fext.torque(1,:);
    ukf_Ty = bagdata.Fext.torque(2,:);
    ukf_Tz = bagdata.Fext.torque(3,:);
    
    has_ukf = true;
else
    disp('UKF DATA ABSENT')
end

%% PLOTS

% Commanded vs actual
if (has_commands)
    figure('position',[10 10 2000 1800])
    sgtitle('UAV Flight State Tracking')

    subplot(3,3,1)
    plot(act_t, act_N, 'k-', ...
         com_t, com_N, 'r--', 'Linewidth', 1.5)
    grid on
    legend('actual','commanded')
    ylabel('N (m)')
    subplot(3,3,2)
    plot(act_t, act_E, 'k-', ...
         com_t, com_E, 'r--', 'Linewidth', 1.5)
    grid on
    ylabel('E (m)')
    subplot(3,3,3)
    plot(act_t, act_D, 'k-', ...
         com_t, com_D, 'r--', 'Linewidth', 1.5)
    grid on
    ylabel('D (m)')

    subplot(3,3,4)
    plot(act_t, act_u, 'k-', ...
         com_t, com_u, 'r--', 'Linewidth', 1.5)
    grid on
    ylabel('u (m/s)')
    subplot(3,3,5)
    plot(act_t, act_v, 'k-', ...
         com_t, com_v, 'r--', 'Linewidth', 1.5)
    grid on
    ylabel('v (m/s)')
    subplot(3,3,6)
    plot(act_t, act_w, 'k-', ...
         com_t, com_w, 'r--', 'Linewidth', 1.5)
    grid on
    ylabel('w (m/s)')

    subplot(3,3,7)
    plot(act_t, act_phi, 'k-', ...
         rsf_t, rsf_phi, 'b--', ...
         com_t, com_phi, 'r--', 'Linewidth', 1.5)
    grid on
    legend('actual','ROSflight estimate','commanded')
    ylabel('\phi (deg)')
    xlabel('t (s)')
    subplot(3,3,8)
    plot(act_t, act_tht, 'k-', ...
         rsf_t, rsf_tht, 'b--', ...
         com_t, com_tht, 'r--', 'Linewidth', 1.5)
    grid on
    ylabel('\theta (deg)')
    xlabel('t (s)')
    subplot(3,3,9)
%     plot(act_t, act_psi, 'k-', ...
%          rsf_t, rsf_psi, 'b--', ...
%          com_t, com_psi, 'r--', 'Linewidth', 1.5)
%     grid on
%     ylabel('\psi (deg)')
    plot(act_t, act_r, 'k-', ...
         rsf_t, rsf_r, 'b--', ...
         com_t, com_r, 'r--', 'Linewidth', 1.5)
    grid on
    ylabel('\r (rad/s)')
    xlabel('t (s)')
end

if (has_ukf)
    figure('position',[10 10 1750 500])
    sgtitle('UKF External Force Estimates: Untethered Flight')
    
    subplot(1,3,1)
    plot(ukf_t, ukf_Fx, 'k-', ...
         [0 ukf_t(end)], [0 0], 'b--', 'Linewidth', 1.5)
    grid on
    legend('estimated','actual')
    xlabel('t (s)')
    ylabel('F_x (N)')
    
    subplot(1,3,2)
    plot(ukf_t, ukf_Fy, 'k-', ...
         [0 ukf_t(end)], [0 0], 'b--', 'Linewidth', 1.5)
    grid on
    xlabel('t (s)')
    ylabel('F_y (N)')
    
    subplot(1,3,3)
    plot(ukf_t, ukf_Fz, 'k-', ...
         [0 ukf_t(end)], [0 0], 'b--', 'Linewidth', 1.5)
    grid on
    xlabel('t (s)')
    ylabel('F_z (N)')
end