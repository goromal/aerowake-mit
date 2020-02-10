% bagfile = '../bags/ukf_bags/tetherless_RC_1_PROCESSED_2020-01-27-23-38-23.bag';
bagfile = '../bags/ukf_bags/tethered_RC_twoflights_2_PROCESSED_2020-01-28-10-59-15.bag';

addpath(genpath('matlab_utilities/'));
bagdata = processAllROSBagTopics(bagfile);

close all

disp('============================')

%% PRE-PROCESSING

% Possible Relevant Bag Fields

has_Fext = false;

% Determine what's in the bag
if isfield(bagdata, 'aerowake_MIT')
    has_sim_truth = true;
    disp('HAS SIM TRUTH')
end
if isfield(bagdata, 'reference')
    has_vicon = true;
    disp('HAS VICON')
end
if isfield(bagdata, 'ukf_est_odometry')
    has_ukf_odom = true;
    disp('HAS UKF ODOMETRY ESTIMATE')
end
if isfield(bagdata, 'odometry')
    has_estimate = true;
    disp('HAS EKF STATE ESTIMATE')
end
if isfield(bagdata, 'Fext')
    has_Fext = true;
    disp('HAS F_EXT ESTIMATE')
end

% Make sure Fext is at least in the bag
if ~has_Fext
    disp('ERROR: NO EXTERNAL FORCE ESTIMATE DATA PRESENT. EXITING.')
    return;
end

% Get initial time
t0 = 0;
if has_sim_truth
    t0 = bagdata.aerowake_MIT.truth.NED.t(1);
elseif has_vicon
    t0 = bagdata.reference.t(1);
elseif has_estimate
    t0 = odometry.t(1);    
elseif has_ukf_odom
    t0 = bagdata.ukf_est_odometry.t(1);
else
    t0 = bagdata.Fext.t(1);
end

%% DATA EXTRACTION

% Sim truth
if has_sim_truth
    t_tru = bagdata.aerowake_MIT.truth.NED.t - t0;
    N_tru = bagdata.aerowake_MIT.truth.NED.pose.position(1,:);
    E_tru = bagdata.aerowake_MIT.truth.NED.pose.position(2,:);
    D_tru = bagdata.aerowake_MIT.truth.NED.pose.position(3,:);
    u_tru = bagdata.aerowake_MIT.truth.NED.twist.linear(1,:);
    v_tru = bagdata.aerowake_MIT.truth.NED.twist.linear(2,:);
    w_tru = bagdata.aerowake_MIT.truth.NED.twist.linear(3,:);
    [phi, tht, psi] = quat_to_euler(bagdata.aerowake_MIT.truth.NED.pose.orientation(4,:),...
                                    bagdata.aerowake_MIT.truth.NED.pose.orientation(1,:),...
                                    bagdata.aerowake_MIT.truth.NED.pose.orientation(2,:),...
                                    bagdata.aerowake_MIT.truth.NED.pose.orientation(3,:));
    phi_tru = phi*180/pi;
    tht_tru = tht*180/pi;
    psi_tru = psi*180/pi;
    p_tru   = bagdata.aerowake_MIT.truth.NED.twist.angular(1,:);
    q_tru   = bagdata.aerowake_MIT.truth.NED.twist.angular(2,:);
    r_tru   = bagdata.aerowake_MIT.truth.NED.twist.angular(3,:);
end
    
% Vicon
if has_vicon
    t_vic = bagdata.reference.t - t0;
    N_vic = bagdata.reference.pose.position(1,:);
    E_vic = bagdata.reference.pose.position(2,:);
    D_vic = bagdata.reference.pose.position(3,:);
    [phi, tht, psi] = quat_to_euler(bagdata.reference.pose.orientation(4,:),...
                                    bagdata.reference.pose.orientation(1,:),...
                                    bagdata.reference.pose.orientation(2,:),...
                                    bagdata.reference.pose.orientation(3,:));
    phi_vic = phi*180/pi;
    tht_vic = tht*180/pi;
    psi_vic = psi*180/pi;
end

% Estimate
if has_estimate
    t_est = bagdata.odometry.t - t0;
    N_est = bagdata.odometry.pose.position(1,:);
    E_est = bagdata.odometry.pose.position(2,:);
    D_est = bagdata.odometry.pose.position(3,:);
    u_est = bagdata.odometry.twist.linear(1,:);
    v_est = bagdata.odometry.twist.linear(2,:);
    w_est = bagdata.odometry.twist.linear(3,:);
    [phi, tht, psi] = quat_to_euler(bagdata.odometry.pose.orientation(4,:),...
                                    bagdata.odometry.pose.orientation(1,:),...
                                    bagdata.odometry.pose.orientation(2,:),...
                                    bagdata.odometry.pose.orientation(3,:));
    phi_est = phi*180/pi;
    tht_est = tht*180/pi;
    psi_est = psi*180/pi;
    p_est   = bagdata.odometry.twist.angular(1,:);
    q_est   = bagdata.odometry.twist.angular(2,:);
    r_est   = bagdata.odometry.twist.angular(3,:);
end

% UKF Odom
if has_ukf_odom
    t_ust = bagdata.ukf_est_odometry.t - t0;
    % In case UKF was post-processed, subtract the necessary time offset
    if t_ust(1) > 60.0
        t_ust = t_ust - t_ust(1) + 0.5; % assuming 0.5 sec latency
    end
    N_ust = bagdata.ukf_est_odometry.pose.position(1,:);
    E_ust = -bagdata.ukf_est_odometry.pose.position(2,:);
    D_ust = -bagdata.ukf_est_odometry.pose.position(3,:);
    u_ust = bagdata.ukf_est_odometry.twist.linear(1,:);
    v_ust = bagdata.ukf_est_odometry.twist.linear(2,:);
    w_ust = bagdata.ukf_est_odometry.twist.linear(3,:);
    [phi, tht, psi] = quat_to_euler(bagdata.ukf_est_odometry.pose.orientation(4,:),...
                                    bagdata.ukf_est_odometry.pose.orientation(1,:),...
                                    bagdata.ukf_est_odometry.pose.orientation(2,:),...
                                    bagdata.ukf_est_odometry.pose.orientation(3,:));
    phi_ust = phi*180/pi;
    tht_ust = -tht*180/pi;
    psi_ust = -psi*180/pi;
    p_ust   = bagdata.ukf_est_odometry.twist.angular(1,:);
    q_ust   = bagdata.ukf_est_odometry.twist.angular(2,:);
    r_ust   = bagdata.ukf_est_odometry.twist.angular(3,:);
end

% External Forces Estimates
t_ukf = bagdata.Fext.t - t0;
% In case UKF was post-processed, subtract the necessary time offset
if t_ukf(1) > 60.0
    t_ukf = t_ukf - t_ukf(1) + 0.5; % assuming 0.5 sec latency
end
x_ukf = bagdata.Fext.force(1,:);
y_ukf = -bagdata.Fext.force(2,:);
z_ukf = -bagdata.Fext.force(3,:);

%% PLOTS

if has_sim_truth || has_vicon || has_ukf_odom || has_estimate
    figure('position',[10 10 2400 1250])
%     sgtitle('UAV States')
    subplot(3,4,1) % N
    legend('-DynamicLegend')
    hold on; grid on
    if has_sim_truth
        plot(t_tru, N_tru, 'k-', 'DisplayName', 'Sim Truth')
    elseif has_vicon
        plot(t_vic, N_vic, 'k-', 'DisplayName', 'Vicon Truth')
    end
    if has_estimate
        plot(t_est, N_est, 'b-', 'DisplayName', 'EKF State Estimate')
    end
    if has_ukf_odom
        plot(t_ust, N_ust, 'r--','DisplayName', 'UKF State Estimate')
    end
    ylabel('N (m)')
    hold off
    subplot(3,4,5) % E
    hold on; grid on
    if has_sim_truth
        plot(t_tru, E_tru, 'k-')
    elseif has_vicon
        plot(t_vic, E_vic, 'k-')
    end
    if has_estimate
        plot(t_est, E_est, 'b-')
    end
    if has_ukf_odom
        plot(t_ust, E_ust, 'r--')
    end
    ylabel('E (m)')
    hold off
    subplot(3,4,9) % D
    hold on; grid on
    if has_sim_truth
        plot(t_tru, D_tru, 'k-')
    elseif has_vicon
        plot(t_vic, D_vic, 'k-')
    end
    if has_estimate
        plot(t_est, D_est, 'b-')
    end
    if has_ukf_odom
        plot(t_ust, D_ust, 'r--')
    end
    ylabel('D (m)')
    hold off
    subplot(3,4,2) % roll
    hold on; grid on
    if has_sim_truth
        plot(t_tru, phi_tru, 'k-')
    elseif has_vicon
        plot(t_vic, phi_vic, 'k-')
    end
    if has_estimate
        plot(t_est, phi_est, 'b-')
    end
    if has_ukf_odom
        plot(t_ust, phi_ust, 'r--')
    end
    ylabel('\phi (deg)')
    hold off
    subplot(3,4,6) % pitch
    hold on; grid on
    if has_sim_truth
        plot(t_tru, tht_tru, 'k-')
    elseif has_vicon
        plot(t_vic, tht_vic, 'k-')
    end
    if has_estimate
        plot(t_est, tht_est, 'b-')
    end
    if has_ukf_odom
        plot(t_ust, tht_ust, 'r--')
    end
    ylabel('\theta (deg)')
    hold off
    subplot(3,4,10) % yaw
    hold on; grid on
    if has_sim_truth
        plot(t_tru, psi_tru, 'k-')
    elseif has_vicon
        plot(t_vic, psi_vic, 'k-')
    end
    if has_estimate
        plot(t_est, psi_est, 'b-')
    end
    if has_ukf_odom
        plot(t_ust, psi_ust, 'r--')
    end
    ylabel('\psi (deg)')
    hold off
    subplot(3,4,3) % u
    hold on; grid on
    if has_sim_truth
        plot(t_tru, u_tru, 'k-')
    end
    if has_estimate
        plot(t_est, u_est, 'b-')
    end
    if has_ukf_odom
        plot(t_ust, u_ust, 'r--')
    end
    ylabel('u (m/s)')
    hold off
    subplot(3,4,7) % v
    hold on; grid on
    if has_sim_truth
        plot(t_tru, v_tru, 'k-')
    end
    if has_estimate
        plot(t_est, v_est, 'b-')
    end
    if has_ukf_odom
        plot(t_ust, v_ust, 'r--')
    end
    ylabel('v (m/s)')
    hold off
    subplot(3,4,11) % w
    hold on; grid on
    if has_sim_truth
        plot(t_tru, w_tru, 'k-')
    end
    if has_estimate
        plot(t_est, w_est, 'b-')
    end
    if has_ukf_odom
        plot(t_ust, w_ust, 'r--')
    end
    ylabel('w (m/s)')
    hold off
    subplot(3,4,4) % p
    hold on; grid on
    if has_sim_truth
        plot(t_tru, p_tru, 'k-')
    end
    if has_estimate
        plot(t_est, p_est, 'b-')
    end
    if has_ukf_odom
        plot(t_ust, p_ust, 'r--')
    end
    ylabel('p (rad/s)')
    hold off
    subplot(3,4,8) % q
    hold on; grid on
    if has_sim_truth
        plot(t_tru, q_tru, 'k-')
    end
    if has_estimate
        plot(t_est, q_est, 'b-')
    end
    if has_ukf_odom
        plot(t_ust, q_ust, 'r--')
    end
    ylabel('q (rad/s)')
    hold off
    subplot(3,4,12) % r
    hold on; grid on
    if has_sim_truth
        plot(t_tru, r_tru, 'k-')
    end
    if has_estimate
        plot(t_est, r_est, 'b-')
    end
    if has_ukf_odom
        plot(t_ust, r_ust, 'r--')
    end
    ylabel('r (rad/s)')
    hold off
end

figure('position',[10 10 1200 500])
subplot(1,2,1)
plot(t_ukf, x_ukf, 'r-', t_ukf, y_ukf, 'g-')
legend('F_x','F_y')
xlabel('Time (s)')
ylabel('Estimated External Force (N)')
grid on
subplot(1,2,2)
plot(t_ukf, z_ukf, 'b-')
legend('F_z')
xlabel('Time (s)')
grid on

figure('position',[10 10 1000 250])
histogram(diff(t_ukf),'Normalization','probability','BinWidth',0.001)
xlabel('\Delta t (s)')
ylabel('Probability')
