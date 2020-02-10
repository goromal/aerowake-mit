bagfile = '../bags/ekf_bags/ekf_notquite_perfect.bag';

addpath(genpath('matlab_utilities/'));
bagdata = processAllROSBagTopics(bagfile);

close all

disp('============================')

%% DATA

t0 = bagdata.aerowake_MIT.truth.NED.t(1);

% Truth

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

% Estimates

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

%% PLOTS

figure('position',[10 10 1000 600])

subplot(2,3,1)
plot(t_tru, N_tru, 'k-', t_est, N_est, 'r--')
grid on
legend('Truth','EKF Estimate')
ylabel('N (m)')
subplot(2,3,2)
plot(t_tru, E_tru, 'k-', t_est, E_est, 'r--')
grid on
ylabel('E (m)')
subplot(2,3,3)
plot(t_tru, D_tru, 'k-', t_est, D_est, 'r--')
grid on
ylabel('D (m)')
subplot(2,3,4)
plot(t_tru, u_tru, 'k-', t_est, u_est, 'r--')
grid on
ylabel('u (m/s)')
xlabel('t (s)')
subplot(2,3,5)
plot(t_tru, v_tru, 'k-', t_est, v_est, 'r--')
grid on
ylabel('v (m/s)')
xlabel('t (s)')
subplot(2,3,6)
plot(t_tru, w_tru, 'k-', t_est, w_est, 'r--')
grid on
ylabel('w (m/s)')
xlabel('t (s)')

figure('position',[10 10 1000 600])

subplot(2,3,1)
plot(t_tru, phi_tru, 'k-', t_est, phi_est, 'r--')
grid on
legend('Truth','EKF Estimate')
ylabel('\phi (rad)')
subplot(2,3,2)
plot(t_tru, tht_tru, 'k-', t_est, tht_est, 'r--')
grid on
ylabel('\theta (rad)')
subplot(2,3,3)
plot(t_tru, psi_tru, 'k-', t_est, psi_est, 'r--')
grid on
ylabel('\psi (rad)')
subplot(2,3,4)
plot(t_tru, p_tru, 'k-', t_est, p_est, 'r--')
grid on
ylabel('p (rad/s)')
xlabel('t (s)')
subplot(2,3,5)
plot(t_tru, q_tru, 'k-', t_est, q_est, 'r--')
grid on
ylabel('q (rad/s)')
xlabel('t (s)')
subplot(2,3,6)
plot(t_tru, r_tru, 'k-', t_est, r_est, 'r--')
grid on
ylabel('r (rad/s)')
xlabel('t (s)')