bagfile = '../bags/control_bags/controller_sim_pres_sim_gains.bag';

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

% Commanded

t_com = bagdata.full_hl_command.t - t0;
N_com = bagdata.full_hl_command.pose.position(1,:);
E_com = bagdata.full_hl_command.pose.position(2,:);
D_com = bagdata.full_hl_command.pose.position(3,:);
u_com = bagdata.full_hl_command.twist.linear(1,:); 
v_com = bagdata.full_hl_command.twist.linear(2,:);
w_com = bagdata.full_hl_command.twist.linear(3,:);
phi_com = bagdata.full_hl_command.pose.orientation(1,:)*180/pi;
tht_com = bagdata.full_hl_command.pose.orientation(2,:)*180/pi;
psi_com = bagdata.full_hl_command.pose.orientation(3,:)*180/pi;
p_com   = bagdata.full_hl_command.twist.angular(1,:);
q_com   = bagdata.full_hl_command.twist.angular(2,:);
r_com   = bagdata.full_hl_command.twist.angular(3,:);

%% PLOTS

figure('position',[10 10 1000 600])

subplot(3,3,1)
plot(t_tru, N_tru, 'k-', t_com, N_com, 'r--')
grid on
legend('Truth','Commanded')
ylabel('N (m)')
subplot(3,3,2)
plot(t_tru, E_tru, 'k-', t_com, E_com, 'r--')
grid on
ylabel('E (m)')
subplot(3,3,3)
plot(t_tru, D_tru, 'k-', t_com, D_com, 'r--')
grid on
ylabel('D (m)')
subplot(3,3,4)
plot(t_tru, u_tru, 'k-', t_com, u_com, 'r--')
grid on
ylabel('u (m/s)')
subplot(3,3,5)
plot(t_tru, v_tru, 'k-', t_com, v_com, 'r--')
grid on
ylabel('v (m/s)')
subplot(3,3,6)
plot(t_tru, w_tru, 'k-', t_com, w_com, 'r--')
grid on
ylabel('w (m/s)')
subplot(3,3,7)
plot(t_tru, phi_tru, 'k-', t_com, phi_com, 'r--')
grid on
ylabel('\phi (deg)')
xlabel('t (s)')
subplot(3,3,8)
plot(t_tru, tht_tru, 'k-', t_com, tht_com, 'r--')
grid on
ylabel('\theta (deg)')
xlabel('t (s)')
subplot(3,3,9)
plot(t_tru, r_tru, 'k-', t_com, r_com, 'r--')
grid on
ylabel('r (rad/s)')
xlabel('t (s)')