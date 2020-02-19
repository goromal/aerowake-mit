%% INPUT FILES

bagfile = '../bags/vision_bags/vision_sim_2020-02-19-00-47-07.bag';

%% FRONTMATTER

addpath(genpath('matlab_utilities/'));
addpath(genpath('matlab-utils/'));
bagdata = processAllROSBagTopics(bagfile);

close all

disp('============================')

%% EXTRACT DATA
                              
% Truth
% t_tru = bagdata.rel_odometry.t;
% t0 = t_tru(1);
% t_tru = t_tru - t0;
% N_tru = bagdata.rel_odometry.pose.position(1,:);
% E_tru = bagdata.rel_odometry.pose.position(2,:);
% D_tru = bagdata.rel_odometry.pose.position(3,:);
% [phi_tru, tht_tru, psi_tru] = ...
%     QuatdDataToRPY(bagdata.rel_odometry.pose.orientation(4,:), ...
%                    bagdata.rel_odometry.pose.orientation(1,:), ...
%                    bagdata.rel_odometry.pose.orientation(2,:), ...
%                    bagdata.rel_odometry.pose.orientation(3,:));
t_tru = bagdata.aerowake_MIT.rel_truth.NED.t;
t0 = t_tru(1);
t_tru = t_tru - t0;
N_tru = bagdata.aerowake_MIT.rel_truth.NED.pose.position(1,:);
E_tru = bagdata.aerowake_MIT.rel_truth.NED.pose.position(2,:);
D_tru = bagdata.aerowake_MIT.rel_truth.NED.pose.position(3,:);
[phi_tru, tht_tru, psi_tru] = ...
    QuatdDataToRPY(bagdata.aerowake_MIT.rel_truth.NED.pose.orientation(4,:), ...
                   bagdata.aerowake_MIT.rel_truth.NED.pose.orientation(1,:), ...
                   bagdata.aerowake_MIT.rel_truth.NED.pose.orientation(2,:), ...
                   bagdata.aerowake_MIT.rel_truth.NED.pose.orientation(3,:));

% Vision Pose Estimate
t_vis = bagdata.vision_pose.t - t0;
N_vis = bagdata.vision_pose.transform.translation(1,:);
E_vis = bagdata.vision_pose.transform.translation(2,:);
D_vis = bagdata.vision_pose.transform.translation(3,:);
[phi_vis, tht_vis, psi_vis] = ...
    QuatdDataToRPY(bagdata.vision_pose.transform.rotation(4,:), ...
                   bagdata.vision_pose.transform.rotation(1,:), ...
                   bagdata.vision_pose.transform.rotation(2,:), ...
                   bagdata.vision_pose.transform.rotation(3,:));

%% PLOTS

figure('position',[10 10 1000 600])

subplot(2,3,1)
plot(t_tru, N_tru, 'k-', t_vis, N_vis, 'r*')
grid on
legend('Truth','Estimate')
ylabel('N (m)')
subplot(2,3,2)
plot(t_tru, E_tru, 'k-', t_vis, E_vis, 'r*')
grid on
ylabel('E (m)')
subplot(2,3,3)
plot(t_tru, D_tru, 'k-', t_vis, D_vis, 'r*')
grid on
ylabel('D (m)')
subplot(2,3,4)
plot(t_tru, 180/pi*phi_tru, 'k-', t_vis, 180/pi*phi_vis, 'r*')
grid on
ylabel('\phi (deg)')
xlabel('t (s)')
subplot(2,3,5)
plot(t_tru, 180/pi*tht_tru, 'k-', t_vis, 180/pi*tht_vis, 'r*')
grid on
ylabel('\theta (deg)')
xlabel('t (s)')
subplot(2,3,6)
plot(t_tru, 180/pi*psi_tru, 'k-', t_vis, 180/pi*psi_vis, 'r*')
grid on
ylabel('\psi (deg)')
xlabel('t (s)')