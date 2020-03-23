%% INPUT FILES

bagfile = '../bags/318_visionsim_bags/notoil_cvcam_2020-03-18-19-27-51.bag';
tstitle = 'Sim-Benchmark-CVCAM-Test';

%% FRONTMATTER

addpath(genpath('matlab_utilities/'));
addpath(genpath('matlab-utils/'));
bagdata = processAllROSBagTopics(bagfile, false);

close all

disp('============================')

%% EXTRACT DATA
                              
t_tru = bagdata.aerowake_MIT.rel_truth.NED.t;
x_tru = bagdata.aerowake_MIT.rel_truth.NED.pose.position(1,:);
y_tru = bagdata.aerowake_MIT.rel_truth.NED.pose.position(2,:);
z_tru = bagdata.aerowake_MIT.rel_truth.NED.pose.position(3,:);
[phi_tru, tht_tru, psi_tru] = ...
    QuatdDataToRPY(bagdata.aerowake_MIT.rel_truth.NED.pose.orientation(4,:), ...
                   bagdata.aerowake_MIT.rel_truth.NED.pose.orientation(1,:), ...
                   bagdata.aerowake_MIT.rel_truth.NED.pose.orientation(2,:), ...
                   bagdata.aerowake_MIT.rel_truth.NED.pose.orientation(3,:));
phi_tru = phi_tru * 180/pi;
tht_tru = tht_tru * 180/pi;
psi_tru = psi_tru * 180/pi;

% Vision Pose Estimate - RAW
t_est = bagdata.vision_pose.t;
x_est = bagdata.vision_pose.transform.translation(1,:);
y_est = bagdata.vision_pose.transform.translation(2,:);
z_est = bagdata.vision_pose.transform.translation(3,:);
phi_est = bagdata.vision_pose.transform.euler(1,:)*180/pi;
tht_est = bagdata.vision_pose.transform.euler(2,:)*180/pi;
psi_est = bagdata.vision_pose.transform.euler(3,:)*180/pi;

% - Filtered by Solution Status
% solution arrived at?
sol_idx = bagdata.vision_pose.sol_status > 0;
t_sol   = t_est(sol_idx);
x_sol   = x_est(sol_idx);
y_sol   = y_est(sol_idx);
z_sol   = z_est(sol_idx);
phi_sol = phi_est(sol_idx);
tht_sol = tht_est(sol_idx);
psi_sol = psi_est(sol_idx);

% character of solution: if vision_bridge was used, indicate that data;
% otherwise, indicate each first solution after a solution dropout
if ismember(2, bagdata.vision_pose.sol_status)
    solspec_idx = bagdata.vision_pose.sol_status == 2;
    solspec_legend = 'Bridge-aided Solution';
else
    dsol_status = [0 diff(bagdata.vision_pose.sol_status)];
    solspec_idx = logical((dsol_status == 1) .* (bagdata.vision_pose.sol_status > 0));
    solspec_legend = 'First Solutions After Dropout';
end
t_solspec   = t_est(solspec_idx);
x_solspec   = x_est(solspec_idx);
y_solspec   = y_est(solspec_idx);
z_solspec   = z_est(solspec_idx);
phi_solspec = phi_est(solspec_idx);
tht_solspec = tht_est(solspec_idx);
psi_solspec = psi_est(solspec_idx);

% - Dynamically valid data
dv_idx = logical((bagdata.vision_pose.sol_status > 0) .* (bagdata.vision_pose.dynamically_valid == 1));
t_est_dv = t_est(dv_idx);
x_est_dv = x_est(dv_idx);
y_est_dv = y_est(dv_idx);
z_est_dv = z_est(dv_idx);
phi_est_dv = phi_est(dv_idx);
tht_est_dv = tht_est(dv_idx);
psi_est_dv = psi_est(dv_idx);

% - Outlier data
ot_idx = logical((bagdata.vision_pose.sol_status > 0) .* ...
                 (bagdata.vision_pose.dynamically_valid == 1) .* ...
                 (bagdata.vision_pose.outlier == 1));
t_ot   = t_est(ot_idx);
x_ot   = x_est(ot_idx);
y_ot   = y_est(ot_idx);
z_ot   = z_est(ot_idx);
phi_ot = phi_est(ot_idx);
tht_ot = tht_est(ot_idx);
psi_ot = psi_est(ot_idx);

%% PLOTS

% figure('position',[10 10 1000 600])
% 
% subplot(2,3,1)
% plot(t_tru, x_tru, 'k-', t_est, x_est, 'r*')
% grid on
% legend('Truth','Estimate')
% ylabel('N (m)')
% subplot(2,3,2)
% plot(t_tru, y_tru, 'k-', t_est, y_est, 'r*')
% grid on
% ylabel('E (m)')
% subplot(2,3,3)
% plot(t_tru, z_tru, 'k-', t_est, z_est, 'r*')
% grid on
% ylabel('D (m)')
% subplot(2,3,4)
% plot(t_tru, 180/pi*phi_tru, 'k-', t_est, 180/pi*phi_est, 'r*')
% grid on
% ylabel('\phi (deg)')
% xlabel('t (s)')
% subplot(2,3,5)
% plot(t_tru, 180/pi*tht_tru, 'k-', t_est, 180/pi*tht_est, 'r*')
% grid on
% ylabel('\theta (deg)')
% xlabel('t (s)')
% subplot(2,3,6)
% plot(t_tru, 180/pi*psi_tru, 'k-', t_est, 180/pi*psi_est, 'r*')
% grid on
% ylabel('\psi (deg)')
% xlabel('t (s)')

figure('position', [50 50 3000 1750])
sgtitle(tstitle)
subplot(2,3,1)
plot(       t_tru, x_tru, 'k-', 'Linewidth', 1.35); hold on; grid on
plot(       t_est, x_est, 'g.')
plot(       t_sol, x_sol, 'bo')
scatter(t_solspec, x_solspec, 150, 'filled', 'MarkerFaceAlpha', 3/8, 'MarkerFaceColor', 'red')
plot(    t_est_dv, x_est_dv, 'c*')
plot(        t_ot, x_ot, 'r+'); hold off
ylabel('x (m)')
legend('Vicon Truth','Vision Raw Output','Vision Valid Solutions',solspec_legend,'Dynamically Valid Vision Estimate','Identified Outliers')
grid on
subplot(2,3,2)
plot(       t_tru, y_tru, 'k-', 'Linewidth', 1.35); hold on; grid on
plot(       t_est, y_est, 'g.')
plot(       t_sol, y_sol, 'bo')
scatter(t_solspec, y_solspec, 150, 'filled', 'MarkerFaceAlpha', 3/8, 'MarkerFaceColor', 'red')
plot(    t_est_dv, y_est_dv, 'c*')
plot(        t_ot, y_ot, 'r+'); hold off
ylabel('y (m)')
grid on
subplot(2,3,3)
plot(       t_tru, z_tru, 'k-', 'Linewidth', 1.35); hold on; grid on
plot(       t_est, z_est, 'g.')
plot(       t_sol, z_sol, 'bo')
scatter(t_solspec, z_solspec, 150, 'filled', 'MarkerFaceAlpha', 3/8, 'MarkerFaceColor', 'red')
plot(    t_est_dv, z_est_dv, 'c*')
plot(        t_ot, z_ot, 'r+'); hold off
ylabel('z (m)')
grid on
subplot(2,3,4)
plot(       t_tru, phi_tru, 'k-', 'Linewidth', 1.35); hold on; grid on
plot(       t_est, phi_est, 'g.')
plot(       t_sol, phi_sol, 'bo')
scatter(t_solspec, phi_solspec, 150, 'filled', 'MarkerFaceAlpha', 3/8, 'MarkerFaceColor', 'red')
plot(    t_est_dv, phi_est_dv, 'c*')
plot(        t_ot, phi_ot, 'r+'); hold off
xlabel('t (s)')
ylabel('\phi (deg)')
grid on
subplot(2,3,5)
plot(       t_tru, tht_tru, 'k-', 'Linewidth', 1.35); hold on; grid on
plot(       t_est, tht_est, 'g.')
plot(       t_sol, tht_sol, 'bo')
scatter(t_solspec, tht_solspec, 150, 'filled', 'MarkerFaceAlpha', 3/8, 'MarkerFaceColor', 'red')
plot(    t_est_dv, tht_est_dv, 'c*')
plot(        t_ot, tht_ot, 'r+'); hold off
xlabel('t (s)')
ylabel('\theta (deg)')
grid on
subplot(2,3,6)
plot(       t_tru, psi_tru, 'k-', 'Linewidth', 1.35); hold on; grid on
plot(       t_est, psi_est, 'g.')
plot(       t_sol, psi_sol, 'bo')
scatter(t_solspec, psi_solspec, 150, 'filled', 'MarkerFaceAlpha', 3/8, 'MarkerFaceColor', 'red')
plot(    t_est_dv, psi_est_dv, 'c*')
plot(        t_ot, psi_ot, 'r+'); hold off
xlabel('t (s)')
ylabel('\psi (deg)')
grid on

savefig(gcf, strcat('./318_visionsim_figs/',tstitle,'.fig'))