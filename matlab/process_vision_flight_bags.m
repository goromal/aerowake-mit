%% INPUTS

bagfile = '../bags/317_vision_bags/2020-03-17-11-40-14.bag';
tstitle = 'Mission-Simulation-Test';

%% FRONTMATTER

addpath(genpath('matlab_utilities/'));
addpath(genpath('matlab-utils/'));
bagdata = processAllROSBagTopics(bagfile, false);

close all

disp('============================')

%% TRANSFORMS

disp('Extracting transforms...')

% Xform from aerowake_uav -> UAV frame
X_XAV_UAV = Xformd_from_tq([0 0 0], Quatd_from_euler(pi, 0, 0));

% Xform from world -> aerowake_uav frame
[t_world_XAV, X_world_XAV] = poseStampedToTandXformd(bagdata.aerowake_uav.world);

% Xform from world -> aerowake_beacons_small frame
[t_world_abs, X_world_abs] = poseStampedToTandXformd(bagdata.aerowake_beacons_small.world);

% Xform from aerowake_beacons_small -> SHIP frame
X_abs_SHIP = Xformd_from_tq([0 0 0], Quatd_from_euler(0, 0, -pi/2));

disp('Interpolating and concatenating transforms...')

% Use interpolation and concatenation to get Xforms from SHIP -> UAV
% frame
X_world_abs_interpolator = GeneralizedInterpolator(t_world_abs, X_world_abs, "linear");
n = length(t_world_XAV);
t_SHIP_UAV = t_world_XAV;
X_SHIP_UAV = Xformd.empty(0, length(t_world_XAV));
for i = 1:1:n
    X_world_abs_i = X_world_abs_interpolator.y(t_world_XAV(i));
    X_SHIP_world_i = X_abs_SHIP.inverse() * X_world_abs_i.inverse();
    X_world_XAV_i = X_world_XAV(i);
    X_SHIP_UAV(i) = X_SHIP_world_i * X_world_XAV_i * X_XAV_UAV;
end

%% RELATIVE STATE MEASUREMENTS

disp('Extracting relative truth and estimates...')

% Relative truth
t_tru   = t_SHIP_UAV;
x_tru   = zeros(size(t_tru));
y_tru   = zeros(size(t_tru));
z_tru   = zeros(size(t_tru));
phi_tru = zeros(size(t_tru));
tht_tru = zeros(size(t_tru));
psi_tru = zeros(size(t_tru));
for i = 1:1:n
    x_tru(i)   = X_SHIP_UAV(i).t(1);
    y_tru(i)   = X_SHIP_UAV(i).t(2);
    z_tru(i)   = X_SHIP_UAV(i).t(3);
    phi_tru(i) = X_SHIP_UAV(i).q.roll()*180/pi;
    tht_tru(i) = X_SHIP_UAV(i).q.pitch()*180/pi;
    psi_tru(i) = X_SHIP_UAV(i).q.yaw()*180/pi;
end

% Relative estimated from vision - RAW
t_est   = bagdata.vision_pose.t;
x_est   = bagdata.vision_pose.transform.translation(1,:);
y_est   = bagdata.vision_pose.transform.translation(2,:);
z_est   = bagdata.vision_pose.transform.translation(3,:);
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

savefig(gcf, strcat('./317_vision_figs/',tstitle,'.fig'))