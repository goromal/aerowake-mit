%% INPUT FILES

bagfile = '../bags/vision_bags/vision_2020-02-13-21-37-30.bag';
visfile = '../ros-packages/aerowake_params/params/vision/mit-uav-sim.yaml';
frmfile = '../ros-packages/aerowake_params/params/frame/mit-uav-frame.yaml';

%% FRONTMATTER

addpath(genpath('matlab_utilities/'));
addpath(genpath('matlab-utils/'));
bagdata = processAllROSBagTopics(bagfile);
visdata = ReadYaml(visfile);
frmdata = ReadYaml(frmfile);

close all

disp('============================')

%% EXTRACT DATA

X_SHP_BCN = Xformd_from_tq([visdata.X_ship_beacon{1}, ...
                            visdata.X_ship_beacon{2}, ...
                            visdata.X_ship_beacon{3}], ...
                           Quatd_from_euler(visdata.X_ship_beacon{4}, ...
                                            visdata.X_ship_beacon{5}, ...
                                            visdata.X_ship_beacon{6}));

X_UAV_CAM = Xformd_from_tq([frmdata.T_UAV_CAM{1}, ...
                            frmdata.T_UAV_CAM{2}, ...
                            frmdata.T_UAV_CAM{3}], ...
                           Quatd_from_euler(frmdata.T_UAV_CAM{4}, ...
                                            frmdata.T_UAV_CAM{5}, ...
                                            frmdata.T_UAV_CAM{6}));
                                            
% Truth
t_tru = bagdata.aerowake_MIT.truth.NED.t;
t0 = t_tru(1);
t_tru = t_tru - t0;
N_tru = bagdata.aerowake_MIT.truth.NED.pose.position(1,:);
E_tru = bagdata.aerowake_MIT.truth.NED.pose.position(2,:);
D_tru = bagdata.aerowake_MIT.truth.NED.pose.position(3,:);
[phi_tru, tht_tru, psi_tru] = quat_to_euler(bagdata.aerowake_MIT.truth.NED.pose.orientation(4,:),...
                                bagdata.aerowake_MIT.truth.NED.pose.orientation(1,:),...
                                bagdata.aerowake_MIT.truth.NED.pose.orientation(2,:),...
                                bagdata.aerowake_MIT.truth.NED.pose.orientation(3,:));

% Ship Truth
t_shp = bagdata.boat_truth_NED.t - t0;
N_shp = bagdata.boat_truth_NED.pose.position(1,:);
E_shp = bagdata.boat_truth_NED.pose.position(2,:);
D_shp = bagdata.boat_truth_NED.pose.position(3,:);
[phi_shp, tht_shp, psi_shp] = quat_to_euler(bagdata.boat_truth_NED.pose.orientation(4,:),...
                                bagdata.boat_truth_NED.pose.orientation(1,:),...
                                bagdata.boat_truth_NED.pose.orientation(2,:),...
                                bagdata.boat_truth_NED.pose.orientation(3,:));

% Vision Pose Estimate
t_vis = bagdata.vision_pose.t - t0;
x_vis = bagdata.vision_pose.transform.translation(1,:);
y_vis = bagdata.vision_pose.transform.translation(2,:);
z_vis = bagdata.vision_pose.transform.translation(3,:);
qw_vis = bagdata.vision_pose.transform.rotation(4,:);
qx_vis = bagdata.vision_pose.transform.rotation(1,:);
qy_vis = bagdata.vision_pose.transform.rotation(2,:);
qz_vis = bagdata.vision_pose.transform.rotation(3,:);
% phi_vis = bagdata.vision_pose.transform.rotation(1,:);
% tht_vis = bagdata.vision_pose.transform.rotation(2,:);
% psi_vis = bagdata.vision_pose.transform.rotation(3,:);

%% DATA PROCESSING

N_shp_s = interp1(t_shp, N_shp, t_vis);
E_shp_s = interp1(t_shp, E_shp, t_vis);
D_shp_s = interp1(t_shp, D_shp, t_vis);
phi_shp_s = interp1(t_shp, phi_shp, t_vis);
tht_shp_s = interp1(t_shp, tht_shp, t_vis);
psi_shp_s = interp1(t_shp, psi_shp, t_vis);

n = length(t_vis);
t_UAV = t_vis;
N_UAV = zeros(1, n);
E_UAV = zeros(1, n);
D_UAV = zeros(1, n);
phi_UAV = zeros(1, n);
tht_UAV = zeros(1, n);
psi_UAV = zeros(1, n);

for i = 1:1:length(t_vis)
    % Vision pose estimate is from beacon frame to camera frame; need to
    % find transform from NED frame to UAV frame from that data
    
    t_NED_SHP = [N_shp_s(i) E_shp_s(i) D_shp_s(i)];
    q_NED_SHP = Quatd_from_euler(phi_shp_s(i), tht_shp_s(i), psi_shp_s(i));
    X_NED_SHP = Xformd_from_tq(t_NED_SHP, q_NED_SHP);
    
%     t_BCN_CAM = [x_vis(i) y_vis(i) z_vis(i)];
%     q_BCN_CAM = Quatd_from_euler(phi_vis(i), tht_vis(i), psi_vis(i));
%     X_BCN_CAM = Xformd_from_tq(t_BCN_CAM, q_BCN_CAM);
    X_CAM_BCN = Xformd([x_vis(i) y_vis(i) z_vis(i) qw_vis(i) ...
                        qx_vis(i) qy_vis(i) qz_vis(i)]);
    
    X_NED_UAV = X_NED_SHP * X_SHP_BCN * X_CAM_BCN.inverse() * X_UAV_CAM.inverse();
    
    N_UAV(i) = X_NED_UAV.t(1);
    E_UAV(i) = X_NED_UAV.t(2);
    D_UAV(i) = X_NED_UAV.t(3);
    phi_UAV(i) = X_NED_UAV.q.roll();
    tht_UAV(i) = X_NED_UAV.q.pitch();
    psi_UAV(i) = X_NED_UAV.q.yaw();
end
    
%% PLOTS

figure('position',[10 10 1000 600])

subplot(2,3,1)
plot(t_tru, N_tru, 'k-', t_UAV, N_UAV, 'r--')
grid on
legend('Truth','Estimate')
ylabel('N (m)')
subplot(2,3,2)
plot(t_tru, E_tru, 'k-', t_UAV, E_UAV, 'r--')
grid on
ylabel('E (m)')
subplot(2,3,3)
plot(t_tru, D_tru, 'k-', t_UAV, D_UAV, 'r--')
grid on
ylabel('D (m)')
subplot(2,3,4)
plot(t_tru, 180/pi*phi_tru, 'k-', t_UAV, 180/pi*phi_UAV, 'r--')
grid on
ylabel('\phi (deg)')
xlabel('t (s)')
subplot(2,3,5)
plot(t_tru, 180/pi*tht_tru, 'k-', t_UAV, 180/pi*tht_UAV, 'r--')
grid on
ylabel('\theta (deg)')
xlabel('t (s)')
subplot(2,3,6)
plot(t_tru, 180/pi*psi_tru, 'k-', t_UAV, 180/pi*psi_UAV, 'r--')
grid on
ylabel('\psi (deg)')
xlabel('t (s)')