%% INPUT FILES

bagfile = '../bags/312_flight_bags/1-freeflights/2020-03-12-12-54-40.bag';

% %% FRONTMATTER
% 
% addpath(genpath('matlab_utilities/'));
% addpath(genpath('matlab-utils/'));
% bagdata = processAllROSBagTopics(bagfile, false);
% 
% close all
% 
% disp('============================')

%% EXTRACT DATA

% t = bagdata.aerowake_uav.world.t;
% x = bagdata.aerowake_uav.world.pose.position(1,:);
t = bagdata.odometry.t;
x = bagdata.odometry.pose.position(1,:);

%% FILTER TESTS

% figure('position',[10 10 3000 1000])
% 
% % time health
% subplot(1,2,1)
% dt_ideal = range(t)/length(t);
% hist(diff(t), 500); hold on; grid on
% plot([dt_ideal dt_ideal],[0 length(t)],'r--','Linewidth',1.5)
% xlim([0 3*dt_ideal])
% hold off
% 
% % isolate trouble clusters
% subplot(1,2,2)
% 
% % acceptable_idx = [0 diff(t)] > 0.95*dt_ideal;
% acceptable_idx = [0 diff(t)] > 1.5*dt_ideal;
% 
% t_s = t(acceptable_idx);
% x_s = x(acceptable_idx);
% plot(t,x,'k.','Linewidth',1.5); hold on; grid on
% plot(t_s, x_s, 'ro')
% t_s_mid = t_s + 0.5*[diff(t_s) 0];
% x_s_mid = interp1(t,x,t_s_mid);
% plot(t_s_mid, x_s_mid, 'b+', 'Linewidth',1.25)
% hold off
% 
% % figure
% % plot(t_s, x_s, 'r-', t_s_mid, x_s_mid, 'b-')
% 
% figure
% n = 2*length(t_s_mid); % try to reconstruct a high-fidelity signal
% t_ss = linspace(t_s_mid(1),t_s_mid(end),n);
% x_ss = interp1(t_s_mid,x_s_mid,t_ss,'pchip');
% plot(t,x,'k-','Linewidth',1.25); hold on; grid on
% plot(t_ss,x_ss,'r-','Linewidth',1.25)
% plot(t_ss,smooth(t_ss,x_ss,0.1),'c-','Linewidth',1.25); hold off

figure('position',[10 10 3000 1000])
[t_ss, x_ss] = smooth_stutter_data(t, x, 5.0);
subplot(2,1,1)
plot(t,x,'k-','Linewidth',1.5)
grid on
subplot(2,1,2)
plot(t_ss,x_ss,'r-','Linewidth',1.5)
grid on

% ----------------------------------------------

function [t_smooth, y_smooth] = smooth_stutter_data(t, y, ideal_dt_factor)

dt_ideal = range(t)/length(t);
acceptable_idx = [0 diff(t)] > ideal_dt_factor*dt_ideal;
t_s = t(acceptable_idx);
t_s_mid = t_s + 0.5*[diff(t_s) 0];
y_s_mid = interp1(t,y,t_s_mid);
n = length(t); % try to reconstruct a high-fidelity signal
t_smooth = linspace(t_s_mid(1),t_s_mid(end),n);
y_smooth = interp1(t_s_mid,y_s_mid,t_smooth,'pchip');

end