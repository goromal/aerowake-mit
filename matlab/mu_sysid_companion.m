% bagfile = '../bags/312_flight_bags/1-freeflights/2020-03-12-12-54-40.bag';
% outputdir = '312_drag_figs/2020-03-12-12-54-40';
% resultstitle = '312 Flights 2020-03-12-12-54-40 Near-zero Accels'; % no illegal file strings!
% t0 = 10;
% tf = 50;

% bagfile = '../bags/312_flight_bags/1-freeflights/2020-03-12-12-57-04.bag';
% outputdir = '312_drag_figs/2020-03-12-12-57-04';
% resultstitle = '312 Flights 2020-03-12-12-57-04 Near-zero Accels'; % no illegal file strings!
% t0 = 10;
% tf = 50;

bagfile = '../bags/312_flight_bags/1-freeflights/2020-03-12-12-59-56.bag';
outputdir = '312_drag_figs/2020-03-12-12-59-56';
resultstitle = '312 Flights 2020-03-12-12-59-56 Near-zero Accels'; % no illegal file strings!
t0 = 0;
tf = 100;

factors = [5 0 5 0 5 5 5 5 5 5]; % leave the zeros
dot_epsilon = 0.1;
% 0 = make logs (make sure outputdir exists), 1 = make and process solution
% (make sure outputdir/config.yaml file exists, maybe run mf_process before and after...)
action = 1;

%%
close all
if action == 0
    disp('EXTRACTING BAG DATA')
    addpath(genpath('matlab_utilities/'));
    addpath(genpath('matlab-utils/'));
    bagdata = processAllROSBagTopics(bagfile, false);
    disp('============================')

    disp('CONVERTING BAG DATA')
    t = bagdata.odometry.t;

    if t0 < 0
        t0 = t(1);
    end
    if tf < 0
        tf = t(end);
    end
    clipped_idx = logical((t >= t0) .* (t <= tf));
    t = t(clipped_idx);

    logdata = zeros(8,length(t));

    logdata(1,:) = bagdata.odometry.twist.linear(1,clipped_idx);
    logdata(2,:) = bagdata.odometry.twist.linear(2,clipped_idx);
    logdata(3,:) = bagdata.odometry.twist.linear(3,clipped_idx);

    [logdata(4,:), logdata(5,:), ~] = ...
        QuatdDataToRPY(bagdata.odometry.pose.orientation(4,clipped_idx), ...
                       bagdata.odometry.pose.orientation(1,clipped_idx), ...
                       bagdata.odometry.pose.orientation(2,clipped_idx), ...
                       bagdata.odometry.pose.orientation(3,clipped_idx));

    logdata(6,:) = bagdata.odometry.twist.angular(1,clipped_idx);
    logdata(7,:) = bagdata.odometry.twist.angular(2,clipped_idx);
    logdata(8,:) = bagdata.odometry.twist.angular(3,clipped_idx);

    disp('SMOOTHING AND DIFFERENTIATING DATA')
    [t_ss, u_ss] = smooth_stutter_data(t, logdata(1,:), factors(1));
    u_dot = dirtyDerivativeVector(t_ss, u_ss);
    [   ~, v_ss] = smooth_stutter_data(t, logdata(2,:), factors(2));
    v_dot = dirtyDerivativeVector(t_ss, v_ss);
    [   ~, w_ss] = smooth_stutter_data(t, logdata(3,:), factors(3));
    [ ~, phi_ss] = smooth_stutter_data(t, logdata(4,:), factors(4));
    [ ~, tht_ss] = smooth_stutter_data(t, logdata(5,:), factors(5));
    [   ~, p_ss] = smooth_stutter_data(t, logdata(6,:), factors(6));
    [   ~, q_ss] = smooth_stutter_data(t, logdata(7,:), factors(7));
    [   ~, r_ss] = smooth_stutter_data(t, logdata(8,:), factors(8));
    zvel_idx = logical((abs(u_dot) < dot_epsilon) .* (abs(v_dot) < dot_epsilon));
    ss_data = [t_ss(zvel_idx); u_ss(zvel_idx); u_dot(zvel_idx); v_ss(zvel_idx); ...
               v_dot(zvel_idx); w_ss(zvel_idx); phi_ss(zvel_idx); tht_ss(zvel_idx); ...
               p_ss(zvel_idx); q_ss(zvel_idx); r_ss(zvel_idx)];
    cp_data = [t; logdata(1,:); zeros(size(t)); logdata(2,:); zeros(size(t)); ...
               logdata(3,:); logdata(4,:); logdata(5,:); logdata(6,:); ...
               logdata(7,:); logdata(8,:)];

    lognames = {'u','u_dot','v','v_dot','w','phi','tht','p','q','r'};

    disp('PLOTTING AND SAVING')
    for i = 1:1:10
        
        figure('position',[10 10 3000 1000])
        if i == 2 || i == 4
            plot(ss_data(1,:), ss_data(i+1,:), 'r.', 'Linewidth', 1.5)
            grid on
        else
            subplot(2,1,1)
            plot(cp_data(1,:), cp_data(i+1,:), 'k.', 'Linewidth', 1.5)
            grid on
            title(strcat(lognames{i},' (Factor: ',num2str(factors(i)),')'))
            subplot(2,1,2)
            plot(ss_data(1,:), ss_data(i+1,:), 'r.', 'Linewidth', 1.5)
            grid on
        end
        savefig(gcf, strcat(outputdir,'/',lognames{i},'.fig'))
        write_log([ss_data(1,:); ss_data(i+1,:)], strcat(outputdir,'/',lognames{i},'.log'))
        close all
    end
%     for i = 1:1:8
%         disp(lognames{i})
%         [t_ss, y_ss] = smooth_stutter_data(t, logdata(i,:), factors(i));
%         figure('position',[10 10 3000 1000])
%         subplot(2,1,1)
%         plot(t, logdata(i,:), 'k.', 'Linewidth', 1.5)
%         grid on
%         title(strcat(lognames{i},' (Factor: ',num2str(factors(i)),')'))
%         subplot(2,1,2)
%         plot(t_ss, y_ss, 'r.', 'Linewidth', 1.5)
%         grid on
%         savefig(gcf, strcat(outputdir,'/',lognames{i},'.fig'))
%         write_log([t_ss; y_ss], strcat(outputdir,'/',lognames{i},'.log'))
%         close all
%         if i == 1
%             disp('u_dot')
%             u_dot = dirtyDerivativeVector(t_ss, y_ss);
%             figure('position',[10 10 3000 1000])
%             plot(t_ss, u_dot, 'r.', 'Linewidth', 1.5)
%             grid on
%             savefig(gcf, strcat(outputdir,'/u_dot.fig'))
%             write_log([t_ss; u_dot], strcat(outputdir,'/u_dot.log'))
%             close all
%         end
%         if i == 2
%             disp('v_dot')
%             v_dot = dirtyDerivativeVector(t_ss, y_ss);
%             figure('position',[10 10 3000 1000])
%             plot(t_ss, v_dot, 'r.', 'Linewidth', 1.5)
%             grid on
%             savefig(gcf, strcat(outputdir,'/v_dot.fig'))
%             write_log([t_ss; v_dot], strcat(outputdir,'/v_dot.log'))
%             close all
%         end
%     end
else
    %%
    % Get solution
    system(['../scripts/sysid/multirotor-drag-sysid/build/multirotor_mu_least_squares ' outputdir '/config.yaml'])
    sols_data = read_log(strcat(outputdir,'/SOLS.log'), 2);
    n_set = 1:1:(size(sols_data,2)-1);
    m = sols_data(1,1);
    g = sols_data(2,1);
    mu_x = sols_data(1,2:end);
    mean_mu_x = median(mu_x);
    mu_y = sols_data(2,2:end);
    mean_mu_y = median(mu_y);
    figure('position',[10 10 2000 500])
    plot(n_set, mu_x, 'r.', [n_set(1) n_set(end)], [mean_mu_x mean_mu_x], 'r--', ...
         n_set, mu_y, 'g.', [n_set(1) n_set(end)], [mean_mu_y mean_mu_y], 'g--')
    grid on
    xlabel('Number of datapoints used in optimization')
    ylabel('\mu')
    legend('\mu_x','median \mu_x','\mu_y','median \mu_y')
    resultsplotstitle = [resultstitle ' (g = ' num2str(g) ', m = ' num2str(m) '): \mu = [' num2str(mean_mu_x) ', ' num2str(mean_mu_y) ']'];
    title(resultsplotstitle)
    savefig(gcf, strcat(outputdir,'/',resultstitle,'_',strrep(num2str(g),'.','p'),'_',strrep(num2str(m),'.','p'), '.fig'))
end

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