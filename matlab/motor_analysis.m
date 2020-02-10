data = readtable('../bags/motor_logs/SHAWMS_E4M4_SECONDTRY_RF_EM2.csv');
figtitle = 'ROSflight Motor 2';
pfn = size(data,1);
 
%% Polynomial Fitting

idxs_1000 = [1 12 13 24];
idxs_1100 = [2 11 14 23];
idxs_1200 = [3 10 15 22];
idxs_1300 = [4  9 16 21];
idxs_1400 = [5  8 17 20];
idxs_1500 = [6  7 18 19];

% idxs_1000 = [1 12 13 24 25 36];
% idxs_1100 = [2 11 14 23 26 35];
% idxs_1200 = [3 10 15 22 27 34];
% idxs_1300 = [4  9 16 21 28 33];
% idxs_1400 = [5  8 17 20 29 32];
% idxs_1500 = [6  7 18 19 30 31];

selection_matrix = [idxs_1000; idxs_1100; idxs_1200; idxs_1300; ...
                    idxs_1400; idxs_1500];

data_pwm    = cherrypicked_mean_data(data.ESCSignal__s_, selection_matrix);
data_s      = (data_pwm - 1000)/1000;
data_thrust = -1*cherrypicked_mean_data(data.Thrust_N_, selection_matrix);
data_torque = cherrypicked_mean_data(data.Torque_N_m_, selection_matrix);
data_omega  = 2*pi/60*cherrypicked_mean_data(data.MotorElectricalSpeed_RPM_, selection_matrix);
          
figure('Position',[10 10 1500 500])
sgtitle(figtitle)

subplot(2,3,1)
plot(data_pwm, data_thrust, 'k*','Linewidth',2)
hold on; grid on
Pthrust = polyfit(data_pwm, data_thrust, 2);
Thrust_info_str = ['Thrust Poly: [' num2str(Pthrust(1)) ', ' num2str(Pthrust(2)) ', ' num2str(Pthrust(3)) ']'];
disp(Thrust_info_str)
plot(data_pwm, calc_poly(data_pwm, Pthrust),'k--')
xlabel('PWM Signal')
ylabel('Motor Thrust (N)')
title('Thrust Data Fit')
hold off

subplot(2,3,2)
plot(data_pwm, data_torque, 'k*','Linewidth',2)
hold on; grid on
Ptorque = polyfit(data_pwm, data_torque, 2);
Torque_info_str = ['Torque Poly: [' num2str(Ptorque(1)) ', ' num2str(Ptorque(2)) ', ' num2str(Ptorque(3)) ']'];
disp(Torque_info_str)
plot(data_pwm, calc_poly(data_pwm, Ptorque),'k--')
xlabel('PWM Signal')
ylabel('Motor Torque (N-m)')
title('Torque Data Fit')
hold off

subplot(2,3,3)
plot(data_s, data_omega, 'k*', 'Linewidth',2)
hold on; grid on
Pomega = polyfit(data_s, data_omega, 1);
Omega_info_str = ['Omega Poly:  [' num2str(Pomega(1)) ', ' num2str(Pomega(2)) ']'];
disp(Omega_info_str)
plot(data_s, calc_poly(data_s, Pomega),'k--')
title('Motor Speed Data Fit')
xlabel('Normalized PWM Signal')
ylabel('\omega (rad/s)')
hold off

subplot(2,3,4)
km_start_idx = 3;
TF_ratio = data_torque(km_start_idx:end) ./ data_thrust(km_start_idx:end);
plot(data_pwm(km_start_idx:end), TF_ratio, 'k*','Linewidth',2)
ylim([0 max(TF_ratio)+0.005])
hold on; grid on
km = mean(TF_ratio);
km_info_str = ['km Fit:       ' num2str(km)];
disp(km_info_str)
plot(data_pwm([km_start_idx,end]),[km km],'k--')
xlabel('PWM Signal')
ylabel('Torque / Thrust')
title('k_m Fit')
hold off

subplot(2,3,5)
kn_start_idx = 5;
T_ratio = data_thrust ./ (calc_poly(data_s, Pomega).^2);
plot(data_pwm, T_ratio, 'k*','Linewidth',2)
hold on; grid on
kn = mean(T_ratio(kn_start_idx:end));
kn_info_str = ['kn Fit:       ' num2str(kn)];
disp(kn_info_str)
plot(data_pwm([1,end]),[kn kn],'k--')
xlabel('PWM Signal')
ylabel('Thrust / \omega^2')
title('k_n Fit')
hold off

hF = subplot(2,3,6);
set(hF, 'color', [1 1 1],'visible','off')
text(0.0,0.9,Thrust_info_str)
text(0.0,0.7,Torque_info_str)
text(0.0,0.5,Omega_info_str)
text(0.0,0.3,km_info_str)
text(0.0,0.1,kn_info_str)

%% Supporting Functions

function cpmd = cherrypicked_mean_data(data, selection_matrix)

n = size(selection_matrix, 1);
cpmd = zeros(n, 1);
for i = 1:1:n
    cpmd(i,1) = mean(data(selection_matrix(i,:)));
end

end

function pval = calc_poly(x, p)

n = length(p);
pval = zeros(size(x));
for i = 1:1:n
    k = n - i;
    pval = pval + p(i) .* x.^k;
end

end

% function pval = pwmpoly(pwm, coeff)
% 
% pval = coeff(1).*pwm.*pwm + coeff(2).*pwm + coeff(3);
% 
% end
    
