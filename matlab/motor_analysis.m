data = readtable('MotorTest.csv');
pfn = size(data,1);
 
%% Polynomial Fitting

idxs_1000 = [1 12 13 24 25 36];
idxs_1100 = [2 11 14 23 26 35];
idxs_1200 = [3 10 15 22 27 34];
idxs_1300 = [4  9 16 21 28 33];
idxs_1400 = [5  8 17 20 29 32];
idxs_1500 = [6  7 18 19 30 31];

selection_matrix = [idxs_1000; idxs_1100; idxs_1200; idxs_1300; ...
                    idxs_1400; idxs_1500];

data_pwm    = cherrypicked_mean_data(data.ESCSignal__s_, selection_matrix);
data_s      = (data_pwm - 1000)/1000;
data_thrust = cherrypicked_mean_data(data.Thrust_N_, selection_matrix);
data_torque = cherrypicked_mean_data(data.Torque_N_m_, selection_matrix);
data_omega  = cherrypicked_mean_data(data.MotorElectricalSpeed_rad_s_, selection_matrix);
          
figure('Position',[10 10 1500 500])

subplot(2,3,1)
plot(data_pwm, data_thrust, 'k*','Linewidth',2)
hold on; grid on
Pthrust = polyfit(data_pwm, data_thrust, 2);
disp(['Thrust Poly: [' num2str(Pthrust(1)) ', ' num2str(Pthrust(2)) ', ' num2str(Pthrust(3)) ']'])
plot(data_pwm, calc_poly(data_pwm, Pthrust),'k--')
title('Thrust Data Fit')
hold off

subplot(2,3,2)
plot(data_pwm, data_torque, 'k*','Linewidth',2)
hold on; grid on
Ptorque = polyfit(data_pwm, data_torque, 2);
disp(['Torque Poly: [' num2str(Ptorque(1)) ', ' num2str(Ptorque(2)) ', ' num2str(Ptorque(3)) ']'])
plot(data_pwm, calc_poly(data_pwm, Ptorque),'k--')
title('Torque Data Fit')
hold off

subplot(2,3,3)
plot(data_s, data_omega, 'k*', 'Linewidth',2)
hold on; grid on
Pomega = polyfit(data_s, data_omega, 1);
disp(['Omega Poly:  [' num2str(Pomega(1)) ', ' num2str(Pomega(2)) ']'])
plot(data_s, calc_poly(data_s, Pomega),'k--')
title('Omega Data Fit')
hold off

subplot(2,3,4)
TF_ratio = data_torque(2:end) ./ data_thrust(2:end);
plot(data_pwm(2:end), TF_ratio, 'k*','Linewidth',2)
ylim([0 max(TF_ratio)+0.005])
hold on; grid on
km = mean(TF_ratio);
disp(['km Fit:       ' num2str(km)])
plot(data_pwm([2,end]),[km km],'k--')
title('k_m Fit')
hold off

subplot(2,3,5)
T_ratio = data_thrust ./ (calc_poly(data_s, Pomega).^2);
plot(data_pwm, T_ratio, 'k*','Linewidth',2)
hold on; grid on
kn = mean(T_ratio(3:end));
disp(['kn Fit:       ' num2str(kn)])
plot(data_pwm([1,end]),[kn kn],'k--')
title('k_n Fit')
hold off

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
    
