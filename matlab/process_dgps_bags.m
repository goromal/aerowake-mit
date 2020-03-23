bagdir = '../bags/dgps_bags/06-03-2020';

%% 

figure('position',[10 10 1000 1000])
SMALLcoords = [  5  5  -5  -5; ...
                -5  5   5  -5] * 0.6;
LARGEcoords = [ 11 11 -12 -12; ...
               -11 13  13 -11] * 0.6;
plot([SMALLcoords(1,:) SMALLcoords(1,1)], [SMALLcoords(2,:) SMALLcoords(2,1)], 'k--', 'Linewidth', 2.0)
hold on; grid on
plot([LARGEcoords(1,:) LARGEcoords(1,1)], [LARGEcoords(2,:) LARGEcoords(2,1)], 'k--', 'Linewidth', 2.0)
plot(0, 0, 'b+', 'Linewidth', 2.0)
xlabel('Ship Y (m)')
ylabel('Ship X (m)')

%%

addpath(genpath('matlab_utilities/'));

LARGEbagdir = strcat(bagdir,'/LARGE');
LARGElisting = dir(LARGEbagdir);

for i = 1:1:length(LARGElisting)
    if contains(LARGElisting(i).name, 'bag')
        bagfile = strcat(LARGEbagdir, '/', LARGElisting(i).name);
        bagdata = processAllROSBagTopics(bagfile);
        posdata = bagdata.rover.navrelposned;
        plot(posdata.E, posdata.N, 'r*', 'Linewidth', 2.0)
    end
end

SMALLbagdir = strcat(bagdir,'/SMALL');
SMALLlisting = dir(SMALLbagdir);

for i = 1:1:length(SMALLlisting)
    if contains(SMALLlisting(i).name, 'bag')
        bagfile = strcat(SMALLbagdir, '/', SMALLlisting(i).name);
        bagdata = processAllROSBagTopics(bagfile);
        posdata = bagdata.rover.navrelposned;
        plot(posdata.E, posdata.N, 'g*', 'Linewidth', 2.0)
    end
end

hold off