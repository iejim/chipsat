%% 
clear all, close all
%% Get data
d6 = readCSV('dime/dime-8.csv');

%% Calculate yaw values 

%from qerror
ye = [];
for a=1:length(d6.time)
    [r,p,y] = MtoEuler(QtoM(d6.error(a,:)));
    ye = [ye; y*180/pi];
end

%from Reference
yr = [];
for a=1:length(d6.time)
    [r,p,y] = MtoEuler(QtoM(d6.ref(a,:)));
    yr = [yr; y*180/pi];
end
%% Plot
figure(1)
plot(d6.time, d6.yaw*180/pi, d6.time, ye, d6.time, yr);
title('Heading and Error (Ref = 0)')
xlabel('Time (s)')
legend('Yaw','e', 'R')
grid on

%% Plot torque command
torque_s = sum(d6.torque');
%Normalize (for simplicity)
torque_s = torque_s/max(torque_s);
figure(2)
plot(d6.time, ye, '--', d6.time, torque_s*180);
title('Heading and Torque (Ref = 0)')
xlabel('Time (s)')
legend('Yaw Error','T')
grid on