%% Test torque creation based on wheel speed
% This should differentiate the speed

d = readCSV('surf/surf-1d.csv');
%Iw = 0.0523;
Iw = 1.352e-4;
%% Differentiate the measured speed

% Low pass filter it first (1Hz cutoff);
radfilt = filter([0.05912, 0.05910],[1, -0.8818],d.speedmeas);
%radfilt = filter([0.01241, 0.01241],[1, -0.9752],-d.speedmeas);
Fs = 50;
raddot = diff(radfilt)*Fs;
raddot = [raddot(1,:); raddot];
torque = Iw*raddot;
torquefilt = filter([0.01241, 0.01241],[1, -0.9752],torque);

%% Differentiate the command speed

% Low pass filter it first (1Hz cutoff);
%cmdfilt = filter([0.05912, 0.05910],[1, -0.8818],d.speedcmd);

Fs = 50;
cmddot = diff(d.speedcmd)*Fs;
cmddot = [cmddot(1,:); cmddot];
torquecmd = Iw*cmddot;
torquecmdfilt = filter([0.01241, 0.01241],[1, -0.9752],torquecmd);
%% Plot
figure
subplot(211)
plot(d.time,d.speedmeas(:,3), d.time, radfilt(:,3),'r', d.time, d.speedcmd(:,3),'k');
legend('Measured Speed', 'Filtered', 'Command')
grid on
subplot(212)
plot(d.time,torque(:,3));
hold on
plot(d.time,d.torque(:,3),'k', d.time, torquefilt(:,3),'r');
plot(d.time,torquecmd(:,3),'Color',[0,200/255,0])
legend('Calculated from Measured Speed', 'Command', 'Filtered', 'From Speed Cmd')
grid on