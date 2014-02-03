%% Test torque creation based on wheel speed
% This should differentiate the speed

d = readCSV('blue/blue-5.csv');
Iw = 0.0523;
%% Differentiate

% Low pass filter it first (1Hz cutoff);
radfilt = filter([0.05912, 0.05910],[1, -0.8818],-d.speedmeas);
%radfilt = filter([0.01241, 0.01241],[1, -0.9752],-d.speedmeas);
Fs = 50;
raddot = diff(radfilt)*Fs;
raddot = [raddot(1,:); raddot];
torque = Iw*raddot;
torquefilt = filter([0.01241, 0.01241],[1, -0.9752],torque);
%% Plot
figure
subplot(211)
plot(d.time,-d.speedmeas(:,3), d.time, radfilt(:,3));
subplot(212)
plot(d.time,torque(:,3));
hold on
plot(d.time,d.torque(:,3),'k', d.time, torquefilt(:,3),'r');