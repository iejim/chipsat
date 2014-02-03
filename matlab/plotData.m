%% Marina Samuels
% 1/25/2014
% plot test data
close all;clear all;clc;

%%
% d=readCSV32('fuzz/fuzz-14.csv');
% d=readCSV('dime/dime-9.csv');
d=readCSV('blue/blue-5.csv');

% convert reference command from quaternion to euler angles
d.refangles=QtoEuler(d.ref);
% convert quat. error to angles error
d.errorangles=QtoEuler(d.error);

% convert duty cycles back to rad/s
d.dc2radps = zeros(size(d.dc));

for i=1:length(d.dc2radps)
    d.dc2radps(i,:) = d.dc(i,:)-10*sign(d.dc(i,:));
end
    d.dc2radps = d.dc2radps*5000/80*pi/30;
    
% calculate torque from current used by wheels
d.torque_curr = -d.amps*36.9e-3*10; % (Nm)

% compare position reference, measured, and calculated error
figure;
plot(d.time,d.yaw.*180/pi,'b',d.time,d.refangles(:,3).*180/pi,'r')
hold on;
plot(d.time,d.errorangles(:,3).*180/pi,'k');
grid on;
% axis([min(d.time),max(d.time)+5,min([min(d.yaw),min(d.refangles)])-5*pi/180,max([max(d.yaw),max(d.refangles)])+5*pi/180]);
a=axis;
axis([a(1),a(2),a(3)-5*pi/180,a(4)+5*pi/180]);
legend('meas','ref','error');
xlabel('time (s)');
ylabel('Degrees');
title('Position (Yaw)');

figure;
% plot motor command input speeds vs. measured output speeds
subplot(2,4,1)
plot(d.time,d.speedcmd(:,1),'r',d.time,d.speedmeas(:,1),'b');grid on;legend('cmd','meas'); % used to mult. speedcmd **(619/7262/80)
hold on;
plot(d.time,d.dc2radps(:,1),'k--');
xlabel('time (s)');
ylabel('speed (rad/s)');
title('Momentum Wheels');
subplot(2,4,2)
plot(d.time,d.speedcmd(:,2),'r',d.time,d.speedmeas(:,2),'b');grid on;
hold on;
plot(d.time,d.dc2radps(:,2),'k--');
subplot(2,4,3)
plot(d.time,d.speedcmd(:,3),'r',d.time,d.speedmeas(:,3),'b');grid on;
hold on;
plot(d.time,d.dc2radps(:,3),'k--');
subplot(2,4,4)
plot(d.time,d.speedcmd(:,4),'r',d.time,d.speedmeas(:,4),'b');grid on;
hold on;
plot(d.time,d.dc2radps(:,4),'k--');

% plot duty cycles
subplot(2,4,5)
plot(d.time,d.dc(:,1),'g');grid on;legend('dc');
xlabel('time (s)');
ylabel('duty cycle');
subplot(2,4,6)
plot(d.time,d.dc(:,2),'g');grid on;
subplot(2,4,7)
plot(d.time,d.dc(:,3),'g');grid on;
subplot(2,4,8)
plot(d.time,d.dc(:,4),'g');grid on;

% plot 4-wheel torques
figure
subplot(1,4,1)
plot(d.time,d.torque(:,1),'k');
hold on; plot(d.time,d.torque_curr(:,1),'b');
grid on;legend('cmd','meas');
xlabel('time (s)');
ylabel('N-m');
title('Command Torque vs. Torque Calculated from Measured Current');
subplot(1,4,2)
plot(d.time,d.torque(:,2),'k');grid on;
hold on; plot(d.time,d.torque_curr(:,2),'b');
subplot(1,4,3)
plot(d.time,d.torque(:,3),'k');grid on;
hold on; plot(d.time,d.torque_curr(:,3),'b');
subplot(1,4,4)
plot(d.time,d.torque(:,4),'k');grid on;
hold on; plot(d.time,d.torque_curr(:,4),'b');

