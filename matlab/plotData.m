%% Marina Samuels
% 1/25/2014
% plot test data
close all;clear all;clc;

%%
% d=readCSV32('fuzz/fuzz-14.csv');
% d=readCSV('dime/dime-9.csv');
d=readCSV('blue/blue-4.csv');

% convert reference command from quaternion to euler angles
d.refangles=QtoEuler(d.ref);
% convert quat. error to angles error
d.errorangles=QtoEuler(d.error);

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

figure;
% plot motor command input speeds vs. measured output speeds
subplot(2,4,1)
plot(d.time,d.speedcmd(:,1)*(619/7262/80),'r',d.time,d.rpm(:,1)*pi/30,'b');grid on;legend('cmd','meas');
subplot(2,4,2)
plot(d.time,d.speedcmd(:,2)*(619/7262/80),'r',d.time,d.rpm(:,2)*pi/30,'b');grid on;
subplot(2,4,3)
plot(d.time,d.speedcmd(:,3)*(619/7262/80),'r',d.time,d.rpm(:,3)*pi/30,'b');grid on;
subplot(2,4,4)
plot(d.time,d.speedcmd(:,4)*(619/7262/80),'r',d.time,d.rpm(:,4)*pi/30,'b');grid on;

% plot duty cycles
subplot(2,4,5)
plot(d.time,d.dc(:,1),'g');grid on;legend('dc');
subplot(2,4,6)
plot(d.time,d.dc(:,2),'g');grid on;
subplot(2,4,7)
plot(d.time,d.dc(:,3),'g');grid on;
subplot(2,4,8)
plot(d.time,d.dc(:,4),'g');grid on;

% plot 4-wheel torques
figure
subplot(1,4,1)
plot(d.time,d.torque(:,1),'k');grid on;legend('torques');
subplot(1,4,2)
plot(d.time,d.torque(:,2),'k');grid on;
subplot(1,4,3)
plot(d.time,d.torque(:,3),'k');grid on;
subplot(1,4,4)
plot(d.time,d.torque(:,4),'k');grid on;