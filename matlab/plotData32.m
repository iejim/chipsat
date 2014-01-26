%% Marina Samuels
% 1/25/2014
% plot test data
close all;clear all;clc;

%%
d=readCSV32('fuzz/fuzz-14.csv');
% d=readCSV32('tia/tia-5.csv');

% convert reference command from quaternion to euler angles
d.refangles=QtoEuler(d.ref);
% convert quat. error to angles error
d.qe=calcQuatError(d.ref,d.quat);
d.errorangles=QtoEuler(d.qe);

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
title('Yaw');
xaxis('time(s)');
yaxis('degrees');

% compare speed commands to outputs
figure;
% plot(d.time,d.speedcmds,'r',d.time,d.rpm*