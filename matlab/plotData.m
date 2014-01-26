%% Marina Samuels
% 1/25/2014
% plot test data
close all;clear all;clc;

%%
d=readCSV32('fuzz/fuzz-14.csv');

% d=readCSV('dime/dime-9.csv');

% convert reference command from quaternion to euler angles
d.refangles=QtoEuler(d.ref);
% convert quat. error to angles error
d.errorangles=QtoEuler(d.error);

figure;
plot(d.time,d.yaw.*180/pi,'b',d.time,d.refangles(:,3).*180/pi,'r')
hold on;
plot(d.time,d.errorangles(:,3).*180/pi,'k');
grid on;
% axis([min(d.time),max(d.time)+5,min([min(d.yaw),min(d.refangles)])-5*pi/180,max([max(d.yaw),max(d.refangles)])+5*pi/180]);
a=axis;
axis([a(1),a(2),a(3)-5*pi/180,a(4)+5*pi/180]);

