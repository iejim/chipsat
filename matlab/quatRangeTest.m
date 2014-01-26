%% check if quaternion error at 180deg references
% 'quatRangeTest.m'
clear all;close all;clc;

% create +-360deg measurements
yawmeas=(-359:359)'.*pi/180;
anglemeas=[zeros(length(yawmeas),1),zeros(length(yawmeas),1),yawmeas];
for i=1:length(yawmeas)
    qmeas(i,:)=EulertoQ(anglemeas(i,:));
end

% create almost 180deg reference
angleref=[0,0,0*179.99*pi/180];
qreff=EulertoQ(angleref);
qref=ones(length(qmeas),1)*qreff;

% calculate quaternion error (with restriction that qe is always positive)
qe=calcQuatError(qref,qmeas);

% convert qe to degrees
error=QtoEuler(qe);

figure;
plot(anglemeas(:,3).*180/pi,'b');
hold on;
plot(error(:,3).*180/pi,'k');
grid on;