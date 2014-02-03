%% Test the Control Law Math
% Performs a check on the math, uses the CSV data
clear all, close all
%% Load
% d6 = readCSV32('fuzz/fuzz-14.csv');
d6 = readCSV('blue/blue-5.csv');

mPIV.KP = 3.0; mPIV.KI = 0.0; mPIV.KV = 0.0;
mFFGains.KVff = 0.0; mFFGains.KAff =0.0;
mSystem.b = 1e-6;
mSystem.Iw = 1.35e-4;
mSystem.wn = 0.2;
mSystem.hDotMax = .25;
mSystem.hMax = .25;

mSystem.Inertia =   [0.029943767017920 -1.040803424000000e-05 9.237942464000000e-05;
                     -1.040803424000000e-05 0.030334333726400 1.848606880000000e-06;
                     9.237942464000000e-05 1.848606880000000e-06 0.046686992545600];
                 
mMotorScale.vToRads = 157.0010928631499 ;
mMotorScale.vToAmps = 2.522522522522;

Kp =    [mPIV.KP*2.2*mSystem.Inertia(1,1)*mSystem.wn*mSystem.wn,0,0,0;
        0,mPIV.KP*2.2*mSystem.Inertia(2,2)*mSystem.wn*mSystem.wn,0,0;
        0,0,mPIV.KP*2.2*mSystem.Inertia(3,3)*mSystem.wn*mSystem.wn,0;];
    
Tc3to4 =    [-0.25,-0.25,0.25,0.25;
             -0.25,0.25,0.25,-0.25;
             0.25,-0.25,0.25,-0.25;
             0.25,0.25,0.25,0.25;];

delta_time = d6.time(2);
%% Calculate the error

% Quaternion structiure
qe = [];
ye = [];
yq= [];
yt=[];
for a=1:length(d6.time) 
    Qt = [d6.ref(a, 4), d6.ref(a, 3), -d6.ref(a, 2), d6.ref(a, 1);
          -d6.ref(a, 3), d6.ref(a, 4), d6.ref(a, 1), d6.ref(a, 2);
          d6.ref(a, 2), -d6.ref(a, 1), d6.ref(a, 4), d6.ref(a, 3);
          -d6.ref(a, 1), -d6.ref(a, 2), -d6.ref(a, 3), d6.ref(a, 4);];
    qs = [-d6.quat(a,1:3) d6.quat(a,4)]';
    error = Qt*qs;
%     if error(4)<0
%         error = -error;
%     end
    qe = [qe; error'];
    [r,p,y] = MtoEuler(QtoM(error));
    ye = [ye; y*180/pi];
    [r,p,y] = MtoEuler(QtoM(d6.quat(a,:)));
    yq = [yq; y*180/pi];
    [r,p,y] = MtoEuler(QtoM(d6.ref(a,:)));
    yt = [yt; y*180/pi];
end

%% Controller
Torque = zeros(length(d6.time),4);
SpeedCmd = zeros(length(d6.time),4);
a =1;
mTc3 =2*Kp*qe(a,:)'*qe(a,4);
Tc3Comp = [mTc3(1), mTc3(2), mTc3(3), 0.]';
Torque(a,:) = (Tc3to4*Tc3Comp)';
for a=2:length(d6.time)
    mTc3 =2*Kp*qe(a,:)'*qe(a,4);
    Tc3Comp = [mTc3(1), mTc3(2), mTc3(3), 0.]';
    Torque(a,:) = (Tc3to4*Tc3Comp)';
    SpeedCmd(a,:) = SpeedCmd(a-1,:)+(Torque(a,:)+Torque(a-1,:))*(delta_time/mSystem.Iw)/2;
end
% SpeedCmd = (cumtrapz(d6.time, Torque)*(delta_time/mSystem.Iw));
DC = SpeedCmd*(763.9437/5000);

DutyC = floor(DC);

%% Plot movement

figure(1)
plot(d6.time, yq, d6.time, ye, d6.time, yt)
legend('State', 'Error', 'Reference')
grid on
%% Plot Command

figure(2)
subplot(241)
plot(d6.time, SpeedCmd(:,1), d6.time, d6.speedcmd(:,1))
axis tight
title('Speed Command')
subplot(242)
plot(d6.time, SpeedCmd(:,2), d6.time, d6.speedcmd(:,2))
axis tight
subplot(243)
plot(d6.time, SpeedCmd(:,3), d6.time, d6.speedcmd(:,3))
axis tight
subplot(244)
plot(d6.time, SpeedCmd(:,4), d6.time, d6.speedcmd(:,4))
axis tight
legend('Sim','CSV')


subplot(245)
plot(d6.time, Torque(:,1),'k', d6.time, d6.torque(:,1),'r')
title('Torque')
axis tight
subplot(246)
plot(d6.time, Torque(:,2),'k', d6.time, d6.torque(:,2),'r')
axis tight
subplot(247)
plot(d6.time, Torque(:,3),'k', d6.time, d6.torque(:,3),'r')
axis tight
subplot(248)
plot(d6.time, Torque(:,4),'k', d6.time, d6.torque(:,4),'r')
axis tight
legend('Sim','CSV')
