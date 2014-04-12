%% Marina Samuels
% 1/25/2014
% plot test data
close all;clear all;clc;

%%
% d=readCSV32('fuzz/fuzz-14.csv');
% d=readCSV('dime/dime-9.csv');
% d=readCSV('blue/blue-5.csv');
% d = readCSV('surf/surf-1d.csv');
% d = readCSV('shark/shark-6.csv');
% d = readCSV('shark/shark-3.csv');
d = readCSV('sail/sail-14.csv');

% convert from quaternion to euler angles
d.refangles=QtoEuler(d.ref); % input reference
d.refstar=QtoEuler(d.quatstar); % trajectory reference command
d.errorangles=QtoEuler(d.error); % quat. error

% convert trajectory reference command from quaternion to euler angles
d.quatstar=QtoEuler(d.ref);


% convert duty cycles back to rad/s
d.dc2radps = zeros(size(d.dc));

for i=1:length(d.dc2radps)
    d.dc2radps(i,:) = d.dc(i,:)-10*sign(d.dc(i,:));
end
    d.dc2radps = d.dc2radps*5000/80*pi/30;
    
% calculate torque from current used by wheels
d.torque_curr = d.amps*36.9e-3;%*10; % (Nm)

Iw=1.352e-4;
b=1e-6;
calc_torque = diff(d.speedcmd)/0.02;
calc_torque = [zeros(1,4);calc_torque]+b.*d.speedcmd;

% calculate torque of the table from gyro speeds
Inertia =   [0.029943767017920 -1.040803424000000e-05 9.237942464000000e-05;
                     -1.040803424000000e-05 0.030334333726400 1.848606880000000e-06;
                     9.237942464000000e-05 1.848606880000000e-06 0.046686992545600];

d.rates = [d.r_dot,d.p_dot,d.y_dot];

% filter coefficients
a=0.01241;%0.05912
b=0.9752; %0.8818

rdot_filt = filter([a, a],[1, -b],d.r_dot);
pdot_filt = filter([a, a],[1, -b],d.p_dot);
ydot_filt = filter([a, a],[1, -b],d.y_dot);
filtRates=[rdot_filt,pdot_filt,ydot_filt];

alpha_r = [0;diff(rdot_filt)]./0.02;
alpha_p = [0;diff(pdot_filt)]./0.02;
alpha_y = [0;diff(ydot_filt)]./0.02;
d.alpha = [alpha_r,alpha_p,alpha_y];
d.torqueSC_calc = d.alpha*Inertia;
                 

% for a=2:length(d6.time)
%     mTc3 =2*Kp*qe(a,:)'*qe(a,4);
%     Tc3Comp = [mTc3(1), mTc3(2), mTc3(3), 0.]';
%     Torque(a,:) = (Tc3to4*Tc3Comp)';
%     SpeedCmd(a,:) = SpeedCmd(a-1,:)+(Torque(a,:)+Torque(a-1,:))*(delta_time/mSystem.Iw)/2;
% end

%% calculate cross term
a=0.05912;
b=0.8818;
speedfilt = filter([a,a],[1,-b],d.speedmeas);

Tc4to3 = [-1, -1, 1, 1;    -1, 1, -1, 1;   1, 1, 1, 1];
for i=1:size(ydot_filt,1)
    w = [filtRates(i,1);filtRates(i,2);filtRates(i,3)];
    wcross =[ 1, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0];
    h = Iw*Tc4to3*speedfilt(i,:)';
    crossterm(i,:) = [wcross*(Inertia*w+h)]';
end

%%
% compare position reference, measured, and calculated error
figure;
plot(d.time,d.yaw.*180/pi,'b',d.time,d.refangles(:,3).*180/pi,'r',d.time,d.refstar(:,3)*180/pi,'g')
hold on;
plot(d.time,d.errorangles(:,3).*180/pi,'k');
% axis([min(d.time),max(d.time)+5,min([min(d.yaw),min(d.refangles)])-5*pi/180,max([max(d.yaw),max(d.refangles)])+5*pi/180]);
a=axis;
axis([a(1),a(2),a(3)-5*pi/180,a(4)+5*pi/180]);
legend('meas','ref','refstar','error');
grid on;xlabel('time (s)'); ylabel('Degrees');
title('Table Position (Yaw)');

% compare table speed command (from trajectory generator) vs. measured
figure;
plot(d.time,d.omegastar(:,3),'r');
hold on; plot(d.time,ydot_filt,'b');
grid on;xlabel('time (sec)');ylabel('speed (rad/s)');
legend('star','meas'); title('Table Speed');

% compare table acceleration command vs. measured
figure;
plot(d.time,d.alphastar(:,3),'r');
hold on; plot(d.time,alpha_y,'b');
grid on;xlabel('time (sec)');ylabel('speed (rad/s^2)');
legend('star','meas'); title('Table Acceleration');

% plot the table torque command (3axis, from controller) vs
% measured-calculated
figure();
% subplot(3,1,1)
% plot(d.time,d.torqueSC_calc(:,1),'b');
% hold on;plot(d.time,d.tc3(:,1),'r');
% grid on;xlabel('time (sec)');ylabel('Roll torque (Nm)');
% legend('meas','cmd');
% title('Torque Command vs. Extrapolated from Measured Gyro Speeds');
% 
% subplot(3,1,2)
% plot(d.time,d.torqueSC_calc(:,2),'b');
% hold on;plot(d.time,d.tc3(:,2),'r');
% grid on;xlabel('time (sec)');ylabel('Pitch torque (Nm)');
% 
% subplot(3,1,3)
plot(d.time,d.tc3(:,3),'r');
hold on;plot(d.time,d.torqueSC_calc(:,3),'b');
grid on;xlabel('time (sec)');ylabel('Yaw torque (Nm)');% title('Torque Command vs. Extrapolated from Measured Gyro Speeds');
legend('cmd','meas');
title('Torque Command vs. Torque Extrapolated from Measured Gyro Speeds');

% plot crossterm values (that may or not contribue to Tc3), depending on
% control law used
figure;plot(d.time,crossterm(:,1),'r',d.time,crossterm(:,2),'b',d.time,crossterm(:,3),'k');
grid on;xlabel('time (sec)');ylabel('torque (Nm)');
legend('roll x','pitch y','yaw z');title('Crossterm Values');

% plot motor speed input command (4) vs. measured (4)
figure;
subplot(2,4,1)
plot(d.time,d.speedcmd(:,1),'r',d.time,d.speedmeas(:,1),'b');
hold on; plot(d.time,d.dc2radps(:,1),'k--'); grid on;% used to mult. speedcmd **(619/7262/80)
xlabel('time (s)');ylabel('speed (rad/s)');
title('Momentum Wheel Speeds');
subplot(2,4,2)
plot(d.time,d.speedcmd(:,2),'r',d.time,d.speedmeas(:,2),'b');grid on;
hold on; plot(d.time,d.dc2radps(:,2),'k--');
subplot(2,4,3)
plot(d.time,d.speedcmd(:,3),'r',d.time,d.speedmeas(:,3),'b');grid on;
hold on; plot(d.time,d.dc2radps(:,3),'k--');
subplot(2,4,4)
plot(d.time,d.speedcmd(:,4),'r',d.time,d.speedmeas(:,4),'b');grid on;
hold on; plot(d.time,d.dc2radps(:,4),'k--');
grid on;legend('cmd','meas','calcfromDC');

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
title('Wheel Command Torque vs. Torque Calculated from Measured Current');
subplot(1,4,2)
plot(d.time,d.torque(:,2),'k');grid on;
hold on; plot(d.time,d.torque_curr(:,2),'b');
subplot(1,4,3)
plot(d.time,d.torque(:,3),'k');grid on;
hold on; plot(d.time,d.torque_curr(:,3),'b');
subplot(1,4,4)
plot(d.time,d.torque(:,4),'k');grid on;
hold on; plot(d.time,d.torque_curr(:,4),'b');

