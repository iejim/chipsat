%% Test the Control Law Math
% Performs a check on the math, uses the CSV data
%clear all, close all
%% Load
% d = readCSV32('fuzz/fuzz-14.csv');
%d = readCSV('blue/blue-5.csv');
% d = readCSV('surf/surf-1d.csv');
d = readCSV('shark/shark-1c.csv');
L = length(d.time);
%Change to false to disable trajectory generation
useTrajectory = 1;

mPIV.KP = 20.0; mPIV.KI = 0.0; mPIV.KV = 0.0;
mFFGains.KVff = 0.0; mFFGains.KAff =0.0;
mSystem.b = 1e-6;
mSystem.Iw = 1.35e-4;
mSystem.wn = 0.2;
mSystem.hDotMax = .25;
mSystem.hMax = .25;
Vmax = 2.0;
Amax = 1.5*Vmax;


mSystem.Inertia =   [0.029943767017920 -1.040803424000000e-05 9.237942464000000e-05;
                     -1.040803424000000e-05 0.030334333726400 1.848606880000000e-06;
                     9.237942464000000e-05 1.848606880000000e-06 0.046686992545600];
              
mMotorScale.vToRads = -157.0010928631499 ;
mMotorScale.vToAmps = -2.522522522522;

Kp =    [mPIV.KP*2.2*mSystem.Inertia(1,1)*mSystem.wn*mSystem.wn,0,0,0;
        0,mPIV.KP*2.2*mSystem.Inertia(2,2)*mSystem.wn*mSystem.wn,0,0;
        0,0,mPIV.KP*2.2*mSystem.Inertia(3,3)*mSystem.wn*mSystem.wn,0;];
    
Tc3to4 =    [-0.25,-0.25,0.25,0.25;
             -0.25,0.25,0.25,-0.25;
             0.25,-0.25,0.25,-0.25;
             0.25,0.25,0.25,0.25;];

delta_time = d.time(2);
%% Calculate the error

% Quaternion structure
qe = [];
ye = [];
yq = [];
yt = [];
mQuatStar = zeros(L,4);
mOmegaStar = zeros(L,3);
mAlphaStar = zeros(L,3);
q0 = d.ref(1,:)';
angle = 0;
ax = [0;0;0];
time = [0;0;0;0];

for a=1:length(d.time)
    % Trajectory generation stuff
    if (a>=2) && useTrajectory && ~isequal(d.ref(a,:),d.ref(a-1,:)) %Changed reference, perform trajectory calculation
        espRef = EulertoQ([0,0,45]*pi/180);
       [q0, angle, ax, time]= trajectorySetup(d.quat(a,:), espRef, Amax, Vmax);
       time = time+d.time(a)
    end
    
    if useTrajectory
        [mQuatStar(a,:), mOmegaStar(a,:),mAlphaStar(a,:)] = trajectoryGenerator(q0, angle, ax, time, Amax, Vmax, d.time(a));
    end
    
    Qt = [d.ref(a, 4), d.ref(a, 3), -d.ref(a, 2), d.ref(a, 1);
          -d.ref(a, 3), d.ref(a, 4), d.ref(a, 1), d.ref(a, 2);
          d.ref(a, 2), -d.ref(a, 1), d.ref(a, 4), d.ref(a, 3);
          -d.ref(a, 1), -d.ref(a, 2), -d.ref(a, 3), d.ref(a, 4);];
    qs = [-d.quat(a,1:3) d.quat(a,4)]';
    error = Qt*qs;
%     if error(4)<0
%         error = -error;
%     end
    qe = [qe; error'];
    [r,p,y] = MtoEuler(QtoM(error));
    ye = [ye; y*180/pi];
    [r,p,y] = MtoEuler(QtoM(d.quat(a,:)));
    yq = [yq; y*180/pi];
    [r,p,y] = MtoEuler(QtoM(d.ref(a,:)));
    yt = [yt; y*180/pi];
        
end
%Save the trajectory generator output in angles
refStar=QtoEuler(mQuatStar)*180/pi; % trajectory reference command
%% Controller
Torque = zeros(length(d.time),4);
SpeedCmd = zeros(length(d.time),4);
a =1;
mTc3 =2*Kp*qe(a,:)'*qe(a,4);
Tc3Comp = [mTc3(1), mTc3(2), mTc3(3), 0.]';
Torque(a,:) = (Tc3to4*Tc3Comp)';
for a=2:length(d.time)
    mTc3 =2*Kp*qe(a,:)'*qe(a,4);
    Tc3Comp = [mTc3(1), mTc3(2), mTc3(3), 0.]';
    Torque(a,:) = (Tc3to4*Tc3Comp)';
    SpeedCmd(a,:) = SpeedCmd(a-1,:)+(Torque(a,:)+Torque(a-1,:))*(delta_time/mSystem.Iw)/2;
end
% SpeedCmd = (cumtrapz(d.time, Torque)*(delta_time/mSystem.Iw));
DC = SpeedCmd*(763.9437/5000);

DutyC = floor(DC);

%% Differentiate the command speed

% Low pass filter it first (1Hz cutoff);
%cmdfilt = filter([0.05912, 0.05910],[1, -0.8818],SpeedCmd);
%radfilt = filter([0.01241, 0.01241],[1, -0.9752],-d.speedmeas);
Fs = 50;
cmddot = diff(SpeedCmd)*Fs;
cmddot = [cmddot(1,:); cmddot];
torquecmd = mSystem.Iw*cmddot;
%torquecmdfilt = filter([0.01241, 0.01241],[1, -0.9752],torquecmd);
%% Plot movement

figure(1)
clf
plot(d.time, yq, d.time, ye, d.time, yt)
grid on
legend('State', 'Error', 'Reference')
if useTrajectory
    hold on
    plot(d.time, refStar(:,3),'k')
    legend('State', 'Error', 'Reference', 'Trajectory')
    hold off
    
    figure(2)
    subplot(211)
    plot(d.time, mOmegaStar(:,3))
    title('Speed')
    legend('OmegaStar')
    grid on
    
    subplot(212)
    plot(d.time, mAlphaStar(:,3))
    title('Acc')
    legend('AlphaStar')
    grid on
end

%% Plot Command
% 
% figure(2)
% clf
% subplot(241)
% plot(d.time, SpeedCmd(:,1), d.time, d.speedcmd(:,1))
% axis tight, grid on
% title('Speed Command')
% subplot(242)
% plot(d.time, SpeedCmd(:,2), d.time, d.speedcmd(:,2))
% axis tight, grid on
% subplot(243)
% plot(d.time, SpeedCmd(:,3), d.time, d.speedcmd(:,3))
% axis tight, grid on
% subplot(244)
% plot(d.time, SpeedCmd(:,4), d.time, d.speedcmd(:,4))
% axis tight, grid on
% legend('Sim','CSV')
% 
% 
% subplot(245), hold on
% plot(d.time, Torque(:,1),'k', d.time, d.torque(:,1),'r')
% plot(d.time,torquecmd(:,1),'Color',[0,200/255,0])
% title('Torque'), grid on
% axis tight
% subplot(246), hold on
% plot(d.time, Torque(:,2),'k', d.time, d.torque(:,2),'r')
% plot(d.time,torquecmd(:,2),'Color',[0,200/255,0])
% axis tight, grid on
% subplot(247), hold on
% plot(d.time, Torque(:,3),'k', d.time, d.torque(:,3),'r')
% plot(d.time,torquecmd(:,3),'Color',[0,200/255,0])
% axis tight, grid on
% subplot(248), hold on
% plot(d.time, Torque(:,4),'k', d.time, d.torque(:,4),'r')
% plot(d.time,torquecmd(:,4),'Color',[0,200/255,0])
% axis tight, grid on
% legend('Sim','CSV', 'From Speed Cmd')


