%% Plot the table behavior

%d = readCSV('surf/surf-1d.csv');
% d = readCSV('shark/shark-1d.csv');
d = readCSV('sail/sail-4.csv');

% Controller values
mPIV.KP = 25.0; mPIV.KI = 0.0; mPIV.KV = 0.0;
mFFGains.KVff = 0.0; mFFGains.KAff =0.0;
% Motor model values
mSystem.b = 1e-6;
mSystem.Iw = 1.35e-4;
mSystem.wn = 0.2;

% Table values
mSystem.hDotMax = .25;
mSystem.hMax = .25;

mSystem.Inertia =   [0.029943767017920 -1.040803424000000e-05 9.237942464000000e-05;
                     -1.040803424000000e-05 0.030334333726400 1.848606880000000e-06;
                     9.237942464000000e-05 1.848606880000000e-06 0.046686992545600];
                 
% Conversions
Kp =    [mPIV.KP*2.2*mSystem.Inertia(1,1)*mSystem.wn*mSystem.wn,0,0,0;
        0,mPIV.KP*2.2*mSystem.Inertia(2,2)*mSystem.wn*mSystem.wn,0,0;
        0,0,mPIV.KP*2.2*mSystem.Inertia(3,3)*mSystem.wn*mSystem.wn,0;];
    
Tc3to4 =    [-0.25,-0.25,0.25,0.25;
             -0.25,0.25,0.25,-0.25;
             0.25,-0.25,0.25,-0.25;
             0.25,0.25,0.25,0.25;];
         
%% Setup data

% Position
error = QtoEuler(d.error)*180/pi;
%pos = QtoEulerd.quat)*180/pi;
ref = QtoEuler(d.ref)*180/pi;

% Torque
%From gyros (filtered at 0.2Hz cutoff)
% radfilt = filter([0.01241, 0.01241],[1, -0.9752],d.y_dot);
radfilt = filter([0.05912, 0.05910],[1, -0.8818],d.y_dot);

Fs = 50;
dt = 1/Fs;
raddot = diff(radfilt)*Fs;
raddot = [raddot(1,:); raddot];
torque_gyros = mSystem.Inertia(3,3)*raddot;

%Speed command from torque command (integration)
sc = zeros(length(d.time),1);
for a=2:length(d.time)
    sc(a) = sc(a-1)+(torque_gyros(a)+torque_gyros(a-1))*(dt/mSystem.Inertia(3,3))/2;
end

% Table acc
acc = diff(radfilt)*Fs;
acc = [acc(1); acc];
%% Plot data

% Table position
    % incl. command and error
figure
plot(d.time, d.yaw*180/pi, d.time, error(:,3), d.time, ref(:,3))
grid on
legend('Yaw','Error', 'Reference')
    
% Table control
    % Tc3 (yaw)
    % Table speed command (calculated)
    % Table speed
    
figure
subplot(311)
plot(d.time, d.tc3(:,3), d.time, torque_gyros)
legend('Command', 'Measured (gyros')
title('Torque')
grid on
subplot(312)
plot(d.time, sc, d.time, d.y_dot, d.time, radfilt)
legend('Command', 'Measured', 'Filtered')
title('Speed')
grid on
subplot(313)
plot(d.time, acc)
% legend('Acc', 'Measured', 'Filtered')
title('Acc')
grid on
       