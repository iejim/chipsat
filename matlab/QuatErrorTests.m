%% Plot the results of Rotation Tests 
% File: Fuzz-4.csv, Fuzz-6.csv
% Moving: 0, +90, 0, +90
% Purpose: What is the error and the reaction on each case?
%% Load
d6 = readCSV32('fuzz/fuzz-14.csv');

%% Using Quaternion structure

%Calculate the error
qe = [];
ye = [];
yq= [];
yt=[];
for a=1:length(d6.time) 
    Qt = [d6.ref(a, 4), d6.ref(a, 3), -d6.ref(a, 2), d6.ref(a, 1);
          -d6.ref(a, 3), d6.ref(a, 4), d6.ref(a, 1), d6.ref(a, 2);
          d6.ref(a, 2), -d6.ref(a, 1), d6.ref(a, 4), d6.ref(a, 3);
          -d6.ref(a, 1), -d6.ref(a, 2), -d6.ref(a, 3), d6.ref(a, 4);];
    qs = [d6.quat(a,1:3) d6.quat(a,4)]';
    error = Qt*qs;
    if error(4)<0
        error = -error;
    end
    qe = [qe; error'];
    [r,p,y] = MtoEuler(QtoM(error));
    ye = [ye; y*180/pi];
    [r,p,y] = MtoEuler(QtoM(d6.quat(a,:)));
    yq = [yq; y*180/pi];
    [r,p,y] = MtoEuler(QtoM(d6.ref(a,:)));
    yt = [yt; y*180/pi];
end

%%
%Concatenate
res = [d6.ref, yt, d6.quat, d6.yaw*180/pi, d6.time, qe, ye, yq-yt];
%% Using Angles

yaw = [-pi:.001/pi:pi];
pitch = rand(1,length(yaw))*0.0;
roll = rand(1,length(yaw))*0.0;
quat = [];
for a=1:length(yaw)
    quat = [quat; MtoQ(EulertoM([roll(a),pitch(a),yaw(a)]))];
end
quat = [yaw'*180/pi, quat];
%matq = angle2quat(roll, pitch, yaw, 'XYZ');
%matq = [matq(:,2:4) matq(:,1)];
%% Plotting

%Plots q(3) and q(4) as a function of the angle
figure(2)
plot(yaw*180/pi, quat(:,4), yaw*180/pi, quat(:,5))
grid on
title('Quaternion k and s')
xlabel('Angle (deg)')
legend('k','s')

%% Go back to yaw (test the opposite algorithm)
yr = [];
for a=1:length(yaw)
    [r,p,y] = MtoEuler(QtoM(quat(a,2:5)));
    yr = [yr, y*180/pi];
end

%% Plot reconstructed yaw
figure(3)
plot(yaw*180/pi,yr)

%% Matlab version
% figure(3)
% plot(yaw*180/pi, matq(:,3), yaw*180/pi, matq(:,4))
% grid on
% title('Quaternion z and s')
% xlabel('Angle (deg)')
% legend('z','s')