%% Plot the results of stationary test 
% File: Fuzz-1.csv, Fuzz-4.csv
% Motors are off. Free drifting
%% Load
d = readCSV32('fuzz/fuzz-14.csv');
%d4 = readCSV('fuzz/fuzz-4.csv');
%Rate of change of the drift
drift_rate = diff(d.yaw);
drift_rate = [drift_rate(1); drift_rate]/0.02;

gyro_yaw = cumtrapz(d.time,d.y_dot);

%% Plot Yaw over time
figure(1)
subplot(321)
plot(d.time, d.yaw, d.time, gyro_yaw+d.yaw(1), 'r')
xlabel('Time (s)')
ylabel('Yaw (rad)')
%legend('Yaw', 'Pitch', 'Roll')
title('Drift in rad (Yaw readings & Gyro Integration)')
grid on

subplot(322)
plot(d.time, d.yaw*180/pi)
xlabel('Time (s)')
ylabel('Yaw (deg)')
%legend('Yaw')
title('Drift in deg')
grid on

subplot(312)
plot(d.time, drift_rate)
xlabel('Time (s)')
ylabel('Drift Rate (rad/s)')
title('Differentiation of Yaw')
grid on

subplot(313)
plot(d.time, d.y_dot, 'r')
xlabel('Time (s)')
ylabel('Yaw Rate (Gyro)(rad/s)')
title('Gyro Rates for Yaw')
grid on