%%
d = readCSV('tia-6.csv');

%% Fixed
roll_f = ((d.roll<0)*(-1)*pi)+((d.roll>0)*pi)-d.roll;
rdot_f = -d.r_dot;
pitch_f = -d.pitch;
pdot_f = d.p_dot;
yaw_f =((d.yaw<0)*(-1)*pi) +((d.yaw>0)*pi)- d.yaw;
ydot_f = d.y_dot;

%% Original
figure
subplot(311)
plot(d.time, d.roll, d.time,d.r_dot);
title('Original')
legend('Roll');
subplot(312)
plot(d.time, d.pitch, d.time,d.p_dot);
legend('Pitch')
subplot(313)
plot(d.time, d.yaw, d.time,d.y_dot);
legend('Yaw')

%% Fixed plot
figure
subplot(311)
plot(d.time, roll_f, d.time,rdot_f);
title('Fixed')
legend('Roll');
subplot(312)
plot(d.time, pitch_f, d.time,pdot_f);
legend('Pitch')
subplot(313)
plot(d.time, yaw_f, d.time,ydot_f);
legend('Yaw')