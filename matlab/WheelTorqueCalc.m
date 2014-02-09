%% Wheel Torque Step Behavior
% In steps of 0.02s, create a torque of 0.1Nm in 0.1 seconds

dt = 0.02;
Iw = 1.35e-4;
time = [0:dt:0.1];
tq = [0:1/(length(time)-1):1]*.1;

sc = zeros(length(time),1);
for a=2:length(time)
    sc(a) = sc(a-1)+(tq(a)+tq(a-1))*(dt/Iw)/2;
end

acc = diff(sc)/dt;
acc = [0; acc];
%%
figure
subplot(211)
plot(time,sc)
legend('Speed Command')
grid on

subplot(212)
plot(time,tq, time,acc)
legend('Torque command', 'Acceleration')
grid on