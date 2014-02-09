%% Table torque step behavior
% In steps of 0.02s, create a torque of 0.1Nm in 0.1 seconds

dt = 0.02;
Inertia =   [0.029943767017920 -1.040803424000000e-05 9.237942464000000e-05;
             -1.040803424000000e-05 0.030334333726400 1.848606880000000e-06;
             9.237942464000000e-05 1.848606880000000e-06 0.046686992545600];
                 
time = [0:dt:0.1];
tq = [0:1/(length(time)-1):1]*.1;

sc = zeros(length(time),1);
for a=2:length(time)
    sc(a) = sc(a-1)+(tq(a)+tq(a-1))*(dt/Inertia(3,3))/2;
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
