% Goal
final = 15*pi/180;
amax = 50*pi/180;
vmax = 40*pi/180;

% Interval?
t0 = 0;
t1 = sqrt(final/amax);
tf = 2*t1;
t = [0:1/100:1]*tf;
theta = zeros(length(t),1);
thetadot = theta;
theta2dot = theta;

for k=1:length(t)
    if t(k)<=t1
        theta(k) = amax/2*t(k)^2;
        thetadot(k) = amax*t(k);
        theta2dot(k) = amax;
    elseif t(k)<tf
        v1 = amax*t1;
        theta1 = amax/2*t1^2;
        
        theta(k) = -amax/2*(t(k)-t1)^2+theta1+v1*(t(k)-t1);
        thetadot(k) = -amax*(t(k)-t1)+v1;
        theta2dot(k) = -amax;
    end
end

%%
figure
subplot(311)
plot(t,theta)
title('Pos')
subplot(312)
plot(t,thetadot)
title('Speed')
subplot(313)
plot(t,theta2dot)
title('Acc')


