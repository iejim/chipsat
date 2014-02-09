function [mQuatStar, mOmegaStar,mAlphaStar] = trajectoryGenerator(q0, angle, ax, time, Amax, Vmax, t)
%     quaternion q0, float angle, vector axis, quaternion time

%     // angle is "yoda_max" from previous function; angle amount need to move
%     x0 =0; a0=0; v0=0; dt=0;
%     float yoda, yodadot, yodadot2;

    if (t< time(1))% initialization
        yoda=0;
        yodadot=0;
        yodadot2=0;
    elseif (t < time(2))
        dt=t-time(1);
        x0=0;
        v0=0;
        a0=Amax;
        yoda=x0+v0*dt+a0*dt*dt/2; % position
        yodadot=v0+a0*dt; % speed
        yodadot2=a0; % accel

    elseif (t< time(3))
        dt=t-time(2);
        x0=Amax*(time(2)-time(1))*(time(2)-time(1))/2;
        v0=Vmax;
        a0=0;
        yoda=x0+v0*dt+a0*dt*dt/2;
        yodadot=v0+a0*dt;
        yodadot2=a0;

    elseif (t< time(4))
        dt=t-time(3);
        x0=Amax/2*(time(2) - time(1))^2 + Vmax*(time(3)-time(2));
%         v0=Vmax;
        v0=Amax*(time(2) - time(1));
        a0=-Amax;
        yoda=x0+v0*dt+a0*dt*dt/2;
        yodadot=v0+a0*dt;
        yodadot2=a0;
    else 
        yoda=angle;
        yodadot=0;
        yodadot2=0;
    end

    q =[ sin(yoda/2)*ax(1),...
         sin(yoda/2)*ax(2),...
         sin(yoda/2)*ax(3),...
         cos(yoda/2)];

    Q =[ q(4),q(3),-q(2),q(1);...
        -q(3),q(4),q(1),q(2);...
         q(2),-q(1),q(4),q(3);...
        -q(1),-q(2),-q(3),q(4)];

    mQuatStar=Q*q0;
    mOmegaStar=yodadot*ax;
    mAlphaStar=yodadot2*ax;

    end