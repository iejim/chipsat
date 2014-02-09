function [q0, angle, axis, time] = trajectorySetup(currentQ, ref, Amax, Vmax)
%     quaternion &q0, float &angle, vector &axis, quaternion &time
    % "Inputs": tstart = 'mImuTime' when mReference changes in input file;
            % q0 = 'mCurrentQuat' when mReference changes in input file;
            % qf = 'mReference' when mReference changes in input file;

    % Calculate difference qe between current position and mReference
    Q = [   ref(4),ref(3),-ref(2),ref(1);...
            -ref(3),ref(4),ref(1),ref(2);...
            ref(2),-ref(1),ref(4),ref(3);...
            -ref(1),-ref(2),-ref(3),ref(4)];

    qs =[-currentQ(1),-currentQ(2),-currentQ(3),currentQ(4)]';
    qe = Q*qs;

    q0 = currentQ';

    angle = 2*acos(qe(4));  %Calculate the angle of rotation (rad)
    axis = [qe(1),qe(2),qe(3)]'/sin(angle/2);

    time(1)=0;                 % Start time for maneuver (sec)
    if (angle > Vmax*Vmax/Amax)       % Trapezoidal trajectory
        time(2)=time(1)+Vmax/Amax;      % End of acceleration (sec)
        time(3)=time(2)+(angle-Vmax*Vmax/Amax)/Vmax;    % End of vel (sec)
        time(4)=time(3)+Vmax/Amax;      % End of deceleration (sec)
        
    else                                % Max accel/decel
        time(2)=time(1)+sqrt(angle/Amax);   % End of acceleration (sec)
        time(3)=time(2);                % No constant vel time
        time(4)=time(3)+sqrt(angle/Amax); % Max accel/decel
    end
end