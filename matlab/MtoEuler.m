function [roll, pitch, yaw] = MtoEuler(M)
    roll = atan2(M(2,3),M(3,3));
    pitch = asin(-M(1,3));
    yaw = atan2(M(1,2),M(1,1));
    
end