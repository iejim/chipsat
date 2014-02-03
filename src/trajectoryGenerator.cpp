#include "controller.h"

using namespace USU;

void Controller::trajectoryGenerator(quaternion q0, float angle, vector axis, quaternion time)
{

    // angle is "yoda_max" from previous function; angle amount need to move
    float x0, a0, v0;
    float yoda, yodadot, yodadot2;

    if (mImuTime< time(0)){// initialization
        yoda=0;
        yodadot=0;
        yodadot2=0;
    }
    else if (mImuTime < time(1)){
        dt=mImuTime-time(0);
        x0=0;
        v0=0;
        a0=Amax;
        yoda=x0+v0*dt+a0*dt*dt/2; // position
        yodadot=v0+a0*dt; // speed
        yodadot2=a0; // accel
    }
    else if (mImuTime< time(2)){
        dt=mImuTime-time(1);
        x0=Amax*(time(1)-time(0))*(time(1)-time(0))/2;
        v0=Vmax;
        a0=0;
        yoda=x0+v0*dt+a0*dt*dt/2;
        yodadot=v0+a0*dt;
        yodadot2=a0;
    }
    else if (mImuTime< time(3)){
        dt=mImuTime-time(2);
        x0=Amax*(time(1)-time(0))*(time(1)-time(0)/2+Vmax*(time(2)-time(1));
        v0=Vmax;
        a0=-Amax;
        yoda=x0+v0*dt+a0*dt*dt/2;
        yodadot=v0+a0*dt;
        yodadot2=a0;
    }
    else {
        yoda=angle;
        yodadot=0;
        yodadot2=0;
    }
    quaternion q;
    q << sin(yoda/2)*E(0),
         sin(yoda/2)*E(1),
         sin(yoda/2)*E(2),
         cos(yoda/2);

    Matrix4f Q;
        Q << q(3),q(2),-q(1),q(0),
            -q(2),q(3),q(0),q(1),
             q(1),-q(0),q(3),q(2),
            -q(0),-q(1),-q(2),q(3);

    mQuatStar=Q*q0;
    mOmegarStar=yodadot*axis;
    mAlphaStar=yodadot2*axis;

    return

}

void Controller::trajectorySetup(quaternion &q0, float &angle, vector &axis, quaternion &time);
{
    // "Inputs": tstart = 'mImuTime' when mReference changes in input file;
            // q0 = 'mCurrentQuat' when mReference changes in input file;
            // qf = 'mReference' when mReference changes in input file;

    // Calculate difference qe between current position and mReference
    Matrix4f Q << mReference.q(3),mReference.q(2),-mReference.q(1),mReference.q(0),
                -mReference.q(2),mReference.q(3),mReference.q(0),mReference.q(1),
                mReference.q(1),-mReference.q(0),mReference.q(3),mReference.q(2),
                -mReference.q(0),-mReference.q(1),-mReference.q(2),mReference.q(3);

    quaternion qs(-mCurrentQuat(0),-mCurrentQuat(1),-mCurrentQuat(2),mCurrentQuat(3));
    quaternion qe = Q*qs;

    q0 = mCurrentQuat;

    angle = 2*acos(qe(3));  //Calculate the angle of rotation (rad)
    axis = vector(qe(0),qe(1),qe(2))/sin(angle/2);

    time(0)=mImuTime;                 // Start time for maneuver (sec)
    if (angle > Vmax*Vmax/Amax){       // Trapezoidal trajectory
        time(1)=time(0)+Vmax/Amax;      // End of acceleration (sec)
        time(2)=time(1)+(angle-Vmax*Vmax/Amax)/Vmax;    // End of vel (sec)
        time(3)=time(2)+Vmax/Amax;      // End of deceleration (sec)
    }
    else{                                // Max accel/decel
        time(1)=time(0)+sqrt(angle/Amax);   // End of acceleration (sec)
        time(2)=time(1);                // No constant vel time
        time(3)=time(2)+sqrt(angle/Amax); // Max accel/decel
    }

}
