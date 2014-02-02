#include "controller.h"

using namespace USU;
#define RAD2RPM 763.9437f

void Controller::controlLaw()
{


    /// Temporary variables for the controller are created before while loop.


    //Trajectory Generation Initalization
    mQuatStar = quaternion(0,0,0,0);
    mOmegaStar = vector(0,0,0);
    mAlphaStar = vector(0,0,0);
    float speed2dc = RAD2RPM/mSystem.motorSpeedMax;

//    float bIw = 1/(mSystem.b+mSystem.Iw); //come back and give number

    Matrix3x4 Kp;;
    Kp <<   mPIV.KP*2.2*mSystem.Inertia(0,0)*mSystem.wn*mSystem.wn,0,0,0,
            0,mPIV.KP*2.2*mSystem.Inertia(1,1)*mSystem.wn*mSystem.wn,0,0,
            0,0,mPIV.KP*2.2*mSystem.Inertia(2,2)*mSystem.wn*mSystem.wn,0;

    Matrix3x4 Kv;;
    Kv <<   mPIV.KI*1.9*mSystem.Inertia(0,0)*mSystem.wn*mSystem.wn,0,0,0,
            0,mPIV.KI*1.9*mSystem.Inertia(1,1)*mSystem.wn*mSystem.wn,0,0,
            0,0,mPIV.KI*1.9*mSystem.Inertia(2,2)*mSystem.wn*mSystem.wn,0;

    Matrix3x4 Ki;;
    Ki <<   mPIV.KP*1*mSystem.Inertia(0,0)*mSystem.wn*mSystem.wn,0,0,0,
            0,mPIV.KP*1*mSystem.Inertia(1,1)*mSystem.wn*mSystem.wn,0,0,
            0,0,mPIV.KP*1*mSystem.Inertia(2,2)*mSystem.wn*mSystem.wn,0;

    //Matrix to convert Torque values from a 3-axis element vector to the 4-wheel model
    Matrix4f Tc3to4;
//    Tc3to4 << 0.5,0,0.25,0.25,  0,0.5,0.25,-0.25,   -0.5,0,0.25,0.25,     0,-0.5,0.25,-0.25;
//correct Tc3to4 conversion matrix below:
    Tc3to4 << -0.25,-0.25,0.25,0.25,    -0.25,0.25,0.25,-0.25,  0.25,-0.25,0.25,-0.25,  0.25,0.25,0.25,0.25;


    Matrix4f Qt;

    //Flags
    bool inSync = false;
    bool gotIMU = false;
    bool gotReference = false;

    // For calculating time values
    struct timeval start, now;

/// In this while loop is where the periodic features of the controller happen.
    while (mKeepRunning){

        //Check if it's time to stop the program
        if (mStopTime <= mClock)
        {
            mKeepRunning = false;
            break;
        }


        if(!inSync){
        //Wait for the first IMU packet to arrive (as a way of synchronization)
            mFirstImuTime = 0.0;
            gotIMU = readIMU(mEuler, mCurrentRates, mFirstImuTime);
            while(!gotIMU){

                usleep(100);
                gotIMU = readIMU(mEuler, mCurrentRates, mFirstImuTime);
                cerr << ",";
            }
            gettimeofday(&start, NULL);
            inSync = true;

        }else {
            gotIMU = readIMU(mEuler, mCurrentRates, mImuTime);
            while(!gotIMU)
            {
                usleep(100); //Give it some time for data to arrive to the queue
                gotIMU = readIMU(mEuler, mCurrentRates, mImuTime);

            }

        }

        gettimeofday(&now, NULL);
        mClock = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec)/1000000.0f;

        //Check if it is time to change reference values
        //The first reference on the input file is already stored in mNextReference
        if (std::abs(mClock - mNextReference.time) <= 0.011 ){
            mReference = mNextReference;
            gotReference = true;

            if(mReference.num < mTotalRefs) //Read only if there are any left
                readNextReference();
        }

        fixRates(mCurrentRates);
        fixAngles(mEuler);
        readMotors(mSpeed, mAmps);
        mCurrentQuat = createQuaternion(mEuler);

        if(gotReference){
            gotReference = false;
        }

//TODO: Call the trajectory generation function
        //

        //Calculate quaternion error
        Qt << mReference.q(3),mReference.q(2),-mReference.q(1),mReference.q(0),
              -mReference.q(2),mReference.q(3),mReference.q(0),mReference.q(1),
              mReference.q(1),-mReference.q(0),mReference.q(3),mReference.q(2),
              -mReference.q(0),-mReference.q(1),-mReference.q(2),mReference.q(3);

        quaternion qs(-mCurrentQuat(0),-mCurrentQuat(1),-mCurrentQuat(2),mCurrentQuat(3));

        mQuatError = Qt*qs; //Save the error (as a quaternion)

        mQuatErrorI = integrateQ(mQuatError,mLastQuatError,mLastQuatErrorI, (mImuTime-mLastImuTime));

/// CONTROL LAW: calculate required torque on each of 3 axes
        mTc3 =2*Kp*mQuatError*mQuatError(3);
//        mTc3=2*Kp*mQuatError*mQuatError(3)+Ki*
        //Tc=2*Kp*qe*qe(3)+Ki*qei+Kv*(w_star-w)+Ka*alpha_star+Td_hat+crossterm;

        //Calculate required torque on each of 4 wheels
        Vector4f Tc3Comp(mTc3(0), mTc3(1), mTc3(2), 0.);
        mTorque = Tc3to4*Tc3Comp;

        //Calculate required speeds (rad/s)
        mSpeedCmd = integrateQ(mTorque,mLastTorque,mLastSpeedCmd,(mImuTime-mLastImuTime),(1/mSystem.Iw)); //units rad/s
        //
        //Calculate required duty cycles
        quaternion DC = mSpeedCmd*speed2dc;
        mDutyC = Vector4i((int)DC(0), (int)DC(1), (int)DC(2), (int)DC(3));  //if mSpeed is in rad/s

        //Adding 10%DC bias
        mDutyC += Vector4i(mDutyC(0)>0?10:-10,
                           mDutyC(1)>0?10:-10,
                           mDutyC(2)>0?10:-10,
                           mDutyC(3)>0?10:-10);

        sendDutyCycles(mDutyC);

        cerr << "Angles: " << mEuler << endl << "Rates: " << mCurrentRates << endl;
        cerr << "Quaternion" << mCurrentQuat(0) << "," <<  mCurrentQuat(1)
             << "," << mCurrentQuat(2) << "," << mCurrentQuat(3) << endl;
//        cerr << "Motor 0: " << "DC: " << mDutyC(0) << " "<< mSpeed(0) << " rad/s, " << mAmps(0) << " A" << endl;
//        cerr << "Motor 1: " << "DC: " << mDutyC(1) << " "<< mSpeed(1) << " rad/s, " << mAmps(1) << " A" << endl;
//        cerr << "Motor 2: " << "DC: " << mDutyC(2) << " "<< mSpeed(2) << " rad/s, " << mAmps(2) << " A" << endl;
//        cerr << "Motor 3: " << "DC: " << mDutyC(3) << " "<< mSpeed(3) << " rad/s, " << mAmps(3) << " A" << endl;

        //Save the data
        if(mLogging)
            logData();

//TODO: Log even if it does not exit gracefully

        updateStates();
        waitPeriod();

    }

}
