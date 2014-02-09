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
    float angle;
    vector axis;
    quaternion time;
    quaternion q0;
    //To go from rad/s to duty cycle for the speed command
    float speed2dc = RAD2RPM/mSystem.motorSpeedMax;

    //Define PIV gains
    Matrix3x4 Kp;
    Kp <<   mPIV.KP*2.2*mSystem.Inertia(0,0)*mSystem.wn*mSystem.wn,0,0,0,
            0,mPIV.KP*2.2*mSystem.Inertia(1,1)*mSystem.wn*mSystem.wn,0,0,
            0,0,mPIV.KP*2.2*mSystem.Inertia(2,2)*mSystem.wn*mSystem.wn,0;

    Matrix3x4 Ki;
    Ki <<   mPIV.KP*1*mSystem.Inertia(0,0)*mSystem.wn*mSystem.wn,0,0,0,
            0,mPIV.KP*1*mSystem.Inertia(1,1)*mSystem.wn*mSystem.wn,0,0,
            0,0,mPIV.KP*1*mSystem.Inertia(2,2)*mSystem.wn*mSystem.wn,0;

    matrix Kv;
    Kv <<   mPIV.KI*1.9*mSystem.Inertia(0,0)*mSystem.wn*mSystem.wn,0,0,
            0,mPIV.KI*1.9*mSystem.Inertia(1,1)*mSystem.wn*mSystem.wn,0,
            0,0,mPIV.KI*1.9*mSystem.Inertia(2,2)*mSystem.wn*mSystem.wn;

    //Matrix to convert Torque values from a 3-axis element vector to the 4-wheel model
    Matrix4f Tc3to4;
    //correct Tc3to4 conversion matrix below:
    Tc3to4 << -0.25,-0.25,0.25,0.25,    -0.25,0.25,0.25,-0.25,  0.25,-0.25,0.25,-0.25,  0.25,0.25,0.25,0.25;

    //Matrix to convert from 4-wheel model to 3-axis (to calculate h, h_dot, crossterm)
    Matrix3x4 Tc4to3;
    Tc4to3 << -1, -1, 1, 1,    -1, 1, -1, 1,   1, 1, 1, 1;

    //Matrix to perform cross product
    matrix wcross;

    //Matrix used to calculate quaternion error
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

        fixRates(mCurrentRates);
        filterRates(mFiltRates);
        fixAngles(mEuler);
        readMotors(mSpeed, mAmps);
        filterMotors(mFiltSpeed, mFiltAmps);
        mCurrentQuat = createQuaternion(mEuler);

        gettimeofday(&now, NULL);
        mClock = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec)/1000000.0f;

        //Check if it is time to change reference values
        //The first reference on the input file is already stored in mNextReference
        if (std::abs(mClock - mNextReference.time) <= 0.011 ){
            mReference = mNextReference;
            gotReference = true;

            if(mReference.num < mTotalRefs) //Read only if there are any left
                readNextReference();
        //step one of trajectory generation: get tstart,q0 and calculate qe, ANGLE,AXIS,TIME
        trajectorySetup(q0, angle, axis, time);
       }

        if(gotReference){
            gotReference = false;
        }

        // Call the trajectory generation function
        trajectoryGenerator(q0,angle,axis,time);

        quaternion q;
//        q = mReference.q;
        q = mQuatStar;


        //Calculate quaternion error:
        //Declare matrix for quaternion error calculation
        Qt << q(3),q(2),-q(1),q(0),
              -q(2),q(3),q(0),q(1),
              q(1),-q(0),q(3),q(2),
              -q(0),-q(1),-q(2),q(3);

        quaternion qs(-mCurrentQuat(0),-mCurrentQuat(1),-mCurrentQuat(2),mCurrentQuat(3));

        mQuatError = Qt*qs; //Save the error (as a quaternion)

        mQuatErrorI = integrateQ(mQuatError,mLastQuatError,mLastQuatErrorI, (mImuTime-mLastImuTime));

/// CONTROL LAW: calculate required torque on each of 3 axes

        //Calculate torque crossterm (combines table + wheels)

        wcross << 0, -mFiltRates(2), mFiltRates(1), mFiltRates(2), 0, -mFiltRates(0), -mFiltRates(1), mFiltRates(0), 0;
        vector hw = mSystem.Iw*Tc4to3*mFiltSpeed;
        vector crossterm = wcross*(mSystem.Inertia*mFiltRates+hw);
//        mTc3 =2*Kp*mQuatError*mQuatError(3);  // P controller, first generation of testing
        mTc3=2*Kp*mQuatError*mQuatError(3);//+Ki*mQuatErrorI+Kv*(mOmegaStar-mFiltRates)+mFFGains.KAff(2)*mAlphaStar+crossterm;

        //Calculate required torque on each of 4 wheels
        Vector4f Tc3Comp(mTc3(0), mTc3(1), mTc3(2), 0.3);
        mTorque = Tc3to4*Tc3Comp;

        //Calculate required speeds (rad/s)
        mSpeedCmd = integrateQ(mTorque,mLastTorque,mLastSpeedCmd,(mImuTime-mLastImuTime),(1/mSystem.Iw)); //units rad/s

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
