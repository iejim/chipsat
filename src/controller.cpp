
#include "controller.h"
#include <unistd.h>
#include <sys/time.h>

using std::cout;
using std::cerr;
using std::endl;
using namespace USU;

/////// Controller class

Controller::Controller(int priority, unsigned int period_us, const char* imuserial, const char* i2cDevice):
     PeriodicRtThread(priority, period_us), mGX3(priority-1, imuserial, period_us/1000), mMotors(i2cDevice), mKeepRunning(false)
{
    mDutyC << 0, 0, 0, 0;
    mUseInput = false;
    mLogging = false;
    mReference = {0, 0.0f, quaternion(0.0f,0.0f,0.0f,0.0f)};
    mNextReference = {0, 0.0f, quaternion(0.0f,0.0f,0.0f,0.0f)};
    mPIV = {0.0f, 0.0f, 0.0f};
    mFFGains = {0.0, 0.0};
    mFirstImuTime = 0.0;
    mStopTime = 0.0;
}

/*!
    In this function, the IMU can be setup with the necessary message(s) and modes and
    the motors are usually set to start at 0 RPM. This is also were the input file is read.
*/
void Controller::initialize()
{
    //Here the settings for the controller are initialized
    //Packets, Gains, Motor, Inputs, etc will be initialized here

    //IMU
    mGX3.addCommand(EULER_ANGLES_ANG_RATES); //Choose what to get
    mGX3.runContinuously(); // Set to send the data continuously
    mGX3.initialize();

    //Motors
    mDutyC = Vector4i(0, 0, 0, 0);
    sendDutyCycles(mDutyC); //Initialize motors to zero

    readInputFile();

}

/*!

    This is the function called when the system creates the thread.
    The Control Law runs in the loop created here.

    This function starts the IMU before the loop is started.
    On exit, the motors are stopped and the data is saved to the output file.
*/
void Controller::run()
{
    //This is where the controller magic begins

    //Before it does anything, it has to wait for one IMU package to arrive
    //so the threads synchronize (or something like that)

    cerr << "Start IMU..."  << endl;
    mGX3.start();


    struct timeval start, now;

    mKeepRunning = true;

    cerr << "Running... " << endl;



///------------------------- CONTROLLER CODE ----------------------------------

/// Temporary variables for the controller are created before while loop.

    //Trajectory Generation Initalization
    mQuatStar = quaternion(0,0,0,0);
    mOmegaStar = vector(0,0,0);
    mAlphaStar = vector(0,0,0);

    float bIw = 1/(mSystem.b+mSystem.Iw); //come back and give number

    Matrix3x4 Kp;;
    Kp <<   mPIV.KP*2.2*mSystem.Inertia(0,0)*mSystem.wn*mSystem.wn,0,0,0,
            0,mPIV.KP*2.2*mSystem.Inertia(1,1)*mSystem.wn*mSystem.wn,0,0,
            0,0,mPIV.KP*2.2*mSystem.Inertia(2,2)*mSystem.wn*mSystem.wn,0;

    //Matrix to convert Torque values from a 3-axis element vector to the 4-wheel model
    Matrix4f Tc3to4;
    Tc3to4 << 0.5,0,0.25,0.25,  0,0.5,0.25,-0.25,   -0.5,0,0.25,0.25,     0,-0.5,0.25,-0.25;

    Matrix4f Qt;

    bool inSync = false;
    bool gotIMU = false;
    bool gotReference = false;

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

        //Calculate quaternion error
        if(gotReference){
            gotReference = false;
        }

//TODO: Call the trajectory generation function
        //

        Qt << mReference.q(3),mReference.q(2),-mReference.q(1),mReference.q(0),
              -mReference.q(2),mReference.q(3),mReference.q(0),mReference.q(1),
              mReference.q(1),-mReference.q(0),mReference.q(3),mReference.q(2),
              -mReference.q(0),-mReference.q(1),-mReference.q(2),-mReference.q(3);

        quaternion qs(-mCurrentQuat(0),-mCurrentQuat(1),-mCurrentQuat(2),-mCurrentQuat(3));

        mQuatError = Qt*qs; //Save the error (as a quaternion)

/// CONTROL LAW: calculate required torque on each of 3 axes
        mTc3 =2*Kp*mQuatError*mQuatError(3);
        //Tc=2*Kp*qe*qe(4)+Ki*qei+Kv*(w_star-w)+Ka*alpha_star+Td_hat+crossterm;

        //Calculate required torque on each of 4 wheels
        Vector4f Tc3Comp(mTc3(0), mTc3(1), mTc3(2), 0.);
        mTorque = Tc3to4*Tc3Comp;

        //Calculate required speeds (rad/s)
        mSpeedCmd = integrateQ(mTorque,mLastTorque,mLastSpeedCmd,(mImuTime-mLastImuTime),(1/mSystem.Iw));

        //Calculate required duty cycles
//TODO Make this number a macro (#define) or a variable
        mSpeedCmd = mSpeedCmd*(80/618.7262f); //This number is part of the Motor Controller configuration
        mDutyC = Vector4i((int)mSpeedCmd(0), (int)mSpeedCmd(1), (int)mSpeedCmd(2), (int)mSpeedCmd(3));  //if mSpeed is in rad/s
        mDutyC += Vector4i(10,10,10,10);
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


    //TODO Try to check if mGX3 is still running, then save the data
/// After the loop stops, the motors are stopped, and the log data is saved.
    cerr << "CONTROLLER: Terminating " << endl;
    mDutyC = Vector4i(0, 0, 0, 0);
    sendDutyCycles(mDutyC); //Stop the motors
    joinIMU(); //Stop the IMU
    saveLogData();

}

/*!
    This function should be called after specifying a file with @ref setInputFile.
    The file is expected to include all values; the data is not validated.

    Only the first reference is mandatory, more can be added one per line
    and UPDATING the value for the total references.

    Refer to the source code for the order.
*/
void Controller::readInputFile()
{

    if (!mInputFile.is_open()){
        throw std::runtime_error("No input file has been set"); //Is this going to work if mInputFile hasn't been correctly set? It should.
        return;
    }

/// This is would only be used when a PID controller is used
//    Read PID gains (KP, KI, KD)
//    mInputFile >> mPID.KP >> mPID.KI >> mPID.KD;

    //Read PIV gains (KP, KI, KVff, KAff)
    mInputFile >> mPIV.KP >> mPIV.KI >> mPIV.KV;
    //Read Trajectory Gen FF gains
    mInputFile >> mFFGains.KVff >>  mFFGains.KAff;

    //Read the system constants
    mInputFile >> mSystem.b >> mSystem.Iw >> mSystem.wn >> mSystem.hDotMax >> mSystem.hMax;

    float i0, i1, i2, i3, i4, i5, i6, i7, i8;
    //Read the Moment of Inertia matrix
    mInputFile >> i0 >> i1 >> i2 >> i3 >> i4 >> i5 >> i6 >> i7 >> i8;
    mSystem.Inertia << i0, i1, i2, i3, i4, i5, i6, i7, i8;

    //Read the ADC scaling parameters
    mInputFile >> mMotorScale.vToRads >> mMotorScale.vToAmps;

    //Read the trajectory inputs (or references) along with timestamps
    mInputFile >> mTotalRefs; //Read how many lines have been set
    mInputFile >> mStopTime;  //Read the running time limit

    //Read the first reference line
    readNextReference();

}

/*!
    This function saves the next Reference Command on the input file into @ref mNextReference.
    It should be called after the controller populates (or overwrites) @ref mReference, unless
    the current reference stored in @ref mNextReference is to be skipped.

    The function will only reads, it does not check which reference (index) is next. If the
    file cannot be read any further, it should return nothing and @ref mReference might stay the
    same. It is up to the Controller to know which reference is being used (using mTotalRefs).

    The data is read as-is and is not validated; it is stored in a @ref RefData struct.
    Refer to the source code for the order.
*/
void Controller::readNextReference()
{
    //Read one line from the input file to create the next reference
    if (!mInputFile.is_open()){
        throw std::runtime_error("Cannot access input file to read reference"); //Is this going to work if mInputFile hasn't been correctly set? It should.
        return;
    }
    float q0, q1, q2, q3;

    mInputFile >> mNextReference.num; //Read reference index number
    mInputFile >> mNextReference.time; //Read timestamp
    mInputFile >> q0 >> q1 >> q2 >> q3; //Read quaternion
    mNextReference.q << q0,q1,q2,q3;

}

/*!
    The specified filename will be used as the input with the settings the system
    This function should be called before @ref initialize.
    \param [in] inputFile C-String with the full name of the input file, e.g: "input.txt"

*/
void Controller::setInputFile(const char* inputFile)
{
    mInputFile.open(inputFile, std::fstream::in);
    //Do other stuff here (like checks)
    if(!mInputFile.is_open())
        throw std::runtime_error("Could not open input file for reading");
    mUseInput = true;

}

/*!
    For the data to be saved, this function should be called before @ref run.
    \param [in] logFile C-String with the full name of the log file, e.g: "data.csv"
*/
void Controller::setLogFile(const char* logFile)
{
    mLogFile.open(logFile);
    if(!mLogFile.is_open())
        throw std::runtime_error("Could not open log file for writing");
    mLogging= true;
}

/*!
    The data generated and sampled by the controller on each loop iteration is
    saved into memory so it can be _later_ written to the log file.
    This function can be called from within the Control Law loop.

    The data is stored in the RAM memory because writing to the SD Card
    can take too much time and interrupt the program's execution.
    During this write time, the program will _STOP_ the execution of the controller,
    which is _dangerous_.

    The amount of data that can be stored is only limited by the amount of
    RAM memory installed. However, the data stored is mostly floating-point
    numbers (many digits) and can take up space very fast. Running the
    program for 90 seconds can easily generate more than 1MB of data.
    This should not be a problem for many systems, but should be taken into
    account when running for a long time.

*/
void Controller::logData()
{

    //Save timestamp

    //1 <- CSV index (MATLAB style)
    mLogBuf << toCSV(mClock);

    //Save calculated values

    //2-5
    mLogBuf << toCSV(mCurrentQuat(0)) << toCSV(mCurrentQuat(1)) << toCSV(mCurrentQuat(2)) << toCSV(mCurrentQuat(3));
    //6-9
    mLogBuf << toCSV(mReference.q(0)) << toCSV(mReference.q(1)) << toCSV(mReference.q(2)) << toCSV(mReference.q(3));
    //10-13
    mLogBuf << toCSV(mQuatStar(0)) << toCSV(mQuatStar(1)) << toCSV(mQuatStar(2)) << toCSV(mQuatStar(3));
    //14-16
    mLogBuf << toCSV(mOmegaStar(0)) << toCSV(mOmegaStar(1)) << toCSV(mOmegaStar(2));
    //17-19
    mLogBuf << toCSV(mAlphaStar(0)) << toCSV(mAlphaStar(1)) << toCSV(mAlphaStar(2));
    //20-23
    mLogBuf << toCSV(mQuatError(0)) << toCSV(mQuatError(1)) << toCSV(mQuatError(2)) << toCSV(mQuatError(3));
    //24-26
    mLogBuf << toCSV(mTc3(0)) << toCSV(mTc3(1)) << toCSV(mTc3(2));
    //27-30
    mLogBuf << toCSV(mTorque(0)) << toCSV(mTorque(1)) << toCSV(mTorque(2)) << toCSV(mTorque(3));
    //31-34
    mLogBuf << toCSV(mSpeedCmd(0)) << toCSV(mSpeedCmd(1)) << toCSV(mSpeedCmd(2)) << toCSV(mSpeedCmd(3));
    //35-38
    mLogBuf << toCSV(mDutyC(0)) << toCSV(mDutyC(1)) << toCSV(mDutyC(2)) << toCSV(mDutyC(3));

    //Save readings
    //39-42
    mLogBuf << toCSV(mImuTime) << toCSV(mEuler(0)) << toCSV(mEuler(1)) << toCSV(mEuler(2));
    //43-45
    mLogBuf << toCSV(mCurrentRates(0)) << toCSV(mCurrentRates(1)) << toCSV(mCurrentRates(2));
    //46-49
    mLogBuf << toCSV(mSpeed(0)) << toCSV(mSpeed(1)) << toCSV(mSpeed(2)) << toCSV(mSpeed(3));
    //50-54
    mLogBuf << toCSV(mAmps(0)) << toCSV(mAmps(1)) << toCSV(mAmps(2)) << toCSV(mAmps(3));

    mLogBuf << endl;


}

/*!
    This function will save the data stored in the memory buffer @ref mLogBuf into
    the file specified with @ref setLogFile. The data is saved as it is formatted
    in @ref logData.

    This function _must_ be called after the Control loop has been stopped since
    it calls the system function sync() to ensure the program will wait for all
    the data to be written to the SD Card.
*/
void Controller::saveLogData()
{
    if(!mLogFile.is_open())
        throw std::runtime_error("Log file is not open for writing");
    mLogFile << mLogBuf.rdbuf();
    mLogFile.flush();
    cerr<< "Writing to file. Please wait...." << endl;
    sync();
}
void Controller::sendDutyCycles(Vector4i dc)
{
    mMotors.setMotor(0, dc(0));
    mMotors.setMotor(1, dc(1));
    mMotors.setMotor(2, dc(2));
    mMotors.setMotor(3, dc(3));

}

bool Controller::readIMU(vector &euler, vector &rates, float &timer)
{
    //Wait to see if there is an IMU packet available
    if (mGX3.size()==0)
        return false;//Should work to synchronize the first time everything in run.

    packet_ptr pack;
    pack  = mGX3.front();
    mGX3.pop();

    pack->getVectors(mVectorQueue);

    euler = mVectorQueue.front();
    mVectorQueue.pop();

    rates = mVectorQueue.front();
    mVectorQueue.pop();

    timer = pack->getTime()/TIMER_TO_S - mFirstImuTime;
    cerr << " Imu: " << timer << "left " << mGX3.size() << endl;
    return true;

}

void Controller::fixAngles(vector& euler)
{
//TODO Make this generic (by accepting a matrix?) for flexibility
//i.e. euler(0) = bias(0) + sign(0)*euler(0);
//This could actually be done with a matrix-vector multiplication

    if(euler(0)>0) //Roll
            euler(0) = M_PI-euler(0);
    else
            euler(0) = -M_PI-euler(0); //Roll

    euler(2) = -euler(2); //Yaw



}

void Controller::fixRates(vector& rates)
{
    rates(0) = -rates(0);
    rates(1) = -rates(1);

}
void Controller::readMotors(Vector4f &speedVec, Vector4f &currentVec)
{
    float currents[4], speeds[4]; //Hold the current and speed readings

    mMotors.getAnalogs(speeds, currents);

    //Rearrange
    speedVec << mMotorScale.vToRads*speeds[0],
                mMotorScale.vToRads*speeds[1],
                mMotorScale.vToRads*speeds[2],
                mMotorScale.vToRads*speeds[3];

    currentVec << mMotorScale.vToAmps*currents[0],
                mMotorScale.vToAmps*currents[1],
                mMotorScale.vToAmps*currents[2],
                mMotorScale.vToAmps*currents[3];
}

bool Controller::joinIMU()
{
    mGX3.stop();
    if (mGX3.join(5000)) //Give it a timeout in case it died before joining
    {
        cerr << "IMU thread joined" << endl;
        cerr << "IMU terminated ... "<< endl;
        return 0;
    } else
    {
        cerr << "IMU thread joining failed" << endl;
        cerr << "IMU terminated ... "<< endl;
        return 1;
    }
}


void Controller::updateStates()
{
    mLastQuat = mCurrentQuat;
    mLastQuatError = mQuatError;
    mLastSpeedCmd = mSpeedCmd;
    mLastEuler = mEuler;
    mLastRates = mCurrentRates;
    mLastSpeed = mSpeed;
    mLastImuTime = mImuTime;

    mLastTorque = mTorque;
    mLastAmps = mAmps;
    mLastDutyC = mDutyC;

}

quaternion Controller::createQuaternion(vector euler)
{
    Eigen::Matrix3f M;
    float theta, phi, psi;
    phi = euler(0); //Roll
    theta = euler(1); //Pitch
    psi = euler(2); //Yaw

    //Create roation matrix
    M << cos(psi)*cos(theta),                           sin(psi)*cos(theta),                            -sin(theta),
        cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi), sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi), cos(theta)*sin(phi),
        cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi), sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi), cos(theta)*cos(phi);

    //Perform quaternion tests
    Eigen::Array4f test; //Array, not vector
    test << M(0,0)+M(1,1)+M(2,2),
            M(0,0)-M(1,1)-M(2,2),
           -M(0,0)+M(1,1)-M(2,2),
           -M(0,0)-M(1,1)+M(2,2);
    int i;
    test.maxCoeff(&i);

    float s;
    quaternion q;

    switch(i){
        case 0:
                s = 2*sqrt(1.+M(0,0)+M(1,1)+M(2,2));
                q <<  s/4.,
                    (M(1,2)-M(2,1))/s,
                    (M(2,0)-M(0,2))/s,
                    (M(0,1)-M(1,0))/s;
                break;
        case 1:
                s = 2*sqrt(1.+M(0,0)-M(1,1)-M(2,2));
                q << (M(2,1)-M(1,2))/s,
                        -s/4.,
                     -(M(1,0)+M(0,1))/s,
                     -(M(0,2)+M(2,0))/s;
                break;
        case 2:
                s = 2*sqrt(1.-M(0,0)+M(1,1)-M(2,2));
                q << (M(0,2)-M(2,0))/s,
                     -(M(1,0)+M(0,1))/s,
                        -s/4.,
                     -(M(2,1)+M(1,2))/s;
                break;
        case 3:
                s = 2*sqrt(1.-M(0,0)-M(1,1)+M(2,2));
                q << (M(1,0)-M(0,1))/s,
                     -(M(0,2)+M(2,0))/s,
                     -(M(2,1)+M(1,2))/s,
                     -s/4.;
                break;

    }

    //The math above places the scalar first in q; we want it at the end
    quaternion qa(q(1),q(2),q(3),q(0));

    //Check that qa(4) is positive, which is the convention we will use here (remember qa=-qa)
    if (qa(4)<0)
        qa=-qa;
    return qa;

}

quaternion Controller::integrateQ(quaternion in, quaternion old_in, quaternion old_out, float delta_time, float gain)
{
    //Implements discrete trapezoidal integration
    //as explained http://www.mathworks.com/help/simulink/slref/discretetimeintegrator.html
    //y[k] = y[k-1] + KI*dt/2*(in[k]+in[k-1]);
    quaternion q;
    q = old_out + gain*delta_time/2*(in+old_in);
    return q;
}

quaternion Controller::multiplyQ(quaternion q1, quaternion q2)
{

    vector b1,b2, b;

    b1 << q1(0), q1(1), q1(2);
    b2 << q2(0), q2(1), q2(2);
    b = q1(3)*b2 + q2(3)*b1 + b1.cross(b2);
    quaternion q(b(0), b(1), b(2),
                 q1(3)*q2(3) - b1.dot(b2));

    return q;

}



Controller::~Controller()
{
    if(mInputFile.is_open())
        mInputFile.close();

    if(mLogFile.is_open())
        mLogFile.close();
}


