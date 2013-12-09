
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
    sendDutyCycles(mDutyC); //Initialized motors to zero

    readInputFile();
    //Gains
    //mPGain = 1;
    //mIGain = 1;
    // etc, etc.

    //Inputs (or something)
    //readInput()


}

void Controller::run()
{
    //This is where the controller magic begins

    //Before it does anything, it has to wait for one IMU package to arrive
    //so the threads synchronize (or something like that)

    cerr << "Start IMU..."  << endl;
    mGX3.start();


    struct timeval start, now;


//    int step = 10;
//    int count  = 200;

    mKeepRunning = true;

    cerr << "Running... " << endl;



    /////--------CONTROLLER CODE------------------//create temporary variables before while
//    float b = 1e-6; //SI units, average damping coefficent of wheels
//    float Iw = 0.462; //lbm-in^2, mass moment of inertia of momentum wheel
    float bIw = 1/(mSystem.b+mSystem.Iw); //come back and give number
//    float wn=0.2; //choose natural damping frequency

//    Matrix3f Inertia;
//    Inertia << 102.322878, -0.035566, 0.315676, -0.035566, 103.65751, 0.006317, 0.315676, 0.006317, 159.537290; //lbm-in^2 inertia of tables
    Matrix3x4 Kp;
    Kp <<   mPIV.KP*2.2*mSystem.Inertia(0,0)*mSystem.wn*mSystem.wn,0,0,0,
            0,mPIV.KP*2.2*mSystem.Inertia(1,1)*mSystem.wn*mSystem.wn,0,0,
            0,0,mPIV.KP*2.2*mSystem.Inertia(2,2)*mSystem.wn*mSystem.wn,0;

    Matrix4f Tc3to4;
    Tc3to4 << 0.5,0,0.25,0.25,  0,0.5,0.25,-0.25,   -0.5,0,0.25,0.25,     0,-0.5,0.25,-0.25;

    //For now
//    mReference.q = quaternion(-0.034439, -0.54683, -0.836534, -0.00150478);
//    (0.772131, 0.0352254, -0.0553593, 0.0.632067);


    ///This is whe  re the periodic features of the controller happen
//    gettimeofday(&start, NULL);
//    gettimeofday(&now, NULL);
//    mClock = now.tv_sec - start.tv_sec + (now.tv_usec - start.tv_usec)/1000.0;
    bool inSync = false;
    bool gotIMU = false;
    while (mKeepRunning){

        //Check if it is time to change reference values
        //The first reference on the input file is already stored in mNextReference
        if (mStopTime <= mClock)
        {
            mKeepRunning = false;
            break;
        }

        if (mNextReference.time == mClock){ //The clock *should* be updated every 20ms
            mReference = mNextReference;

            if(mReference.num < mTotalRefs) //Read only if there are any left
                readNextReference();
        }
        //Read IMU
//        readIMU(mEuler, mCurrentRates, mImuTime);

        if(!inSync){
        //Wait for the first IMU packet to arrive (as a way of synchronization)
            mFirstImuTime = 0.0;
            gotIMU = readIMU(mEuler, mCurrentRates, mFirstImuTime);
            while(!gotIMU){
                //cerr << "." ;
                usleep(100);
                gotIMU = readIMU(mEuler, mCurrentRates, mFirstImuTime);
                cerr << ",";
            }
            gettimeofday(&start, NULL);
            inSync = true;
            //cerr << endl;
        }else {
            gotIMU = readIMU(mEuler, mCurrentRates, mImuTime);
            while(!gotIMU)
            {
                usleep(100); //Give it some time for data to arrive to the queue
                gotIMU = readIMU(mEuler, mCurrentRates, mImuTime);
                cerr << ".";
            }
            cerr <<endl;
        }

        gettimeofday(&now, NULL);
        mClock = (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec)/1000000.0;
        cerr << "Clock: " << mClock << endl;

//        if(gotIMU){

            fixAngles(mEuler);
            fixRates(mCurrentRates);
            readMotors(mSpeed, mAmps);
            mCurrentQuat = createQuaternion(mEuler);

            //calculate quaternion error
            //The quaternion used here will eventually change to one coming from the trajectory generator
            Matrix4f Qt;
            Qt << mReference.q(3),mReference.q(2),-mReference.q(1),mReference.q(0),
                  -mReference.q(2),mReference.q(3),mReference.q(0),mReference.q(1),
                  mReference.q(1),-mReference.q(0),mReference.q(3),mReference.q(2),
                  -mReference.q(0),-mReference.q(1),-mReference.q(2),mReference.q(3);
            quaternion qs(-mCurrentQuat(0),-mCurrentQuat(1),-mCurrentQuat(2),-mCurrentQuat(3));
    //        Vector4f qe = Qt*qs;
            mQuatError = Qt*qs; //Save the error (as a quaternion)

            //calculate required torque on each of 3 axes
            Vector3f Tc3 =2*Kp*mQuatError*mQuatError(3);

            //calculate required torque on each of 4 wheels

            Vector4f Tc3Comp(Tc3(0), Tc3(1), Tc3(2), 0.);
            Vector4f Tc4 = Tc3to4*Tc3Comp;

            //calculate required speeds (rad/s)
            Vector4f speedscmd =(Tc4+mSystem.Iw*mLastSpeed)*bIw; // check bIw

            //calculate required duty cycles
            speedscmd = speedscmd*(80/618.7262);
            mDutyC = Vector4i((int)speedscmd(0), (int)speedscmd(1), (int)speedscmd(2), (int)speedscmd(3));  //if speedscmd is in rad/s
            mDutyC += Vector4i(10,10,10,10);
            sendDutyCycles(mDutyC);

            readMotors(speedscmd, mAmps);
//                cerr << "Angles: " << mEuler << endl << "Rates: " << mCurrentRates << endl;
//                cerr << "Quaternion" << mCurrentQuat(0) << "," <<  mCurrentQuat(1)
//                     << "," << mCurrentQuat(2) << "," << mCurrentQuat(3) << endl;
//                cerr << "Motor 0: " << "DC: " << mDutyC(0) << " "<< speedscmd(0) << " rad/s, " << mAmps(0) << " A" << endl;
//                cerr << "Motor 1: " << "DC: " << mDutyC(1) << " "<< speedscmd(1) << " rad/s, " << mAmps(1) << " A" << endl;
//                cerr << "Motor 2: " << "DC: " << mDutyC(2) << " "<< speedscmd(2) << " rad/s, " << mAmps(2) << " A" << endl;
//                cerr << "Motor 3: " << "DC: " << mDutyC(3) << " "<<
//                speedscmd(3) << " rad/s, " << mAmps(3) << " A" << endl;

            //Save the data
            if(mLogging)
                logData();
//        }
            updateStates();

            waitPeriod();
            //TODO I could use the system time for this (in case something doesn't work correctly)



    }


    cerr << "CONTROLLER: Terminating " << endl;
    mDutyC = Vector4i(0, 0, 0, 0);
    sendDutyCycles(mDutyC); //Stop the motors
    joinIMU(); //Stop the IMU
    saveLogData();

}


void Controller::readInputFile()
{
    //Read the input file from here
    //Probably one line at a time
    //and store it somewhere

    if (!mInputFile.is_open()){
        throw std::runtime_error("No input file has been set"); //Is this going to work if mInputFile hasn't been correctly set? It should.
        return;
    }


//    //Read PID gains (KP, KI, KD)
//    mInputFile >> mPID.KP >> mPID.KI >> mPID.KD;

    //Read PIV gains (KP, KI, KVff, KAff)
    mInputFile >> mPIV.KP >> mPIV.KI >> mPIV.KV;
    //Read Trajectory Gen FF gains
    mInputFile >> mFFGains.KVff >>  mFFGains.KAff;

    //Read the system constants
    mInputFile >> mSystem.b >> mSystem.Iw >> mSystem.wn >> mSystem.hDotMax >> mSystem.hMax;

    float i0, i1, i2, i3, i4, i5, i6, i7, i8;
    mInputFile >> i0 >> i1 >> i2 >> i3 >> i4 >> i5 >> i6 >> i7 >> i8;
    mSystem.Inertia << i0, i1, i2, i3, i4, i5, i6, i7, i8;

    //Read the trajectory inputs (or references) along with timestamps
    mInputFile >> mTotalRefs; //Read how many lines have been set
    mInputFile >> mStopTime;  //Read the running time limit
    //Read the first line
    readNextReference();

}
//50.0 0.0 0.0
//0.0 0.0
//0.000001 0.462 0.2 .25 .25
//102.322878 -0.035566 0.315676 -0.035566 103.65751 0.006317 0.315676 0.006317 159.537290
//1
//1 0.0 -0.034439 -0.54683 -0.836534 -0.00150478

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

void Controller::setInputFile(const char* inputFile)
{
    mInputFile.open(inputFile, std::fstream::in);
    //Do other stuff here (like checks)
    if(!mInputFile.is_open())
        throw std::runtime_error("Could not open input file for reading");
    mUseInput = true;

}

void Controller::setLogFile(const char* logFile)
{
    mLogFile.open(logFile);
    if(!mLogFile.is_open())
        throw std::runtime_error("Could not open log file for writing");
    mLogging= true;
}

void Controller::logData()
{
    //if(!mLogFile.is_open())
    //    throw std::runtime_error("Log file is not open for writing");
    //Save timestamp
    mLogBuf << toCSV(mClock);
    //Save calculated values
    //mLogBuf << toCSV(mCurrentQuat(0)) << toCSV(mCurrentQuat(1)) << toCSV(mCurrentQuat(2)) << toCSV(mCurrentQuat(3));
    //mLogBuf << toCSV(mQuatError(0)) << toCSV(mQuatError(1)) << toCSV(mQuatError(2)) << toCSV(mQuatError(3));
    //mLogBuf << toCSV(mTorque(0)) << toCSV(mTorque(1)) << toCSV(mTorque(2)) << toCSV(mTorque(3));
    mLogBuf << toCSV(mDutyC(0)) << toCSV(mDutyC(1)) << toCSV(mDutyC(2)) << toCSV(mDutyC(3));
//    mLogBuf.flush();
    //Save readings
    mLogBuf << toCSV(mImuTime) << toCSV(mEuler(0)) << toCSV(mEuler(1)) << toCSV(mEuler(2));
    mLogBuf << toCSV(mCurrentRates(0)) << toCSV(mCurrentRates(1)) << toCSV(mCurrentRates(2));
    mLogBuf << toCSV(mSpeed(0)) << toCSV(mSpeed(1)) << toCSV(mSpeed(2)) << toCSV(mSpeed(3));
    mLogBuf << toCSV(mAmps(0)) << toCSV(mAmps(1)) << toCSV(mAmps(2)) << toCSV(mAmps(3));
//    mLogBuf.flush();
    mLogBuf << endl;
    //sync(); //Synchronize ?
}

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
    //TODO Is this
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
    //TODO Make this generic (by accepting bias and sign vectors?) for flexibility
    //i.e. euler(0) = bias(0) + sign(0)*euler(0);
    //This could actually be done with a matrix-vector multiplication
    euler(0) = ((euler(0)>0)?M_PI:-M_PI) - euler(0); //Roll
    euler(1) = -euler(1); //Pitch
    euler(2) = -euler(2); //Yaw

}

void Controller::fixRates(vector& rates)
{
    rates(0) = -rates(0);

}
void Controller::readMotors(Vector4f &speedVec, Vector4f &currentVec)
{
    float currents[4], speeds[4]; //Hold the current and speed readings

    mMotors.getAnalogs(speeds, currents); //TODO Check the orded

    //Rearrange
    //TODO perform the necessary conversions
    speedVec << V_TO_RADS*speeds[0],
                V_TO_RADS*speeds[1],
                V_TO_RADS*speeds[2],
                V_TO_RADS*speeds[3];

    currentVec << V_TO_CURR*currents[0],
                V_TO_CURR*currents[1],
                V_TO_CURR*currents[2],
                V_TO_CURR*currents[3];
}

bool Controller::joinIMU()
{
    mGX3.stop();
    if (mGX3.join())
    {
        cerr << "IMU thread joined" << endl;
        cerr << "IMU terminaning ... "<< endl;
        return 0;
    } else
    {
        cerr << "IMU thread joining failed" << endl;
        cerr << "IMU terminaning ... "<< endl;
        return 1;
    }
}


void Controller::updateStates()
{
    mLastQuat = mCurrentQuat;
    mLastQuatError = mQuatError;
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
    theta = euler(0);
    phi = euler(1);
    psi = euler(2);

    //Create roation matrix
    M << cos(psi)*cos(theta),                           sin(psi)*cos(theta),                              -sin(theta),
        cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi), sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi), cos(theta)*sin(phi),
        cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi), sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi), cos(theta)*cos(phi);

    //Perform quaternion tests
    Eigen::Array4f test;
    test << M(0,0)+M(1,1)+M(2,2),
            M(0,0)-M(1,1)-M(2,2),
           -M(0,0)+M(1,1)-M(2,2),
           -M(0,0)-M(1,1)+M(2,2);
    int i;
    test.maxCoeff(&i);

    float s;
    Eigen::Vector4f q;

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

    quaternion quat;
    quat = q;

    return quat;
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


