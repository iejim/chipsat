
#include "controller.h"



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
    mFFGains = {vector(0.0f,0.0f,0.0f), vector(0.0f,0.0f,0.0f)};
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


    //Ensure the control law loop will run
    mKeepRunning = true;

    cerr << "Running... " << endl;



///------------- RUN THE CONTROL LAW CODE ----------------------------------

    controlLaw();


//TODO Try to check if mGX3 is still running, then save the data
/// After the control law loop stops, the motors are stopped, and the log data is saved.
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
    float k0, k1, k2;
    mInputFile >> k0 >> k1 >> k2;
    mFFGains.KVff << k0, k1, k2;

    mInputFile >> k0 >> k1 >> k2;
    mFFGains.KAff << k0, k1, k2;


    //Read the system constants
    mInputFile >> mSystem.b >> mSystem.Iw >> mSystem.wn >> mSystem.hDotMax >> mSystem.hMax >> mSystem.Amax >> mSystem.Vmax;

    float i0, i1, i2, i3, i4, i5, i6, i7, i8;
    //Read the Moment of Inertia matrix
    mInputFile >> i0 >> i1 >> i2 >> i3 >> i4 >> i5 >> i6 >> i7 >> i8;
    mSystem.Inertia << i0, i1, i2, i3, i4, i5, i6, i7, i8;
    mInputFile >> mSystem.motorSpeedMax;

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
    //50-53
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
/*!
    Sends the specified duty cycles commands to all motors.

    \param [in] dc A four integer vector with commands for motors 0-3
*/
void Controller::sendDutyCycles(Vector4i dc)
{
    mMotors.setMotor(0, dc(0));
    mMotors.setMotor(1, dc(1));
    mMotors.setMotor(2, dc(2));
    mMotors.setMotor(3, dc(3));

}
/*!
    Reads the data from an IMU packet stored in the Queue shared between @ref mGX3
    and the controller thread. It reads only the latest packet in the Queue and
    extracts the data into the passed arguments. The time from the IMU is normalized
    to the program's running time.

    As a debug tool, the IMU time and post-process Shared Queue size are printed to
    standard error (cerr). The size should be zero, otherwise packets where skipped,
    which means some delay in the controller happened.

    The data order is handled as Roll, Pitch, Yaw.

    \param [out] euler The vector used to store the Euler angles (rads)
    \param [out] rates The vector used to store the Angular Rates (rad/s)
    \param [out] timer The variable to store the timestamp of the packet
            in seconds since the first packet received.(s)

    \return false if the Shared Queue is empty.
*/
bool Controller::readIMU(vector &euler, vector &rates, float &timer)
{
    //Wait to see if there is an IMU packet available
    if (mGX3.size()==0)
        return false;//Should work to synchronize the first time everything is run.

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

/*!
    Transforms the IMU Euler Angle readings to match the platform's coordinate system.
    This assumes the IMU is installed in its default position in the center of the table.
    There is the possibility to expand this function by accepting a rotation matrix.

    \param [in|out] euler The euler angles extracted from an IMU packet to be rewritten
                            relative to the table's reference frame.
*/
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

/*!
    Transforms the IMU Angular Rates readings to match the platform's coordinate system.
    This assumes the IMU is installed in its default position in the center of the table.
    There is the possibility to expand this function by accepting a rotation matrix.

    \param [in|out] rates The angular rates extracted from an IMU packet to be rewritten
                            relative to the table's reference frame.
*/
void Controller::fixRates(vector& rates)
{
    rates(0) = -rates(0);
    rates(1) = -rates(1);

}

/*!
    Performs a low-pass filter of the angular rates so we can use it as a cleaner measure
    of the table's rotational speed. The cutoff frequency was selected to be 1Hz.

*/
void Controller::filterRates()
{
    mCurrentRates = firstOrderFilterV(mRawRates,mLastRawRates, mLastRates, mImuTime-mLastImuTime);
}

/*!
    Samples the angular speed and current readings from the motor controllers. These values
    are read in Volts and converted the appropiate units using the user-defined Motor
    parameters in the input file.

    \param [out] speedVec Four-element vector to hold the speed of each motor (rad/s)
    \param [out] currentVec Four-element vector to hold the current used by each motor (A)
    \sa AdcGains, mMotorScale
*/
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

/*!
    This function will stop execution of the controller thread and wait for the IMU communicator
    thread to terminate. A timeout of 5ms is given to the IMU thread to terminate in order to
    prevent the system from hanging if anything goes wrong during the IMU termination process.

    \return Wether the IMU thread was successfully joined before termination.
    \sa GX3Communicator
*/
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

/*!
    Updates the internal variables to store the current values for the next iteration.
*/
void Controller::updateStates()
{
    mLastQuat = mCurrentQuat;
    mLastQuatError = mQuatError;
    mLastSpeedCmd = mSpeedCmd;
    mLastEuler = mEuler;
    mLastRates = mCurrentRates;
    mLastSpeed = mSpeed;
    mLastImuTime = mImuTime;

    mLastQuatErrorI = mQuatErrorI;
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

    //Check that the scalar is positive, which is the convention we will use here (remember qa=-qa)
    //The math above places the scalar first in q; we want it at the end
    quaternion qa;
    if (q(0)<0)
        qa << -q(1),-q(2),-q(3),-q(0);
    else
        qa << q(1),q(2),q(3),q(0);

    return qa;

}

/*!
    Implements the discrete trapezoidal integration of a quaternion. The discrete integrator is based
    on the Bilinear Transformation of an integrator.
*/
quaternion Controller::integrateQ(quaternion in, quaternion old_in, quaternion old_out, float delta_time, float gain)
{
    //Implements discrete trapezoidal integration
    //as explained http://www.mathworks.com/help/simulink/slref/discretetimeintegrator.html
    //y[k] = y[k-1] + KI*dt/2*(in[k]+in[k-1]);
    quaternion q;
    q = old_out + gain*delta_time/2*(in+old_in);
//    cerr << "spd " << q(0) << " , " << q(1) << " , " << q(2) << " , " << q(3); //look at output
    return q;
}

/*!
    Implements the multiplication of two quaternions.
*/
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

/*!
    This filter implements a generic discrete first-order filter:
      az + a
     --------
       z - b
 */
float firstOrderFilterF(float in, float old_in, float old_out, float a, float b)
{
    return b*old_out + a*(in-old_in);
}
/*!
    This filter implements the Discrete Bilinear Transformation of a first-order, continuous LTI low-pass filter:
        w_c
     -------
     s  + w_c

    This function is designed to be used on different filter by specifying the 3dB cufoff frequency and sampling time.
    If the sampling time between calls changes, the function will not behave correcty for it won't follow Discrete Systems rules.

    The center frequency defaults to 2*PI (1Hz).
 */
quaternion firstOrderFilterQ(quaternion in, quaternion old_in, quaternion old_out, float sampling_time, float w_c)
{
    float tf = 2.0/sampling_time;
    float a = w_c/(tf +w_c);
    float b = (tf - w_c)/(2.0/tf+w_c);
    quaternion q;
    q << firstOrderFilterF(in(0), old_in(0), old_out(0), a,b),
         firstOrderFilterF(in(1), old_in(1), old_out(1), a,b),
         firstOrderFilterF(in(2), old_in(2), old_out(2), a,b),
         firstOrderFilterF(in(3), old_in(3), old_out(3), a,b);

    return q;
}

/*!
    This filter implements the Discrete Bilinear Transformation of a first-order, continuous LTI low-pass filter:
        w_c
     -------
     s  + w_c

    This function is designed to be used on different filter by specifying the 3dB cufoff frequency and sampling time.
    If the sampling time between calls changes, the function will not behave correcty for it won't follow Discrete Systems rules.

    The center frequency defaults to 2*PI (1Hz).
 */
vector firstOrderFilterV(vector in, vector old_in, vector old_out, float sampling_time, float w_c)
{
    float tf = 2.0/sampling_time;
    float a = w_c/(tf +w_c);
    float b = (tf - w_c)/(2.0/tf+w_c);
    vector v;
    v << firstOrderFilterF(in(0), old_in(0), old_out(0), a,b),
         firstOrderFilterF(in(1), old_in(1), old_out(1), a,b),
         firstOrderFilterF(in(2), old_in(2), old_out(2), a,b);

    return v;
}

Controller::~Controller()
{
    if(mInputFile.is_open())
        mInputFile.close();

    if(mLogFile.is_open())
        mLogFile.close();
}


