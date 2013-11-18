
#include "controller.h"
#include "messages.h"

using namespace USU;

Controller::Controller(int priority, unsigned int period_us, const char* imuserial, const char* i2cDevice):
     PeriodicRtThread(priority, period_us), mGX3(priority, imuserial, period_us/1000), mMotors(i2cDevice), mKeepRunning(false)
{
    mRefTime = 0;
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
    sendDutyCycles(0,0,0,0); //Initialized motors to zero

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

    //readInput()

    //Call state = mGX3.front()

    //euler  = state.getVectors[0]
    //rate = state.getVectors[1]

    //mCurrentState = createQuaternion(euler);

    //if (time == mRefTime){
    //   qerror = mRefence - mCurrentState
    //}
}

void Controller::readInput()
{
    //Read the input file from here
    //Probably one line at a time
    //and store it somewhere
    float q0, q1, q2, q3;
    mInputFile >> mRefTime; //Read timestamp
    mInputFile >> q0 >> q1 >> q2 >> q3;
    Eigen::Vector4f quat(q0,q1,q2,q3);
    mReference = quat;
}

void Controller::setInputFile(const char* inputFile)
{
    mInputFile.open(inputFile);
    //Do other stuff here (like checks)
}

void Controller::sendDutyCycles(int d0, int d1, int d2, int d3)
{
    mMotors.setMotor(0, d0);
    mMotors.setMotor(1, d1);
    mMotors.setMotor(2, d2);
    mMotors.setMotor(3, d3);

}

Controller::~Controller()
{
    //dtor
}
