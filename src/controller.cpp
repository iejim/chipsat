
#include "controller.h"
#include <unistd.h>

using std::cout;
using std::endl;
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

    //if (time == mRefTime)
    //   qerror = mRefence - mCurrentState
    //
    cout << "Start IMU..."  << endl;
    mGX3.start();

    mKeepRunning = true;
    int step = 10;
    int count  = 200;

    vector euler, rates;
    float motorData0[2], motorData1[2], motorData2[2], motorData3[2];

    //Wait for the first IMU packet to arrive (as a way of synchronization)
    while(!readIMU(euler, rates)){
        cout << "." ;
        usleep(5000);
    }
    cout << endl;

    std::cout << "Running..." << std::endl;
    bool gotIMU = false;
    while (mKeepRunning){

        //Read IMU
        gotIMU = readIMU(euler, rates);
        readMotors(motorData0, motorData1, motorData2, motorData3);

        mCurrentState = createQuaternion(euler);
        count--;
        if(!count){
            step++;
            sendDutyCycles(step, step,  step, step); //Send the command
            cout << "Command: " << step << endl;
            count = 100;


            cout << "Angles: " << euler << endl << "Rates: " << rates << endl;
            cout << "Quaternion" << mCurrentState.x() << mCurrentState.y() << mCurrentState.z() << mCurrentState.w() << endl;
            cout << "Motor 0: " << motorData0[0] << " rad/s, " << motorData0[1] << " A" << endl;
            cout << "Motor 1: " << motorData1[0] << " rad/s, " << motorData1[1] << " A" << endl;
            cout << "Motor 2: " << motorData2[0] << " rad/s, " << motorData2[1] << " A" << endl;
            cout << "Motor 3: " << motorData3[0] << " rad/s, " << motorData3[1] << " A" << endl;
        }

        if(step == 60)
            step = 10; //restart



        waitPeriod();
    }


    std::cout << "CONTROLLER: Terminating " << std::endl;
    sendDutyCycles(0,0,0,0);
    joinIMU();
}

bool Controller::joinIMU()
{
    mGX3.stop();
    if (mGX3.join())
    {
        cout << "IMU thread joined" << endl;
        cout << "IMU terminaning ... "<< endl;
        return 0;
    } else
    {
        cout << "IMU thread joining failed" << endl;
        cout << "IMU terminaning ... "<< endl;
        return 1;
    }
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


bool Controller::readIMU(vector &euler, vector &rates)
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
    return true;

}

void Controller::readMotors(float* m0, float* m1, float* m2, float* m3)
{
    float currents[4], speeds[4]; //Hold the current and speed readings

    mMotors.getAnalogs(speeds, currents); //TODO Check the orded

    //Rearrange
    //TODO perform the necessary conversions
    m0[0] = 261.7994*speeds[0];
    m0[1] = V_TO_CURR*currents[0];
    m1[0] = 261.7994*speeds[1];
    m1[1] = V_TO_CURR*currents[1];
    m2[0] = V_TO_RADS*speeds[2];
    m2[1] = V_TO_CURR*currents[2];
    m3[0] = V_TO_RADS*speeds[3];
    m3[1] = V_TO_CURR*currents[3];


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

quaternion integrate(quaternion state)
{
    quaternion q(0.,0.,0.,0.);
    return q;
}

Controller::~Controller()
{
    //dtor
}
