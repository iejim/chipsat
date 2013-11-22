
#include "controller.h"
#include <unistd.h>

using std::cout;
using std::endl;
using namespace Eigen; //so that I don't have to write Eigen a bunch of times
using namespace USU;

typedef Matrix<float,3,4> Matrix3x4; //come back to this and make some more

Controller::Controller(int priority, unsigned int period_us, const char* imuserial, const char* i2cDevice):
     PeriodicRtThread(priority, period_us), mGX3(priority, imuserial, period_us/1000), mMotors(i2cDevice), mKeepRunning(false)
{
    mRefTime = 0;
    mDutyC << 0, 0, 0, 0;
    mLastSpeed << 0,0,0,0;
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

    //Wait for the first IMU packet to arrive (as a way of synchronization)
    while(!readIMU(mEuler, mCurrentRates)){
        cout << "." ;
        usleep(5000);
    }
    cout << endl;

    std::cout << "Running..." << std::endl;
    bool gotIMU = false;
/////--------CONTROLLER CODE------------------//create temporary variables before while
    float b = 1e-6; //SI units, average damping coefficent of wheels
    float Iw = 0.462; //lbm-in^2, mass moment of inertia of momentum wheel
    float bIw = 1/(b+Iw); //come back and give number
    Matrix3f Inertia;
    Inertia << 102.322878, -0.035566, 0.315676, -0.035566, 103.65751, 0.006317, 0.315676, 0.006317, 159.537290; //lbm-in^2 inertia of tables
    Matrix3x4 Kp;
    float wn=0.2; //choose natural damping frequency
    Kp <<   2.2*Inertia(0,0)*wn*wn,0,0,0,
            0,2.2*Inertia(1,1)*wn*wn,0,0,
            0,0,2.2*Inertia(2,2)*wn*wn,0;

    //For now
    mReference = quaternion(1.2, 0.3, 0.4, 0.1);

    while (mKeepRunning){

        //Read IMU
        gotIMU = readIMU(mEuler, mCurrentRates);

        readMotors(mSpeed, mAmps);

        mCurrentQuat = createQuaternion(mEuler);

        //calculate quaternion error
        Matrix4f Qt;
        Qt << mReference.w(),mReference.z(),-mReference.y(),mReference.x(),
              -mReference.z(),mReference.w(),mReference.x(),mReference.y(),
              mReference.y(),-mReference.x(),mReference.w(),mReference.z(),
              -mReference.x(),-mReference.y(),-mReference.z(),mReference.w();
        Vector4f qs(-mCurrentQuat.x(),-mCurrentQuat.y(),-mCurrentQuat.z(),-mCurrentQuat.w());
        Vector4f qe = Qt*qs;

        //calculate required torque on each of 3 axes
        Vector3f Tc3 =2*Kp*qe*qe.w();

        //calculate required torque on each of 4 wheels
        Matrix4f Tc3to4;
        Tc3to4 << 0.5,0,0.25,0.25,  0,0.5,0.25,-0.25,   -0.5,0,0.25,0.25,     0,-0.5,0.25,-0.25;
        Vector4f Tc3Comp(Tc3(0), Tc3(1), Tc3(2), 0.);
        Vector4f Tc4 = Tc3to4*Tc3Comp;

        //calculate required speeds (rad/s)
        Vector4f speedscmd =(Tc4+Iw*mLastSpeed)*bIw; // check bIw

        //calculate required duty cycles
        speedscmd = speedscmd*(80/618.7262);
        mDutyC = Vector4i(int(speedscmd(0)), int(speedscmd(1)), int(speedscmd(2)), int(speedscmd(3)));  //if speedscmd is in rad/s
        mDutyC += Vector4i(10,10,10,10);
        sendDutyCycles(mDutyC);
//        count--;
//        if(!count){
//            step++;
//            mDutyC = Vector4i(step, step, step, step);
//            sendDutyCycles(mDutyC); //Send the command
//            cout << "Command: " << step << endl;
//            count = 100;
//
        readMotors(speedscmd, mAmps);
            cout << "Angles: " << mEuler << endl << "Rates: " << mCurrentRates << endl;
            cout << "Quaternion" << mCurrentQuat.x() << "," <<  mCurrentQuat.y()
                 << "," << mCurrentQuat.z() << "," << mCurrentQuat.w() << endl;
            cout << "Motor 0: " << "DC: " << mDutyC(0) << " "<< speedscmd(0) << " rad/s, " << mAmps(0) << " A" << endl;
            cout << "Motor 1: " << "DC: " << mDutyC(1) << " "<< speedscmd(1) << " rad/s, " << mAmps(1) << " A" << endl;
            cout << "Motor 2: " << "DC: " << mDutyC(2) << " "<< speedscmd(2) << " rad/s, " << mAmps(2) << " A" << endl;
            cout << "Motor 3: " << "DC: " << mDutyC(3) << " "<<
            speedscmd(3) << " rad/s, " << mAmps(3) << " A" << endl;
//        }

//        if(step == 60)
//            step = 10; //restart


        updateStates();
        waitPeriod();
    }


    std::cout << "CONTROLLER: Terminating " << std::endl;
    mDutyC = Vector4i(0, 0, 0, 0);
    sendDutyCycles(mDutyC);
    joinIMU();
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

void Controller::sendDutyCycles(Vector4i dc)
{
    mMotors.setMotor(0, dc(0));
    mMotors.setMotor(1, dc(1));
    mMotors.setMotor(2, dc(2));
    mMotors.setMotor(3, dc(3));

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

void Controller::readMotors(Vector4f &speedVec, Vector4f &currentVec)
{
    float currents[4], speeds[4]; //Hold the current and speed readings

    mMotors.getAnalogs(speeds, currents); //TODO Check the orded

    //Rearrange
    //TODO perform the necessary conversions
    speedVec << 261.7994*speeds[0],
                261.7994*speeds[1],
                V_TO_RADS*speeds[2],
                V_TO_RADS*speeds[3];

    currentVec << V_TO_CURR*currents[0],
                V_TO_CURR*currents[1],
                V_TO_CURR*currents[2],
                V_TO_CURR*currents[3];
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


void Controller::updateStates()
{
    mLastQuat = mCurrentQuat;
    mLastQuatError = mQuatError;
    mLastEuler = mEuler;
    mLastRates = mCurrentRates;
    mLastSpeed = mSpeed;

    mLastTorque = mTorque;
    mLastAmps = mAmps;
    mLastDutyC = mDutyC;

}
Controller::~Controller()
{
    //dtor
}
