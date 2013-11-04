/**
 * @file kalmanfilter.cpp
 *
 * C++ class for the sensor fusion and stated estimated.
 * Based on the PeriodicRtThread class.
 *
 * @author Jan Sommer
 *  Created on: Apr 20, 2013
 *
 */

#include<iostream>
using std::cout;
using std::endl;

#include <sys/time.h>
#include <unistd.h>


#include "kalmanfilter.h"
#include "vector.h"

using namespace USU;


int timeval_subtract (struct timeval * result, struct timeval * x, struct timeval * y)
{
    /* Perform the carry for the later subtraction by updating y. */
    if (x->tv_usec < y->tv_usec) {
        int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
        y->tv_usec -= 1000000 * nsec;
        y->tv_sec += nsec;
    }
    if (x->tv_usec - y->tv_usec > 1000000) {
        int nsec = (x->tv_usec - y->tv_usec) / 1000000;
        y->tv_usec += 1000000 * nsec;
        y->tv_sec -= nsec;
    }

    /* Compute the time remaining to wait.
          tv_usec is certainly positive. */
    result->tv_sec = x->tv_sec - y->tv_sec;
    result->tv_usec = x->tv_usec - y->tv_usec;

    /* Return 1 if result is negative. */
    return x->tv_sec < y->tv_sec;
}

KalmanFilter::KalmanFilter(int priority, unsigned int period_us, const char *i2cImu, const char *i2cMotor)
    :PeriodicRtThread(priority, period_us), mMode(CollectPololuData), mImu(i2cImu),
      mGX3(priority, "/dev/ttyO4"), mKeepRunning(false)
{
}

void KalmanFilter::run()
{
    switch(mMode)
    {
    case SimpleControl:          runSimpleControl();
                                 break;
    case CollectPololuData:      runCollectPololu();
                                 break;
    case CollectMicroStrainData: runCollectMicroStrain();
                                 break;
    case CollectData:            runCollectBoth();
                                 break;
    }

    std::cerr << "KALMANFILTER: Terminating now..." << std::endl;
}

bool KalmanFilter::getState()
{
    ScopedLock scLock(mStateLock);
    return mState;
}

void KalmanFilter::initializeModeSimpleControl(std::string trajFilename, float pgain)
{
    std::ifstream inFile;
    inFile.open(trajFilename);
    if(!inFile.is_open())
        throw std::runtime_error("MotorController: Could not open input file");

    Command temp;
    while(true)
    {
        inFile >> temp.time;

        float x,y,z;
        inFile >> x >> y >> z;
        temp.angVel << x, y, z;

        if(inFile.eof())
            break;
        mCommandList.push_back(temp);
    }
    inFile.close();

    cout << "Read " << mCommandList.size() << " commands." << endl;

    mMotors.setPGain(pgain);
}

void KalmanFilter::runSimpleControl()
{
    vector gyro;
    mKeepRunning = true;
    struct timeval start, now, elapsed;

    if(mCommandList.empty())
    {
        std::cerr << "Error: No command list loaded. Terminating";
        return;
    }

    mImu.enable();

    std::vector<Command>::const_iterator commandIt = mCommandList.begin();
    mMotors.setSetValue(commandIt->angVel);
    int countdown = commandIt->time;


    gettimeofday(&start, NULL);
    unsigned lastTime = 0;
    waitPeriod();

    while(mKeepRunning)
    {
        gettimeofday(&now, NULL);

        gyro = mImu.readGyro();

        // Run countdown
        timeval_subtract(&elapsed, &now, &start);
        unsigned time = elapsed.tv_sec * 1000 + elapsed.tv_usec / 1000; // in ms since start
        countdown -= time - lastTime;
        lastTime = time;

        // if countdown over execute next command from list
        if(countdown <=0)
        {
            commandIt++;
            // if at the end of the commandList start again from the beginning
            if(commandIt == mCommandList.end())
                commandIt = mCommandList.begin();

            countdown = commandIt->time;
            mMotors.setSetValue(commandIt->angVel);
        }

        // Alwasy use mutex, when changing state
         mMotors.controlFromGyro(gyro);

        waitPeriod();
    }

    std::cerr << "KALMANFILTER: Got signal to terminate" << std::endl;
}

void KalmanFilter::runCollectPololu()
{
    vector acc, mag, gyro;
    mKeepRunning = true;
    struct timeval start, now, elapsed;

    mImu.enable();

    gettimeofday(&start, NULL);
    waitPeriod();

    // Create an object to set the output format for the vectors
    Eigen::IOFormat csv(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ");

    while(mKeepRunning)
    {
        gettimeofday(&now, NULL);

        acc  = mImu.readAcc();
        mag  = mImu.readMag();
        gyro = mImu.readGyro();

        timeval_subtract(&elapsed, &now, &start);
        unsigned time = elapsed.tv_sec * 1000 + elapsed.tv_usec / 1000; // in ms since start

        // print data
        cout << time << ",\t" << acc.format(csv) << ",\t" << mag.format(csv) << ",\t" << gyro.format(csv) << endl;

        waitPeriod();
    }

    std::cerr << "KALMANFILTER: Got signal to terminate" << std::endl;
}

void KalmanFilter::runCollectMicroStrain()
{

    mKeepRunning = true;

    mGX3.initialize();
    mGX3.start();
    while(mKeepRunning)
    {
        packet_ptr lastState;

        if(mGX3.isEmpty() == false)
        {
            int length = mGX3.size();
            while(length-->1)
            {
                mGX3.pop();
            }

            lastState = mGX3.front();
            mGX3.pop();

            cout << (*lastState) << endl;
        }

        waitPeriod();
    }

    std::cerr << "KALMANFILTER: Got signal to terminate" << std::endl;
    std::cerr << "KALMANFILTER: Stopping Gx3-communicator..." << std::endl;
    mGX3.stop();
    if(mGX3.join() )
    {
        std::cerr << "KALMANFILTER: Gx3-communicator joined" << std::endl;
    }
    else
    {
        std::cerr << "KALMANFILTER: Joining Gx3-communicator failed" << std::endl;
    }
}

void KalmanFilter::runCollectBoth()
{
    vector acc, mag, gyro;
    mKeepRunning = true;
    struct timeval start, now, elapsed;

    mImu.enable();
    mGX3.initialize();
    mGX3.start();

    gettimeofday(&start, NULL);
    waitPeriod();

    // Create an object to set the output format for the vectors
    Eigen::IOFormat csv(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ");

    while(mKeepRunning)
    {
        gettimeofday(&now, NULL);

        acc  = mImu.readAcc();
        mag  = mImu.readMag();
        gyro = mImu.readGyro();

        timeval_subtract(&elapsed, &now, &start);
        unsigned time = elapsed.tv_sec * 1000 + elapsed.tv_usec / 1000; // in ms since start

        packet_ptr lastState;

        if(mGX3.isEmpty() == false)
        {
            int length = mGX3.size();
            while(length-->1)
            {
                mGX3.pop();
            }

            lastState = mGX3.front();
            mGX3.pop();
        }

        // print data
        cout << (*lastState) << "\t" << time << ",\t" << acc.format(csv) << ",\t" << mag.format(csv) << ",\t" << gyro.format(csv) << endl;

        waitPeriod();
    }

    std::cerr << "KALMANFILTER: Got signal to terminate" << std::endl;
    std::cerr << "KALMANFILTER: Stopping Gx3-communicator..." << std::endl;
    mGX3.stop();
    if(mGX3.join() )
    {
        std::cerr << "KALMANFILTER: Gx3-communicator joined" << std::endl;
    }
    else
    {
        std::cerr << "KALMANFILTER: Joining Gx3-communicator failed" << std::endl;
    }
}
KalmanFilter::Mode KalmanFilter::getMode() const
{
    return mMode;
}

void KalmanFilter::setMode(const Mode &value)
{
    mMode = value;
}

