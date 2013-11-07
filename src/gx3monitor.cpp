/**
 * @file gx3monitor.cpp
 *
 * C++ class used to monitor the data output
 * of the GX3 IMU.
 * Based on the PeriodicRtThread class.
 *
 * @author Ivan Jimenez
 *  Created on: Nov 03, 2013
 *
 */

#include<iostream>
using std::cout;
using std::endl;

#include <sys/time.h>
#include <unistd.h>


#include "gx3monitor.h"
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

GX3Monitor::GX3Monitor(int priority, unsigned int period_us, const char *imuserial)
    :PeriodicRtThread(priority, period_us), mGX3(priority, imuserial), mKeepRunning(false)
{

}

void GX3Monitor:setCommandList(uint8_t* cList, uint8_t cNum)
{
    for(int i=0; i<cNum; i++)
    {
        mGX3.addCommand(cList[i]);
    }
}
void GX3Monitor::run()
{
    mKeepRunning = true;

    //TODO Should probably be set before run()
    //Add Commands
    //mGX3.addCommand();

    mGX3.initialize();
    mGX3.start();

    std::cerr << "GX3MONITOR: Terminating now..." << std::endl;
}

bool GX3Monitor::getState()
{
    ScopedLock scLock(mStateLock);
    return mState;
}

void GX3Monitor::runCollect()
{

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

    std::cerr << "GX3MONITOR: Got signal to terminate" << std::endl;
    std::cerr << "GX3MONITOR: Stopping Gx3-communicator..." << std::endl;
    mGX3.stop();
    if(mGX3.join() )
    {
        std::cerr << "GX3MONITOR: Gx3-communicator joined" << std::endl;
    }
    else
    {
        std::cerr << "GX3MONITOR: Joining Gx3-communicator failed" << std::endl;
    }
}


