/**
 * @file periodicrtthread.cpp
 *
 * Small C++ wrapper class to create a realtime scheduled pthread
 * with periodic timer events.
 *
 * @author Jan Sommer
 *  Created on: Apr 10, 2013
 *
 */

#include <time.h>
#include <sys/timerfd.h>
#include <stdio.h>
#include <unistd.h>
#include <cerrno>

#include "periodicrtthread.h"
using namespace USU;


PeriodicRtThread::PeriodicRtThread(int priority, unsigned int period_us)
    :RtThread(priority), mMissedWakeUps(0), mPeriod_us(period_us)
{
    makeThreadPeriodic();
}


void PeriodicRtThread::makeThreadPeriodic()
{
    int ret;
    unsigned int ns;
    unsigned int sec;
    struct itimerspec itval;
    /* Create the timer */
    if ( (mTimerFd = timerfd_create (CLOCK_MONOTONIC, 0)) == -1)
    {
        perror("timer_create ");
        return;
    }

    /* Make the timer periodic */
    sec = mPeriod_us/1000000;
    ns = (mPeriod_us - (sec * 1000000)) * 1000;
    itval.it_interval.tv_sec = sec;
    itval.it_interval.tv_nsec = ns;
    itval.it_value.tv_sec = sec;
    itval.it_value.tv_nsec = ns;
    if ( (ret = timerfd_settime(mTimerFd, 0, &itval, NULL)) != 0)
    {
        perror("timerfd_settime ");
        return;
    }
}

void PeriodicRtThread::waitPeriod()
{
    unsigned long long missed;
    int ret;

    /* Wait for the next timer event. If we have missed any the
           number is written to "missed"
       While loop to continue the wait if interrupted by a signal.
    */
    while( (ret = read (mTimerFd, &missed, sizeof (missed)) ) == -1)
    {
        perror("timer read ");
        return;
    }

    /* "missed" should always be >= 1, but just to be sure, check it is not 0 anyway */
    if (missed > 0)
        mMissedWakeUps += (missed - 1);
}
