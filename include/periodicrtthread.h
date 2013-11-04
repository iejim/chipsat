/**
 * @file periodicrtthread.h
 *
 * Small C++ wrapper class to create a realtime scheduled pthread
 * with periodic timer events.
 *
 * @author Jan Sommer
 *  Created on: Apr 10, 2013
 *
 */

#ifndef PERIODICRTTHREAD_H
#define PERIODICRTTHREAD_H

#include "RtThread.h"

namespace USU {

///TODO: Make some proper exceptions

/*!
 \brief Abstract wrapper class for a periodic thread usign the pthread library with RT-priority

 Based on RtThread this class uses pthread underneath but creates a periodic timer
 event it can wait for in a (forever) loop. This is more accurate than the use of
 nanosleep as the execution time of the loop will not be taken into account. It is
 therefore designed for periodic work where high accuracy is desired.

 \ingroup threading

*/
class PeriodicRtThread : public RtThread
{
private:
    int mTimerFd; /*!< Handle to the timer */

    /*!
      \brief mMissedWakeUps Stores the number of missed wakeups since the thread starts (if any)

      The missed wake-ups will be counted from when waitPeriod is called.
      Workaround call waitPeriod once before entering the periodic loop and
      set mMissedWakeUps = 0;
      TODO: Find a better solution
     */
    unsigned int mMissedWakeUps;
    unsigned int mPeriod_us; /*!< The periodic time in us */

    PeriodicRtThread(const PeriodicRtThread& thread); /*!< Copy constructor made inaccessible by declaring it private */
    PeriodicRtThread& operator=(const PeriodicRtThread& rhs); /*!< Assignment constructor made inaccessible by declaring it private */

protected:
    /*!
     \brief Registers the Periodic timer

        TODO: create exception
    */
    void makeThreadPeriodic();

    /*!
     \brief Blocks the thread until the next timer event

     Waits the remaining time until the next timer event happens.
     Thus waitTime = mPeriod_us - runtime since last timer event

    */
    void waitPeriod();

public:
    /*!
     \brief Creates the PeriodicRtThread object

      Calls the constructor of the parent RtThread and registers the
      periodic timer

     \param priority  the Priority of the Thread (Linux: 1..99)
     \param period_us Period of the thread in us
    */
    PeriodicRtThread(int priority = 0, unsigned int period_us = 1000000);

    /*!
     \brief Actual method of the thread is running

        Every child class has to implement this function
        in order to do some threaded work.
    */
    virtual void run() = 0;
};

}

#endif // PERIODICRTTHREAD_H
