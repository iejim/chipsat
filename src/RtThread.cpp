/**
 * @file RtThread.cpp
 *
 * Small C++ wrapper class to create a realtime scheduled pthread
 *
 * @author Jan Sommer
 *  Created on: Apr 10, 2013
 *
 */
#include <stdio.h>
#include <iostream>
#include <time.h>
#include <errno.h>
#include <stdexcept>
#include "RtThread.h"
using namespace USU;



RtThread::RtThread(int priority):
    mPriority(priority), mId(-1), mStarted(false)
{
    int ret;
    if ( (ret = pthread_attr_init(&mAttr)) != 0)
    {
        perror("phtread_attr_init ");
        throw "Error";
    }
    // Set scheduler to (realtime) FIFO
    if ( (ret = pthread_attr_setschedpolicy(&mAttr, SCHED_FIFO)) != 0)
    {
        perror("pthread_attr_setschedpolicy");
        throw "Error";
    }

    // Change priority for the thread to mPriority
    struct sched_param param;
    if ( (ret = pthread_attr_getschedparam(&mAttr, &param)) != 0)
    {
        perror("pthread_attr_getschedparam");
        throw "Error";
    }
    param.__sched_priority = mPriority;

    if ( (ret = pthread_attr_setschedparam(&mAttr, &param)) != 0)
    {
        perror("pthread_attr_setschedparam");
        throw "Error";
    }

    /*
     * Set inherit scheduler attribut to PTHREAD_EXPLICIT_SCHED
     * otherwise the schedule attributes in mAttr will be ignored
     * and the same settings as the main thread will be inherited.
     */
    if ( (ret = pthread_attr_setinheritsched(&mAttr, PTHREAD_EXPLICIT_SCHED)) != 0)
    {
        perror("pthread_attr_setinheritsched ");
        throw "Error";
    }
}

RtThread::~RtThread()
{
    /*
     * Make sure that the thread terminated properly
     * before deleting the instance
     */
    this->join();
    int ret;
    if ( (ret = pthread_attr_destroy(&mAttr)) != 0)
    {
        perror("pthread_attr_destroy");
        throw "Error";
    }
}

inline
pthread_t RtThread::getThreadId() const
{
    return mId;
}

inline
int RtThread::getPriority() const
{
    return mPriority;
}

void RtThread::start(void *arg)
{
    int ret;
    mArgs = arg;
    /*
    * Since pthread_create is a C library function, the 3rd argument is
    * a global function that will be executed by the thread. In C++, we
    * emulate the global function using the static member function that
    * is called exec. The 4th argument is the actual argument passed to
    * the function exec. Here we use this pointer, which is an instance
    * of the Thread class.
    */
    ret = pthread_create(&mId, &mAttr, &RtThread::exec, (void *)this);
    if ( ret !=0)
    {
        perror("thread_create ");
        if (ret == EAGAIN)
            throw std::runtime_error("Cannot create a new thread");
        else if (ret == EINVAL)
            throw std::runtime_error("Attr invalid");
        else// (ret == EPERM)
            throw std::runtime_error("Insufficient permisions");
    }
    mStarted = true;
}

bool RtThread::join(int timeout_ms)
{
    //Allow the thread to wait for the termination status
    if (mStarted)
    {
        if(timeout_ms == 0)
        {
            if (pthread_join(mId, NULL) != 0) return false;
        }
        else
        {
            struct timespec ts;
            if (clock_gettime(CLOCK_REALTIME, &ts) == -1)
            {
                perror("clock_gettime ");
                throw "Error";
            }
            ts.tv_sec  += timeout_ms / 1000;
            ts.tv_nsec += timeout_ms * 1000000;

            int result =  pthread_timedjoin_np(mId, NULL, &ts);
            if (result == ETIMEDOUT)
                return false;
        }
        return true;
    }

    return false;
}

// Function which is actually executed by the thread
void * RtThread::exec(void *thr)
{
    reinterpret_cast<RtThread *> (thr)->run();
    return NULL;
}


