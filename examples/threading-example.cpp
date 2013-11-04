
#include <iostream>
using namespace std;

#include<csignal>
#include<cstdlib>
#include<unistd.h>

#include "periodicrtthread.h"
#include "Lock.h"
using namespace USU;


class SharedObject
{
private:
    Lock mLock1;
    Lock mLock2;
    int mData1;
    int mData2;

public:
    SharedObject():mData1(0), mData2(0)
    {}
    void increaseData1()
    {
        mLock1.lock(); // Lock mutex manually
            mData1++;
        mLock1.unlock(); // Unlock mutex manually
    }

    void increaseData2()
    {
        /*
         * ScopedLock locks the mutex when its created
         * and unlocks it automatically when it is destroyed
         * at the end of its scope. Therefore the mutex will be
         * unlocked even if the code returns in an
         * unexpected way (exception or error)
         */
        ScopedLock scLock(mLock2);
        mData2++;
    }

    int getData1()
    {
        int temp; // needed to unlock mutex before return
        mLock1.lock();
            temp = mData1;
        mLock1.unlock();
        return temp;
    }

    int getData2()
    {
        ScopedLock scLock(mLock2);
        return mData2;
    } // Mutex will be released automatically here
};


class MyThread: public PeriodicRtThread
{
public:
    MyThread(int prio)
        : PeriodicRtThread(prio, 1000000), keepRunning(true)
    {}

    void run()
    {
        SharedObject* counter = reinterpret_cast<SharedObject*> (mArgs);
        cout << "Hello from Thread: " << mId << endl;

        while(keepRunning)
        {
            counter->increaseData1();
            counter->increaseData2();

            cout << "Thread " << mId << ": Data1 " << counter->getData1() << endl;
            cout << "Thread " << mId << ": Data2 " << counter->getData2() << endl;
            waitPeriod();
        }

        cout << "Thread " << mId << ": terminating..." << endl;
    }

    volatile bool keepRunning;
};


MyThread thr1(5), thr2(4);

void endProgram(int s)
{
    cout << "got signal" << endl;

    thr1.keepRunning = false;
    thr2.keepRunning = false;


    usleep(2000000);

    exit(1);

}

int main()
{
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = endProgram;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    SharedObject sO;
    cout << "Starting threads" << endl;
    thr1.start((void*) &sO);
    thr2.start((void*) &sO);

    thr1.join();
    thr2.join();
    cout << "Threads joined" << endl;


 return 0;
}
