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


//int timeval_subtract (struct timeval * result, struct timeval * x, struct timeval * y)
//{
//    /* Perform the carry for the later subtraction by updating y. */
//    if (x->tv_usec < y->tv_usec) {
//        int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
//        y->tv_usec -= 1000000 * nsec;
//        y->tv_sec += nsec;
//    }
//    if (x->tv_usec - y->tv_usec > 1000000) {
//        int nsec = (x->tv_usec - y->tv_usec) / 1000000;
//        y->tv_usec += 1000000 * nsec;
//        y->tv_sec -= nsec;
//    }
//
//    /* Compute the time remaining to wait.
//          tv_usec is certainly positive. */
//    result->tv_sec = x->tv_sec - y->tv_sec;
//    result->tv_usec = x->tv_usec - y->tv_usec;
//
//    /* Return 1 if result is negative. */
//    return x->tv_sec < y->tv_sec;
//}


GX3Monitor::GX3Monitor(int priority, unsigned int period_us, const char *imuserial)
    :PeriodicRtThread(priority, period_us), mGX3(priority, imuserial), mKeepRunning(false)
{

}

void GX3Monitor::setCommandList(uint8_t* cList, uint8_t cNum)
{
    for(int i=0; i<cNum; i++)
    {
        mGX3.addCommand(cList[i]);
    }


}

void GX3Monitor::setContinuousMode()
{
    mGX3.runContinuously();
}
void GX3Monitor::run()
{   cout << "MONITOR: Running" <<endl;
    mKeepRunning = true;

    //At this point, the ser should have already added the desired commands
    //And setup continous mode, if needed
    cout << "MONITOR: Initializing Communicator... "<< endl;
    mGX3.initialize();
    cout << "MONITOR: Starting Communicator... " << endl;
    mGX3.start();
    cout << "MONITOR: Entering the loop ... " <<endl;
    uint8_t count = mGX3.getCommandCount();
    packet_ptr* packetList = new packet_ptr[count]; //Since dealing with addresses, should not need be more dynamic than this.
    uint8_t i;
    uint8_t j;
    SharedQueue<vector> dataVecQueue;
    SharedQueue<matrix> dataMatQueue;
    SharedQueue<quaternion> dataQuatQueue;
    while(mKeepRunning){
        //Get te packets to read their data, as many as were requested
        //TODO Check if the Queue is always the same size of the count
        if(count >= mGX3.size()){ //They should be at least the same
            //TODO What should it do if its larger?
            //     Maybe pop until only _count_ are left
            //     That's what Jan did.
            for(i=0; i<count; i++){
                packetList[i] = mGX3.front();
                mGX3.pop();
            }
        }

        /*
        At this point I have the data. What should we do with it?
        For now, just check what kind of data it contains and print.
        Two ways to do this:
            - Check the Packet Type with getPacketType()
              and do something about it
            - Check if the packet has vectors and/or a matrix
              with hasVectors() and hasMatrix()
        Remember the Monitor class is generic, so it can use both
        */
        for (i=0; i<count;i++){ //Better to do this in the previous loop
            //If done above, packetList does not need to be an array.
            if(packetList[i]->hasVectors()){
                packetList[i]->getVectors(dataVecQueue);
                for(j=0; j<packetList[i]->hasVectors(); j++){
                    printVector(dataVecQueue.front());
                    dataVecQueue.pop();
                }
            }
            if(packetList[i]->hasMatrix()){
                packetList[i]->getMatrix(dataMatQueue);
                printMatrix(dataMatQueue.front());
                dataMatQueue.pop();
            }
            if(packetList[i]->getPacketType() == QUATERNION){
                //The maybe way
                //Quaternion* quat = static_cast<Quaternion*>(packetList[i]);
                packet_ptr quat = packetList[i];
                //quat->getQuaternion(dataQuatQueue);
                cout << (*quat) << endl;
                //dataQuatQueue.pop();
                /*
                //The forceful way
                SharedQueue<vector> tempQueue;
                packetList[i]->getVectors(tempQueue);
                float qData[4];
                vector* part = tempQueue.pop();
                qData[0] = *part[0];
                qData[1] = *part[1];
                qData[2] = *part[2];
                part = tempQueue.pop();

                qData[3] = *part[0];
                dataQuatQueue.push(new quaternion(qData));
                */

            }

        }
        /*
        Right now the Queues contain
        */

        //Wait the rest of the time
        waitPeriod();
    }


    std::cerr << "GX3MONITOR: Got signal to terminate" << std::endl;
    std::cerr << "GX3MONITOR: Stopping Gx3-communicator..." << std::endl;
    mGX3.stop();
    if(mGX3.join())
        std::cerr << "KALMANFILTER: Gx3-communicator joined" << std::endl;
    else
        std::cerr << "KALMANFILTER: Joining Gx3-communicator failed" << std::endl;

    std::cerr << "GX3MONITOR: Terminating now..." << std::endl;
}

void GX3Monitor::printVector(vector vec){
    cout << "\t" << vec(0)  << ", " << vec(1)  << ", " << vec(2)
         << endl;
}

void GX3Monitor::printMatrix(matrix mat){
    cout << "\t" << mat(0,0) << ", " << mat(0,1) << ", " << mat(0,1)
        << ",\t" << mat(1,0) << ", " << mat(1,1) << ", " << mat(1,2)
        << ",\t" << mat(2,0) << ", " << mat(2,1) << ", " << mat(2,2)
        << endl;
}

/*
bool GX3Monitor::getState()
{
    ScopedLock scLock(mStateLock);
    return mState;
}
*/
