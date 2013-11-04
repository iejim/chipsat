/**
 * @file gx3communicator.cpp
 *
 * Contains the thread which handles the communication to the
 * 3DM-GX3-25.
 *
 * @author Jan Sommer
 *  Created on: Apr 26, 2013
 *
 */

#include <stdint.h>
#include <iostream>
#include <iomanip>
#include <stdexcept>

#include <sys/time.h>

#include "gx3communicator.h"
using namespace USU;

#include "messages.h"

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

//    /* Compute the time remaining to wait.
//          tv_usec is certainly positive. */
//    result->tv_sec = x->tv_sec - y->tv_sec;
//    result->tv_usec = x->tv_usec - y->tv_usec;

//    /* Return 1 if result is negative. */
//    return x->tv_sec < y->tv_sec;
//}

GX3Communicator::GX3Communicator(int priority, const char *serialDevice, SerialPort::BaudRate baudRate)
    :RtThread(priority), mSerialPort(serialDevice), mBaudRate(baudRate), mKeepRunning(false)
{

}

void GX3Communicator::initialize()
{
    mSerialPort.Open(mBaudRate);
    if(mSerialPort.IsOpen() == false)
        throw std::runtime_error("Opening SerialPort failed");

    /*
       Set up the 3DM-GX25 with the following settings (different from default):
        - Data rate 50 Hz (20 ms)
        - Enable little endian for floating points
     */
    SamplingSettings initSettings(SamplingSettings::Change,  20,
                                  SamplingSettings::FlagDefault | SamplingSettings::FlagFloatLittleEndian
                                  | SamplingSettings::FlagEnableQuaternion);

    if(initSettings.sendCommand(mSerialPort) == false)
        throw std::runtime_error("Setting SamplingSettings failed");

}

void GX3Communicator::run()
{

    mKeepRunning = true;

    // Activate Continuous mode
    SetContinuousMode setCont(ACC_ANG_MAG_VEC_ORIENTATION_MAT);
    if(setCont.sendCommand(mSerialPort) == false)
        std::cerr << " Set continuous mode failed " << std::endl;/// TODO: Error

//    struct timeval start, now, elapsed;

//    gettimeofday(&start, NULL);
    while(mKeepRunning)
    {
        packet_ptr data(new AccAngMagOrientationMat);
        if(data->readFromSerial(mSerialPort))
        {
            mQueue.push(data);
            //        gettimeofday(&now, NULL);
            //        timeval_subtract(&elapsed, &now, &start);
//                    unsigned long long timestamp = elapsed.tv_sec * 1000 + elapsed.tv_usec / 1000; // in ms since start
//                    std::cout << (*data) << std::endl;
        }
        else
        {
//            std::cout << "readFromSerial failed" << std::endl;
            //            throw std::runtime_error("Getting PackageData failed"); /// TODO: Error?

        }
    }

    std::cerr << "GX3COMMUNICATOR: Got signal to terminate" << std::endl;
    std::cerr << "GX3COMMUNICATOR: Stopping IMU continuous mode..." << std::endl;
    // Stop continuous mode
    setCont.mCommand[3] = 0;
    if(setCont.sendCommand(mSerialPort) == false)
        ; /// TODO: Error?

    std::cerr << "GX3COMMUNICATOR: IMU continuous mode stopped" << std::endl;
    std::cerr << "GX3COMMUNICATOR: Terminating now..." << std::endl;
}


