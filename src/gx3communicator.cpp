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
using std::cout;
using std::endl;

using namespace USU;

#include "messages.h"



GX3Communicator::GX3Communicator(int priority, const char *serialDevice, uint16_t samplingPeriod_ms, SerialPort::BaudRate baudRate)
    :RtThread(priority), mSerialPort(serialDevice), mBaudRate(baudRate), mSamplingPeriod(samplingPeriod_ms), mKeepRunning(false)
{
    mRunContinuous = false;
    mCommandList = NULL;
    mCommandNumber = 0;
}

void GX3Communicator::initialize()
{
    cout << "GX3COMMUNICATOR: Opening serial port ..." <<endl;
    mSerialPort.Open(mBaudRate);
    cout << "GX3COMMUNICATOR: Checking for serial connection... " << endl;
    if(mSerialPort.IsOpen() == false)
        throw std::runtime_error("Opening SerialPort failed");

    /*
       Set up the 3DM-GX25 with the following settings (different from IMU default):
        - Data rate defaults to 20ms (50 Hz)
        - Enable little endian for floating points
//        - Enable quaternions
     */

    SamplingSettings initSettings(SamplingSettings::Change,  mSamplingPeriod,
                                  SamplingSettings::FlagDefault | SamplingSettings::FlagFloatLittleEndian);
//                                  | SamplingSettings::FlagEnableQuaternion);

    std::cerr << "GX3COMMUNICATOR: Initializing IMU " << std::endl;

    try{
        initSettings.sendCommand(mSerialPort);
    } catch (std::exception e){
        throw std::runtime_error("Setting SamplingSettings failed");
    }

    //Prepare the list of commands
    mCommandNumber = mCommandQueue.size();
    if (mCommandNumber>0){
        mCommandList = new uint8_t[mCommandNumber];
        mPacketList = new packet_ptr[mCommandNumber];
        for (int i=0; i<mCommandNumber; i++){
            mCommandList[i] = mCommandQueue.front();
            mCommandQueue.pop();

            packet_ptr* newPack;
            switch(mCommandList[i]){
                case RAW_ACC_ANG:
                    newPack = new packet_ptr(new RawAccAng);
//                    mPacketList[i] = new RawAccAng();
                    break;
                case ACC_ANG_MAG_VEC:
                    newPack = new packet_ptr(new AccAngMag);
//                    mPacketList[i] = new AccAngMag();
                    break;
                case QUATERNION:
                    newPack = new packet_ptr(new Quaternion);
//                    mPacketList[i] = new Quaternion();
                    break;
                case ACC_ANG_MAG_VEC_ORIENTATION_MAT:
                    newPack = new packet_ptr(new AccAngMagOrientationMat);
//                    mPacketList[i] = new AccAngMagOrientationMat();
                    break;
                case EULER_ANGLES:
                    newPack = new packet_ptr(new Euler);
//                    mPacketList[i] = new Euler();
                    break;
                case EULER_ANGLES_ANG_RATES:
                    newPack = new packet_ptr(new EulerAng);
//                    mPacketList[i] = new EulerAng();
                    break;
                case ORIENTATION_MATRIX:
                    newPack = new packet_ptr(new OrientationMat);
//                    mPacketList[i] = new OrientationMat();
                    break;
                case ACC_ANG_ORIENTATION_MAT:
                    newPack = new packet_ptr(new AccAngOrientationMat);
//                    mPacketList[i] = new AccAngOrientationMat();
                    break;
                default:
                    break;
                    //mPacketList[i] = new GX3Packet; //Will throw and error (abstract class)
            }
            mPacketList[i] = *newPack;
        }


    } else {
        throw std::runtime_error("No commands are set to be requested");
    }
    std::cerr << "GX3COMMUNICATOR: Initialization successful! " << std::endl;
}

void GX3Communicator::run()
{

    mKeepRunning = true;


    //Create a package with all the commands that will be sent on every request.
    //Here for scope reasons
    RequestCommands sessionCommands(mCommandList, mCommandNumber, mRunContinuous);
    //That way it takes just one call no matter how many commands are sent

//    struct timeval start, now, elapsed;

//    gettimeofday(&start, NULL);

    if(mRunContinuous){
        std::cerr << "GX3COMMUNICATOR: Setting up continuous data " << std::endl;
        sessionCommands.sendCommand(mSerialPort);
    }
    std::cerr << "GX3COMMUNICATOR: Begin reading data... " << std::endl;
    while(mKeepRunning)
    {

        if (!mRunContinuous) //No need to keep resend if continuous
        {
            //This line will request the data and has to be
            //sent before we try to read any data
//            std::cerr << "GX3COMMUNICATOR: Requesting commands " << std::endl;
            if(sessionCommands.sendCommand(mSerialPort) == false)
                std::cerr << "GX3COMMUNICATOR: Requesting multiple commands failed " << std::endl;

        }
//        std::cerr << "GX3COMMUNICATOR: Fetching data " << std::endl;
        for(int i=0; i<mCommandNumber; i++)
        {
            //I think this method fails because the IMU might not be sending
            //the packets in the same order
            if(mPacketList[i]->readFromSerial(mSerialPort))
                mQueue.push(mPacketList[i]);
            else
                std::cout << "readFromSerial failed. Packet: " << std::endl;
        }
//        std::cerr << "GX3COMMUNICATOR: Done fetching data " << std::endl;
        //throw std::runtime_error("Getting PackageData failed"); /// TODO: Error?

    }

    std::cerr << "GX3COMMUNICATOR: Got signal to terminate" << std::endl;

    //Clean up the queue
    std::cout << "Left " << mQueue.size() << " packets in the queue. Removing ..." << std::endl;
    while (mQueue.size()>0){
        mQueue.pop();
    }

    if (mRunContinuous){
        std::cerr << "GX3COMMUNICATOR: Stopping IMU continuous mode..." << std::endl;
        sessionCommands.stopContinuous(mSerialPort);

        std::cerr << "GX3COMMUNICATOR: IMU continuous mode stopped" << std::endl;
    }
    mSerialPort.Close();

    std::cerr << "GX3COMMUNICATOR: Terminating now..." << std::endl;
}


