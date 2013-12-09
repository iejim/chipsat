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
using std::cerr;
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
    cerr << "GX3COMMUNICATOR: Opening serial port ..." <<endl;
    mSerialPort.Open(mBaudRate);
    cerr << "GX3COMMUNICATOR: Checking for serial connection... " << endl;
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

    cerr << "GX3COMMUNICATOR: Initializing IMU " << endl;

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
        throw std::runtime_error("GX3COMMUNICATOR: No commands are set to be requested");
    }
    cerr << "GX3COMMUNICATOR: Initialization successful! " << endl;
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

    try
    {
        if(mRunContinuous){
            cerr << "GX3COMMUNICATOR: Setting up continuous data " << endl;
            sessionCommands.sendCommand(mSerialPort);
        }
        cerr << "GX3COMMUNICATOR: Begin reading data... " << endl;
        while(mKeepRunning)
        {

            if (!mRunContinuous) //No need to keep resend if continuous
            {
                //This line will request the data and has to be
                //sent before we try to read any data
    //            cerr << "GX3COMMUNICATOR: Requesting commands " << endl;
                if(sessionCommands.sendCommand(mSerialPort) == false)
                    cerr << "GX3COMMUNICATOR: Requesting multiple commands failed " << endl;

            }
    //        std::cerr << "GX3COMMUNICATOR: Fetching data " << std::endl;
            for(int i=0; i<mCommandNumber; i++)
            {
                //I think this method fails because the IMU might not be sending
                //the packets in the same order
                if(mPacketList[i]->readFromSerial(mSerialPort))
                {
                    mQueue.push(mPacketList[i]);
                    int s = mQueue.size();
                    cerr << "P: " << s << endl;
                    if (s>2)
                        throw std::runtime_error("Stuck");
                }
                else
                    cerr << "readFromSerial failed. Packet: " << i+1 << " of " << int(mCommandNumber) << endl;
            }
    //        cerr << "GX3COMMUNICATOR: Done fetching data " << endl;
            //throw std::runtime_error("Getting PackageData failed"); /// TODO: Error?

        }
    }catch (std::exception e)
    {
        cerr << "Error: " << e.what() <<endl <<  "GX3COMMUNICATOR: Stopping ... " << endl;
        mKeepRunning = false;
    }

    cerr << "GX3COMMUNICATOR: Got signal to terminate" << endl;

    //Clean up the queue
    cerr << "GX3COMMUNICATOR: Left " << mQueue.size() << " packets in the queue. Removing ..." << endl;
    while (mQueue.size()>0){
        mQueue.pop();
    }

    if (mRunContinuous){
        cerr << "GX3COMMUNICATOR: Stopping IMU continuous mode..." << endl;
        sessionCommands.stopContinuous(mSerialPort);

        cerr << "GX3COMMUNICATOR: IMU continuous mode stopped" << endl;
    }
    mSerialPort.Close();

    cerr << "GX3COMMUNICATOR: Terminating now..." << endl;
}


