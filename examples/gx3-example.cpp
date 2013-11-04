
#include <iostream>
#include <iomanip>
//using namespace std;
//#include "gx3communicator.h"
#include "messages.h"
#include <stdint.h>
#include <SerialPort.h>
#include <time.h>
#include <unistd.h>


using namespace USU;

int main()
{
    SerialPort mSerialPort("/dev/ttyO4");

    mSerialPort.Open(SerialPort::BAUD_115200);
    if(mSerialPort.IsOpen() == false)
        throw std::runtime_error("Opening SerialPort failed");

    /*
       Set up the 3DM-GX25 with the following settings (different from default):
        - Data rate 50 Hz
        - Enable little endian for floating points
        - Enable Quaternion calculation
     */
    SamplingSettings initSettings(SamplingSettings::Change, 20,
                                  SamplingSettings::FlagDefault | SamplingSettings::FlagFloatLittleEndian
                                                                | SamplingSettings::FlagEnableQuaternion);


    if(initSettings.sendCommand(mSerialPort) == false)
        throw std::runtime_error("Setting SamplingSettings failed");

    for(int i=0; i<10; ++i)
    {
        unsigned char byte = QUATERNION;
        mSerialPort.WriteByte(byte);
        Quaternion lastState;

        if(lastState.readFromSerial(mSerialPort))
            std::cout << "True\n";
        else
            std::cout << "False\n";

        std::cout << lastState.quat.w() << "\t" << lastState.quat.x() << "\t"
                  << lastState.quat.y() << "\t" << lastState.quat.z() << std::endl;
        usleep(1000000);
    }


    return 0;
}
