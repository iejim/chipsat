#include <unistd.h>
#include <exception>
#include <stdexcept>
#include <iostream>

#include "messages.h"
using std::cerr;
using std::endl;

using namespace USU;

class testSettings
{

public:

    testSettings(){};

    void stop(){ running = false;}

    void run(){
        SerialPort mSerial("/dev/ttyO4");
        Euler pack;
        running = true;

        cerr << "Opening serial port ..." <<endl;
        mSerial.Open(SerialPort::BAUD_115200);
        cerr << "Checking for serial connection... " << endl;
        if(mSerial.IsOpen() == false)
            throw std::runtime_error("Opening SerialPort failed");

    ////////////////////////////////////////////////////////
        while (running){
            cerr << "Reading port ..." << endl;
            try {
                mSerial.WriteByte(0xCE);
                if(pack.readFromSerial(mSerial)){
                    cerr << pack << endl;
                }else{
                    cerr << "No data" << endl;
                }
            }
            catch (std::exception e){
                cerr << "Exception: " << e.what() <<endl;
            }
            sleep(2);
        }
        cerr << "Done testing ... " << endl;
    /////////////////////////////////////////////////////////

        SamplingSettings initSettings(SamplingSettings::Change,  20,
                                      SamplingSettings::FlagDefault | SamplingSettings::FlagFloatLittleEndian
                                      | SamplingSettings::FlagEnableQuaternion);

        std::cerr << "Initializing IMU " << std::endl;

        try{
            initSettings.sendCommand(mSerial);
        } catch (std::exception e){
            throw std::runtime_error("Setting SamplingSettings failed");
        }
    }

private:
    bool running;

};
