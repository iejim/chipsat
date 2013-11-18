#include <unistd.h>
#include <exception>
#include <stdexcept>
#include <iostream>

#include "messages.h"
using std::cout;
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

        cout << "Opening serial port ..." <<endl;
        mSerial.Open(SerialPort::BAUD_115200);
        cout << "Checking for serial connection... " << endl;
        if(mSerial.IsOpen() == false)
            throw std::runtime_error("Opening SerialPort failed");

    ////////////////////////////////////////////////////////
        while (running){
            cout << "Reading port ..." << endl;
            try {
                mSerial.WriteByte(0xCE);
                if(pack.readFromSerial(mSerial)){
                    cout << pack << endl;
                }else{
                    cout << "No data" << endl;
                }
            }
            catch (std::exception e){
                cout << "Exception: " << e.what() <<endl;
            }
            sleep(2);
        }
        cout << "Done testing ... " << endl;
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
