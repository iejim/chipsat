#include<csignal>
#include<cstdlib>
#include<unistd.h>

#include <iostream>
#include <string>
#include <exception>
#include <stdexcept>
using std::string;
using std::cout;
using std::endl;
using std::cerr;

#include "tclap/CmdLine.h"
#include "controller.h"

//#include "testSamplingSettings.cpp"
using namespace USU;


// Parse the command line arguments
// Define possible arguments

TCLAP::CmdLine cmd("Program for the attitude determination and control of the USU simulation table",' ', "0.1");

TCLAP::ValueArg<string> inputFile("i", "inputFile", "Input file with desired gains, constants and references", true, "input.txt", "filename");
TCLAP::ValueArg<string> logFile("o", "logFile", "Log file to store the generated data", false, "datalog.csv", "filename");

// Example for switching arg
//TCLAP::SwitchArg stats("s", "stats", "Print statistics (number of spots, number of identified spots, ratio");

//Controller control(5, 20000, "/dev/ttyS0", "/dev/i2c-1"); //ninkasi
Controller control(5, 20000);



void endProgram(int s)
{
    cerr << "MAIN: Got signal for termination" << endl;
    cerr << "MAIN: Stopping controller thread..." << endl;
    control.stop();
}


int main(int argc, char **argv)
{
    // Register endProgram function as
    // signal handler for the kill signal (ctrl+c)
    cerr << "MAIN: Begin ..." << endl;
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = endProgram;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    try
    {
        // Register commandline options to parser
        cmd.add(inputFile);
        cmd.add(logFile);
        cmd.parse(argc, argv);

        control.setInputFile(inputFile.getValue().data());

        if(!logFile.isSet())
            cerr << "Logging to default file 'datalog.csv'..." << endl;
        control.setLogFile(logFile.getValue().data());

        cerr  << "Initializing..." << endl;
        control.initialize();

        cerr << "Start ..."  << endl;
        control.start();

        if (control.join()) //Three second timeout, in case any thread dies first
        {
            cerr << "Thread joined" << endl;
            cerr << "MAIN: Terminating ... "<< endl;
            return 0;
        } else
        {
            cerr << "Thread joining failed" << endl;
            cerr << "MAIN: Terminating ... "<< endl;
            return 1;
        }

////////////////////////////
//        std::cerr << "MAIN: Creating command list ..." << std::endl;
////        uint8_t commandList[3] = {EULER_ANGLES, QUATERNION, ORIENTATION_MATRIX};
//        uint8_t commandList = EULER_ANGLES_ANG_RATES;
//        imuMonitor.setCommandList(&commandList, 1);
//        std::cerr << "MAIN: Start Monitor ..." << std::endl;
//        imuMonitor.setContinuousMode();
//        imuMonitor.start();
//
//        if(imuMonitor.join() )
//        {
//            std::cerr << "MAIN: IMU Monitor thread joined" << std::endl;
//            std::cerr << "MAIN: Terminating now..." << std::endl;
//            return 0;
//        }
//        else
//        {
//            std::cerr << "MAIN: Joining IMU Monitor thread failed" << std::endl;
//            std::cerr << "MAIN: Terminating now..." << std::endl;
//            return 1;
//        }
///////////////////////////////



    } catch (std::exception e)  // catch any exceptions
    {
        cerr << "MAIN: Error: " << e.what() << endl;
        return 1;
    }

}
