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

#include "tclap/CmdLine.h"
#include "controller.h"

//#include "testSamplingSettings.cpp"
using namespace USU;

/*
// Text to explain the different modes (more elegant way to split strings over several lines?)
const string modeText = string("Operation mode: \n\t") +
                                 string("- pololu: Collect data from Pololu IMU and print it in csv format\n\t") +
                                 string("- microstrain: Collect data from MicroStrain IMU and print it in csv format\n\t") +
                                 string("- collect: Collect data from both IMUs and print it in csv format\n\t") +
                                 string("- simpleControl: Run simple angular velocity control scheme");

// Parse the command line arguments
// Define possible arguments
1
TCLAP::CmdLine cmd("Program for the attitude determination and control of the USU simulation table",' ', "0.1");

TCLAP::ValueArg<string> trajFile("", "trajfile", "Input file for the trajectory the table should follow", false, "input.txt", "filename");
TCLAP::ValueArg<float> pgain("", "pgain", "The P-Gain for the simple proportional speed controller", false, 1.0, "float");
TCLAP::ValueArg<string> mode("", "mode",  modeText , true, string(), "mode name");
*/
// Example for switching arg
//TCLAP::SwitchArg stats("s", "stats", "Print statistics (number of spots, number of identified spots, ratio");

Controller control(5, 20000);


void endProgram(int s)
{
    std::cerr << "MAIN: Got signal for termination" << std::endl;
    std::cerr << "MAIN: Stopping controller thread..." << std::endl;
    control.stop();
}


int main(int argc, char **argv)
{
    // Register endProgram function as
    // signal handler for the kill signal (ctrl+c)
    std::cerr << "MAIN: Begin ..." << std::endl;
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = endProgram;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    try
    {
        // Register commandline options to parser
//        cmd.add(trajFile);
//        cmd.add(pgain);
//        cmd.add(mode);
//6
//        cmd.parse(argc, argv);
//////////////////////
        cout  << "Initializing..." << endl;
        control.initialize();

        cout << "Start ..."  << endl;
        control.start();

        if (control.join())
        {
            cout << "Thread joined" << endl;
            cout << "MAIN: Terminaning ... "<< endl;
            return 0;
        } else
        {
            cout << "Thread joining failed" << endl;
            cout << "MAIN: Terminaning ... "<< endl;
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
        std::cerr << "MAIN: Error: " << e.what() << std::endl;
        return 1;
    }

}
