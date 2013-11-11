#include<csignal>
#include<cstdlib>
#include<unistd.h>

#include <iostream>
#include <string>
#include <exception>
using std::string;

#include "tclap/CmdLine.h"
#include "gx3monitor.h"
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


GX3Monitor imuMonitor(5, 20000, "/dev/tty04");
//bool run = true;

void endProgram(int s)
{
    std::cerr << "MAIN: Got signal for termination" << std::endl;
    std::cerr << "MAIN: Stopping monitor thread..." << std::endl;
    imuMonitor.stop();
}


int main(int argc, char **argv)
{
    // Register endProgram function as
    // signal handler for the kill signal (ctrl+c)
    std::cerr << "MAIN: Begin ..." << std::endl;
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = endProgram;
    std::cerr << "MAIN: 1 ..." << std::endl;
    sigemptyset(&sigIntHandler.sa_mask);
    std::cerr << "MAIN: 2 ..." << std::endl;
    sigIntHandler.sa_flags = 0;
    std::cerr << "MAIN: 3 ..." << std::endl;
    sigaction(SIGINT, &sigIntHandler, NULL);
    std::cerr << "MAIN: Try ..." << std::endl;
    try
    {
        // Register commandline options to parser
//        cmd.add(trajFile);
//        cmd.add(pgain);
//        cmd.add(mode);
//
//        cmd.parse(argc, argv);
        std::cerr << "MAIN: Creating command list ..." << std::endl;
        uint8_t commandList[3] = {EULER_ANGLES, QUATERNION, ORIENTATION_MATRIX};
        imuMonitor.setCommandList(commandList, 3);

        /*imuMonitor.start();

        if(imuMonitor.join() )
        {
            std::cerr << "MAIN: IMU Monitor thread joined" << std::endl;
            std::cerr << "MAIN: Terminating now..." << std::endl;
            return 0;
        }
        else
        {
            std::cerr << "MAIN: Joining IMU Monitor thread failed" << std::endl;
            std::cerr << "MAIN: Terminating now..." << std::endl;
            return 1;
        }

        */
    } catch (exception &e)  // catch any exceptions
    {
        std::cerr << "error: " << e.error() std::endl;
        return 1;
    }

}
