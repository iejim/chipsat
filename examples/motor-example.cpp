/**
 * @file motor-example.cpp
 *
 * Example program to run a trajectory of the motors
 * defined in an input file and collect the analog measurements for each motor.
 *
 * @author Jan Sommer
 *  Created on: July, 13 2013
 *
 */


#include <iostream>
using std::cout;
using std::endl;

#include "motorprogrammer.hpp"


/*!
 \brief The main program.

 Parses the arguments and creates a MotorProgrammer object
 and starts the test routine.


 \param argc
 \param argv[]
 \return int
*/
int main(int argc, char *argv[])
{

    if (argc != 3 && argc != 2)
    {
        cout << "Wrong number of arguments" << endl;
        cout << "Usage:" <<endl;
        cout << "motor-test <input file> [<output file>]" << endl;
        cout << "Terminating" << endl;
        return 1;
    }

    const char * inputFile = argv[1];
    const char * outputFile;
    if (argc == 3)
    {
        outputFile = argv[2];
    }
    else
    {
        outputFile = "output.csv";
    }


    MotorProgrammer mp(6, inputFile, outputFile, 10000);

    mp.start();

    mp.join();

   return 0;
}



