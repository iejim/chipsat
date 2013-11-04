/**
 * @file MotorProgrammer.hpp
 *
 * Runs the given trajectory for the 4 motors.
 *
 * Based on RtThread
 *
 * @author Jan Sommer
 *  Created on: Apr 26, 2013
 *
 */


#ifndef MOTOR_PROGRAMMER_H
#define MOTOR_PROGRAMMER_H

#include <time.h>
#include <sys/timerfd.h>
#include <unistd.h>
#include <cerrno>
#include <stdexcept>

#include <queue>
#include <fstream>
#include <iostream>
using std::cout;
using std::endl;

#include "RtThread.h"
#include "motorcontrol.h"
#include "datacollector.hpp"
using namespace USU;

/*!
 \brief Class which reads the input file and runs the trajectory for each motor
*/
class MotorProgrammer: public RtThread
{
public:
    /*!
     \brief Struct representing a single command point

     At the point time the corresponding motor will be
     set to the desired speed

    */
    struct Command
    {
        unsigned int time; /*!< Time (in ms) from start*/
        unsigned int motor; /*!< The motor ought to be set [0,3] */
        int speed; /*!< The speed the motor will be set to at time*/
    };

    /*!
    \brief Constructor

    Prepares the underlying thread.
    Parses the input file.


    \param priority Priority for the underlying RtThread
    \param inputFile    filename of the input file
    \param outputFile   filename of the output file
    \param period_us    sampling time (in us) for the data collector thread
    */
    MotorProgrammer(int priority, const char* inputFile, const char *outputFile, unsigned int period_us);

    /*!
     \brief Starts the thread

     Sets the motors speeds according go the input file

    */
    virtual void run();

    volatile bool mKeepRunning; /*!< Possibility to interrupt thread */

private:
    int mTimerFd; /*!< Handle to the timer */
    struct timespec mStartTime; /*!< Start time all other times are relative to*/
    std::queue<Command> mCommandQueue; /*!< The list with the commands*/
    MotorControl mMotors; /*!< Object which controls the motors*/
    DataCollector mDataCollector; /*!< Thread which collects the data periodically*/

};


MotorProgrammer::MotorProgrammer(int priority, const char *inputFile,
                                 const char* outputFile, unsigned int period_us)
    :RtThread(priority), mDataCollector(priority-1, period_us, outputFile, mMotors)
{
    // Initialize the timer
    mTimerFd = timerfd_create (CLOCK_MONOTONIC, 0);
    if ( mTimerFd == -1)
    {
        perror("timer_create ");
        return;
    }


    // Read input file
    std::fstream file;
    file.open(inputFile, std::ios_base::in);

    Command temp;
    unsigned lastTime = 0;
    while(true)
    {
        file >> temp.time;
        if (temp.time < lastTime)
            throw std::runtime_error("Error while parsing input file: Commands have to be sorted chronologically");
        lastTime = temp.time;

        file >> temp.motor;
        if(temp.motor >3)
            throw std::runtime_error("Error while parsing input file: Motor has to be between 0 and 3");

        file >> temp.speed;

        if(file.eof())
            break;
        mCommandQueue.push(temp);
    }

    file.close();

    cout << "Read " << mCommandQueue.size() << " commands." << endl;
}

void MotorProgrammer::run()
{
    // Get the start time
    if (clock_gettime(CLOCK_MONOTONIC, &mStartTime) == -1)
    {
        perror("clock_gettime ");
        return;
    }


    cout << "Starting data collector " << endl;
    mDataCollector.start();

    //    1. Read the next event from the list.
    //    2. Create a timer signal for the time.
    //    3. Wait for the timer.
    //    4. Set the motor(s) to the desired duty cycle
    //    5. Go back to 1
    mKeepRunning = true;
    while(mKeepRunning && !mCommandQueue.empty())
    {

        Command nextEvent = mCommandQueue.front();

        // Set the timer event:
        struct itimerspec nextTimer;
        // single event (no period)
        nextTimer.it_interval.tv_sec  = 0;
        nextTimer.it_interval.tv_nsec = 0;

        // next alarm in ms from mStartTime
        int sec  = nextEvent.time /1000;
        int nsec = (nextEvent.time - sec*1000) * 1000000;

        nextTimer.it_value.tv_sec  = mStartTime.tv_sec  + sec;
        nextTimer.it_value.tv_nsec = mStartTime.tv_nsec + nsec;

        // check for overflow
        if (nextTimer.it_value.tv_nsec >= 1000000000)
        {
            nextTimer.it_value.tv_sec++;
            nextTimer.it_value.tv_nsec -= 1000000000;
        }

        int ret;
        // start timer
        if ( (ret = timerfd_settime(mTimerFd, TFD_TIMER_ABSTIME, &nextTimer, NULL)) != 0)
        {
            perror("timerfd_settime ");
            return;
        }

        // wait for timer
        unsigned long long buffer;
        if ( (ret = read(mTimerFd, &buffer, sizeof(buffer) )) == -1 )
        {
            perror ("timer read ");
            return;
        }

        // Do command

        // do{} while -loop would make more sense?
        mMotors.setMotor(nextEvent.motor, nextEvent.speed);
        mCommandQueue.pop();

        cout << "Motor: " << nextEvent.motor
             << "\tDutyCycle: " << nextEvent.speed << endl;

        // Do subsequent commands (if any)
        while(nextEvent.time == mCommandQueue.front().time)
        {
            nextEvent = mCommandQueue.front();
            mMotors.setMotor(nextEvent.motor, nextEvent.speed);
            mCommandQueue.pop();
            cout << "Motor: " << nextEvent.motor
                 << "\tDutyCycle: " << nextEvent.speed << endl;
        }
    }

    cout << "Finished commandlist" << endl;
    cout << "Stopping data collector" << endl;
    mDataCollector.mKeepRunning = false;
    mDataCollector.join();
    cout << "Data collector joined" << endl;
    cout << "Terminating" << endl;
}

#endif
