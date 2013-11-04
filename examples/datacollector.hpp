#ifndef DATA_COLLECTOR_H
#define DATA_COLLECTOR_H

#include <sys/time.h>
//#include <unistd.h>

#include "periodicrtthread.h"
#include "motorcontrol.h"
using namespace USU;

/*!
 \brief Simple class which manages the motors and collects data at an periodic intervall

    Inherited from PeriodicRtThread
*/
class DataCollector: public PeriodicRtThread
{
public:

    /*!
     \brief Constructor

     Sets up the underlying PeriodicRtThread.

    \param priority Priority of the PeriodicRtThread
    \param period_us sampling period (in us) of the PeriodicRtThread
     \param filename  Filename of the output file
    \param motors     Reference to the MotorControl object for accessing the ADC
    */
    DataCollector(int priority, unsigned int period_us, const char *filename, MotorControl& motors);

    /*!
     \brief Runs the sampling loop

     Reads the channels and set speeds of all 4 motors and
     prints the results to the output file.

    */
    virtual void run();

    volatile bool mKeepRunning; /*!<  Possibility to interrupt thread */

private:

    /*!
     \brief Substract 2 timeval structures

     result = x - y;

     \param result
     \param x
     \param y
     \return int 1 if result is negative
    */
    int timeval_subtract (struct timeval * result, struct timeval * x, struct timeval * y);

    MotorControl& mMotors; /*!< Object to access the motors for ADC measurements */
    const char * mFilename; /*!< Filename of the output file */

};

/*!
 \brief

 \param priority
 \param period_us
 \param filename
 \param motors
*/
DataCollector::DataCollector::DataCollector(int priority, unsigned int period_us, const char* filename, MotorControl &motors)
    :PeriodicRtThread(priority, period_us), mMotors(motors), mFilename(filename)
{}


void DataCollector::run()
{
    mKeepRunning = true;

    struct timeval start, now, elapsed;

    // open output file
    std::fstream file;
    file.open(mFilename, std::ios_base::out);

    // get initial clock value
    gettimeofday(&start, NULL);

    while(mKeepRunning)
    {
        gettimeofday(&now, NULL);

        // read analog sensor
        float a1[4], a2[4];
        mMotors.getAnalogs(a1, a2);

        int dutyCycles[4];
        mMotors.getDutyCycles(dutyCycles);

        // Compute time stamp
        timeval_subtract(&elapsed, &now, &start);
        unsigned long long timestamp = elapsed.tv_sec * 1000 + elapsed.tv_usec / 1000; // in ms since start


        //print line of data to output file
        file << timestamp << "," << a1[0] << "," << a2[0] << "," << dutyCycles[0]
                          << "," << a1[1] << "," << a2[1] << "," << dutyCycles[1]
                          << "," << a1[2] << "," << a2[2] << "," << dutyCycles[2]
                          << "," << a1[3] << "," << a2[3] << "," << dutyCycles[3] <<  std::endl;

        waitPeriod();
    }

    file.close();
}

int DataCollector::timeval_subtract(timeval *result, timeval *x, timeval *y)
{
    /* Perform the carry for the later subtraction by updating y. */
    if (x->tv_usec < y->tv_usec) {
        int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
        y->tv_usec -= 1000000 * nsec;
        y->tv_sec += nsec;
    }
    if (x->tv_usec - y->tv_usec > 1000000) {
        int nsec = (x->tv_usec - y->tv_usec) / 1000000;
        y->tv_usec += 1000000 * nsec;
        y->tv_sec -= nsec;
    }

    /* Compute the time remaining to wait.
          tv_usec is certainly positive. */
    result->tv_sec = x->tv_sec - y->tv_sec;
    result->tv_usec = x->tv_usec - y->tv_usec;

    /* Return 1 if result is negative. */
    return x->tv_sec < y->tv_sec;
}

#endif
