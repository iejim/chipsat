#include <iostream>
#include <sys/time.h>

#include "minimu.h"
#include "vector.h"
#include "periodicrtthread.h"

using namespace USU;

class DataCollector : public PeriodicRtThread
{
private:
    MinImu mIMU;
    volatile bool keepRunning;
public:
    DataCollector(int priority, int period_us, const char* i2cdevice)
        :PeriodicRtThread(priority, period_us), mIMU(i2cdevice), keepRunning(false)
    {}
    int timeval_subtract (struct timeval * result, struct timeval * x, struct timeval * y)
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


    virtual void run()
    {
        keepRunning = true;
        mIMU.enable();
        struct timeval start, now, elapsed;
        gettimeofday(&start, NULL);
        
        while(keepRunning)
        {
            vector acc, mag, gyro;
            acc  = mIMU.readAcc();
            mag  = mIMU.readMag();
            gyro = mIMU.readGyro();
            gettimeofday(&now, NULL);
            timeval_subtract(&elapsed, &now, &start);

           unsigned long long timestamp = elapsed.tv_sec * 1000 + elapsed.tv_usec / 1000; // in ms since start
//             unsigned long long timestamp = elapsed.tv_usec;
            std::cout << timestamp << "," << acc[0] << "," << acc[1] << "," << acc[2]
                                   << "," << mag[0] << "," << mag[1] << "," << mag[2]
                                   << "," << gyro[0] << "," << gyro[1] << "," << gyro[2] << std::endl;

            waitPeriod();
        }
    }

    void stop()
    {
        keepRunning = false;
    }

};

int main()
{

    DataCollector dc(5, 20000, "/dev/i2c-2");
    dc.start();
    dc.join();

    return 0;
}
