/**
 * @file gx3monitor.h
 *
 * C++ class used to monitor the data output
 * of the GX3 IMU.
 * Based on the PeriodicRtThread class.
 *
 * @author Ivan Jimenez
 *  Created on: Nov 03, 2013
 *
 */

#ifndef GX3MONITOR_H_INCLUDED
#define GX3MONITOR_H_INCLUDED

#include "periodicrtthread.h"
#include "minimu.h"
#include "Lock.h"
#include "gx3communicator.h"
#include "messages.h"
//#include "motorcontrol.h"

namespace USU
{

/*!
 \brief Represents the Periodic Thread class for state estimation

 This class is derived from PeriodicRtThread. It initializes the interface to the
 3DM-25 IMU and reads the requested data.
 The data can be accessed from other threads (protected by mutex).

 The class allows requesting for different combinations of outputs:
    - Euler angles
    - Euler angles and angular rates
    - Quaternions
    - Acceleration, angular rates and Orientation Matrix
    - Orientation Matrix

 These outputs can be combined freely - so the user needs remember that
 some combinations would send the same data twice.

*/
class GX3Monitor : public PeriodicRtThread
{
public:

    /*!
     \brief Constructor of the class

     Initializes the interface to the MinIMU9 sensors and
     to the 3DM-GX3.
     Sets up the motor controller.

     \param priority    priority of the underlying periodic thread
     \param period_us   period (in us) of the underlying periodic thread
     \param imuserial   path to serial port the IMU is at (e.g. /dev/ttyUSBn)
    */
    GX3Monitor(int priority, unsigned int period_us, const char* imuserial);

    /*!
     \brief Thread routine

     Connects to the IMU and reads the requested data from the serial line.

    */
    virtual void run();

    /*!
     \brief Signals the thread to stop

    */
    void stop() { mKeepRunning = false; }

    /*!
     \brief Returns the current system state estimate

     Copies the current system state estimate. Acquires
     mutex before acessing the internal variable to avoid
     read/write-conflicts.

     \return bool Current system state
     TODO: Currently only dummy variable. Replace with actual state representation (quaternion?)
           Probably not necessary anymore
    */
    bool getState();

    void setContinuous(bool runCont = true);
    void setCommandList(uint8_t* cList, uint8_t cNum);

private:



    Mode mMode;

    /*!
     \brief Struct representing a single command point

     At the point time the corresponding motor will be
     set to the desired speed

    */
    struct Command
    {
        unsigned int time; /*!< Time (in ms)*/
        Eigen::Vector3f angVel; /*!< The angular velocity set until the command is processed*/
    };
    std::vector<Command> mCommandList; /*!< The list with the commands*/

    GX3Communicator mGX3;  /*!< Class for accessing the 3DM-GX3*/

    ///TODO: queue to ethernet

    Lock mStateLock; /*!< Mutex to access the State variable*/
    volatile bool mKeepRunning; /*!< Indicates if the Thread should keep running. volatile to prevent optimizing */


    GX3Monitor(const GX3Monitor& thread); /*!< Copy constructor made inaccessible by declaring it private */
    GX3Monitor& operator=(const GX3Monitor& rhs); /*!< Assignment constructor made inaccessible by declaring it private */
};

}



#endif // GX3MONITOR_H_INCLUDED
