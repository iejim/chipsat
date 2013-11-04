/**
 * @file kalmanfilter.h
 *
 * C++ class for the sensor fusion and state estimated.
 * Based on the PeriodicRtThread class.
 *
 * @author Jan Sommer
 *  Created on: Apr 20, 2013
 *
 */



#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "periodicrtthread.h"
#include "minimu.h"
#include "Lock.h"
#include "gx3communicator.h"
#include "messages.h"
#include "motorcontrol.h"

namespace USU
{

/*!
 \brief Represents the Periodic Thread class for state estimation

 This class is derived from PeriodicRtThread. It initializes the interface to the
 MinIMU9v2 and estimates the system state using Kalman filtering techniques. The state
 estimate can be accessed from other threads (protected by mutex).

 TODO:
    - Implement kalman filter for state estimate
    - change name to something more meaningful?
*/
class KalmanFilter : public PeriodicRtThread
{
public:
    enum Mode
    {
        SimpleControl,
        CollectPololuData,
        CollectMicroStrainData,
        CollectData
    };

    /*!
     \brief Constructor of the class

     Initializes the interface to the MinIMU9 sensors and
     to the 3DM-GX3.
     Sets up the motor controller.

     \param priority    priority of the underlying periodic thread
     \param period_us   period (in us) of the underlying periodic thread
     \param i2cImu      name of the I2C-device for the IMU (e.g. /dev/i2c-1)
     \param i2cMotor    name of the I2C-device for the Motors (e.g. /dev/i2c-2)
    */
    KalmanFilter(int priority, unsigned int period_us, const char* i2cImu, const char *i2cMotor);

    /*!
     \brief Thread routine

     Current scenario is:
        - Get quaternion data from MicroStrain at constant rate
        - Hand this state estimate to the motor controller.

        TODO: Develop scenario using Kalman-Filter
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

    void initializeModeSimpleControl(std::string trajFilename, float pgain);

    Mode getMode() const;
    void setMode(const Mode &value);

private:
    void runSimpleControl();

    void runCollectPololu();

    void runCollectMicroStrain();

    void runCollectBoth();

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


    MinImu mImu; /*!< Class for accessing the MinIMU9v2*/
    GX3Communicator mGX3;  /*!< Class for accessing the 3DM-GX3*/
    MotorControl mMotors;

    ///TODO: queue to ethernet
    bool mState; ///TODO: Actual representation of the state estimate /*!< TODO */
    Lock mStateLock; /*!< Mutex to access the State variable*/
    volatile bool mKeepRunning; /*!< Indicates if the Thread should keep running. volatile to prevent optimizing */


    KalmanFilter(const KalmanFilter& thread); /*!< Copy constructor made inaccessible by declaring it private */
    KalmanFilter& operator=(const KalmanFilter& rhs); /*!< Assignment constructor made inaccessible by declaring it private */
};

}

#endif // KALMANFILTER_H
