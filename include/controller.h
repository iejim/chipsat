#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdexcept>
#include <fstream>
#include "sharedqueue.h"
#include "vector.h"

#include "periodicrtthread.h"
#include "gx3communicator.h"
#include "messages.h"
#include "motorcommunicator.h"
#include "messages.h"

#define _USE_MATH_DEFINES
#include <cmath>

using Eigen::Vector4f;
using Eigen::Vector4i;

namespace USU
{

#define V_TO_RADS 157.0010928631499 // 5997.0/4.0*2.0*M_PI/60.0
#define V_TO_CURR 2.522522522522 // 9.0/3.996


class Controller : public PeriodicRtThread
{
    public:
        /** Default constructor */
        Controller(int priority, unsigned int period_us, const char* imuserial = "/dev/ttyO4", const char* i2cDevice = "/dev/i2c-3");

        void initialize();

        virtual void run();

        void stop() { mKeepRunning = false; }

        void setInputFile(const char* inputFile);

        void sendDutyCycles(Vector4i dc); /*!< Sets the duty cycle for all motors */

        bool readIMU(vector &euler, vector &rates);

        void readMotors(Vector4f &speeds, Vector4f &currents); /*!< Reads the motors and return the scaled values */

//        void controlLaw(); /*!< Implements the control law using the class variables*/

        static quaternion createQuaternion(vector euler); /*!< Creates a quaternion based on euler angles */

        static quaternion integrate(quaternion state);

        /** Default destructor */
        ~Controller();

    private:

        void readInput(); /*!< Reads an the reference input from a file (for now) */
        bool joinIMU();

        GX3Communicator mGX3;               /*!< Class for accessing the 3DM-GX3*/
        MotorCommunicator mMotors;          /*!< Class for accessing the motors */
        volatile bool mKeepRunning;         /*!< Indicates if the Thread should keep running. volatile to prevent optimizing */

        SharedQueue<vector> mVectorQueue;   /*!< Queue used to store the vectors from the IMU packet */


        quaternion mCurrentQuat;            /*!< Current state */
        quaternion mQuatError;              /*!< Current error */
        vector mEuler;                      /*!< Current Euler angles */
        vector mCurrentRates;               /*!< Current angular rates */
        Vector4f mSpeed;                    /*!< Current motor speed (rad/s) state for ALL motors */

        quaternion mLastQuat;               /*!< Last state */
        quaternion mLastQuatError;          /*!< Last error */
        vector mLastEuler;                  /*!< Last Euler angles */
        vector mLastRates;                  /*!< Last angular rates */
        Vector4f mLastSpeed;                /*!< Last motor speed state for ALL motors */

        Vector4f mTorque;                   /*!< Current motor torque state for ALL motors */
        Vector4f mAmps;                     /*!< Current motor current (Amps) state for ALL motors */
        Vector4i mDutyC;                    /*!< Current motor duty cycle */

        Vector4f mLastTorque;               /*!< Last motor torque state for ALL motors */
        Vector4f mLastAmps;                 /*!< Last motor current (Amps) state for ALL motors */
        Vector4i mLastDutyC;                /*!< Last motor duty cycle */


        std::ifstream mInputFile;           /*!< Input file with trajectory */
        quaternion mReference;              /*!< Reference input state */
        uint16_t mRefTime;                  /*!< Timestamp for input state */

        Controller(const Controller& thread); /*!< Copy constructor made inaccessible by declaring it private */
        Controller& operator=(const Controller& rhs); /*!< Assignment constructor made inaccessible by declaring it private */

};

}



#endif // CONTROLLER_H
