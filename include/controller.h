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

namespace USU
{

class Controller : public PeriodicRtThread
{
    public:
        /** Default constructor */
        Controller(int priority, unsigned int period_us, const char* imuserial = "/dev/ttyO4", const char* i2cDevice = "/dev/i2c-3");

        void initialize();

        virtual void run();

        void stop() { mKeepRunning = false; }

        void setInputFile(const char* inputFile);

        void sendDutyCycles(int d0, int d1, int d2, int d3); /*!< Sets the duty cycle for all motors */

        static quaternion createQuaternion(vector euler); /*!< Creates a quaternion based on euler angles */

        static quaternion integrate(quaternion state);

        /** Default destructor */
        ~Controller();
    private:

        void readInput(); /*!< Reads an the reference input from a file (for now) */

        GX3Communicator mGX3; /*!< Class for accessing the 3DM-GX3*/
        MotorCommunicator mMotors; /*!< Class for accessing the motors */
        volatile bool mKeepRunning; /*!< Indicates if the Thread should keep running. volatile to prevent optimizing */

        quaternion mCurrentState; /*!< Current state */

        std::ifstream mInputFile; /*!< Input file with trajectory */
        quaternion mReference; /*!< Reference input state */
        uint16_t mRefTime;  /*!< Timestamp for input state */

        Controller(const Controller& thread); /*!< Copy constructor made inaccessible by declaring it private */
        Controller& operator=(const Controller& rhs); /*!< Assignment constructor made inaccessible by declaring it private */

};

}



#endif // CONTROLLER_H
