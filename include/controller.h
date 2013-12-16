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

using namespace Eigen;

namespace USU
{

//#define V_TO_RADS 157.0010928631499 // 5997.0/4.0*2.0*M_PI/60.0
//#define V_TO_CURR 2.522522522522 // 9.0/3.996
#define TIMER_TO_S 62500.0f

#define toCSV(name) name << ','

typedef Matrix<float,3,4> Matrix3x4;

struct pidGains{
    float KP;
    float KI;
    float KD;
};

//For use with trajectory generation
struct pivGains{
    float KP;
    float KI;
    float KV;
};

struct trajFFGains{
    float KVff;  //KV feed-forward  (velocity)
    float KAff;  //KA feed-forward  (acceleration)
};

//Structure to hold the system constants
//All in SI units
struct sysConsts{
    float b;        //Average damping coefficent of wheels
    float Iw;       //lbm-in^2, mass moment of inertia of momentum wheel
    float wn;       //Natural damping frequency
    Matrix3f Inertia; //Moment of inertia matrix
    float hDotMax;  //Wheel torque saturation
    float hMax;     //Wheel momentum saturation
};

//Struct containing reference information
struct refData{
    uint16_t num;   //Reference number (to keep track)
    float time;     //Reference time (when to execute)
    quaternion q;   //Reference quaternion
};

//Struct containing the motor conversion values
struct adcGains{
    float vToRads;
    float vToAmps;
};



class Controller : public PeriodicRtThread
{
    public:
        /** Default constructor */
        Controller(int priority, unsigned int period_us, const char* imuserial = "/dev/ttyO4", const char* i2cDevice = "/dev/i2c-3");

        void initialize();

        virtual void run();

        void stop() { mKeepRunning = false; }

        void setInputFile(const char* inputFile);

        void setLogFile(const char* logFile);

        void sendDutyCycles(Vector4i dc); /*!< Sets the duty cycle for all motors */

        bool readIMU(vector &euler, vector &rates, float &timer);

        void fixAngles(vector &euler); /*!< Fix the rates vector in case the position of the IMU requires it */

        void fixRates(vector &rates); /*!< Fix the rates vector in case the position of the IMU requires it */

        void readMotors(Vector4f &speeds, Vector4f &currents); /*!< Reads the motors and return the scaled values */

//        void controlLaw(); /*!< Implements the control law using the class variables*/

        void readNextReference(); /*!< Reads a new reference from the input file */

        void logData(); /*!< Saves the current state data and readings to memory as CSV*/

        void saveLogData(); /*! Writes the log data to a CSV file */

        static quaternion createQuaternion(vector euler); /*!< Creates a quaternion based on euler angles */

        static quaternion integrateQ(quaternion input, quaternion old_input, quaternion old_output, float delta_time, float gain  = 1);

        static quaternion multiplyQ(quaternion q1, quaternion q2);


        /** Default destructor */
        ~Controller();

    private:

        void readInputFile();               /*!< Reads an the reference input from a file (for now) */
        bool joinIMU();
        void updateStates();                /*!< Updates the members representing "Last" states */

        GX3Communicator mGX3;               /*!< Class for accessing the 3DM-GX3*/
        MotorCommunicator mMotors;          /*!< Class for accessing the motors */
        volatile bool mKeepRunning;         /*!< Indicates if the Thread should keep running. volatile to prevent optimizing */
        SharedQueue<vector> mVectorQueue;   /*!< Queue used to store the vectors from the IMU packet */

        //// Program settings
        float mClock;                       /*!< Program timer (for references and else) */
        float mStopTime;
        float mFirstImuTime;                /*!< Hold the timestamp of the first IMU reading */
        std::ifstream mInputFile;           /*!< Input file with desired gains, constants and references */
        std::ofstream mLogFile;             /*!< Log file to store calculated data */
        std::stringstream mLogBuf;         /*!< Buffer stream to store the data to be saved (stores on memory)*/
        bool mUseInput;                     /*!< Flag to know if we're using an input file */
        bool mLogging;                      /*!< Flag to know if we are saving data */

        //// System settings
        pivGains mPIV;                      /*!< Struct holding the gains for the PIV controller */
        sysConsts mSystem;                  /*!< Struct holdin1g the system constants */
        trajFFGains mFFGains;               /*!< Trajectory generator feed-forward gains */
        uint16_t mTotalRefs;                /*!< Total number of references to be used */
        refData mReference;                 /*!< Current reference input and its data */
        refData mNextReference;             /*!< Next reference input and its data */
        adcGains mMotorScale;               /*!< Struct containing the values to scale the ADC readings to units */

        //// State data
        quaternion mCurrentQuat;            /*!< Current state */
        quaternion mQuatError;              /*!< Current error */
        quaternion mSpeedCmd;               /*!< Current speed command (rad/s)*/

        //Sampled data
        vector mEuler;                      /*!< Current Euler angles */
        vector mCurrentRates;               /*!< Current angular rates */
        Vector4f mSpeed;                    /*!< Current motor speed (rad/s) state for ALL motors */
        float mImuTime;                     /*!< Time stamp of the IMU packet */

        //Storage
        quaternion mLastQuat;               /*!< Last state */
        quaternion mLastQuatError;          /*!< Last error */
        quaternion mLastSpeedCmd;           /*!< Last speed command (rad/s)*/
        vector mLastEuler;                  /*!< Last Euler angles */
        vector mLastRates;                  /*!< Last angular rates */
        float mLastImuTime;                 /*!< Last time stamp of the IMU packet */
        Vector4f mLastSpeed;                /*!< Last motor speed state for ALL motors */
        Vector4f mAmps;                     /*!< Current motor current (Amps) state for ALL motors */


        //// Calculated quantities
        Vector4f mTorque;                   /*!< Current motor torque state for ALL motors */
        Vector4i mDutyC;                    /*!< Current motor duty cycle */

        //Storage
        Vector4f mLastTorque;               /*!< Last motor torque state for ALL motors */
        Vector4f mLastAmps;                 /*!< Last motor current (Amps) state for ALL motors */
        Vector4i mLastDutyC;                /*!< Last motor duty cycle */



        Controller(const Controller& thread); /*!< Copy constructor made inaccessible by declaring it private */
        Controller& operator=(const Controller& rhs); /*!< Assignment constructor made inaccessible by declaring it private */

};

}



#endif // CONTROLLER_H
