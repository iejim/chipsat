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

#define _USE_MATH_DEFINES
#include <cmath>
#include <unistd.h>
#include <sys/time.h>

using std::cout;
using std::cerr;
using std::endl;

using namespace Eigen;

namespace USU
{

/*! \brief Constant to convert from IMU timestamp to seconds. */
#define TIMER_TO_S 62500.0f
/*! \brief Adds a ',' next to 'name' in an output stream. */
#define toCSV(name) name << ','

/*! \brief Struct to hold the KP, KI and KD values for a controller using PID. */
struct PidGains{
    float KP; /*!< Proportional gain */
    float KI; /*!< Integral gain */
    float KD; /*!< Derivative gain */
};

/*! \brief Struct to hold KP, KI and KV for a controller using PIV and/or trajectory generation. */
struct PivGains{
    float KP; /*!< Proportional gain */
    float KI; /*!< Integral gain */
    float KV; /*!< Velocity gain */
};
/*! \brief Struct to hold the Feed-Forward gains for a trajectory generator. */
struct TrajFFGains{
    float KVff;   /*!< KV feed-forward (velocity) */
    float KAff;   /*!< KA feed-forward (acceleration) */
};

/*! \brief Struct to hold the system constants. All in SI units. */
struct SysConsts{
    float b;            /*!< Average damping coefficent of wheels */
    float Iw;           /*!< lbm-in^2, mass moment of inertia of momentum wheel */
    float wn;           /*!< Natural damping frequency */
    Matrix3f Inertia;   /*!< Moment of Inertia matrix (3x3)*/
    float motorSpeedMax; /*!< Controller max speed at 90% duty cycle - controller min speed at 10% duty cycle */
    float hDotMax;      /*!< Wheel torque saturation */
    float hMax;         /*!< Wheel momentum saturation */
    float Amax;         /*!< Table maximum acceleration */
    float Vmax;         /*!< Table maximum velocity */
};

/*! \brief Struct to store position reference information. */
struct RefData{
    uint16_t num;   /*!< Reference number (index) */
    float time;     /*!< Reference time (when to execute) */
    quaternion q;   /*!< Reference quaternion (4x1) */
};

/*! \brief Struct containing the motor conversion values from voltages. */
struct AdcGains{
    float vToRads;  /*!< V to Radian/s */
    float vToAmps;  /*!< V to Amps */
};





class Controller : public PeriodicRtThread
{
    public:
        /** Default constructor */
        Controller(int priority, unsigned int period_us, const char* imuserial = "/dev/ttyO4", const char* i2cDevice = "/dev/i2c-3");

        /*! \brief Initializes the settings and values for the controller (IMU, Gains, Motors, etc). */
        void initialize();

        /*! \brief This is where the Control Law is run. */
        virtual void run();

        /*! \brief Stops the loop in the thread.

            Stopping the thread should end the contoller.
        */
        void stop() { mKeepRunning = false; }

        /*! \brief Opens the specified input file so it can be read. */
        void setInputFile(const char* inputFile);

        /*!  \brief Opens the specified file so the CSV log data can be saved. */
        void setLogFile(const char* logFile);

        /*! \brief Sets the duty cycle for all motors */
        void sendDutyCycles(Vector4i dc);

        /*! \brief Recovers the last data packet sent by the IMU */
        bool readIMU(vector &euler, vector &rates, float &timer);

        /*! \brief Fix the rates vector in case the position of the IMU requires it */
        void fixAngles(vector &euler);

        /*! \brief Fix the rates vector in case the position of the IMU requires it */
        void fixRates(vector &rates);

        /*! \brief Reads the motors and return the scaled values */
        void readMotors(Vector4f &speeds, Vector4f &currents);

        /*! \brief Implements the control law structure and logic using the class variables */
        void controlLaw();

        /*! \brief Trajectory setup outputs TIME, etc. used by trajtory generator*/
        void  trajectorySetup(float tstart, quaternion reference, quaternion qinit, quaternion &q0, float &angle, vector &axis, vector4f &time);

        /*! \brief trajectory generator outputs mQuatStar, mOmegaStar, mAlphaStar*/
        void trajectoryGenerator(quaternion q0, float angle, vector axis, vector4f time);

        /*! \brief Reads the next Reference Command line from the input file. */
        void readNextReference();

        /*! \brief Stores the CSV runtime data in memory for saving. */
        void logData();

        /*! \brief Writes the log data to the CSV log file. */
        void saveLogData();

        /*! \brief Creates a quaternion based on euler angles */
        static quaternion createQuaternion(vector euler);

        /*! \brief Performs quaternion vector integration. */
        static quaternion integrateQ(quaternion in, quaternion old_in, quaternion old_out, float delta_time, float gain  = 1);

        /*! \brief Multiply two quaternion vectors */
        static quaternion multiplyQ(quaternion q1, quaternion q2);

        /*! \brief Performs First-order filtering based on the supplied time constant */
        static quatertion firstOrderFilterQ(quaternion in, quaternion old_in, quaternion old_out, float delta_time, float w_c  = 2*pi);

        /** Default destructor */
        ~Controller();

    private:

        void readInputFile();               /*!< \brief Reads the input file with the system settings. */
        bool joinIMU();                     /*!< \brief Joins the IMU thread and waits for its termination. */
        void updateStates();                /*!< \brief Updates the members representing "Last" (previous) state. */

        GX3Communicator mGX3;               //!< Class for accessing the 3DM-GX3.
        MotorCommunicator mMotors;          //!< Class for accessing the motors.
        volatile bool mKeepRunning;         //!< Indicates if the Thread should keep running. volatile to prevent optimizing
        SharedQueue<vector> mVectorQueue;   //!< Queue used to store the vectors from the IMU packet.

        //// Program settings
        float mClock;                       //!< Program timer (for references and else).
        float mStopTime;
        float mFirstImuTime;                //!< Hold the timestamp of the first IMU reading.
        std::ifstream mInputFile;           //!< Input file with desired gains, constants and references.
        std::ofstream mLogFile;             //!< Log file to store calculated data.
        std::stringstream mLogBuf;          //!< Buffer stream to store the data to be saved (stores on memory).
        bool mUseInput;                     //!< Flag to know if we're using an input file.
        bool mLogging;                      //!< Flag to know if we are saving data.

        // System settings
//        PidGains mPID;                      //!< Struct holding the gains for a PID controller.
        PivGains mPIV;                      //!< Struct holding the gains for the PIV controller.
        SysConsts mSystem;                  //!< Struct holding the system constants.
        TrajFFGains mFFGains;               //!< Trajectory generator feed-forward gains.
        uint16_t mTotalRefs;                //!< Total number of references to be used.
        RefData mReference;                 //!< Current reference input and its data.
        RefData mNextReference;             //!< Next reference input and its data.
        AdcGains mMotorScale;               //!< Struct containing the values to scale the ADC readings to units.

        // State data
        quaternion mCurrentQuat;            //!< Current state, measured.
        quaternion mQuatError;              //!< Current error, calculated.
        quaternion mSpeedCmd;               //!< Current speed command (rad/s), calculated.
        quaternion mQuatStar;               //!< Current quaternion command from trajectory generation, calculated.
        vector mOmegaStar;                  //!< Current speed command from trajectory generation, calculated.
        vector mAlphaStar;                  //!< Current acceleration command from trajectory generation, calculated.

        // Sampled data
        vector mEuler;                      //!< Current Euler angles.
        vector mCurrentRates;               //!< Current angular rates.
        Vector4f mSpeed;                    //!< Current motor speed (rad/s) state for ALL motors.
        float mImuTime;                     //!< Time stamp of the IMU packet.

        // Storage
        quaternion mLastQuat;               //!< Last state.
        quaternion mLastQuatError;          //!< Last error.
        quaternion mLastSpeedCmd;           //!< Last speed command (rad/s).
        vector mLastEuler;                  //!< Last Euler angles.
        vector mLastRates;                  //!< Last angular rates.
        float mLastImuTime;                 //!< Last time stamp of the IMU packet.
        Vector4f mLastSpeed;                //!< Last motor speed state for ALL motors.
        Vector4f mAmps;                     //!< Current motor current (Amps) state for ALL motors.

        // Calculated quantities
        quaternion mQuatErrorI;             //!< Integrated error, calculated.
        vector mTc3;                        //!< Current 3-axis torque.
        Vector4f mTorque;                   //!< Current motor torque state for ALL motors.
        Vector4i mDutyC;                    //!< Current motor duty cycle.

        // Storage
        quaternion mLastQuatErrorI;         //!< Last integrated current error.
        Vector4f mLastTorque;               //!< Last motor torque state for ALL motors.
        Vector4f mLastAmps;                 //!< Last motor current (Amps) state for ALL motors.
        Vector4i mLastDutyC;                //!< Last motor duty cycle.



        Controller(const Controller& thread); /*!< Copy constructor made inaccessible by declaring it private. */
        Controller& operator=(const Controller& rhs); /*!< Assignment constructor made inaccessible by declaring it private. */

};

}



#endif // CONTROLLER_H
