#ifndef MOTORCOMMUNICATOR_H
#define MOTORCOMMUNICATOR_H

#include <vector>

#include "cPWM.h"
#include "Beagle_GPIO.h"
#include "motor.h"
#include "max127.h"
#include "vector.h"
#include "messages.h"


namespace USU
{

class MotorCommunicator
{
public:
    /** Default constructor */
    MotorCommunicator(const char* i2cDevice="/dev/i2c-3");

    /*!
     \brief Sets the speed of a motor

     \param motor which motor [0..3]
     \param dutyCycle  which speed [-100..100]
    */
    void setMotor(int motor, int dutyCycle);

    /*!
     \brief Returns the Analog measurements of a motor

     \param motor which motor [0..3]
     \param aOut1   reference to a variable to store the first analog measurement
     \param aOut2   reference to a variable to store the second analog measurement
    */
    void getAnalog(int motor, float &aOut1, float &aOut2);

    /*!
     \brief Returns the Analog measurements of all motors

     \param aOut1 Float array to store the first analog measurement of each motor
     \param aOut2 Float array to store the second analog measurement of each motor
    */
    void getAnalogs(float * aOut1, float* aOut2);

    /*!
     \brief Returns the dutycycles of all motors

     \param dc Int array to store the duty cycle of each motor
    */
    void getDutyCycles(int* dc);

    /** Default destructor */
    ~MotorCommunicator();

private:


    cPWM mPwm1; /*!< First PWM module (has 2 channels) */
    cPWM mPwm2; /*!< Second PwM module (has 2 channels) */

    Beagle_GPIO mBeagleGpio; /*!< Representation of the BeagleBoard Gpios (access via mmap) */
    Motor *mMotor[4]; /*!< Array of the 4 motors */

    Max127 mAnalog; /*!< Representation of the ADC for the motor feedback */

    MotorCommunicator(const MotorCommunicator& thread); /*!< Copy constructor made inaccessible by declaring it private */

    MotorCommunicator& operator=(const MotorCommunicator& rhs); /*!< Assignment constructor made inaccessible by declaring it private */
};

}
#endif // MOTORCOMMUNICATOR_H
