/**
 * @file motorcontrol.h
 *
 * C++ class for the calculation of the control response.
 * Based on the PeriodicRtThread class.
 *
 * @author Jan Sommer
 *  Created on: Apr 22, 2013
 *
 */

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <vector>

#include "cPWM.h"
#include "Beagle_GPIO.h"
#include "motor.h"
#include "max127.h"
#include "messages.h"

namespace USU
{

/*!
 \brief Represents the class for motor control

 It initializes the interface to the 4 motors. It receives the last system state estimate from the Kalman
 filter, calculates the appropiate control response and sets the speed (duty cycle)
 of the motors.

 TODO: Get the desired state from ground station to calculate the control response.

 \ingroup motor

*/
class MotorControl
{
public:


    /*!
     \brief Constructor of the class

     Initializes the underlying GPIO-class,
     the PWMs, the 4 Motors and the ADC.

     \param i2cDevice  name of the i2cDevice of the ADC
    */
    MotorControl(const char* i2cDevice="/dev/i2c-3");

    virtual ~MotorControl();

    /*!
     \brief Calculate the control response from the current state estimate

     TODO: Doesn't do anything at the moment

     \param state the current state estimate from the IMU
   */
    void calculateControlResponse(Quaternion state);

    /*!
     \brief Uses a simple algorithm to control the speed only from gyro data

     \param gyro Vector with the current angular rates
    */
    void controlFromGyro(Eigen::Vector3f gyro);

    /*!
     \brief For testing: sets the speed of a motor

     \param motor which motor [0..3]
     \param dutyCycle  which speed [-100..100]
    */
    void setMotor(int motor, int dutyCycle);


    /*!
     \brief For testing: returns the Analog measurements of a motor

     \param motor which motor [0..3]
     \param aOut1   reference to a variable to store the first analog measurement
     \param aOut2   reference to a variable to store the second analog measurement
    */
    void getAnalog(int motor, float &aOut1, float &aOut2);

    /*!
     \brief For testing: returns the Analog measurements of all motors

     \param aOut1 Float array to store the first analog measurement of each motor
     \param aOut2 Float array to store the second analog measurement of each motor
    */
    void getAnalogs(float * aOut1, float* aOut2);

    /*!
     \brief For testing: returns the dutycycles of all motors

     \param dc Int array to store the duty cycle of each motor
    */
    void getDutyCycles(int* dc);

    float getPGain() const;
    void setPGain(float value);

    Eigen::Vector3f getSetValue() const;
    void setSetValue(const Eigen::Vector3f value);

private:

    cPWM mPwm1; /*!< First PWM module (has 2 channels) */
    cPWM mPwm2; /*!< Second PwM module (has 2 channels) */

    Beagle_GPIO mBeagleGpio; /*!< Representation of the BeagleBoard Gpios (access via mmap) */
    Motor *mMotor[4]; /*!< Array of the 4 motors */

    Max127 mAnalog; /*!< Representation of the ADC for the motor feedback */
    float mPGain;
    Eigen::Vector3f mSetValue;

    MotorControl(const MotorControl& thread); /*!< Copy constructor made inaccessible by declaring it private */

    MotorControl& operator=(const MotorControl& rhs); /*!< Assignment constructor made inaccessible by declaring it private */
};

}

#endif // MOTORCONTROL_H
