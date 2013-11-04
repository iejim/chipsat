/**
 * @file motor.h
 *
 * Class to represent a motor
 *
 * @author Jan Sommer
 *  Created on: Apr 22, 2013
 *
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "cPWM.h"
#include "Beagle_GPIO.h"

/*!
 \brief Function-pointer to the SetDutyCyle-method of cPWM class

 Each cPWM object has 2 channels (A and B). Each motor gets assigned to one of the
 channels using the corresponding function pointer.
*/
typedef void (cPWM::*SetDutyCyle)(unsigned int);

namespace USU
{

/*!
 \brief Class which represents a motor

 It controls 2 digital pins to set motor spin direction
 and one PWM channel to set motor speed.

 \ingroup motor

*/
class Motor
{
public:
    /*!
    \brief Constructor

     \param beagleGpio Reference to a Beagle_GPIO object to set the pins
     \param clockwise  First pin needed to set motor direction
     \param counterClockwise Second pin needed to set motor direction
     \param pwm Reference to the cPWM-object, which controls the PWM
     \param dutyCycle Function to set the dutyCycle of the PWM-channel assigned to the motor
    */
    Motor(Beagle_GPIO& beagleGpio, Beagle_GPIO::Pins clockwise,  Beagle_GPIO::Pins counterClockwise,
          cPWM& pwm, SetDutyCyle dutyCycle);
    /*!
     \brief Set the speed of the motor in percent.

     \param speed desired motor speed (-100, 100)
    */
    void setSpeed(int speed);

    /*!
     \brief Return the current speed of the motor

     \return int current Speed of the motor
    */
    int getSpeed() const { return mSpeed; } /// TODO: protect with mutex?

private:
    Beagle_GPIO& mBeagleGpio; /*!< Object which represents the interface to the GPIOs of the BB */
    Beagle_GPIO::Pins mClockwise; /*!< Pin for the Clockwise signal*/
    Beagle_GPIO::Pins mCounterClockwise; /*!< Pin for the Counterclockwise signal*/

    cPWM &mPwm; /*!< Object of the PWM */
    SetDutyCyle mSetDutyCycle; /*!< Function to set the assigned PWM channel */
    int mSpeed; /*!< Keeps track of the set speed */
};

}

#endif // MOTOR_H
