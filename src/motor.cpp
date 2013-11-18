/**
 * @file motor.cpp
 *
 * Class to represent a motor
 *
 * @author Jan Sommer
 *  Created on: Apr 22, 2013
 *
 */

#include "motor.h"
using namespace USU;

Motor::Motor(Beagle_GPIO &beagleGpio, Beagle_GPIO::Pins clockwise, Beagle_GPIO::Pins counterClockwise, cPWM &pwm, SetDutyCyle dutyCycle)
    :mBeagleGpio(beagleGpio), mClockwise(clockwise), mCounterClockwise(counterClockwise),
      mPwm(pwm), mSetDutyCycle(dutyCycle), mSpeed(0)
{
    mBeagleGpio.configurePin(mClockwise, Beagle_GPIO::kOUTPUT);
    mBeagleGpio.enablePinInterrupts( mClockwise, false );
    mBeagleGpio.writePin(mClockwise, 0);

    mBeagleGpio.configurePin(mCounterClockwise, Beagle_GPIO::kOUTPUT);
    mBeagleGpio.enablePinInterrupts( mCounterClockwise, false );
    mBeagleGpio.writePin(mCounterClockwise, 0);

    setSpeed(mSpeed);
}

void Motor::setSpeed(int speed)
{
    if (speed > 0)
    {
        // Make sure speed <100
        speed = speed<100 ? speed : 99;
        mBeagleGpio.writePin(mClockwise, 1);
        mBeagleGpio.writePin(mCounterClockwise, 0);
        (mPwm.*mSetDutyCycle)(speed);
        mSpeed = speed;
    }
    else if (speed < 0)
    {
        // Make sure speed >-100
        speed = speed>-100 ? speed : -99;
        mBeagleGpio.writePin(mClockwise, 0);
        mBeagleGpio.writePin(mCounterClockwise, 1);
        (mPwm.*mSetDutyCycle)(-speed);
        mSpeed = speed;
    }
    else
    {
        mBeagleGpio.writePin(mClockwise, 0);
        mBeagleGpio.writePin(mCounterClockwise, 0);
        (mPwm.*mSetDutyCycle)(9);
        mSpeed = 0;
    }
}
