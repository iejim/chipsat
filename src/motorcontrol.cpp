/**
 * @file motorcontrol.cpp
 *
 * C++ class for the calculation of the control response.
 * Based on the PeriodicRtThread class.
 *
 * @author Jan Sommer
 *  Created on: Apr 22, 2013
 *
 */

#include <fstream>
#include <stdexcept>

#include "motorcontrol.h"
using namespace USU;

MotorControl::MotorControl(const char *i2cDevice)
    :mPwm1(1), mPwm2(2), mAnalog(i2cDevice), mPGain(1.0)
{
    // Initizalize the four motors
    mMotor[0] = new Motor(mBeagleGpio, Beagle_GPIO::P8_31,Beagle_GPIO::P8_29, mPwm1, &cPWM::DutyA_percent);
    mMotor[1] = new Motor(mBeagleGpio, Beagle_GPIO::P8_27,Beagle_GPIO::P8_25, mPwm1, &cPWM::DutyB_percent);
    mMotor[2] = new Motor(mBeagleGpio, Beagle_GPIO::P8_23,Beagle_GPIO::P8_21, mPwm2, &cPWM::DutyA_percent);
    mMotor[3] = new Motor(mBeagleGpio, Beagle_GPIO::P8_18,Beagle_GPIO::P8_17, mPwm2, &cPWM::DutyB_percent);
    mPwm1.Period_freq(100);
    mPwm2.Period_freq(100);
    mPwm1.RunA();
    mPwm1.RunB();
    mPwm2.RunA();
    mPwm2.RunB();
}

MotorControl::~MotorControl()
{
    mPwm1.StopA();
    mPwm1.StopB();
    mPwm2.StopA();
    mPwm2.StopB();
}

void MotorControl::calculateControlResponse(Quaternion state)
{
    /// TODO: Make some control magic
//    mMotor[0]->setSpeed(20);
    /// [...]
}

void MotorControl::controlFromGyro(Eigen::Vector3f gyro)
{    
    float speeds[4];
    float currents[4];
    getAnalogs(speeds, currents);

    int speeds_input[4];
    getDutyCycles(speeds_input);

    // mPGain I already have this, do the math to go from speeds to rpms (or rad/s), then Eigen::Vector3f err = (gyro - mSetValue)
    // From err to 4 * pwms

}

void MotorControl::setMotor(int motor, int dutyCycle)
{
    mMotor[motor]->setSpeed(dutyCycle);
}

void MotorControl::getAnalog(int motor, float& aOut1, float& aOut2)
{
    aOut1 =  mAnalog.readVoltage(motor*2);
    aOut2 =  mAnalog.readVoltage(motor*2 + 1);
}

void MotorControl::getAnalogs(float *aOut1, float *aOut2)
{
    aOut1[0] =  mAnalog.readVoltage(0);
    aOut2[0] =  mAnalog.readVoltage(1);
    aOut1[1] =  mAnalog.readVoltage(2);
    aOut2[1] =  mAnalog.readVoltage(3);
    aOut1[2] =  mAnalog.readVoltage(4);
    aOut2[2] =  mAnalog.readVoltage(5);
    aOut1[3] =  mAnalog.readVoltage(6);
    aOut2[3] =  mAnalog.readVoltage(7);
}

void MotorControl::getDutyCycles(int *dc)
{
    dc[0] = mMotor[0]->getSpeed();
    dc[1] = mMotor[1]->getSpeed();
    dc[2] = mMotor[2]->getSpeed();
    dc[3] = mMotor[3]->getSpeed();
}
float MotorControl::getPGain() const
{
    return mPGain;
}

void MotorControl::setPGain(float value)
{
    mPGain = value;
}
Eigen::Vector3f MotorControl::getSetValue() const
{
    return mSetValue;
}

void MotorControl::setSetValue(const Eigen::Vector3f value)
{
    mSetValue = value;
}


