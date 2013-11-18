#include "motorcommunicator.h"

using namespace USU;


MotorCommunicator::MotorCommunicator(const char *i2cDevice)
    :mPwm1(1), mPwm2(2), mAnalog(i2cDevice)
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


MotorCommunicator::~MotorCommunicator()
{
    mPwm1.StopA();
    mPwm1.StopB();
    mPwm2.StopA();
    mPwm2.StopB();
}

void MotorCommunicator::setMotor(int motor, int dutyCycle)
{
    mMotor[motor]->setSpeed(dutyCycle);
}

void MotorCommunicator::getAnalog(int motor, float& aOut1, float& aOut2)
{
    aOut1 =  mAnalog.readVoltage(motor*2);
    aOut2 =  mAnalog.readVoltage(motor*2 + 1);
}

void MotorCommunicator::getAnalogs(float *aOut1, float *aOut2)
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

void MotorCommunicator::getDutyCycles(int *dc)
{
    dc[0] = mMotor[0]->getSpeed();
    dc[1] = mMotor[1]->getSpeed();
    dc[2] = mMotor[2]->getSpeed();
    dc[3] = mMotor[3]->getSpeed();
}
