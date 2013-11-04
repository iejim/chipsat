/**
 * @file max127.cpp
 *
 * C++ class for the ADC Max127.
 *
 * @author Jan Sommer
 *  Created on: May 20, 2013
 *
 */

#include "max127.h"
using namespace USU;

Max127::Max127(const char *i2cdevice)
    :mI2c(i2cdevice)
{
    mI2c.addressSet(I2C_ADDRESS);
}

int16_t Max127::readRaw(uint8_t channel)
{
    // Compile the full control byte by setting the channel bits
    mI2c.writeByte(CONTROL_BYTE | (channel << SEL0) );

    uint16_t rawValue = mI2c.readWord();


    // From the read word use use the high byte as low byte and vice versa
    // Then move all bits 4 to the left because it is only a 12 bit number.
    return ( (int16_t) ( ( (rawValue & 0xFF00)>>8) | ( (rawValue & 0x00FF )<<8) ) >> 4 );


}

float Max127::readVoltage(unsigned int channel)
{
    // fullscale = +-5V --> resolution = 10V/4096
    const float scaleVoltage = 2.4414063e-3f;
    return (readRaw(channel) * scaleVoltage);
}
