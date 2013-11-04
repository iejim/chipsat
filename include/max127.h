/**
 * @file max127.h
 *
 * C++ class for the ADC Max127.
 *
 * @author Jan Sommer
 *  Created on: May 20, 2013
 *
 */

#ifndef MAX127_H
#define MAX127_H

#include <stdint.h>

#include "I2CBus.h"

namespace USU
{

/*!
 \brief I2C-address of the ADC

 It is assumed that the PINs A0-A2 are connected to GND.
 If the PINs are connected to VCC change accordingly.
*/
const uint8_t I2C_ADDRESS  = 0b00101000;

/*!
 \brief Template of the control byte

 The used settings are_
    - fullscale range +-5V
    - Standby Power-Down mode

 The bits for channel selection are set to 0.
 Send CONTROL_BYTE | (CH<<SEL0) with CH being the
 desired channel via the I2CBus.
*/
const uint8_t CONTROL_BYTE = 0b10000110;
const uint8_t SEL0         = 4; /*!< Bit offset for channel selection */

/*!
 \brief Class representing the MAX127 ADC

 Provides simple functionality to read the channels.
 Uses the I2CBus class for communication.

 \ingroup motor

*/
class Max127
{
public:
   /*!
    \brief Constructor

    Initializes the I2C-connection

    \param i2cdevice device name of the i2c-bus (e.g. /dev/i2c-1)
   */
    Max127(const char *i2cdevice);

    /*!
     \brief Returns the raw integer measurement of the selected channel

     At the moment assumens bipolar operation.
     The range is [-2048, 2047]

     \param channel channel to read
     \return int16_t signed integer representing the measurement
    */
    int16_t readRaw(uint8_t channel);
    /*!
     \brief Returns the measurement of the selected channel in volts

     At the moment assumes fullscale of 10 V (bipolar +-5V or unipolar)

     \param channel channel to read
     \return float  measured voltage in V
    */
    float readVoltage(unsigned int channel);

private:
    I2CBus mI2c; /*!< Object for the used I2C-bus */


};

}

#endif // MAX127_H
