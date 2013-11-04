#ifndef _L3G_h
#define _L3G_h

#include "I2CBus.h"

#define L3G_WHO_AM_I      0x0F

#define L3G_CTRL_REG1     0x20
#define L3G_CTRL_REG2     0x21
#define L3G_CTRL_REG3     0x22
#define L3G_CTRL_REG4     0x23
#define L3G_CTRL_REG5     0x24
#define L3G_REFERENCE     0x25
#define L3G_OUT_TEMP      0x26
#define L3G_STATUS_REG    0x27

#define L3G_OUT_X_L       0x28
#define L3G_OUT_X_H       0x29
#define L3G_OUT_Y_L       0x2A
#define L3G_OUT_Y_H       0x2B
#define L3G_OUT_Z_L       0x2C
#define L3G_OUT_Z_H       0x2D

#define L3G_FIFO_CTRL_REG 0x2E
#define L3G_FIFO_SRC_REG  0x2F

#define L3G_INT1_CFG      0x30
#define L3G_INT1_SRC      0x31
#define L3G_INT1_THS_XH   0x32
#define L3G_INT1_THS_XL   0x33
#define L3G_INT1_THS_YH   0x34
#define L3G_INT1_THS_YL   0x35
#define L3G_INT1_THS_ZH   0x36
#define L3G_INT1_THS_ZL   0x37
#define L3G_INT1_DURATION 0x38


/*!
 \brief Class to manage the communication to the L3G gyroscope via the I2C-bus

 \ingroup minimu

*/
class L3G
{
public:
    /*!
    \brief

    \param i2cDeviceName
    */
    L3G(const char * i2cDeviceName);

    int g[3]; /*!< Gyro raw angular velocity readings */

    /*!
     \brief Puts the chip into active sampling mode

    */
    void enable(void);

    /*!
     \brief Write value to register reg

     TODO: Make registers enum, so that writing to wrong register impossible?

     \param reg Register address to write to
     \param value Value to write to the register reg
    */
    void writeReg(uint8_t reg, uint8_t value);

    /*!
     \brief Read the value from register reg

     \param reg Register address to read from
     \return uint8_t Value read from the register reg
    */
    uint8_t readReg(uint8_t reg);

    /*!
     \brief Reads the current raw angular rates into \ref g
    */
    void read();

private:
    /*!
     \brief Tries to determine the correct I2C address of the gyroscope.

    */
    void detectAddress();
    I2CBus i2c; /*!< Handle to the I2C-bus */
};

#endif
