/**
 * @file minimu.h
 *
 * C++ MinIMU9v2.
 *
 * @author Jan Sommer
 *  Created on: Apr 20, 2013
 *
 */

#ifndef MINIMU_H
#define MINIMU_H

#include <iostream>

#include "IMU.h"
#include "LSM303.h"
#include "L3G.h"
#include "vector.h"

namespace USU
{



/*!
 \brief Class to manage the communication to the Pololu MinIMU9

 \ingroup minimu

*/
class MinImu : public IMU
{
public:
    LSM303 compass; /*!< Compass (i.e. Accelerometer and Magnetometer of the IMU) */
    L3G gyro; /*!< Gyroscope of the IMU*/

    /*!
    \brief Constructor

    Initializes the compass and gyroscope.

    \param i2cDeviceName  Name of the I2C device the IMU is connected to
    */
    MinImu(const char * i2cDeviceName);

    /*!
     \brief Reads the magnetometer and return a \ref vector of raw values

     TODO: Transform into gauss?

     \return vector
    */
    virtual vector readMag();  // In body coords, currently without scaling

    /*!
     \brief Reads the accelerometer and return a \ref vector with units in g

     \return vector
    */
    virtual vector readAcc();  // In body coords, with units = g

    /*!
     \brief Reads the gyroscope and returns a \ref vector with units in degrees/s

     \return vector
    */
    virtual vector readGyro(); // In body coords, with units = degrees/sec

    /*!
     \brief Enables compass and gyroscope, i.e. starts the sampling on these devices
    */
    virtual void enable();
};

}

#endif // MINIMU_H
