#ifndef _IMU_H
#define _IMU_H

#include "vector.h"

/*!
 \brief Virtual base class for IMU

 Derive this class to make your own IMU-class.

 \ingroup minimu
*/
class IMU {
public:
    // Scaled readings
    virtual vector readMag() = 0;  // In body coords, scaled to -1..1 range
    virtual vector readAcc() = 0;  // In body coords, with units = g
    virtual vector readGyro() = 0; // In body coords, with units = rad/sec
    void read(){ readAcc(); readMag(); readGyro(); }

    virtual void enable() = 0;
//    virtual void measureOffsets() = 0;
//    virtual void loadCalibration() = 0;

//    vector gyro_offset;
//    matrix calMatrix;
//    vector calOffset;

    int_vector raw_m, raw_a, raw_g;
};

#endif
