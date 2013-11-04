#include "minimu.h"
using namespace USU;
#include "IMU.h"

MinImu::MinImu(const char *i2cDeviceName)
    :compass(i2cDeviceName), gyro(i2cDeviceName)
{

}

void MinImu::enable()
{
    compass.enable();
    gyro.enable();
}

vector MinImu::readGyro()
{
    // At the full-scale=250 dps setting, the gyro datasheet says
    // we get 8.75 mdps/digit.
    const float gyro_scale = 0.00875; // in °/s

    gyro.read();
    IMU::raw_g = int_vector_from_ints(&gyro.g);
    return ( vector_from_ints(&gyro.g)  * gyro_scale);
}

vector MinImu::readAcc()
{
    // LSM303 accelerometer: At 2 g sensitivity, the datasheet says
    // we get 1 mg/digit.
    const float accel_scale = 0.0010;

    compass.readAcc();
    IMU::raw_a = int_vector_from_ints(&compass.a);
    return vector_from_ints(&compass.a) * accel_scale;
}

vector MinImu::readMag()
{
    compass.readMag();
    IMU::raw_m = int_vector_from_ints(&compass.m);

    return vector_from_ints(&compass.m);
}
