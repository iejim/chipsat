#include <iostream>
using std::cin;
using std::cout;
using std::endl;

#include <cmath>
#include "Eigen/Dense"
#include "Eigen/Geometry"

// Function to read in a matrix from an input stream
namespace Eigen {
template<typename Derived>
std::istream & operator >>(std::istream & Inp, MatrixBase<Derived> & m)
{
  for (int i = 0; i < m.rows(); ++i)
    for (int j = 0; j < m.cols(); j++)
      Inp >> m(i,j);

  return Inp;
}

}


/*!
 \brief Control function

 Uses the input from the IMU (orientation matrix and gyro vector) and
 the ADC voltage values for each motor (ch1: current? ch2: ??) to
 compute the set value for the motors

 \param orientation Orientation matrix from the IMU
 \param gyro Angular rate vector from the IMU
 \param ch1 Voltages representing the current(??) for each motor
 \param ch2 Voltages represnting the ???? for each motor
*/
void controlTest(Eigen::Matrix3f &orientation, Eigen::Vector3f &gyro, Eigen::Vector4f &ch1, Eigen::Vector4f &ch2)
{
    // Create quaternion from orientation matrix
    Eigen::Quaternion<float> q(orientation);

    // 4-Element Vector to store the results
    Eigen::Vector4f motorSpeeds;
    // [...]
    // computation
    // [...]
    // final set Values in motorSpeeds


    // Will be replaced by the correct set-function in the final program
    cout << "Resulting motor speed values are:" << endl;
    cout << motorSpeeds << endl;
}


/*!
 \brief Read simulation values from Input and call control function

 Reads a number of simulation values from cin and passes them
 to the test function. The resulting data can be used to validate the
 implementation.

 \return int
*/
int main()
{
    // No validation of input values! Make sure you match the expected types.
    unsigned numDataSet; // number data sets with sampling data
    cin >> numDataSet;

    for(unsigned i=0; i<numDataSet; ++i)
    {
        Eigen::Matrix3f m;
        Eigen::Vector3f gyro;
        Eigen::Vector4f ch1, ch2, quat;
        Eigen::Quaternionf qref;

        cin >> m;
        cin >> gyro;
        cin >> ch1;
        cin >> ch2;
        cin >> quat;
        qref = quat;

        controlTest(m, gyro, ch1, ch2);
    }

 return 0;
}
