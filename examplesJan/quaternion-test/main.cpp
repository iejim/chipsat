#include <iostream>
using std::cin;
using std::cout;
using std::endl;

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// constant for cos(45°) == sin(45°)
const float rot = 0.5*sqrt(2.0);

// Helper functions for manual quaternion computation
inline float SIGN(float x) {return (x >= 0.0f) ? +1.0f : -1.0f;}
inline float NORM(float a, float b, float c, float d) {return sqrt(a * a + b * b + c * c + d * d);}


// Function to read in a matrix from cin
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
 \brief Use a orientation matrix and compute corresponding quaternion

 Ask the user to input a rotation matrix. If "n" use default rotation
 matrix to rotate 45° around the z-axis.

 Compute the corresponding quaterion:
    1. Using the inbuilt functionality of the Eigen-library
    2. Manually

    Print out both results.

 \return int
*/
int main()
{   
    
  Eigen::Matrix3f m;
  // Rotation matrix for rotation around z-axis with 45° (default value)
  m << rot, -rot ,  0.0f,
       rot,  rot ,  0.0f,
       0.0f, 0.0f,  1.0f;
  
  cout << "Use custom rotation matrix? (y/n)";
  char c;
  cin >> c;
  if (c == 'y')
    cin >> m;  
  
  cout << endl << "Rotation matrix is: " << endl;     
  cout << m << endl;
  
  // Convert rotation matrix to quaternion (the easy way)
  Eigen::Quaternion<float> q(m);
  cout << "Corresponding quaternion is:" << endl;
  cout << q.w() << "\t" << q.x() << "\t" << q.y() << "\t" << q.z()  << endl;
  
  
  // Compute quaternion manually using:
  // http://www.cg.info.hiroshima-cu.ac.jp/~miyazaki/knowledge/teche52.html

  cout << "Manual quaternion is: " << endl; 
  float r11 = m(0,0), r12 = m(0,1), r13 = m(0,2);
  float r21 = m(1,0), r22 = m(1,1), r23 = m(1,2);
  float r31 = m(2,0), r32 = m(2,1), r33 = m(2,2);
  float r;
  float q0, q1, q2, q3;
  q0 = ( r11 + r22 + r33 + 1.0f) / 4.0f;
  q1 = ( r11 - r22 - r33 + 1.0f) / 4.0f;
  q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
  q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;
  if(q0 < 0.0f) q0 = 0.0f;
  if(q1 < 0.0f) q1 = 0.0f;
  if(q2 < 0.0f) q2 = 0.0f; 
  if(q3 < 0.0f) q3 = 0.0f;
  q0 = sqrt(q0);
  q1 = sqrt(q1);
  q2 = sqrt(q2);
  q3 = sqrt(q3);
  if(q0 >= q1 && q0 >= q2 && q0 >= q3) {
     q0 *= +1.0f;
     q1 *= SIGN(r32 - r23);
     q2 *= SIGN(r13 - r31);
     q3 *= SIGN(r21 - r12);
 } else if(q1 >= q0 && q1 >= q2 && q1 >= q3) {
     q0 *= SIGN(r32 - r23);
     q1 *= +1.0f;
     q2 *= SIGN(r21 + r12);
     q3 *= SIGN(r13 + r31);
 } else if(q2 >= q0 && q2 >= q1 && q2 >= q3) {
     q0 *= SIGN(r13 - r31);
     q1 *= SIGN(r21 + r12);
     q2 *= +1.0f;
     q3 *= SIGN(r32 + r23);
 } else if(q3 >= q0 && q3 >= q1 && q3 >= q2) {
     q0 *= SIGN(r21 - r12);
     q1 *= SIGN(r31 + r13);
     q2 *= SIGN(r32 + r23);
     q3 *= +1.0f;
 } else {
     cout << "coding error" << endl;
 }
 r = NORM(q0, q1, q2, q3);
 q0 /= r;
 q1 /= r;
 q2 /= r;
 q3 /= r;
 
 cout << q0 << "\t" << q1 << "\t" << q2 << "\t" << q3 << endl;
 
 return 0;
}
