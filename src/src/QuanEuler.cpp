# include "fusion.hpp"
#define PI 3.1415926535897932384626433832795


float State::eulerAngle(float GYRO, float gyro_sen, float gyro_samp) // Function to compute Euler angle
{
  float temp;
  temp = (GYRO  * gyro_samp);
  return temp;
}

// Computing the Quaternions
void State::quaternion(float phi, float theta, float psi, float &q_w, float &q_x, float &q_y, float &q_z)
{
  float degtorad = PI / (float)180;
  float _x = phi * degtorad;
  float _y = theta * degtorad;
  float _z = psi * degtorad;
  // float cphi = cos(_x / (float)2);
  // float cthe = cos(_y / (float)2);
  // float cpsi = cos(_z / (float)2);
  // float sphi = sin(_x / (float)2);
  // float sthe = sin(_y / (float)2);
  // float spsi = sin(_z / (float)2);
  // q_w = cos ((phi + psi) / 2) * cos (theta / 2);
  // q_x = cos ((phi - psi) / 2) * sin (theta / 2);
  // q_y = sin ((phi - psi) / 2) * sin (theta / 2);
  // q_z = sin ((phi + psi) / 2) * cos (theta / 2);

  q_w = cos ((_x + _z) / 2) * cos (_y / 2);
  q_x = cos ((_x - _z) / 2) * sin (_y / 2);
  q_y = sin ((_x - _z) / 2) * sin (_y / 2);
  q_z = sin ((_x + _z) / 2) * cos (_y / 2);


  // q_w = (cphi * cthe * cpsi) + (sphi * sthe * spsi);
  // q_x = (sphi * cthe * cpsi) - (cphi * sthe * spsi);
  // q_y = (cphi * sthe * cpsi) + (sphi * cthe * spsi);
  // q_z = (cphi * cthe * spsi) - (sphi * sthe * cpsi);
  // Roll or phi (X), pitch or theta (Y), and yaw or psi (Z)
  // alpha = z, beta = x, gamma = y
}