# include "fusion.hpp"

float State::eulerAngle(float GYRO, float gyro_sen, float gyro_samp) // Function to compute Euler angle
{
  float temp;
  return temp = (GYRO / gyro_sen) * gyro_samp;
}

// Computing the Quaternions
void State::quaternion(float phi, float theta, float psi, float &q_w, float &q_x, float &q_y, float &q_z)
{
    float cy = cos(psi * 0.5);
    float sy = sin(psi * 0.5);
    float cp = cos(theta * 0.5);
    float sp = sin(theta * 0.5);
    float cr = cos(phi * 0.5);
    float sr = sin(phi * 0.5);

    q_w = cy * cp * cr + sy * sp * sr;
    q_x = cy * cp * sr - sy * sp * cr;
    q_y = sy * cp * sr + cy * sp * cr;
    q_z = sy * cp * cr - cy * sp * sr;
}