/**
 * @file fusion.hpp
 * @brief The main header file for the Code Base
 * 
 */

#ifndef _FUSION_HPP_
#define _FUSION_HPP_

#include "Arduino.h"
#include "Eigen.h"
#include <Eigen/Eigen>
#include <Adafruit_BNO055.h>
#include "Thread.h"
#include <ThreadController.h>

//! IMU Struct
/*!
*   This structs holds the BNO055 sample at a point in time to be stored and processed.
*/
struct IMUdata {
    double GYRO[3] = {0,0,0}; // Gyro in [DPS]
    double LINEAR_ACCEL[3] = {0,0,0}; // Accelerometer in [m/s^2]
    double GRAVITY_ACCEL[3] = {0,0,0}; // Accelerometer in [mg]
    double MAG[3] = {0,0,0}; // Magnetometer in [uT]
    uint32_t t = 0;
};

//! Main State Estimation Class
/*!
*
*/
class State
{
    private:

    public:
        State();
        ~State();

    protected:
};

//! Main Kalman Filter Class
/*!
*
*/
class Kalman
{
    private:

    public:
        Kalman();
        ~Kalman();

    protected:
};

//! BNO055 IMU Class
/*!
*   Class to manage the Adafruit BNO055 Absolute Orientation IMU Fusion breakout board
*/
class DigitalIMU {
    private:
        Adafruit_BNO055 board;
        sensors_event_t event;
        imu::Quaternion quat;
        imu::Vector<3> accel;
    public:
        DigitalIMU();
        DigitalIMU(int32_t sensorID, uint8_t address);
        bool begin();
        void sample(IMUdata* data);
};

#endif
