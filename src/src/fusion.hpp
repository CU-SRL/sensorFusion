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
    double Temp = 0;
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

/* All lower case variables are vectors || all uppercase are Matrices*/
namespace matrices
{
    // BOTH DISCARDED AFTER FIRST RUN
    extern Eigen::VectorXd x_0; // Initial State Vector 
    extern Eigen::MatrixXd P_0; // Initial Process Covariance Matrix

    // VECTORS
    extern Eigen::VectorXd x_m; /* Observation Vector 9x1 */
    extern Eigen::VectorXd x_k; /* Final State Vector 21x1 */
    extern Eigen::VectorXd x_kp; /* State Prediction 21x1 */
    extern Eigen::VectorXd x_k_1; /* Previous State Vector (K-1) 21x1 */
    extern Eigen::VectorXd w_k; /* Predicted State Noise 21x1 */
    extern Eigen::VectorXd z_k; /* Measurement Noise 21x1 */

     // MATRICES
    extern Eigen::MatrixXd C; /* Observation Transition Matrix*/
    extern Eigen::MatrixXd A; /* State Transition Matrix */
    extern Eigen::MatrixXd P_kp; /* Predicted Process Covariance */
    extern Eigen::MatrixXd P_k_1; /* Previous Predicted Process Covariance */
    extern Eigen::MatrixXd R; /* Sensor Covariance Matrix */

    // Identity Matrix
    extern Eigen::MatrixXd I; /* 21x21 Identity Matrix */
 };

#endif
