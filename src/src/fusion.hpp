/**
 * @file fusion.hpp
 * @brief The main header file for the Code Base
 * 
 */

#ifndef _FUSION_HPP_
#define _FUSION_HPP_


//INITIAL STATE ENTER BEFORE FLIGHT
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

#define x_x_0 0 //initial position in x direction
#define x_y_0 0 //initial position in y direction
#define x_z_0 0 //initial position in z direction
#define v_x_0 0 // initial velocity in the x direction
#define v_y_0 0 // initial velocity in the y direction
#define v_z_0 0 // initial velocity in the z direction
#define a_x_0 0 // initial acceleration in x direction
#define a_y_0 0 // initial acceleration in y direction
#define a_z_0 0 // initial acceleration in z direction
#define theta_x_0 0 // initial attitude in the x direction
#define theta_y_0 0 // initial attitude in the y direction
#define theta_z_0 0 // initial attitude in the z direction
#define omega_x_0 0 // initial angular velocity in the x direction
#define omega_y_0 0 // initial angular velocity in the y direction
#define omega_z_0 0 // initial angular velocity in the z direction
#define alpha_x_0 0 // initial angular acceleration in the x direction
#define alpha_y_0 0 // initial angular acceleration in the y direction
#define alpha_z_0 0 // initial angular acceleration in the z direction
#define mag_x_0 0 //  nitial expected magnetic field reading in the x direction
#define mag_y_0 0 // initial expected magnetic field reading in the y direction
#define mag_z_0 0 // initial expected magnetic field reading in the z direction


#define p_x_x_0 0.1 //initial position variance in x direction
#define p_x_y_0 0.1 //initial position variance in y direction
#define p_x_z_0 0.1 //initial position variance in z direction
#define p_v_x_0 0.1 // initial velocity variance in the x direction
#define p_v_y_0 0.1 // initial velocity variance in the y direction
#define p_v_z_0 0.1 // initial velocity variance in the z direction
#define p_a_x_0 0.01 // initial acceleration variance in x direction
#define p_a_y_0 0.01 // initial acceleration variance in y direction
#define p_a_z_0 0.01 // initial acceleration variance in z direction
#define p_theta_x_0 0.1 // initial attitude variance in the x direction
#define p_theta_y_0 0.1 // initial attitude variance in the y direction
#define p_theta_z_0 0.1 // initial attitude variance in the z direction
#define p_omega_x_0 0.1 // initial angular velocity variance in the x direction
#define p_omega_y_0 0.1 // initial angular velocity variance in the y direction
#define p_omega_z_0 0.1 // initial angular velocity variance in the z direction
#define p_alpha_x_0 0.01 // initial angular acceleration variance in the x direction
#define p_alpha_y_0 0.01 // initial angular acceleration variance in the y direction
#define p_alpha_z_0 0.01 // initial angular acceleration variance in the z direction
#define p_mag_x_0 0 // initial expected magnetic field reading variance in the x direction
#define p_mag_y_0 0 // initial expected magnetic field reading variance in the y direction
#define p_mag_z_0 0 // initial expected magnetic field reading variance in the z direction


///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////



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
    float GYRO[3] = {0,0,0};             // Gyro in [DPS]
    float LINEAR_ACCEL[3] = {0,0,0};     // Accelerometer in [m/s^2]
    float GRAVITY_ACCEL[3] = {0,0,0};    // Accelerometer in [mg]
    float MAG[3] = {0,0,0};              // Magnetometer in [uT]
    float Quat[4] = {0,0,0,0};           // {w, x, y, z}
    float Temp = 0;
    uint32_t t = 0;

    float phi, theta, psi;                // Roll or phi (X), pitch or theta (Y), and yaw or psi (Z)
    float q_w, q_x, q_y, q_z;             // Quaternion
};

//! Main State Estimation Class
/*!
*
*/
class State
{
    private:
        float accel_error = 0.0;

        float gyro_error = 0.0;
        IMUdata *data;

    public:
        State(IMUdata*);
        ~State();

        void dataAq(IMUdata *data);

        void predict();
        void processCovarianceMatrix();
        void calculateKalmanGain();
        void stateDetermination();
        void updatePreviousState();
        void updateProcessCovarianceMatrix();
        void updateDynamics();
        // Eigen::MatrixXd dcmBodyToEarth(double theta, double phi, double psi);

        template<typename T>
        void print_mtxd(const T& X); 

        float calcAccelSystematicError();
        float calcGyroSystematicError();

        float eulerAngle(float GYRO, float gyro_sen, float gyro_samp); // Function to compute Euler angle
        void quaternion(float phi, float theta, float psi, float &q_w, float &q_x, float &q_y, float &q_z); // Function to computer Quaternion

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
        void sample(IMUdata* data, State* ptr);
};

namespace constants
{
    extern int interval_IMU;
    extern double dt;
    extern double baseAccel_error;
    extern double baseGyro_error;
    extern float gyro_sen;           // = 900 rad/sec
};

/* All lower case variables are vectors || all uppercase are Matrices*/
namespace matrices
{
    // BOTH DISCARDED AFTER FIRST RUN
    extern Eigen::VectorXd x_0; // Initial State Vector 
    extern Eigen::MatrixXd P_0; // Initial Process Covariance Matrix

    // VECTORS
    extern Eigen::VectorXd y; /* Observation Vector */
    extern Eigen::VectorXd x_m; /* Direct Observation Vector */
    extern Eigen::VectorXd x_k; /* Final State Vector */
    extern Eigen::VectorXd x_kp; /* State Prediction */
    extern Eigen::VectorXd x_k_1; /* Previous State Vector (K-1) */
    extern Eigen::VectorXd w_k; /* Predicted State Noise */
    extern Eigen::VectorXd z_k; /* Measurement Noise */

    // MATRICES
    extern Eigen::MatrixXd C; /* Observation Transition Matrix*/
    extern Eigen::MatrixXd A; /* State Transition Matrix */
    extern Eigen::MatrixXd P_kp; /* Predicted Process Covariance */
    extern Eigen::MatrixXd P_k_1; /* Previous Predicted Process Covariance */
    extern Eigen::MatrixXd R; /* Sensor Covariance Matrix */
    extern Eigen::MatrixXd H; /* Observation model mapping matrix*/
    extern Eigen::MatrixXd K; /* Kalman Gain */

    // Identity Matrix
    extern Eigen::MatrixXd I; /* 21x21 Identity Matrix */
 };

#endif
