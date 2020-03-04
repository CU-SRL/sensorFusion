/**
 * @file namespaces.cpp
 * @brief The main namespace source file for the Code Base
 * 
 * 
 */

#include "fusion.hpp"

namespace constants
{
    int interval_IMU = 10; // IMU sample rate (OLD 45)
    double dt = (double)interval_IMU/(double)1000; // delta t in seconds
    double baseAccel_error = 0.001862*sqrt(1/dt); // in m/s^2
    double baseGyro_error = 0.1; // in DPS
    float gyro_sen = 51566.2;            // = 900 rad/sec
};

namespace matrices
{
    // BOTH DISCARDED AFTER FIRST RUN
    Eigen::VectorXd x_0(21); // Initial State Vector 
    Eigen::MatrixXd P_0 = Eigen::MatrixXd::Zero(21,21); // Initial Process Covariance Matrix

    // VECTORS
    Eigen::VectorXd y(21); /* Corrected Observation Vector */
    Eigen::VectorXd x_m(21); /* Direct Observation Vector */
    Eigen::VectorXd x_k(21); /* Final State Vector */
    Eigen::VectorXd x_kp(21); /* State Prediction */
    Eigen::VectorXd x_k_1(21); /* Previous State Vector (K-1) */
    Eigen::VectorXd w_k(21); /* Predicted State Noise */
    Eigen::VectorXd z_k(21); /* Measurement Noise */

    // MATRICES
    // Eigen::MatrixXd C(21,9); /* Observation Transition Matrix*/
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(21,21); /* State Transition Matrix */
    Eigen::MatrixXd P_kp(21,21); /* Predicted Process Covariance */
    Eigen::MatrixXd P_k_1(21,21); /* Previous Predicted Process Covariance */
    Eigen::MatrixXd R(21,21); /* Sensor Covariance Matrix */
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6,21); /* H or C matrix, This transitions our state to what we expect our measurements to be
                                Essentially it deletes all of the elements of our state prediction that can't be measure directly */
    Eigen::MatrixXd K(21,6); /* Kalman Gain */

    // Identity Matrix
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(21,21); /* 21x21 Identity Matrix */
};