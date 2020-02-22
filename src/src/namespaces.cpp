/**
 * @file namespaces.cpp
 * @brief The main namespace source file for the Code Base
 * 
 * 
 */

#include "fusion.hpp"

namespace matrices
{
    // BOTH DISCARDED AFTER FIRST RUN
    Eigen::VectorXd x_0(21); // Initial State Vector 
    Eigen::MatrixXd P_0(21,21); // Initial Process Covariance Matrix

    // VECTORS
    Eigen::VectorXd x_m(9); /* Observation Vector 9x1 */
    Eigen::VectorXd x_k(21); /* Final State Vector 21x1 */
    Eigen::VectorXd x_kp(21); /* State Prediction 21x1 */
    Eigen::VectorXd x_k_1(21); /* Previous State Vector (K-1) 21x1 */
    Eigen::VectorXd w_k(21); /* Predicted State Noise 21x1 */
    Eigen::VectorXd z_k(21); /* Measurement Noise 21x1 */

    // MATRICES
    Eigen::MatrixXd C(21,9); /* Observation Transition Matrix*/
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(21,21); /* State Transition Matrix */
    Eigen::MatrixXd P_kp(21,21); /* Predicted Process Covariance */
    Eigen::MatrixXd P_k_1(21,21); /* Previous Predicted Process Covariance */
    Eigen::MatrixXd R(21,21); /* Sensor Covariance Matrix */

    // Identity Matrix
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(21,21); /* 21x21 Identity Matrix */
};