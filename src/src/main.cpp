/**
 * @file main.cpp
 * @brief The main source file for the Code Base
 * 
 * Furthermore, the void setup() and void loop() functions are defined here also.
 * 
 */

/***************************************************************/
/*                                                             */
/*                                                             */
/*                    CU-SRL AVIONICS TEAM                     */
/*            STATE ESTIMATION OF A DYNAMIC VEHICLE            */
/*                                                             */
/*                       Lyon Foster                           */
/*                       Jason Popich                          */
/*                   Madisen Purifoy-Frie                      */
/*                       Daniel Crook                          */
/*                                                             */
/***************************************************************/
#include "fusion.hpp"


// GLOBAL VARIABLES I KNOW BAD... FIX LATER
DigitalIMU IMU = DigitalIMU(55,0x28);
IMUdata imu_data;
State *state;

ThreadController thread_control = ThreadController();
Thread* ThreadIMU = new Thread(); 
Thread* ThreadKalman = new Thread();

void IMU_LOOP() {
    IMU.sample(&imu_data, state); /*!< Sample the IMU by calling the IMU Sample function */
}

void Kalman_LOOP()
{
    state->updateDynamics();
}

void KILLSYSTEM()
{
    while(true)
    {
        Serial.println("Failed to Init!!");
    }
}

void setup()
{
    delay(5000); /*!< Wait 2.5 seconds before starting everything up */

    Serial.begin(115200); /*!< Start serial comms */

    /* Initialize BNO055 IMU sensor */
    if (!IMU.begin()) {
        KILLSYSTEM();
    }

    state = new State(&imu_data);

    ThreadIMU->onRun(IMU_LOOP); /*!< Set the IMU looping function for the ThreadController */
    ThreadIMU->setInterval(constants::interval_IMU); /*!< Set the IMU refresh rate (Interval) */

    ThreadKalman->onRun(Kalman_LOOP);
    ThreadKalman->setInterval(constants::interval_IMU);

    /* Add threads to ThreadController */
    thread_control.add(ThreadIMU);
    thread_control.add(ThreadKalman);

    matrices::x_0 << 
                    x_x_0,
                    x_y_0,
                    x_z_0,
                    v_x_0,
                    v_y_0,
                    v_z_0,
                    a_x_0,
                    a_y_0,
                    a_z_0,
                    theta_x_0,
                    theta_y_0,
                    theta_z_0,
                    omega_x_0,
                    omega_y_0,
                    omega_z_0,
                    alpha_x_0,
                    alpha_y_0,
                    alpha_z_0,
                    mag_x_0,
                    mag_y_0,
                    mag_z_0;

    Eigen::Matrix3d p;
    double x = p_x_x_0;
    p << pow(x,2), 0, 0,
         0, pow(x,2), 0,
         0, 0, pow(x,2);
    matrices::P_0.block<3,3>(0,0) = p;

    double v = p_v_x_0;
    p << pow(v,2), 0, 0,
         0, pow(v,2), 0,
         0, 0, pow(v,2);
    matrices::P_0.block<3,3>(3,3) = p;

    double a = p_a_x_0;
    p << pow(a,2), 0, 0,
         0, pow(a,2), 0,
         0, 0, pow(a,2);
    matrices::P_0.block<3,3>(6,6) = p;

    double theta = p_theta_x_0;
    p << pow(theta,2), 0, 0,
         0, pow(theta,2), 0,
         0, 0, pow(theta,2);
    matrices::P_0.block<3,3>(9,9) = p;

    double omega = p_omega_x_0;
    p << pow(omega,2), 0, 0,
         0, pow(omega,2), 0,
         0, 0, pow(omega,2);
    matrices::P_0.block<3,3>(12,12) = p;

    double alpha = p_alpha_x_0;
    p << pow(alpha,2), 0, 0,
         0, pow(alpha,2), 0,
         0, 0, pow(alpha,2);
    matrices::P_0.block<3,3>(15,15) = p;

    double mag = p_mag_x_0;
    p << pow(mag,2), 0, 0,
         0, pow(mag,2), 0,
         0, 0, pow(mag,2);
    matrices::P_0.block<3,3>(18,18) = p;

    matrices::x_k_1 = matrices::x_0;
    matrices::P_k_1 = matrices::P_0;

    // while (1 = 1)
    // {
    //     if (sqrt(imu_data.LINEAR_ACCEL[0]^2 + imu_data.LINEAR_ACCEL[1]^2 + imu_data.LINEAR_ACCEL[2]^2) > 20)
    //     {
    //         break;
    //     }  
    // } 
}

void loop()
{
    thread_control.run();
}