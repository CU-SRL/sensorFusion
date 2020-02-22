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

void IMU_LOOP() {
    IMU.sample(&imu_data); /*!< Sample the IMU by calling the IMU Sample function */
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
    Eigen::MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);

    delay(2500); /*!< Wait 2.5 seconds before starting everything up */

    Serial.begin(115200); /*!< Start serial comms */

    /* Initialize BNO055 IMU sensor */
    if (!IMU.begin()) {
        KILLSYSTEM();
    }
}

void loop()
{
    IMU_LOOP();
}