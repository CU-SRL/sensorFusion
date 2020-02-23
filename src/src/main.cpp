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

ThreadController thread_control = ThreadController();
Thread* ThreadIMU = new Thread(); 

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
    delay(5000); /*!< Wait 2.5 seconds before starting everything up */

    Serial.begin(115200); /*!< Start serial comms */

    /* Initialize BNO055 IMU sensor */
    if (!IMU.begin()) {
        KILLSYSTEM();
    }
    State state(&imu_data);

    ThreadIMU->onRun(IMU_LOOP); /*!< Set the IMU looping function for the ThreadController */
    ThreadIMU->setInterval(constants::interval_IMU); /*!< Set the IMU refresh rate (Interval) */

    /* Add threads to ThreadController */
    thread_control.add(ThreadIMU);
}

void loop()
{
    thread_control.run();
}